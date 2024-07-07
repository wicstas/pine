#include <pine/core/jit_compiler.h>
#include <pine/core/profiler.h>
#include <pine/core/vecmath.h>
#include <pine/core/atomic.h>

#include <psl/algorithm.h>
#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/memory.h>
#include <psl/array.h>
#include <psl/set.h>

#include "llvm/ADT/STLExtras.h"
#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "llvm/ExecutionEngine/GenericValue.h"
#include "llvm/ExecutionEngine/MCJIT.h"
#include "llvm/IR/Argument.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/ConstantFolder.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/DynamicLibrary.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Passes/OptimizationLevel.h"

namespace pine {

// ================================================
// Source
// ================================================
struct SourceLoc {
  SourceLoc() = default;
  SourceLoc(size_t row, size_t column) : row(row), column(column) {
  }
  size_t row = size_t(-1);
  size_t column = size_t(-1);
};
struct SourceLines {
  SourceLines() = default;
  SourceLines(psl::string_view tokens, size_t paddings);

  psl::optional<psl::string_view> next_line(size_t row) const;

  psl::optional<char> next(SourceLoc sl) const;

  template <typename... Args>
  [[noreturn]] void error(SourceLoc sl, const Args&... args) const {
    error_impl(sl, psl::to_string(args...));
  }

  [[noreturn]] void error_impl(SourceLoc sl, psl::string_view message) const;

private:
  psl::vector<psl::string> lines;
  size_t paddings = invalid;
  static constexpr size_t invalid = size_t(-1);
};
static psl::vector<psl::string> split(psl::string_view input, auto pred) {
  auto parts = psl::vector<psl::string>{};
  auto start = input.begin();
  while (true) {
    auto end = psl::find_if(psl::range(start, input.end()), pred);
    parts.push_back(psl::string(start, end));
    if (end == input.end())
      break;
    else
      start = psl::next(end);
  }

  return parts;
}
SourceLines::SourceLines(psl::string_view tokens, size_t paddings)
    : lines(split(tokens, psl::equal_to_any('\n', '\r', '\f'))), paddings(paddings) {
}
psl::optional<psl::string_view> SourceLines::next_line(size_t row) const {
  if (row == lines.size())
    return psl::nullopt;
  return lines[row];
}
psl::optional<char> SourceLines::next(SourceLoc sl) const {
  if (auto line = next_line(sl.row))
    return (*line)[sl.column];
  return psl::nullopt;
}
[[noreturn]] void SourceLines::error_impl(SourceLoc sl, psl::string_view message) const {
  auto line_at = [this](int64_t i) {
    if (i < 0 || i >= int64_t(lines.size()))
      return psl::string();
    else
      return lines[i];
  };

  auto vicinity = psl::string();
  for (int64_t i = sl.row - paddings; i <= int64_t(sl.row); i++)
    vicinity += " | " + line_at(i) + "\n";
  vicinity += "  -" + psl::string_n_of(sl.column, '-') + "^\n";
  for (size_t i = sl.row + 1; i <= sl.row + paddings; i++)
    vicinity += " | " + line_at(i) + "\n";
  if (vicinity.size())
    vicinity.pop_back();

  Fatal(message, "\n", vicinity);
}

// ================================================
// Module
// ================================================
struct Value {
  enum ValueCategory { L, R };
  Value() = default;
  Value(llvm::Value* ptr, psl::string type_name, ValueCategory category = ValueCategory::R)
      : ptr(ptr), type_name(MOVE(type_name)), category(category) {
  }
  explicit operator bool() const {
    return ptr;
  }
  bool is_l_value() const {
    return category == ValueCategory::L;
  }
  bool is_r_value() const {
    return category == ValueCategory::R;
  }

  llvm::Value* ptr = nullptr;
  psl::string type_name;
  ValueCategory category;
};
struct Module {
  struct TypeInfo {
    llvm::Type* ptr = nullptr;
    llvm::Function* destructor = nullptr;
  };
  struct FunctionInfo {
    llvm::Function* ptr = nullptr;
    const Function* f = nullptr;
  };

  // ================================================
  // Module construction
  // ================================================
  [[noreturn]] void error(SourceLoc loc, const auto&... args) const {
    source.error(loc, args...);
  }

  Context::FindUniqueFResult find_unique_f(psl::string_view name) const {
    return context.find_unique_f(name);
  }
  size_t find_unique_f(SourceLoc sl, psl::string_view name) const {
    if (auto fr = context.find_unique_f(name))
      return fr.fi;
    else if (fr.error == fr.FindNone)
      error(sl, "Unable to find function `", name, '`');
    else
      error(sl, "`", name, " is ambiguous");
  }
  auto find_f(psl::string_view name, psl::span<const pine::TypeTag> atypes) const {
    return context.find_f(name, atypes);
  }
  auto find_f(psl::string_view name, psl::span<const pine::Value> args) const {
    return find_f(name, psl::transform_vector(args, [](auto&& x) { return TypeTag(x.type_name); }));
  }

  void add_type(psl::string name, llvm::Type* ptr, llvm::Function* destructor) {
    types[name] = {ptr, destructor};
  }
  TypeInfo type(const TypeTag& tag) {
    if (tag.is_ref)
      return {llvm::PointerType::get(C, 0), nullptr};
    else if (is_function_type(tag.name))
      return {function_object_type(), nullptr};
    else if (auto type_info = psl::find_or_nullopt(types, tag.name))
      return *type_info;
    else
      Fatal("Type `", tag.name, "` is not registered");
  }
  const TypeInfo type(psl::string name) {
    return type(TypeTag(name));
  }
  size_t type_size(psl::string_view name) {
    if (is_function_type(name))
      return sizeof(psl::pair<void*, void*>);
    else
      return context.get_type_trait(name)->byte_size;
  }

  struct ClassMemberInfo {
    explicit operator bool() const {
      return index != size_t(-1);
    }
    psl::string type_name;
    size_t index = size_t(-1);
  };
  void set_class_member_info(psl::string class_name, psl::string member_type_name, size_t index) {
    class_member_infos.insert({class_name, ClassMemberInfo{member_type_name, index}});
  }
  ClassMemberInfo find_class_member_info(psl::string_view name) {
    return psl::find_or(class_member_infos, name, ClassMemberInfo());
  }

  void add_function(const Function& f) {
    auto f_ptr =
        llvm::Function::Create(get_type_of(f.rtype(), f.ptypes()), llvm::Function::ExternalLinkage,
                               f.unique_name().c_str(), M);
    if (f.ptr())
      llvm::sys::DynamicLibrary::AddSymbol(f.unique_name().c_str(), f.ptr());
    functions.emplace_back(f_ptr, &f);
  }
  void add_created_function(llvm::Function* ptr, const Function* f) {
    functions.emplace_back(ptr, f);
  }
  const FunctionInfo& function(size_t i) const {
    return functions[i];
  }

  void add_constant(psl::string name, const Variable& var) {
    auto type_name = context.type_name_from_id(var.type_id());
    llvm::Constant* const_ptr;
    if (type_name == "f32")
      const_ptr = llvm::ConstantFP::get(C, llvm::APFloat(var.as<float>()));
    else if (type_name == "i32")
      const_ptr = builder.getInt32(var.as<int>());
    else if (type_name == "vec3") {
      auto v = var.as<vec3>();
      const_ptr = llvm::ConstantArray::get(llvm::ArrayType::get(builder.getInt32Ty(), 3),
                                           {llvm::ConstantFP::get(C, llvm::APFloat(v.x)),
                                            llvm::ConstantFP::get(C, llvm::APFloat(v.y)),
                                            llvm::ConstantFP::get(C, llvm::APFloat(v.z))});
    } else
      Fatal("Constant `", type_name, "` is not yet supported");

    constants[name] =
        Value(new llvm::GlobalVariable(*M, const_ptr->getType(), true,
                                       llvm::GlobalValue::PrivateLinkage, const_ptr, name.c_str()),
              type_name, Value::L);
  }

  llvm::FunctionType* get_type_of(const TypeTag& f_rtype, psl::span<const TypeTag> f_ptypes) {
    llvm::Type* rtype;
    auto ptypes = std::vector<llvm::Type*>();

    if (f_rtype.is_ref) {
      rtype = builder.getPtrTy();
    } else {
      rtype = builder.getVoidTy();
      if (f_rtype.name != "void")
        ptypes.push_back(builder.getPtrTy());
    }

    for (auto& ptype : f_ptypes)
      ptypes.push_back(builder.getPtrTy());

    return llvm::FunctionType::get(rtype, ptypes, false);
  }
  llvm::FunctionType* get_type_of(psl::string_view name) {
    auto ptypes = psl::vector<TypeTag>();
    auto depth = 0;
    auto p = size_t(0);
    ptypes.push_back(psl::string(name));
    do {
      if (name[p] == '(')
        ++depth;
      else if (name[p] == ')') {
        if (depth == 1) {
          ptypes.push_back(name.substr(1, p - 1));
        }
        --depth;
      } else if (depth == 1 && name[p] == ',') {
        ptypes.push_back(name.substr(1, p - 1));
        name = name.subview(p + 1);
        p = 0;
      }
      ++p;
    } while (depth);

    return get_type_of(name.substr(p + 2), ptypes);
  }
  llvm::ArrayType* function_object_type() {
    return llvm::ArrayType::get(builder.getPtrTy(), 2);
  }

  // ================================================
  // Compilation
  // ================================================
  llvm::BasicBlock* create_block() {
    return llvm::BasicBlock::Create(C, "", F);
  }
  void cond_br(SourceLoc sl, Value cond, llvm::BasicBlock* true_block,
               llvm::BasicBlock* false_block) {
    if (cond.type_name == "bool") {
      builder.CreateCondBr(load(cond).ptr, true_block, false_block, (llvm::Instruction*)nullptr);
      return;
    }

    if (cond.type_name != "bool") {
      if (auto fr = context.find_unique_f("@convert." + cond.type_name + ".bool"))
        cond = call(fr.fi, psl::array_of(cond));
      else
        error(sl, "Expect a boolean value");
    }

    builder.CreateCondBr(load(cond).ptr, true_block, false_block, (llvm::Instruction*)nullptr);
  }
  llvm::Value* alloc(TypeInfo type) {
    auto current = builder.GetInsertBlock();
    builder.SetInsertPoint(entry, entry->begin());
    auto value = builder.CreateAlloca(type.ptr);
    builder.SetInsertPoint(current);

    if (type.destructor)
      stack.back().blocks.back().pending_destruction.emplace_back(value, type.destructor);

    return value;
  }
  Value alloc(psl::string type_name) {
    return {alloc(type(type_name)), type_name};
  }
  llvm::Value* alloc_return(TypeInfo type) {
    auto current = builder.GetInsertBlock();
    builder.SetInsertPoint(entry, entry->begin());
    auto value = builder.CreateAlloca(type.ptr);
    builder.SetInsertPoint(current);
    return value;
  }
  llvm::Value* malloc(size_t bytes) {
    auto size = alloc("u64");
    builder.CreateStore(builder.getInt64(bytes), size.ptr);
    auto ptr = call(SourceLoc(), "malloc", psl::vector_of(size)).ptr;
    return builder.CreateLoad(builder.getPtrTy(), ptr);
  }
  Value load(const Value& x) {
    return {builder.CreateLoad(type(x.type_name).ptr, x.ptr), x.type_name};
  }

  void push_variable(psl::string name, Value variable) {
    stack.back().blocks.back().variables[name] = MOVE(variable);
  }
  void enter_function(TypeTag return_type) {
    stack.push_back(FunctionScope());
    return_types.push_back(return_type);
  }
  TypeTag exit_function() {
    CHECK(stack.size() > 1);
    stack.pop_back();
    auto type = return_types.back();
    return_types.pop_back();
    return type;
  }
  void enter_scope() {
    stack.back().blocks.push_back(BlockScope());
  }
  void exit_scope() {
    CHECK(stack.back().blocks.size() > 1);
    auto&& block = stack.back().blocks.back();
    for (auto [object, destructor] : block.pending_destruction)
      builder.CreateCall(destructor, object);
    stack.back().blocks.pop_back();
  }
  Value find_variable(psl::string_view name) const {
    for (auto& block : psl::reverse_adapter(stack.back().blocks))
      if (auto it = block.variables.find(name); it != block.variables.end())
        return it->second;
    return psl::find_or(constants, name, Value());
  }

  Value call(size_t fi, psl::span<const Value> args, llvm::Value* res = nullptr) {
    auto [ptr, f] = function(fi);
    auto arg_values = std::vector<llvm::Value*>();
    auto rtype = f->rtype();

    if (args.size() == 2 && args[0].type_name == "i32" && args[1].type_name == "i32") {
      if (f->name() == "=") {
        builder.CreateStore(load(args[1]).ptr, args[0].ptr);
        return {};
      }
      auto a = load(args[0]).ptr;
      auto b = load(args[1]).ptr;
      if (f->name() == "*") {
        auto res = alloc("i32");
        builder.CreateStore(builder.CreateMul(a, b), res.ptr);
        return res;
      } else if (f->name() == "/") {
        auto res = alloc("i32");
        builder.CreateStore(builder.CreateSDiv(a, b), res.ptr);
        return res;
      } else if (f->name() == "+") {
        auto res = alloc("i32");
        builder.CreateStore(builder.CreateAdd(a, b), res.ptr);
        return res;
      } else if (f->name() == "-") {
        auto res = alloc("i32");
        builder.CreateStore(builder.CreateSub(a, b), res.ptr);
        return res;
      } else if (f->name() == "%") {
        auto res = alloc("i32");
        builder.CreateStore(builder.CreateSRem(a, b), res.ptr);
        return res;
      } else if (f->name() == "*=") {
        builder.CreateStore(builder.CreateMul(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "/=") {
        builder.CreateStore(builder.CreateSDiv(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "+=") {
        builder.CreateStore(builder.CreateAdd(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "-=") {
        builder.CreateStore(builder.CreateSub(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "%=") {
        builder.CreateStore(builder.CreateSRem(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "==") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateICmpEQ(a, b), res.ptr);
        return res;
      } else if (f->name() == "!=") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateICmpNE(a, b), res.ptr);
        return res;
      } else if (f->name() == "<") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateICmpSLT(a, b), res.ptr);
        return res;
      } else if (f->name() == ">") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateICmpSGT(a, b), res.ptr);
        return res;
      } else if (f->name() == "<=") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateICmpSLE(a, b), res.ptr);
        return res;
      } else if (f->name() == ">=") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateICmpSGE(a, b), res.ptr);
        return res;
      }
    } else if (args.size() == 2 && args[0].type_name == "f32" && args[1].type_name == "f32") {
      if (f->name() == "=") {
        builder.CreateStore(load(args[1]).ptr, args[0].ptr);
        return {};
      }
      auto a = load(args[0]).ptr;
      auto b = load(args[1]).ptr;
      if (f->name() == "*") {
        auto res = alloc("f32");
        builder.CreateStore(builder.CreateFMul(a, b), res.ptr);
        return res;
      } else if (f->name() == "/") {
        auto res = alloc("f32");
        builder.CreateStore(builder.CreateFDiv(a, b), res.ptr);
        return res;
      } else if (f->name() == "+") {
        auto res = alloc("f32");
        builder.CreateStore(builder.CreateFAdd(a, b), res.ptr);
        return res;
      } else if (f->name() == "-") {
        auto res = alloc("f32");
        builder.CreateStore(builder.CreateFSub(a, b), res.ptr);
        return res;
      } else if (f->name() == "%") {
        auto res = alloc("f32");
        builder.CreateStore(builder.CreateFRem(a, b), res.ptr);
        return res;
      } else if (f->name() == "*=") {
        builder.CreateStore(builder.CreateFMul(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "/=") {
        builder.CreateStore(builder.CreateFDiv(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "+=") {
        builder.CreateStore(builder.CreateFAdd(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "-=") {
        builder.CreateStore(builder.CreateFSub(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "%=") {
        builder.CreateStore(builder.CreateFRem(a, b), args[0].ptr);
        return args[0];
      } else if (f->name() == "==") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateFCmpOEQ(a, b), res.ptr);
        return res;
      } else if (f->name() == "!=") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateFCmpONE(a, b), res.ptr);
        return res;
      } else if (f->name() == "<") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateFCmpOLT(a, b), res.ptr);
        return res;
      } else if (f->name() == ">") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateFCmpOGT(a, b), res.ptr);
        return res;
      } else if (f->name() == "<=") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateFCmpOLE(a, b), res.ptr);
        return res;
      } else if (f->name() == ">=") {
        auto res = alloc("bool");
        builder.CreateStore(builder.CreateFCmpOGE(a, b), res.ptr);
        return res;
      }
    } else if (args.size() == 1 && args[0].type_name == "i32") {
      auto a = load(args[0]).ptr;
      if (f->name() == "+x") {
        return args[0];
      } else if (f->name() == "-x") {
        auto res = alloc("i32");
        builder.CreateStore(builder.CreateNeg(a), res.ptr);
        return res;
      } else if (f->name() == "++x") {
        builder.CreateStore(builder.CreateAdd(a, builder.getInt32(1)), args[0].ptr);
        return args[0];
      } else if (f->name() == "--x") {
        builder.CreateStore(builder.CreateSub(a, builder.getInt32(1)), args[0].ptr);
        return args[0];
      } else if (f->name() == "x++") {
        auto res = alloc("i32");
        builder.CreateStore(a, res.ptr);
        builder.CreateStore(builder.CreateAdd(a, builder.getInt32(1)), args[0].ptr);
        return res;
      } else if (f->name() == "x--") {
        auto res = alloc("i32");
        builder.CreateStore(a, res.ptr);
        builder.CreateStore(builder.CreateSub(a, builder.getInt32(1)), args[0].ptr);
        return res;
      }
    } else if (args.size() == 1 && args[0].type_name == "f32") {
      auto a = load(args[0]).ptr;
      if (f->name() == "+x") {
        return args[0];
      } else if (f->name() == "-x") {
        auto res = alloc("f32");
        builder.CreateStore(builder.CreateFNeg(a), res.ptr);
        return res;
      }
    }

    if (!rtype.is_ref && rtype.name != "void")
      arg_values.push_back(res ? res : alloc(type(rtype.name)));

    for (auto&& arg : args)
      arg_values.push_back(arg.ptr);

    if (rtype.is_ref) {
      return {builder.CreateCall(ptr, arg_values), rtype.name, Value::L};
    } else {
      builder.CreateCall(ptr, arg_values);
      return {arg_values[0], rtype.name, Value::R};
    }
  }

  Value call(SourceLoc sl, psl::string_view name, psl::vector<Value> args,
             llvm::Value* res = nullptr) {
    auto fr = find_f(name, args);
    if (!fr)
      error(sl, "Unable to find function `", name, "`");

    for (auto cv : fr.converts)
      args[cv.position] = call(cv.converter_index, psl::vector_of(args[cv.position]));

    return call(fr.fi, args, res);
  }
  void return_value(SourceLoc sl, Value value) {
    for (auto&& block : stack.back().blocks)
      for (auto [object, destructor] : block.pending_destruction)
        builder.CreateCall(destructor, object);

    if (return_types.back().is_ref) {
      if (value.is_r_value())
        error(sl, "Can't return a r-value when the function expects reference");
      else
        builder.CreateRet(value.ptr);
    } else {
      if (is_function_type(value.type_name))
        builder.CreateStore(builder.CreateLoad(function_object_type(), value.ptr), F->getArg(0));
      else
        call(sl, "@move", psl::vector_of(value), F->getArg(0));
      builder.CreateRetVoid();
    }
  }
  void setup_params(const TypeTag& rtype, psl::span<const TypeTag> ptypes,
                    psl::span<psl::string> pnames) {
    auto arg_i = 0;
    if (rtype.is_ref)
      arg_i++;

    for (size_t i = 0; i < ptypes.size(); i++) {
      auto&& ptype = ptypes[i];
      if (ptype.is_ref) {
        push_variable(pnames[i], {F->getArg(arg_i++), ptype.name});
      } else {
        auto arg = alloc(type(ptype.name));
        builder.CreateStore(F->getArg(arg_i++), arg);
        push_variable(pnames[i], {arg, ptype.name});
      }
    }
  }

  void finalize_function() {
    builder.CreateBr(exit);

    for (auto& block : *F) {
      if (&block == exit)
        continue;

      for (auto it = block.begin(); it != block.end(); ++it) {
        if (llvm::BranchInst::classof(&*it)) {
          block.erase(std::next(it), block.end());
          break;
        } else if (llvm::ReturnInst::classof(&*it)) {
          builder.SetInsertPoint(&block, it);
          for (auto& inst : *exit)
            builder.Insert(inst.clone());

          for (auto it_ = block.begin(); it_ != block.end(); ++it_)
            if (llvm::ReturnInst::classof(&*it_))
              block.erase(std::next(it_), block.end());
          break;
        }
      }
    }

    builder.SetInsertPoint(exit);
    builder.CreateRetVoid();

    auto blocks_without_predecessor = psl::vector<llvm::BasicBlock*>();
    for (auto& block : *F)
      if (&block != entry && llvm::pred_empty(&block))
        blocks_without_predecessor.push_back(&block);

    for (auto block : blocks_without_predecessor) {
      block->removeFromParent();
      block->deleteValue();
    }
  }

  Context& context;
  SourceLines source;

  llvm::LLVMContext& C;
  llvm::Module* M;
  llvm::Function* F;
  llvm::BasicBlock* entry;
  llvm::BasicBlock* exit;
  llvm::IRBuilder<> builder;
  int block_counter = 0;

  psl::map<psl::string, Value> constants = {};
  psl::vector<FunctionInfo> functions = {};
  psl::map<psl::string, TypeInfo> types = {};

  struct LoopInfo {
    llvm::BasicBlock* at_inc;
    llvm::BasicBlock* end;
  };
  psl::vector<LoopInfo> loop_stack = {};
  psl::vector<psl::pair<psl::string, size_t>> function_to_be_compiled = {};

  psl::map<psl::string, ClassMemberInfo> class_member_infos = {};
  int lambda_counter = 0;

  psl::vector<TypeTag> return_types = {};

  struct BlockScope {
    psl::map<psl::string, Value> variables;
    psl::vector<psl::pair<llvm::Value*, llvm::Function*>> pending_destruction;
  };
  struct FunctionScope {
    psl::vector<BlockScope> blocks{1};
  };

  psl::vector<FunctionScope> stack{1};
};

// ================================================
// AST declaration
// ================================================
struct PExpr;
struct Expr0;
struct BlockElem;
struct FunctionDefinition;

struct Expr {
  enum Op {
    // clang-format off
    None = 0,
    Mul  = 1000000000,
    Div  = 1000000001,
    Mod  = 1000000010,
    Pow  = 1000000011,
    Add  = 0100000000,
    Sub  = 0100000001,
    Lt   = 0010000000,
    Gt   = 0010000001,
    Le   = 0010000010,
    Ge   = 0010000011,
    Eq   = 0010000100,
    Ne   = 0010000101,
    And  = 0001000000,
    Or   = 0001000001,
    AddE = 0000100000,
    SubE = 0000100001,
    MulE = 0000100010,
    DivE = 0000100011,
    ModE = 0000100100,
    Assi = 0000100101,
    Init= 0000100102,
    // clang-format on
  };

  Expr() = default;
  template <typename T>
  requires requires(T x) { x.sl; }
  Expr(T x);
  Expr(Expr0 x);
  Expr(Expr a, Expr b, Op op);
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  Op op;
  psl::Box<Expr0> x;
  psl::Box<Expr> a;
  psl::Box<Expr> b;
};

struct Id {
  Id(SourceLoc sl, psl::string value) : sl(sl), value(MOVE(value)) {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::string value;
};
struct NumberLiteral {
  NumberLiteral(const psl::string& str);
  Value emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
  bool is_float = false;
  float valuef;
  int valuei;
};
struct BooleanLiteral {
  BooleanLiteral(bool value) : value(value) {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
  bool value;
};
struct StringLiteral {
  StringLiteral(psl::string value) : value(MOVE(value)) {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
  psl::string value;
};
struct Vector {
  Vector(SourceLoc sl, psl::vector<Expr> args) : sl(sl), args{MOVE(args)} {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::vector<Expr> args;
};
struct FunctionCall {
  FunctionCall(SourceLoc sl, psl::string name, psl::vector<Expr> args)
      : sl(sl), name{MOVE(name)}, args{MOVE(args)} {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::string name;
  psl::vector<Expr> args;
};
struct MemberAccess {
  MemberAccess(SourceLoc sl, PExpr pexpr, Id id);
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::Box<PExpr> pexpr;
  Id id;
};
struct Subscript {
  Subscript(SourceLoc sl, PExpr pexpr, Expr index);
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::Box<PExpr> pexpr;
  Expr index;
};
struct LambdaExpr {
  LambdaExpr(SourceLoc sl, FunctionDefinition body);
  Value emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
  psl::Box<FunctionDefinition> body;
};
struct TypeInitExpr {
  TypeInitExpr(SourceLoc sl, psl::string type_name) : sl(sl), type_name(MOVE(type_name)) {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
  psl::string type_name;
};
struct PExpr : psl::variant<Id, NumberLiteral, BooleanLiteral, StringLiteral, Vector, FunctionCall,
                            MemberAccess, Subscript, Expr, LambdaExpr, TypeInitExpr> {
  using variant::variant;
  Value emit(Module& m) const {
    return dispatch([&](auto&& x) { return x.emit(m); });
  }
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
    return dispatch([&](auto&& x) { return x.get_undeclared_variable(m, vars); });
  }
};

struct Expr0 {
  enum Op { None, PreInc, PreDec, PostInc, PostDec, Positive, Negate, Invert };
  template <typename T>
  Expr0(T x) : sl(x.sl), x{MOVE(x)}, op{None} {
  }
  Expr0(SourceLoc sl, PExpr x, Op op = None) : sl(sl), x{MOVE(x)}, op{op} {
  }
  Value emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  PExpr x;
  Op op;
};
struct Declaration {
  enum Flag { None = 0, AsRef = 1, AssignIfExist = 2 };
  Declaration(SourceLoc sl, psl::string name, Expr expr, Flag flag = Flag::None)
      : sl(sl), name{MOVE(name)}, expr{MOVE(expr)}, flag(flag) {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::string name;
  Expr expr;
  Flag flag;
};
struct Semicolon {
  void emit(Module&) const {
  }
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }
};
struct BreakStmt {
  BreakStmt(SourceLoc sl) : sl(sl) {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
};
struct ContinueStmt {
  ContinueStmt(SourceLoc sl) : sl(sl) {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

  SourceLoc sl;
};
struct ReturnStmt {
  ReturnStmt(SourceLoc sl, psl::optional<Expr> expr) : sl(sl), expr(MOVE(expr)) {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

private:
  SourceLoc sl;
  psl::optional<Expr> expr;
};
struct Stmt : psl::variant<Semicolon, Expr, Declaration, BreakStmt, ContinueStmt, ReturnStmt> {
  using variant::variant;
  void emit(Module& m) const {
    dispatch([&](auto&& x) { x.emit(m); });
  }
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
    dispatch([&](auto&& x) { x.get_undeclared_variable(m, vars); });
  }
};
struct Block {
  Block(psl::vector<BlockElem> elems) : elems{MOVE(elems)} {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::vector<BlockElem> elems;
};
struct While {
  While(SourceLoc sl, Expr condition, Block body)
      : sl(sl), condition(MOVE(condition)), body(MOVE(body)) {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  Expr condition;
  Block body;
};
struct For {
  For(SourceLoc sl, Stmt init, Expr condition, Expr inc, Block body)
      : sl(sl), init{MOVE(init)}, condition{MOVE(condition)}, inc{MOVE(inc)}, body{MOVE(body)} {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  Stmt init;
  Expr condition;
  Expr inc;
  Block body;
};
struct If {
  If(SourceLoc sl, Expr condition, Block body)
      : sl(sl), condition{MOVE(condition)}, body{MOVE(body)} {
  }
  SourceLoc sl;
  Expr condition;
  Block body;
};
struct Else {
  Else(SourceLoc sl, Block body) : sl(sl), body{MOVE(body)} {
  }
  SourceLoc sl;
  Block body;
};
struct IfElseChain {
  IfElseChain(psl::vector<If> ifs, psl::optional<Else> else_) : ifs{MOVE(ifs)}, else_{MOVE(else_)} {
  }
  void emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  psl::vector<If> ifs;
  psl::optional<Else> else_;
};
struct ParameterDeclaration {
  ParameterDeclaration(Id name, Id type) : name(name), type(type) {
  }
  Id name;
  Id type;
};
struct FunctionDefinition {
  FunctionDefinition(SourceLoc sl, psl::string name, Id return_type,
                     psl::vector<ParameterDeclaration> params, Block body)
      : sl(sl),
        name(MOVE(name)),
        return_type(MOVE(return_type)),
        params(MOVE(params)),
        body(MOVE(body)) {
  }

  llvm::Function* emit(Module& m) const;
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const;

  SourceLoc sl;
  psl::string name;
  Id return_type;
  psl::vector<ParameterDeclaration> params;
  Block body;
};
struct MemberDefinition {
  MemberDefinition(Id name, Id type) : name(MOVE(name)), type(MOVE(type)) {
  }

  Id name;
  Id type;
};
struct ClassDefinition {
  ClassDefinition(SourceLoc sl, psl::string name, psl::vector<FunctionDefinition> ctors,
                  psl::vector<FunctionDefinition> methods, psl::vector<MemberDefinition> members)
      : sl(sl),
        name(MOVE(name)),
        ctors(MOVE(ctors)),
        methods(MOVE(methods)),
        members(MOVE(members)) {
  }

  void emit(Module& m) const;
  void get_undeclared_variable(Module&, psl::set<psl::string>&) const {
  }

private:
  SourceLoc sl;
  psl::string name;
  psl::vector<FunctionDefinition> ctors;
  psl::vector<FunctionDefinition> methods;
  psl::vector<MemberDefinition> members;
};
struct BlockElem
    : psl::variant<Block, Stmt, While, For, IfElseChain, FunctionDefinition, ClassDefinition> {
  using variant::variant;
  void emit(Module& m) const {
    return dispatch([&](auto&& x) { x.emit(m); });
  }
  void get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
    return dispatch([&](auto&& x) { x.get_undeclared_variable(m, vars); });
  }
};

// ================================================
// AST definitions
// ================================================
Value Id::emit(Module& m) const {
  if (auto x = m.find_variable(value)) {
    x.category = x.L;
    return x;
  } else if (auto fr = m.find_unique_f(value)) {
    return {m.function(fr.fi).ptr, m.function(fr.fi).f->signature()};
  } else {
    m.error(sl, "Variable `", value, "` is not found");
  }
}
void Id::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  if (auto x = m.find_variable(value))
    ;
  else if (auto fr = m.find_unique_f(value))
    ;
  else
    vars.insert(value);
}
NumberLiteral::NumberLiteral(const psl::string& str) {
  if ((is_float = psl::contains(str, '.')))
    valuef = psl::stof(str);
  else
    valuei = psl::stoi(str);
}
Value NumberLiteral::emit(Module& m) const {
  auto value = m.alloc(m.type(is_float ? "f32" : "i32"));
  if (is_float)
    m.builder.CreateStore(llvm::ConstantFP::get(m.C, llvm::APFloat(valuef)), value);
  else
    m.builder.CreateStore(m.builder.getInt32(valuei), value);
  return {value, is_float ? "f32" : "i32"};
}
Value BooleanLiteral::emit(Module& m) const {
  auto value = m.alloc(m.type("bool"));
  m.builder.CreateStore(m.builder.getInt8(this->value), value);
  return {value, "bool"};
}
Value StringLiteral::emit(Module& m) const {
  auto str_constant = m.builder.CreateGlobalString(value.c_str());
  auto cstr = Value(m.alloc(m.type("cstr")), "cstr");
  m.builder.CreateStore(str_constant, cstr.ptr);
  return m.call(sl, "str", psl::vector_of(cstr));
}
Value Vector::emit(Module& m) const {
  if (args.size() < 2 || args.size() > 4)
    m.error(sl, "Only 2, 3, or 4 items can exist inside []");

  auto arg_values = psl::transform_vector(args, [&](auto&& x) { return x.emit(m); });

  if (psl::any(arg_values, [](auto&& x) { return x.type_name == "f32"; }))
    return m.call(sl, "vec" + psl::to_string(args.size()), arg_values);
  else
    return m.call(sl, "vec" + psl::to_string(args.size()) + "i", arg_values);
}
void Vector::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  for (auto& arg : args)
    arg.get_undeclared_variable(m, vars);
}
static psl::string get_rtype(psl::string_view sig) {
  auto depth = 0;
  auto p = size_t(0);
  do {
    if (sig[p] == '(')
      ++depth;
    else if (sig[p] == ')')
      --depth;
    ++p;
  } while (depth);
  return sig.substr(p + 2);
}
Value FunctionCall::emit(Module& m) const {
  if (auto res = m.find_variable(name)) {
    auto ptr_ty = m.builder.getPtrTy();
    auto obj = m.builder.CreateLoad(
        ptr_ty, m.builder.CreateConstGEP2_32(m.function_object_type(), res.ptr, 0, 0));
    auto f = m.builder.CreateLoad(
        ptr_ty, m.builder.CreateConstGEP2_32(m.function_object_type(), res.ptr, 0, 1));
    auto arg_values = std::vector<llvm::Value*>();
    arg_values.push_back(obj);
    for (auto&& arg : args)
      arg_values.push_back(arg.emit(m).ptr);
    auto r = m.alloc(get_rtype(res.type_name));
    arg_values.insert(arg_values.begin(), r.ptr);
    m.builder.CreateCall(m.get_type_of(res.type_name), f, arg_values);
    return r;
  } else {
    auto arg_values = psl::transform_vector(args, [&](auto&& x) { return x.emit(m); });
    return m.call(sl, name, arg_values);
  }
}
void FunctionCall::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  if (m.find_unique_f(name).error == Context::FindUniqueFResult::FindNone && !m.find_variable(name))
    vars.insert(name);
  for (auto& arg : args)
    arg.get_undeclared_variable(m, vars);
}
MemberAccess::MemberAccess(SourceLoc sl, PExpr pexpr, Id id)
    : sl(sl), pexpr(MOVE(pexpr)), id(MOVE(id)) {
}
Value MemberAccess::emit(Module& m) const {
  auto x = pexpr->emit(m);
  if (auto info = m.find_class_member_info("@member." + x.type_name + "." + id.value))
    return {m.builder.CreateStructGEP(m.type(x.type_name).ptr, x.ptr, info.index), info.type_name,
            Value::L};

  if (auto fr = m.find_unique_f("@ma." + x.type_name + "." + id.value))
    return m.call(fr.fi, psl::vector_of(x));
  else
    m.error(id.sl, "Can't find member `", id.value, "` in type `", x.type_name, '`');
}
void MemberAccess::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  pexpr->get_undeclared_variable(m, vars);
}
Subscript::Subscript(SourceLoc sl, PExpr pexpr, Expr index)
    : sl(sl), pexpr(MOVE(pexpr)), index(MOVE(index)) {
}
Value Subscript::emit(Module& m) const {
  auto args = psl::vector_of(pexpr->emit(m), index.emit(m));
  return m.call(sl, "[]", args);
}
void Subscript::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  pexpr->get_undeclared_variable(m, vars);
}
LambdaExpr::LambdaExpr(SourceLoc sl, FunctionDefinition body_) : sl(sl), body(MOVE(body_)) {
}
Value LambdaExpr::emit(Module& m) const {
  auto class_name = psl::to_string("@lambda.", m.lambda_counter++);

  // Setup the members of the lambda
  auto vars = psl::set<psl::string>();
  body->get_undeclared_variable(m, vars);
  for (auto&& param : body->params)
    vars.erase(param.name.value);

  //   if (vars.size() == 0) {
  //     auto body = this->body;
  //     body->name = class_name;
  //     body->emit(m);

  //     auto fr = m.find_unique_f(class_name);
  //     return {m.function(fr.fi).ptr, m.function(fr.fi).f->signature()};
  //   }

  auto member_args = psl::transform_vector(
      vars, [&](auto&& var) { return psl::make_pair(Id(sl, var).emit(m), var); });
  auto member_types = std::vector<llvm::Type*>();

  auto index = size_t(0);
  auto byte_size = size_t(0);
  for (auto& [value, name] : member_args) {
    byte_size += m.type_size(value.type_name);
    member_types.push_back(m.type(value.type_name).ptr);
    m.set_class_member_info("@member." + class_name + "." + name, value.type_name, index++);
  }

  auto type = llvm::StructType::create(m.C, member_types, class_name.c_str());
  m.add_type(class_name, type, nullptr);
  m.context.create_type_trait(class_name, byte_size);

  auto body = this->body;
  body->name = class_name;
  body->params.push_front(ParameterDeclaration(Id(sl, "self"), Id(sl, class_name)));
  for (const auto& member : member_args) {
    body->body.elems.push_front(
        Stmt(Declaration(sl, member.second, MemberAccess(sl, Id(sl, "self"), Id(sl, member.second)),
                         Declaration::AsRef)));
  }

  auto lambda_function = body->emit(m);

  auto lambda_object = m.malloc(byte_size);
  for (size_t i = 0; i < member_args.size(); i++) {
    if (is_function_type(member_args[i].first.type_name))
      m.builder.CreateStore(m.load(member_args[i].first).ptr,
                            m.builder.CreateStructGEP(type, lambda_object, i));
    else
      m.builder.CreateStore(m.load(m.call(sl, "@copy", psl::vector_of(member_args[i].first))).ptr,
                            m.builder.CreateStructGEP(type, lambda_object, i));
  }

  auto atype = m.function_object_type();
  auto res = m.builder.CreateAlloca(atype);
  m.builder.CreateStore(lambda_object, m.builder.CreateConstGEP2_32(atype, res, 0, 0));
  m.builder.CreateStore(lambda_function, m.builder.CreateConstGEP2_32(atype, res, 0, 1));

  return {res, signature_from(TypeTag(this->body->return_type.value),
                              psl::transform_vector(this->body->params, [](auto&& x) {
                                return TypeTag(x.type.value);
                              }))};
}
Value TypeInitExpr::emit(Module& m) const {
  return {m.alloc(m.type(type_name)), type_name};
}
Value Expr0::emit(Module& m) const {
  auto args = psl::vector_of(x.emit(m));
  switch (op) {
    case None: return args[0];
    case PreInc: return m.call(sl, "++x", args);
    case PreDec: return m.call(sl, "--x", args);
    case PostInc: return m.call(sl, "x++", args);
    case PostDec: return m.call(sl, "x--", args);
    case Positive: return m.call(sl, "+x", args);
    case Negate: return m.call(sl, "-x", args);
    case Invert: return m.call(sl, "!x", args);
    default: PINE_UNREACHABLE;
  }
}
void Expr0::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  x.get_undeclared_variable(m, vars);
}
template <typename T>
requires requires(T x) { x.sl; }
Expr::Expr(T x) : Expr(Expr0(x.sl, MOVE(x))) {
}
Expr::Expr(Expr0 x) : sl(x.sl), op{None}, x{MOVE(x)} {
}
Expr::Expr(Expr a, Expr b, Op op) : sl(a.sl), op{op}, a{MOVE(a)}, b{MOVE(b)} {
}
Value Expr::emit(Module& m) const {
  if (op == None)
    return x->emit(m);

  auto args = psl::vector_of(a->emit(m), b->emit(m));

  if (op == Init) {
    m.builder.CreateStore(m.load(args[1]).ptr, args[0].ptr);
    return {};
  }

  switch (op) {
    case None: PINE_UNREACHABLE;
    case Mul: return m.call(sl, "*", args);
    case Div: return m.call(sl, "/", args);
    case Mod: return m.call(sl, "%", args);
    case Pow: return m.call(sl, "^", args);
    case Add: return m.call(sl, "+", args);
    case Sub: return m.call(sl, "-", args);
    case Lt: return m.call(sl, "<", args);
    case Gt: return m.call(sl, ">", args);
    case Le: return m.call(sl, "<=", args);
    case Ge: return m.call(sl, ">=", args);
    case Eq: return m.call(sl, "==", args);
    case Ne: return m.call(sl, "!=", args);
    case And: return m.call(sl, "&&", args);
    case Or: return m.call(sl, "||", args);
    case AddE: return m.call(sl, "+=", args);
    case SubE: return m.call(sl, "-=", args);
    case MulE: return m.call(sl, "*=", args);
    case DivE: return m.call(sl, "/=", args);
    case ModE: return m.call(sl, "%=", args);
    case Assi: return m.call(sl, "=", args);
    case Init: PINE_UNREACHABLE;
    default: PINE_UNREACHABLE;
  }
}
void Expr::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  if (op == None) {
    x->get_undeclared_variable(m, vars);
  } else {
    a->get_undeclared_variable(m, vars);
    b->get_undeclared_variable(m, vars);
  }
}
void Declaration::emit(Module& m) const {
  if (flag & AssignIfExist)
    if (auto x = m.find_variable(name)) {
      m.call(sl, "=", psl::vector_of(x, expr.emit(m)));
      return;
    }

  auto x = expr.emit(m);
  if (flag & AsRef || x.is_r_value())
    m.push_variable(name, x);
  else
    m.push_variable(name, m.call(sl, "@copy", psl::vector_of(x)));
}
void Declaration::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  expr.get_undeclared_variable(m, vars);
  m.push_variable(name, Value((llvm::Value*)1, "@placeholder"));
}
void ReturnStmt::emit(Module& m) const {
  if (m.return_types.size() == 0)
    m.error(sl, "`return` can only be used inside a function");
  auto return_type = m.return_types.back();
  if (expr) {
    if (return_type.name == "void")
      m.error(sl, "Can only return void");

    auto x = expr->emit(m);
    if (x.type_name != return_type.name) {
      if (auto fr = m.find_unique_f("@convert." + x.type_name + "." + return_type.name))
        m.return_value(sl, m.call(fr.fi, psl::vector_of(x)));
      else
        m.error(sl, "Returned `", x.type_name, "` is incompatible with function return type `",
                return_type.name, "`");
    } else {
      m.return_value(sl, x);
    }
  } else {
    if (return_type.name == "void")
      m.builder.CreateRetVoid();
    else
      m.error(sl, "Can't return void");
  }
}
void ReturnStmt::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  if (expr)
    expr->get_undeclared_variable(m, vars);
}
void BreakStmt::emit(Module& m) const {
  if (m.loop_stack.size() == 0)
    m.error(sl, "Can only be used in a loop");
  m.builder.CreateBr(m.loop_stack.back().end);
}
void ContinueStmt::emit(Module& m) const {
  if (m.loop_stack.size() == 0)
    m.error(sl, "Can only be used in a loop");
  m.builder.CreateBr(m.loop_stack.back().at_inc);
}
void Block::emit(Module& m) const {
  m.enter_scope();
  for (const auto& block_elem : elems)
    block_elem.emit(m);
  m.exit_scope();
}
void Block::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  for (const auto& block_elem : elems)
    block_elem.get_undeclared_variable(m, vars);
}
void While::emit(Module& m) const {
  auto cond_block = m.create_block();
  auto body_block = m.create_block();
  auto seq = m.create_block();

  m.builder.CreateBr(cond_block);
  m.builder.SetInsertPoint(cond_block);
  m.cond_br(condition.sl, condition.emit(m), body_block, seq);

  m.builder.SetInsertPoint(body_block);
  m.loop_stack.emplace_back(cond_block, seq);
  body.emit(m);
  m.loop_stack.pop_back();
  m.builder.CreateBr(cond_block);

  m.builder.SetInsertPoint(seq);
}
void While::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  condition.get_undeclared_variable(m, vars);
  body.get_undeclared_variable(m, vars);
}
void For::emit(Module& m) const {
  init.emit(m);
  auto cond_block = m.create_block();
  auto body_block = m.create_block();
  auto inc_block = m.create_block();
  auto seq = m.create_block();

  m.builder.CreateBr(cond_block);
  m.builder.SetInsertPoint(body_block);
  m.loop_stack.emplace_back(inc_block, seq);
  body.emit(m);
  m.loop_stack.pop_back();
  m.builder.CreateBr(inc_block);
  m.builder.SetInsertPoint(inc_block);
  inc.emit(m);
  m.builder.CreateBr(cond_block);

  m.builder.SetInsertPoint(cond_block);
  m.cond_br(condition.sl, condition.emit(m), body_block, seq);

  m.builder.SetInsertPoint(seq);
}
void For::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  init.get_undeclared_variable(m, vars);
  condition.get_undeclared_variable(m, vars);
  body.get_undeclared_variable(m, vars);
  inc.get_undeclared_variable(m, vars);
}
void IfElseChain::emit(Module& m) const {
  auto cond_blocks = psl::vector<llvm::BasicBlock*>(ifs.size());
  auto body_blocks = psl::vector<llvm::BasicBlock*>(ifs.size());
  psl::fill_f(cond_blocks, [&]() { return m.create_block(); });
  psl::fill_f(body_blocks, [&]() { return m.create_block(); });
  auto seq = m.create_block();
  auto else_block = else_ ? m.create_block() : (llvm::BasicBlock*)nullptr;

  m.builder.CreateBr(cond_blocks[0]);

  for (size_t i = 0; i < ifs.size(); i++) {
    m.builder.SetInsertPoint(cond_blocks[i]);
    llvm::BasicBlock* false_block;
    if (i + 1 < ifs.size())
      false_block = cond_blocks[i + 1];
    else if (else_)
      false_block = else_block;
    else
      false_block = seq;
    m.cond_br(ifs[i].condition.sl, ifs[i].condition.emit(m), body_blocks[i], false_block);

    m.builder.SetInsertPoint(body_blocks[i]);
    ifs[i].body.emit(m);
    m.builder.CreateBr(seq);
  }
  if (else_) {
    m.builder.SetInsertPoint(else_block);
    else_->body.emit(m);
    m.builder.CreateBr(seq);
  }

  m.builder.SetInsertPoint(seq);
}
void IfElseChain::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  for (auto& if_ : ifs) {
    if_.condition.get_undeclared_variable(m, vars);
    if_.body.get_undeclared_variable(m, vars);
  }
  if (else_)
    else_->body.get_undeclared_variable(m, vars);
}

llvm::Function* FunctionDefinition::emit(Module& m) const {
  auto rtype = TypeTag(return_type.value);
  auto rtype_public = rtype;
  auto ptypes = psl::transform_vector(params, [](auto&& x) { return TypeTag(x.type.value); });
  auto ptypes_public = ptypes;
  auto pnames = psl::transform_vector(params, [](auto&& x) { return x.name.value; });
  auto unique_name = name + signature_from(rtype, ptypes);

  if (!rtype.is_ref && rtype.name != "void") {
    rtype.is_ref = true;
    pnames.push_front("@res");
    ptypes.push_front(psl::exchange(rtype, TypeTag("void")));
  }
  for (auto& ptype : ptypes)
    ptype.is_ref = true;
  auto F = llvm::Function::Create(m.get_type_of(rtype, ptypes), llvm::Function::ExternalLinkage,
                                  unique_name.c_str(), m.M);

  auto entry = llvm::BasicBlock::Create(m.C, "entry", F);
  auto exit = llvm::BasicBlock::Create(m.C, "exit", F);

  auto backup_F = psl::exchange(m.F, F);
  auto backup_entry = psl::exchange(m.entry, entry);
  auto backup_exit = psl::exchange(m.exit, exit);
  auto backup_block = m.builder.GetInsertBlock();
  m.builder.SetInsertPoint(entry);

  m.enter_function(rtype_public);
  m.setup_params(rtype, ptypes, pnames);
  body.emit(m);
  m.finalize_function();
  rtype_public = m.exit_function();

  //   F->dump();
  m.context.add_f(name.c_str(), Function(unique_name, rtype_public, ptypes_public, nullptr));
  m.add_created_function(F, &m.context.functions.back());
  m.function_to_be_compiled.push_back({unique_name, m.context.functions.size() - 1});

  m.builder.SetInsertPoint(backup_block);
  m.exit = backup_exit;
  m.entry = backup_entry;
  m.F = backup_F;

  return F;
}
void FunctionDefinition::get_undeclared_variable(Module& m, psl::set<psl::string>& vars) const {
  m.enter_function(TypeTag(return_type.value));
  body.get_undeclared_variable(m, vars);
  m.exit_function();
}
void ClassDefinition::emit(Module& m) const {
  auto member_types = std::vector<llvm::Type*>();
  auto index = size_t(0);
  auto byte_size = size_t(0);
  for (auto& member : members) {
    byte_size += m.type_size(member.type.value);
    member_types.push_back(m.type(member.type.value).ptr);
    m.set_class_member_info("@member." + name + "." + member.name.value, member.type.value,
                            index++);
  }
  m.add_type(name, llvm::StructType::create(m.C, member_types, name.c_str()), nullptr);
  m.context.create_type_trait(name, byte_size);

  for (auto ctor : ctors)
    ctor.emit(m);
  for (auto method : methods)
    method.emit(m);
}

struct Parser {
  template <typename T>
  struct ExplicitParameter {
    explicit ExplicitParameter(T value) : value(MOVE(value)) {
    }
    operator T&() {
      return value;
    }
    T value;
  };

  Parser(psl::string_view tokens) : sl(tokens, row_padding) {
    to_valid_pos();
  }

  Block block(bool top_level = false) {
    consume_spaces();

    if (top_level)
      accept("{");
    else
      consume("{", "to begin block");

    auto block_elems = psl::vector<BlockElem>{};
    while (!expect("}") && next())
      block_elems.push_back(block_elem());

    if (top_level)
      accept("}");
    else
      consume("}", "to end block");
    return Block{block_elems};
  }

  BlockElem block_elem() {
    if (expect("{"))
      return BlockElem{block()};
    else if (expect("while"))
      return BlockElem{while_()};
    else if (expect("for"))
      return BlockElem{for_()};
    else if (expect("if"))
      return BlockElem{if_else_chain()};
    else if (expect("fn"))
      return BlockElem{function_definition()};
    else if (expect("class"))
      return BlockElem{class_definition()};
    else
      return BlockElem{stmt()};
  }
  While while_() {
    consume("while");
    auto loc = source_loc();
    auto cond = expr();
    auto body = block();
    return While{loc, MOVE(cond), MOVE(body)};
  }
  For for_() {
    consume("for");

    backup();
    auto id_ = id();
    if (accept("in")) {
      commit();
      auto range_begin = expr();
      if (accept("..")) {
        auto range_end = expr();
        auto init = Stmt(Declaration(id_.sl, id_.value, range_begin));
        auto cond = Expr(Expr0(range_end.sl, id_), range_end, Expr::Lt);
        auto body = block();
        return For{range_begin.sl, MOVE(init), MOVE(cond),
                   Expr(Expr0(range_begin.sl, id_, Expr0::PreInc)), MOVE(body)};
      } else {
        consume("~", "or .. to specify range");
        auto range_inc = expr();
        consume("~", "to specify range end");
        auto range_end = expr();
        auto init = Stmt(Declaration(id_.sl, id_.value, range_begin));
        auto cond = Expr(Expr0(range_end.sl, id_), range_end, Expr::Le);
        auto body = block();
        return For{range_begin.sl, MOVE(init), MOVE(cond),
                   Expr(Expr0(range_begin.sl, id_), range_inc, Expr::AddE), MOVE(body)};
      }
    } else {
      undo();
      auto init = stmt();
      auto loc = source_loc();
      auto cond = expr();
      consume(";");
      auto inc = expr();
      auto body = block();
      return For{loc, MOVE(init), MOVE(cond), MOVE(inc), MOVE(body)};
    }
  }
  IfElseChain if_else_chain() {
    auto ifs = psl::vector<If>{};
    ifs.push_back(if_());
    while (true) {
      backup();
      if (accept("else")) {
        if (expect("if")) {
          undo();
          ifs.push_back(else_if_());
        } else {
          undo();
          break;
        }
      } else {
        undo();
        break;
      }
    }
    auto else_clause = psl::optional<Else>{};
    if (expect("else"))
      else_clause = else_();
    return IfElseChain{MOVE(ifs), MOVE(else_clause)};
  }
  If if_() {
    consume("if");
    auto loc = source_loc();
    auto cond = expr();
    auto body = block();
    return If{loc, MOVE(cond), MOVE(body)};
  }
  If else_if_() {
    consume("else");
    consume("if");
    auto loc = source_loc();
    auto cond = expr();
    auto body = block();
    return If{loc, MOVE(cond), MOVE(body)};
  }
  Else else_() {
    consume("else");
    auto loc = source_loc();
    return Else{loc, block()};
  }
  ClassDefinition class_definition() {
    auto loc = source_loc();
    consume("class");
    auto name = id().value;
    consume("{", "to begin class definition");

    auto ctors = psl::vector<FunctionDefinition>();
    auto ctor_init_sizes = psl::vector<size_t>();
    auto methods = psl::vector<FunctionDefinition>();
    auto members = psl::vector<MemberDefinition>();

    while (!accept("}")) {
      if (expect("ctor")) {
        auto [ctor, init_size] = ctor_definition(name);
        ctors.push_back(MOVE(ctor));
        ctor_init_sizes.push_back(init_size);
      } else if (expect("fn")) {
        methods.push_back(method_definition(name));
      } else {
        members.push_back(member_definition());
        consume(";", "to end the previous member definition");
      }
      while (accept(";"))
        ;
    }

    for (auto [ctor, init_size] : psl::tie_adapter(ctors, ctor_init_sizes))
      for (const auto& member : members) {
        auto loc = ctor.sl;
        CHECK_GE(ctor.body.elems.size(), 1);
        auto it = psl::next(ctor.body.elems.begin(), init_size);
        it = ctor.body.elems.insert(
            it,
            Stmt(Declaration(member.name.sl, member.name.value,
                             MemberAccess(loc, Id(loc, "self"), member.name), Declaration::AsRef)));
      }
    for (auto& method : methods)
      for (const auto& member : members) {
        auto loc = method.sl;
        method.body.elems.push_front(
            Stmt(Declaration(member.name.sl, member.name.value,
                             MemberAccess(loc, Id(loc, "self"), member.name), Declaration::AsRef)));
      }
    return ClassDefinition(loc, MOVE(name), MOVE(ctors), MOVE(methods), MOVE(members));
  }
  MemberDefinition member_definition() {
    auto name = id();
    consume(":", "to specify its type");
    auto type = id();
    return MemberDefinition(MOVE(name), MOVE(type));
  };
  psl::pair<FunctionDefinition, size_t> ctor_definition(psl::string class_name) {
    auto loc = source_loc();
    consume("ctor");
    auto ctor_name = id().value;
    consume("(", "to begin parameter definition");
    auto params = param_list();
    consume(")", "to end parameter definition");
    auto init_stmts = psl::vector<Stmt>();
    // Initialize each member variables
    if (accept(":")) {
      while (!expect("{")) {
        auto loc = source_loc();
        auto name = id().value;
        auto expr_ = expr();
        init_stmts.push_back(
            Expr(Expr(MemberAccess(loc, Id(loc, "self"), Id(loc, name))), expr_, Expr::Init));
        if (!accept(",")) {
          if (!expect("{")) {
            error("Expect either `,` to continue or '{' to begin function definition");
          }
        }
      }
    }
    auto block_ = block();
    // Create `self`
    auto it = block_.elems.begin();
    it = psl::next(block_.elems.insert(
        it, Stmt(Declaration(loc, "self", Expr0(TypeInitExpr(loc, class_name))))));
    // Add the initializer of its members
    for (const auto& init_stmt : init_stmts)
      it = psl::next(block_.elems.insert(it, init_stmt));
    // Return `self` in the ctor
    block_.elems.push_back(Stmt(ReturnStmt({loc}, Id(loc, "self"))));
    return {FunctionDefinition(loc, ctor_name, Id(loc, class_name), MOVE(params), MOVE(block_)),
            1 + init_stmts.size()};
  }
  FunctionDefinition method_definition(psl::string class_name) {
    auto loc = source_loc();
    consume("fn", "to start method definition");
    auto name = id().value;
    consume("(", "to begin parameter definition");
    auto params = param_list();
    params.push_front(ParameterDeclaration(Id(loc, "self"), Id(loc, class_name + "&")));
    consume(")", "to end parameter definition");
    consume(":", "to specify return type");
    auto type = id();
    auto block_ = block();
    return FunctionDefinition(loc, MOVE(name), MOVE(type), MOVE(params), MOVE(block_));
  }
  FunctionDefinition function_definition() {
    auto loc = source_loc();
    consume("fn", "to start function definition");
    auto name = id().value;
    consume("(", "to begin parameter definition");
    auto params = param_list();
    consume(")", "to end parameter definition");
    consume(":", "to specify return type");
    auto return_type = type_name();
    auto block_ = block();
    return FunctionDefinition(loc, MOVE(name), MOVE(return_type), MOVE(params), MOVE(block_));
  }
  Stmt stmt() {
    auto stmt = Stmt{};
    auto loc = source_loc();
    if (accept(";")) {
      return Stmt(Semicolon());
    }
    if (accept("break")) {
      stmt = Stmt(BreakStmt(loc));
    } else if (accept("continue")) {
      stmt = Stmt(ContinueStmt(loc));
    } else if (accept("return")) {
      if (expect(";"))
        stmt = Stmt(ReturnStmt(loc, psl::nullopt));
      else
        stmt = Stmt(ReturnStmt(loc, expr()));
    } else {
      if (auto n = next(); psl::isalpha(*n) || *n == '_') {
        backup();
        auto id_ = id();
        auto loc = source_loc();

        if (accept("=")) {
          stmt = Declaration{loc, MOVE(id_.value), expr(), Declaration::AssignIfExist};
        } else if (accept(":=")) {
          stmt = Declaration{loc, MOVE(id_.value), expr()};
        } else if (accept("&=")) {
          stmt = Declaration{loc, MOVE(id_.value), expr(), Declaration::AsRef};
        } else {
          undo();
          stmt = Stmt(expr());
        }
      } else {
        stmt = Stmt(expr());
      }
    }
    consume(";", "to end statement");
    return stmt;
  }

  Expr expr() {
    auto exprs = psl::vector<Expr>{};
    auto ops = psl::vector<int>{};
    if (expect("(")) {
      auto is_lambda = false;
      backup();
      consume("(");
      if (accept(")")) {
        undo();
        return lambda();
      } else if (maybe_id()) {
        if (expect(":")) {
          undo();
          return lambda();
        }
      }

      if (!is_lambda) {
        undo();
        consume("(");
        exprs.push_back(expr());
        consume(")", "to balance the parenthesis");
      }
    } else {
      exprs.push_back(Expr{expr0()});
    }
    while (true) {
      // clang-format off
           if (accept("+=")) ops.push_back(0000100000);
      else if (accept("-=")) ops.push_back(0000100001);
      else if (accept("*=")) ops.push_back(0000100010); 
      else if (accept("/=")) ops.push_back(0000100011);
      else if (accept("%=")) ops.push_back(0000100100);
      else if (accept("||")) ops.push_back(0001000001);
      else if (accept("&&")) ops.push_back(0001000000);
      else if (accept("!=")) ops.push_back(0010000101);
      else if (accept("==")) ops.push_back(0010000100);
      else if (accept(">=")) ops.push_back(0010000011);
      else if (accept("<=")) ops.push_back(0010000010);
      else if (accept(">"))  ops.push_back(0010000001);
      else if (accept("<"))  ops.push_back(0010000000);
      else if (accept("-"))  ops.push_back(0100000001);
      else if (accept("+"))  ops.push_back(0100000000);
      else if (accept("^"))  ops.push_back(1000000011);
      else if (accept("%"))  ops.push_back(1000000010);
      else if (accept("/"))  ops.push_back(1000000001);
      else if (accept("*"))  ops.push_back(1000000000);
    //   else if (accept("="))  ops.push_back(0000100101);
      else break;
      // clang-format on
      if (accept("(")) {
        exprs.push_back(expr());
        accept(")");
      } else {
        exprs.push_back(Expr{expr0()});
      }
    }

    while (ops.size() != 0) {
      auto max_precedence = 0;
      auto index = size_t{0};
      for (size_t i = 0; i < ops.size(); i++)
        if (ops[i] > max_precedence) {
          max_precedence = ops[i];
          index = i;
        }
      auto a = MOVE(exprs[index]);
      auto b = MOVE(exprs[index + 1]);
      ops.erase(ops.begin() + index);
      exprs.erase(exprs.begin() + index);
      exprs.erase(exprs.begin() + index);
      exprs.insert(exprs.begin() + index, Expr{MOVE(a), MOVE(b), Expr::Op(max_precedence)});
    }

    return exprs[0];
  }
  Expr0 expr0() {
    auto loc = source_loc();
    if (accept("++")) {
      return Expr0{loc, pexpr(), Expr0::PreInc};
    } else if (accept("--")) {
      return Expr0{loc, pexpr(), Expr0::PreDec};
    } else if (accept("+")) {
      return Expr0{loc, pexpr(), Expr0::Positive};
    } else if (accept("-")) {
      return Expr0{loc, pexpr(), Expr0::Negate};
    } else if (accept("!")) {
      return Expr0{loc, pexpr(), Expr0::Invert};
    } else {
      auto pexpr_ = pexpr();
      if (accept("++"))
        return Expr0{loc, MOVE(pexpr_), Expr0::PostInc};
      else if (accept("--"))
        return Expr0{loc, MOVE(pexpr_), Expr0::PostDec};
      else
        return Expr0{loc, MOVE(pexpr_), Expr0::None};
    }
  }
  PExpr pexpr() {
    auto pexpr_ = pexpr_base();
    while (true) {
      if (accept("[")) {
        auto loc = source_loc();
        auto index = expr();
        consume("]", "to end subscription operator");
        pexpr_ = Subscript{loc, MOVE(pexpr_), MOVE(index)};
      } else if (expect("..")) {
        break;
      } else if (accept(".")) {
        auto loc = source_loc();
        auto id_ = id();
        pexpr_ = MemberAccess{loc, MOVE(pexpr_), MOVE(id_)};
      } else if (expect("(")) {
        if (pexpr_.is<Id>()) {
          consume("(");
          pexpr_ = FunctionCall{pexpr_.as<Id>().sl, pexpr_.as<Id>().value, arg_list()};
          consume(")");
        } else if (pexpr_.is<MemberAccess>()) {
          consume("(");
          auto& p = pexpr_.as<MemberAccess>();
          auto args = arg_list();
          auto name = MOVE(p.id).value;
          args.insert(args.begin(), Expr{Expr0{p.sl, *MOVE(p.pexpr), Expr0::None}});
          pexpr_ = FunctionCall{p.id.sl, MOVE(name), MOVE(args)};
          consume(")");
        } else {
          error("An identifier must precedes function call operator ()");
        }
      } else {
        break;
      }
    }

    return pexpr_;
  }
  PExpr pexpr_base() {
    backup();
    if (accept("false")) {
      if (auto n = next(); !n || (!psl::isalpha(*n) && *n != '_')) {
        commit();
        return PExpr{BooleanLiteral{false}};
      }
    } else if (accept("true")) {
      if (auto n = next(); !n || (!psl::isalpha(*n) && *n != '_')) {
        commit();
        return PExpr{BooleanLiteral{true}};
      }
    }
    undo();
    if (expect("\"") || expect("'"))
      return PExpr{string_literal()};
    else if (expect("[")) {
      return PExpr(vector());
    } else if (expect("(")) {
      auto pexpr = PExpr{expr()};
      consume(")", "to balance the parenthesis");
      return pexpr;
    } else if ((expect(psl::isdigit, 0) || expect("-") || expect(".")) && !expect(".."))
      return PExpr{number()};
    else if (expect(psl::isalpha, 0) || expect("_"))
      return PExpr{id()};
    else {
      error("Expect a primary expression");
    }
  }
  LambdaExpr lambda() {
    auto loc = source_loc();
    consume("(", "to start parameter defintion");
    auto params = param_list();
    consume(")", "to end parameter defintion");
    consume(":", "to specify return type");
    auto return_type = type_name();
    auto body = block();
    return LambdaExpr(loc, FunctionDefinition(loc, "<>", return_type, params, body));
  }
  Vector vector() {
    auto loc = source_loc();
    consume("[", "to start short vector definition");
    auto args = psl::vector<Expr>{};
    if (!accept("]"))
      while (true) {
        args.push_back(expr());
        if (accept("]"))
          break;
        else
          consume(",", "to specify more element");
      }
    return Vector{loc, MOVE(args)};
  }
  psl::vector<ParameterDeclaration> param_list() {
    auto args = psl::vector<ParameterDeclaration>();
    if (!expect(")"))
      while (true) {
        auto name = id();
        consume(":", "to specify its type");
        auto type = type_name();
        args.push_back({MOVE(name), MOVE(type)});
        if (expect(")"))
          break;
        else
          consume(",", "to continue specify parameter");
      }
    return args;
  }
  psl::string type_names() {
    auto r = psl::string();
    while (!expect(")"))
      r += type_name().value + ", ";
    if (r.size())
      r.pop_back(2);
    return r;
  }
  Id type_name() {
    auto loc = source_loc();
    if (accept("(")) {
      auto params = type_names();
      consume(")");
      consume(":");
      auto rtype = type_name().value;
      return Id(loc, "(" + params + "): " + rtype);
    } else {
      auto id_ = id();
      if (accept("&"))
        id_.value += "&";
      return id_;
    }
  }
  psl::vector<Expr> arg_list() {
    auto args = psl::vector<Expr>();
    if (!expect(")"))
      while (true) {
        args.push_back(expr());
        if (expect(")"))
          break;
        else
          consume(",", "to continue specifying argument");
      }
    return args;
  }
  Id id() {
    auto loc = source_loc();
    auto pred = [](char c) { return psl::isalpha(c) || c == '_'; };
    if (!expect(pred, 0))
      error("Expect a letter or `_` to start an identifier");
    auto str = psl::string{};
    while (true) {
      str.push_back(*next());
      proceed();
      auto n = next();
      if (!n || (!pred(*n) && !psl::isdigit(*n)))
        break;
    }
    consume_spaces();
    return Id{loc, MOVE(str)};
  }
  psl::optional<Id> maybe_id() {
    auto loc = source_loc();
    auto pred = [](char c) { return psl::isalpha(c) || c == '_'; };
    if (!expect(pred, 0))
      return psl::nullopt;
    auto str = psl::string{};
    while (true) {
      str.push_back(*next());
      proceed();
      auto n = next();
      if (!n || (!pred(*n) && !psl::isdigit(*n)))
        break;
    }
    consume_spaces();
    return Id{loc, MOVE(str)};
  }
  NumberLiteral number() {
    if (!expect(psl::isdigit, 0) && !expect("-") && !expect("."))
      error("Expect a digit, `-`, or `.` to start a number");
    CHECK(!expect(".."));
    auto pass_decimal_point = false;
    auto str = psl::string{};
    while (true) {
      str.push_back(*next());
      proceed();
      if (str.back() == '.') {
        pass_decimal_point = true;
      }
      if (expect(".."))
        break;
      auto n = next();
      if (!n || !((psl::isdigit(*n) || (!pass_decimal_point && *n == '.'))))
        break;
    }
    if (!pass_decimal_point) {
      if (str.size() > 15 || psl::stoi64(str) > psl::numeric_limits<int>::max())
        error("This number is too large, need to be < ", psl::numeric_limits<int>::max());
    }
    consume_spaces();
    return NumberLiteral{MOVE(str)};
  }
  StringLiteral string_literal() {
    auto str = psl::string();
    auto single_quote = false;
    if (!accept("'", ExplicitParameter(false))) {
      consume("\"", ExplicitParameter(false));
    } else {
      single_quote = true;
    }
    auto escape = false;
    while (auto n = next()) {
      if (escape) {
        if (*n == 'n')
          str.back() = '\n';
        else if (*n == 't')
          str.back() = '\t';
        else if (*n == '"')
          str.back() = '"';
        else
          error("Unknown escape character");
        escape = false;
        proceed();
        continue;
      }
      if (*n == (single_quote ? '\'' : '"'))
        break;
      proceed();
      str.push_back(*n);
      escape = *n == '\\';
    }
    if (single_quote)
      consume("'", "to end string literal");
    else
      consume("\"", "to end string literal");
    return StringLiteral(str);
  }

  SourceLines sl;

private:
  void consume_spaces() {
    auto in_comment = false;
    auto row_ = row;
    while (true) {
      if (row > row_) {
        in_comment = false;
        row_ = row;
      }

      if (auto n = next()) {
        if (*n == '#') {
          in_comment = true;
          proceed();
          continue;
        }
        if (in_comment) {
          proceed();
          continue;
        }
        if (psl::isspace(*n)) {
          proceed();
          continue;
        }
      }
      break;
    }
  }
  bool expect(auto pred, size_t offset) {
    if (auto n = next(offset)) {
      if (pred(*n))
        return true;
      else
        return false;
    } else {
      return false;
    }
  }
  bool expect(psl::string_view str) {
    for (size_t i = 0; i < str.size(); i++) {
      if (auto n = next(i)) {
        if (*n != str[i])
          return false;
      } else {
        return false;
      }
    }

    return true;
  }
  bool accept(psl::string_view str,
              ExplicitParameter<bool> remove_tail_spaces = ExplicitParameter<bool>(true)) {
    if (expect(str)) {
      proceed(str.size());
      if (remove_tail_spaces)
        consume_spaces();
      return true;
    } else {
      return false;
    }
  }
  void consume(psl::string_view str, psl::string_view message) {
    for (char c : str) {
      if (auto n = next()) {
        if (*n != c)
          error("Expect `", c, "` ", message);
        else
          proceed();
      } else {
        error("Expect `", c, "` ", message);
      }
    }
    consume_spaces();
  }
  void consume(psl::string_view str,
               ExplicitParameter<bool> remove_tail_spaces = ExplicitParameter<bool>(true)) {
    for (char c : str) {
      if (auto n = next()) {
        if (*n != c)
          error("Expect `", c, "`", (str.size() > 1 ? " as part of `" + str + "`" : ""), ", get `",
                *n, "`");
        else
          proceed();
      } else {
        error("Expect `", c, "`", (str.size() > 1 ? " as part of `" + str + "`" : ""),
              ", but reach end-of-file");
      }
    }
    if (remove_tail_spaces)
      consume_spaces();
  }

  psl::optional<char> next(size_t n) {
    backup();
    proceed(n);
    auto r = next();
    undo();
    return r;
  }

  void proceed(size_t n = 1) {
    for (; n != 0; n--) {
      column++;
      to_valid_pos();
    }
  }
  void to_valid_pos() {
    while (auto line = next_line()) {
      CHECK_LE(column, line->size());
      if (column == line->size()) {
        row++;
        column = 0;
      } else {
        break;
      }
    }
  }

  void backup() {
    backup_stack.emplace_back(row, column);
  }
  void commit() {
    CHECK(backup_stack.size() != 0);
    backup_stack.pop_back();
  }
  void undo() {
    CHECK(backup_stack.size() != 0);
    row = backup_stack.back().first;
    column = backup_stack.back().second;
    backup_stack.pop_back();
  }
  psl::optional<psl::string_view> next_line() const {
    return sl.next_line(row);
  }
  psl::optional<char> next() const {
    return sl.next(source_loc());
  }

  SourceLoc source_loc() const {
    return {row, column};
  }
  template <typename... Args>
  [[noreturn]] void error(const Args&... args) {
    sl.error(source_loc(), args...);
  }

  size_t row = 0;
  size_t column = 0;
  psl::vector<psl::pair<size_t, size_t>> backup_stack;
  static constexpr size_t row_padding = 1;
  static constexpr size_t invalid = size_t(-1);
};

static psl::pair<Block, SourceLines> parse_as_block(psl::string source) {
  auto parser = Parser{source};
  return {parser.block(true), MOVE(parser.sl)};
}
void jit_interpret(Context& context, psl::string source) {
  using namespace llvm;

  struct MyObj {
    void call() {
      Log(f());
    }
    psl::function<float()> f;
  };
  context.type<MyObj>("MyObj").ctor<psl::function<float()>>().method<&MyObj::call>("call");

  auto shutdown_obj = llvm_shutdown_obj();
  InitializeNativeTarget();
  InitializeNativeTargetAsmPrinter();

  auto C = LLVMContext();
  auto M = std::make_unique<llvm::Module>("@module.pine", C);
  auto F = llvm::Function::Create(llvm::FunctionType::get(llvm::Type::getVoidTy(C), {}, false),
                                  llvm::Function::ExternalLinkage, "main", M.get());

  auto [block, sl] = parse_as_block(MOVE(source));

  auto entry = BasicBlock::Create(C, "entry", F);
  auto exit = BasicBlock::Create(C, "exit", F);

  auto m = Module{.context = context,
                  .source = MOVE(sl),
                  .C = C,
                  .M = M.get(),
                  .F = F,
                  .entry = entry,
                  .exit = exit,
                  .builder = IRBuilder<>(entry)};

  {
    Profiler _("Parsing");

    auto type_map = psl::map<psl::string, llvm::Type*>();
    auto add_type = [&](auto& add_type, const pine::TypeTrait* trait) -> llvm::Type* {
      llvm::Type*& ptr = type_map[trait->name];
      if (ptr)
        return ptr;
      if (trait->is_fundamental()) {
        if (trait->name == "void")
          ptr = Type::getVoidTy(C);
        else if (trait->name == "bool")
          ptr = Type::getInt1Ty(C);
        else if (trait->name == "f32")
          ptr = Type::getFloatTy(C);
        else {
          switch (trait->byte_size) {
            case 1: ptr = Type::getInt8Ty(C); break;
            case 2: ptr = Type::getInt16Ty(C); break;
            case 4: ptr = Type::getInt32Ty(C); break;
            case 8: ptr = Type::getInt64Ty(C); break;
            case 16: ptr = Type::getInt128Ty(C); break;
            default:
              if (trait->byte_size % 8 == 0)
                ptr = ArrayType::get(Type::getInt64Ty(C), trait->byte_size / 8);
              else if (trait->byte_size % 4 == 0)
                ptr = ArrayType::get(Type::getInt32Ty(C), trait->byte_size / 4);
              else if (trait->byte_size % 2 == 0)
                ptr = ArrayType::get(Type::getInt16Ty(C), trait->byte_size / 2);
              else
                ptr = ArrayType::get(Type::getInt8Ty(C), trait->byte_size);
              break;
              // Fatal("Did you forget to set the layout of `", trait->name, "`?");
          }
        }
      } else {
        auto members = std::vector<llvm::Type*>();
        for (auto&& member : trait->members)
          members.push_back(add_type(add_type, member));
        ptr = llvm::StructType::create(C, members, trait->name.c_str());
      }

      llvm::Function* destructor = nullptr;
      if (trait->destructor) {
        auto name = "@dtor." + trait->name;
        destructor = llvm::Function::Create(
            llvm::FunctionType::get(llvm::Type::getVoidTy(C), {m.builder.getPtrTy()}, false),
            llvm::Function::ExternalLinkage, name.c_str(), M.get());
        llvm::sys::DynamicLibrary::AddSymbol(name.c_str(), trait->destructor);
      }

      m.add_type(trait->name, ptr, destructor);
      return ptr;
    };
    for (const auto& [name, type] : context.types)
      add_type(add_type, type.get());

    for (const auto& f : context.functions)
      m.add_function(f);

    for (const auto& [name, var] : context.constants)
      m.add_constant(name, var);

    block.emit(m);

    m.finalize_function();
    // F->dump();

    LoopAnalysisManager LAM;
    FunctionAnalysisManager FAM;
    CGSCCAnalysisManager CGAM;
    ModuleAnalysisManager MAM;

    PassBuilder PB;

    PB.registerModuleAnalyses(MAM);
    PB.registerCGSCCAnalyses(CGAM);
    PB.registerFunctionAnalyses(FAM);
    PB.registerLoopAnalyses(LAM);
    PB.crossRegisterProxies(LAM, FAM, CGAM, MAM);

    ModulePassManager MPM = PB.buildPerModuleDefaultPipeline(OptimizationLevel::O3);
    MPM.run(*M, MAM);
  }

  auto EE = psl::unique_ptr<llvm::ExecutionEngine>(llvm::EngineBuilder(MOVE(M)).create());
  EE->setVerifyModules(true);
  for (auto& [name, fi] : m.function_to_be_compiled) {
    Debug("Compiling: ", name);
    context.functions[fi].set_ptr((void*)EE->getFunctionAddress(name.c_str()));
  }
  Debug("Compiling: main");
  auto main = (void (*)())(void*)EE->getFunctionAddress("main");

  {
    Profiler _("Execution");
    main();
  }
}

}  // namespace pine