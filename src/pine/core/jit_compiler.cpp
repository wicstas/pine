#include <pine/core/jit_compiler.h>
#include <pine/core/profiler.h>
#include <pine/core/atomic.h>

#include <psl/algorithm.h>
#include <psl/variant.h>

namespace pine {
struct PExpr;
struct Expr0;
struct Block;
struct BlockElem;
struct FunctionDefinition;

template <typename T>
struct ExplicitParameter {
  explicit ExplicitParameter(T value) : value(MOVE(value)) {
  }
  operator T&() {
    return value;
  }
  T value;
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
// static psl::string extract_return_type(psl::string_view type) {
//   auto depth = 0;
//   for (size_t i = 0; i < type.size(); i++) {
//     if (type[i] == '(')
//       depth++;
//     if (type[i] == ')')
//       depth--;
//     if (depth == 0 && type[i] == ':')
//       return psl::string(type.substr(i + 2));
//   }
//   return "";
// }
static TypeTag type_tag_from_string(psl::string_view name) {
  if (name.size() && name.back() == '&')
    return TypeTag(psl::string(name.substr(0, name.size() - 1)), true);
  else
    return TypeTag(psl::string(name), false);
}

SourceLines::SourceLines(psl::string_view tokens, size_t paddings)
    : lines(split(tokens, psl::equal_to_any('\n', '\r', '\f'))), paddings(paddings) {
}
psl::optional<psl::string_view> SourceLines::next_line(size_t row) const {
  CHECK_LE(row, lines.size());
  if (row == lines.size())
    return psl::nullopt;
  return lines[row];
}
psl::optional<char> SourceLines::next(SourceLoc sl) const {
  if (auto line = next_line(sl.row)) {
    DCHECK_LT(sl.column, line->size());
    return (*line)[sl.column];
  }
  return psl::nullopt;
}
[[noreturn]] void SourceLines::error_impl(SourceLoc sl, psl::string_view message) const {
  CHECK(paddings != invalid);
  CHECK(sl.column != size_t(-1));
  CHECK(sl.row != size_t(-1));
  CHECK_LE(sl.row, lines.size());

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

  ReturnControlToMain(message, "\n", vicinity);
}

struct ValueResult {
  operator bool() const {
    return ptr;
  }

  llvm::Value* ptr = nullptr;
  size_t type_idx;
};

struct Module {
  struct FunctionInfo {
    const Function* f = nullptr;
    llvm::Function* llvm_f = nullptr;
    llvm::Value* f_value = nullptr;
    size_t param_byte_size = 0;
  };
  struct TypeInfo {
    psl::string name;
    size_t byte_size = 0;
    llvm::Type* ptr = nullptr;
    llvm::Function* copy_operator = nullptr;
  };

  auto find_f(psl::string_view name, psl::span<const pine::TypeTag> atypes) {
    return context.find_f(name, atypes);
  }
  auto find_f(psl::string_view name, psl::span<const pine::ValueResult> args) {
    return context.find_f(name, psl::vector<TypeTag>(psl::transform(args, [this](auto&& x) {
                            return TypeTag(type(x.type_idx).name);
                          })));
  }
  ValueResult find_variable(psl::string_view name) {
    for (auto& map : psl::reverse_adapter(variables))
      if (auto it = map.find(name); it != map.end())
        return it->second;
    return {};
  }
  [[noreturn]] void error(SourceLoc loc, const auto&... args) const {
    source.error(loc, args...);
  }

  llvm::BasicBlock* create_block() {
    return llvm::BasicBlock::Create(C, psl::to_string("block.", block_counter).c_str(), F);
  }
  void cond_br(ValueResult cond, llvm::BasicBlock* true_block, llvm::BasicBlock* false_block) {
    builder.CreateCondBr(
        builder.CreateICmpNE(
            builder.CreateLoad(builder.getInt8Ty(),
                               builder.CreateConstGEP2_32(type(cond.type_idx).ptr, cond.ptr, 0, 0)),
            builder.getInt8(0)),
        true_block, false_block, (llvm::Instruction*)nullptr);
  }

  llvm::Value* alloc(llvm::Type* type) {
    auto current = builder.GetInsertBlock();
    builder.SetInsertPoint(&F->getEntryBlock(), --F->getEntryBlock().end());
    auto value = builder.CreateAlloca(type);
    builder.SetInsertPoint(current);
    return value;
  }

  void add_type(psl::string name, llvm::Type* type, size_t byte_size, void* copy_operator) {
    if (copy_operator) {
      auto c_name = "copy_" + name;
      llvm::sys::DynamicLibrary::AddSymbol(c_name.c_str(), copy_operator);
      name2type_idx[name] = types.size();
      types.emplace_back(
          MOVE(name), byte_size, type,
          llvm::Function::Create(llvm::FunctionType::get(type, {type}, false),
                                 llvm::Function::ExternalLinkage, c_name.c_str(), M));
    } else {
      name2type_idx[name] = types.size();
      types.emplace_back(MOVE(name), byte_size, type, nullptr);
    }
  }
  void add_function(const Function& f) {
    auto rtype = llvm::Type::getVoidTy(C);
    auto param_byte_size = f.byte_size() + type(f.rtype().name).byte_size +
                           psl::sum<size_t>(psl::transform(f.ptypes(), [&](auto&& x) {
                             return x.is_ref ? 8 : type(x.name).byte_size;
                           }));
    auto ptype = llvm::PointerType::getUnqual(
        llvm::ArrayType::get(llvm::Type::getInt8Ty(C), param_byte_size));
    auto function =
        llvm::Function::Create(llvm::FunctionType::get(rtype, ptype, false),
                               llvm::Function::ExternalLinkage, f.signature().c_str(), M);
    llvm::sys::DynamicLibrary::AddSymbol(f.signature().c_str(), f.ptr());
    functions.emplace_back(&f, function, create_function_value(f), param_byte_size);
  }
  llvm::Value* create_function_value(const Function& f) {
    using namespace llvm;
    auto byte_type = Type::getInt8Ty(C);
    auto type = ArrayType::get(byte_type, f.byte_size());

    auto ptr = (uint8_t*)f.object_ptr();
    auto values = std::vector<Constant*>(f.byte_size());
    for (uint32_t i = 0; i < f.byte_size(); i++)
      values[i] = ConstantInt::get(byte_type, ptr[i]);
    auto value = ConstantArray::get(type, values);

    return new GlobalVariable(*M, type, true, GlobalValue::PrivateLinkage, value);
  }

  const FunctionInfo& function(size_t i) const {
    return functions[i];
  }

  size_t type_idx(psl::string_view name) const {
    if (psl::contains(name, '('))
      return 0;
    if (auto it = name2type_idx.find(name); it != name2type_idx.end())
      return it->second;
    else
      Fatal("Type `", name, "` is not found");
  }

  const TypeInfo& type(psl::string_view name) const {
    return type(type_idx(name));
  }
  const TypeInfo& type(size_t idx) const {
    return types[idx];
  }

  ValueResult call(size_t function_index, psl::span<const ValueResult> args) {
    auto& [f, llvm_f, f_data, param_byte_size] = function(function_index);
    auto param_type = llvm::ArrayType::get(builder.getInt8Ty(), param_byte_size);
    auto param = alloc(param_type);
    builder.CreateMemCpy(param, llvm::Align(1), f_data, llvm::Align(1), f->byte_size());
    auto offset = f->byte_size() + type(f->rtype().name).byte_size;
    for (size_t i = 0; i < args.size(); i++) {
      auto ptr = builder.CreateConstGEP2_32(param_type, param, 0, offset);
      auto copy_as_is = f->ptypes()[i].is_ref || f->ptypes()[i].name == "cstr";
      auto byte_size = copy_as_is ? 8 : type(args[i].type_idx).byte_size;
      if (copy_as_is)
        builder.CreateAlignedStore(args[i].ptr, builder.CreateBitCast(ptr, builder.getPtrTy()),
                                   llvm::Align(1));
      else
        builder.CreateMemCpy(ptr, llvm::Align(1), args[i].ptr, llvm::Align(1), byte_size);
      offset += byte_size;
    }
    builder.CreateCall(llvm_f, {param});
    auto res = alloc(type(f->rtype().name).ptr);
    auto ptr = builder.CreateConstGEP2_32(param_type, param, 0, f->byte_size());
    builder.CreateMemCpy(res, llvm::Align(1), ptr, llvm::Align(1), type(f->rtype().name).byte_size);
    return {res, type_idx(f->rtype().name)};
  }

  Context& context;
  SourceLines source;

  llvm::LLVMContext& C;
  llvm::Module* M;
  llvm::Function* F;
  llvm::IRBuilder<> builder;
  int block_counter = 0;

  psl::vector<psl::map<psl::string, ValueResult>> variables =
      psl::vector<psl::map<psl::string, ValueResult>>(1);
  psl::vector<FunctionInfo> functions = {};
  psl::vector<TypeInfo> types = psl::vector_of(TypeInfo{"@function", 16, nullptr, nullptr});
  psl::map<psl::string, size_t> name2type_idx = {};

  struct LoopInfo {
    llvm::BasicBlock* at_inc;
    llvm::BasicBlock* end;
  };
  psl::vector<LoopInfo> loop_stack = {};
};

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
    // clang-format on
  };

  Expr() = default;
  template <typename T>
  requires requires(T x) { x.sl; }
  Expr(T x);
  Expr(Expr0 x);
  Expr(Expr a, Expr b, Op op);
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  Op op;
  psl::Box<Expr0> x;
  psl::Box<Expr> a;
  psl::Box<Expr> b;
};

struct Id {
  Id(SourceLoc sl, psl::string value) : sl(sl), value(MOVE(value)) {
  }
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::string value;
};
struct NumberLiteral {
  NumberLiteral(const psl::string& str);
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  bool is_float = false;
  float valuef;
  int valuei;
};
struct BooleanLiteral {
  BooleanLiteral(bool value) : value(value) {
  }
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  bool value;
};
struct StringLiteral {
  StringLiteral(psl::string value) : value(MOVE(value)) {
  }
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::string value;
};
struct Vector {
  Vector(SourceLoc sl, psl::vector<Expr> args) : sl(sl), args{MOVE(args)} {
  }
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::vector<Expr> args;
};
struct FunctionCall {
  FunctionCall(SourceLoc sl, psl::string name, psl::vector<Expr> args)
      : sl(sl), name{MOVE(name)}, args{MOVE(args)} {
  }
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::string name;
  psl::vector<Expr> args;
};
struct MemberAccess {
  MemberAccess(SourceLoc sl, PExpr pexpr, Id id);
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::Box<PExpr> pexpr;
  Id id;
};
struct Subscript {
  Subscript(SourceLoc sl, PExpr pexpr, Expr index);
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::Box<PExpr> pexpr;
  Expr index;
};
struct LambdaExpr {
  LambdaExpr(SourceLoc sl, psl::vector<Id> captures, FunctionDefinition body);
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  psl::vector<Id> captures;
  psl::Box<FunctionDefinition> body;
};

struct PExpr : psl::variant<Id, NumberLiteral, BooleanLiteral, StringLiteral, Vector, FunctionCall,
                            MemberAccess, Subscript, Expr, LambdaExpr> {
  using variant::variant;
  ValueResult emit(Module& m) const {
    return dispatch([&](auto&& x) { return x.emit(m); });
  }
};

struct Expr0 {
  enum Op { None, PreInc, PreDec, PostInc, PostDec, Positive, Negate, Invert };
  Expr0(SourceLoc sl, PExpr x, Op op = None) : sl(sl), x{MOVE(x)}, op{op} {};
  ValueResult emit(Module& m) const;

  SourceLoc sl;
  PExpr x;
  Op op;
};
struct Declaration {
  Declaration(SourceLoc sl, psl::string name, Expr expr, bool as_ref = false)
      : sl(sl), name{MOVE(name)}, expr{MOVE(expr)}, as_ref(as_ref) {
  }
  void emit(Module& m) const;

  SourceLoc sl;
  psl::string name;
  Expr expr;
  bool as_ref;
};
struct Semicolon {
  void emit(Module&) const {
  }
};
struct BreakStmt {
  BreakStmt(SourceLoc sl) : sl(sl) {
  }
  void emit(Module& m) const;

  SourceLoc sl;
};
struct ContinueStmt {
  ContinueStmt(SourceLoc sl) : sl(sl) {
  }
  void emit(Module& m) const;

  SourceLoc sl;
};
struct ReturnStmt {
  ReturnStmt(SourceLoc sl, psl::optional<Expr> expr) : sl(sl), expr(MOVE(expr)) {
  }
  void emit(Module& m) const;

private:
  SourceLoc sl;
  psl::optional<Expr> expr;
};
struct Stmt : psl::variant<Semicolon, Expr, Declaration, BreakStmt, ContinueStmt, ReturnStmt> {
  using variant::variant;
  void emit(Module& m) const;
};
struct Block {
  Block(psl::vector<BlockElem> elems);
  void emit(Module& m, bool noop = false) const;

  SourceLoc sl;
  psl::vector<BlockElem> elems;
};
struct While {
  While(SourceLoc sl, Expr condition, Block body);
  void emit(Module& m) const;

  SourceLoc sl;
  Expr condition;
  Block body;
};
struct For {
  For(SourceLoc sl, Stmt init, Expr condition, Expr inc, Block body);
  void emit(Module& m) const;

  SourceLoc sl;
  Stmt init;
  Expr condition;
  Expr inc;
  Block body;
};
struct If {
  If(SourceLoc sl, Expr condition, Block body);

  SourceLoc sl;
  Expr condition;
  Block body;
};
struct Else {
  Else(SourceLoc sl, Block body);

  SourceLoc sl;
  Block body;
};
struct IfElseChain {
  IfElseChain(psl::vector<If> ifs, psl::optional<Else> else_) : ifs{MOVE(ifs)}, else_{MOVE(else_)} {
  }
  void emit(Module& m) const;

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

  void emit(Module& m) const;

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
struct InternalClass {
  psl::vector<Variable> members;
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
};

// ================================================
// ================================================
// ================================================
ValueResult Id::emit(Module& m) const {
  if (auto x = m.find_variable(value))
    return x;
  else
    m.error(sl, "Variable `", value, "` is not found");
}
NumberLiteral::NumberLiteral(const psl::string& str) {
  if ((is_float = psl::find(str, '.') != str.end()))
    valuef = psl::stof(str);
  else
    valuei = psl::stoi(str);
}
ValueResult NumberLiteral::emit(Module& m) const {
  auto value = m.alloc(is_float ? llvm::Type::getFloatTy(m.C) : llvm::Type::getInt32Ty(m.C));
  if (is_float)
    m.builder.CreateStore(llvm::ConstantFP::get(m.C, llvm::APFloat(valuef)), value);
  else
    m.builder.CreateStore(m.builder.getInt32(valuei), value);
  return {value, m.type_idx(is_float ? "f32" : "i32")};
}
ValueResult BooleanLiteral::emit(Module& m) const {
  auto value = m.alloc(llvm::Type::getInt8Ty(m.C));
  m.builder.CreateStore(m.builder.getInt8(this->value), value);
  return {value, m.type_idx("bool")};
}
ValueResult StringLiteral::emit(Module& m) const {
  auto cstr = ValueResult(m.builder.CreateGlobalString(value.c_str()), m.type_idx("cstr"));
  return m.call(m.find_f("str", psl::array_of(TypeTag("cstr"))).function_index,
                psl::vector_of(cstr));
}
ValueResult Vector::emit(Module& m) const {
  if (args.size() < 2 || args.size() > 3)
    m.error(sl, "Only 2, 3, or 4 items can exist inside []");

  auto arg_values =
      psl::vector<ValueResult>(psl::transform(args, [&](auto&& x) { return x.emit(m); }));
  auto is_float = false;
  for (size_t i = 0; i < args.size(); i++)
    if (m.type(arg_values[i].type_idx).name == "f32")
      is_float = true;

  auto fr = Context::FindFResult();
  if (is_float) {
    if (args.size() == 2)
      fr = m.find_f("vec2", arg_values);
    else if (args.size() == 3)
      fr = m.find_f("vec3", arg_values);
    else if (args.size() == 4)
      fr = m.find_f("vec4", arg_values);
  } else {
    if (args.size() == 2)
      fr = m.find_f("vec2i", arg_values);
    else if (args.size() == 3)
      fr = m.find_f("vec3i", arg_values);
    else if (args.size() == 4)
      fr = m.find_f("vec4i", arg_values);
  }

  for (auto cv : fr.converts)
    arg_values[cv.position] = m.call(cv.converter_index, psl::vector_of(arg_values[cv.position]));

  return m.call(fr.function_index, arg_values);
}
ValueResult FunctionCall::emit(Module& m) const {
  auto arg_values =
      psl::vector<ValueResult>(psl::transform(args, [&](auto&& x) { return x.emit(m); }));
  auto fr = m.find_f(name, arg_values);
  for (auto cv : fr.converts) {
    arg_values[cv.position] = m.call(cv.converter_index, psl::vector_of(arg_values[cv.position]));
  }

  return m.call(fr.function_index, arg_values);
}
MemberAccess::MemberAccess(SourceLoc sl, PExpr pexpr, Id id)
    : sl(sl), pexpr(MOVE(pexpr)), id(MOVE(id)) {
}
ValueResult MemberAccess::emit(Module& m) const {
  auto x = pexpr->emit(m);
  auto fi = m.context.get_type_trait(m.type(x.type_idx).name).find_member_accessor_index(id.value);
  if (fi == size_t(-1))
    m.error(id.sl, "Can't find member `", id.value, "` in type `", m.type(x.type_idx).name, '`');
  return m.call(fi, psl::vector_of(x));
}
Subscript::Subscript(SourceLoc sl, PExpr pexpr, Expr index)
    : sl(sl), pexpr(MOVE(pexpr)), index(MOVE(index)) {
}
ValueResult Subscript::emit(Module& m) const {
  auto x = pexpr->emit(m);
  auto i = index.emit(m);
  auto fr = m.find_f("[]", psl::array_of(x, i));
  for (auto cv : fr.converts) {
    if (cv.position == 0)
      x = m.call(cv.converter_index, psl::array_of(x));
    if (cv.position == 1)
      i = m.call(cv.converter_index, psl::array_of(i));
  }
  return m.call(fr.function_index, psl::vector_of(x));
}
LambdaExpr::LambdaExpr(SourceLoc sl, psl::vector<Id> captures, FunctionDefinition body)
    : sl(sl), captures(MOVE(captures)), body(MOVE(body)) {
}
ValueResult LambdaExpr::emit(Module& m) const {
  (void)m;
  return {};
}
ValueResult Expr0::emit(Module& m) const {
  if (op == None)
    return x.emit(m);

  auto x_value = x.emit(m);
  auto fr = Context::FindFResult();
  auto ptypes = psl::array_of(TypeTag(m.type(x_value.type_idx).name));
  switch (op) {
    case None: break;
    case PreInc: fr = m.find_f("++x", ptypes); break;
    case PreDec: fr = m.find_f("--x", ptypes); break;
    case PostInc: fr = m.find_f("x++", ptypes); break;
    case PostDec: fr = m.find_f("x--", ptypes); break;
    case Positive: fr = m.find_f("+x", ptypes); break;
    case Negate: fr = m.find_f("-x", ptypes); break;
    case Invert: fr = m.find_f("!x", ptypes); break;
  }
  CHECK(fr.function_index != size_t(-1));
  for (auto cv : fr.converts) {
    x_value = m.call(cv.converter_index, psl::vector_of(x_value));
  }
  return m.call(fr.function_index, psl::vector_of(x_value));
}
template <typename T>
requires requires(T x) { x.sl; }
Expr::Expr(T x) : Expr(Expr0(x.sl, MOVE(x))) {
}
Expr::Expr(Expr0 x) : sl(x.sl), op{None}, x{MOVE(x)} {
}
Expr::Expr(Expr a, Expr b, Op op) : sl(a.sl), op{op}, a{MOVE(a)}, b{MOVE(b)} {
}
ValueResult Expr::emit(Module& m) const {
  if (op == None)
    return x->emit(m);

  try {
    auto a_value = a->emit(m);
    auto b_value = b->emit(m);
    auto ptypes = psl::array_of(TypeTag(m.type(a_value.type_idx).name),
                                TypeTag(m.type(b_value.type_idx).name));
    auto fr = Context::FindFResult();
    switch (op) {
      case None: CHECK(false); break;
      case Mul: fr = m.find_f("*", ptypes); break;
      case Div: fr = m.find_f("/", ptypes); break;
      case Mod: fr = m.find_f("%", ptypes); break;
      case Pow: fr = m.find_f("^", ptypes); break;
      case Add: fr = m.find_f("+", ptypes); break;
      case Sub: fr = m.find_f("-", ptypes); break;
      case Lt: fr = m.find_f("<", ptypes); break;
      case Gt: fr = m.find_f(">", ptypes); break;
      case Le: fr = m.find_f("<=", ptypes); break;
      case Ge: fr = m.find_f(">=", ptypes); break;
      case Eq: fr = m.find_f("==", ptypes); break;
      case Ne: fr = m.find_f("!=", ptypes); break;
      case And: fr = m.find_f("&&", ptypes); break;
      case Or: fr = m.find_f("||", ptypes); break;
      case AddE: fr = m.find_f("+=", ptypes); break;
      case SubE: fr = m.find_f("-=", ptypes); break;
      case MulE: fr = m.find_f("*=", ptypes); break;
      case DivE: fr = m.find_f("/=", ptypes); break;
      case ModE: fr = m.find_f("%=", ptypes); break;
      case Assi: fr = m.find_f("=", ptypes); break;
    }
    CHECK(fr.function_index != size_t(-1));
    for (auto cv : fr.converts) {
      if (cv.position == 0)
        a_value = m.call(cv.converter_index, psl::vector_of(a_value));
      if (cv.position == 1)
        b_value = m.call(cv.converter_index, psl::vector_of(b_value));
    }

    return m.call(fr.function_index, psl::vector_of(a_value, b_value));
  } catch (const Exception& e) {
    m.error(sl, e.what());
  }
}
void Declaration::emit(Module& m) const {
  m.variables.back()[name] = expr.emit(m);
}
void ReturnStmt::emit(Module& m) const {
  if (expr)
    m.builder.CreateRet(expr->emit(m).ptr);
  else
    m.builder.CreateRetVoid();
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
void Stmt::emit(Module& m) const {
  dispatch([&](auto&& x) { x.emit(m); });
}
Block::Block(psl::vector<BlockElem> elems) : elems{MOVE(elems)} {
}
void Block::emit(Module& m, bool noop) const {
  for (const auto& block_elem : elems)
    block_elem.emit(m);
  if (!noop) {
    auto block = m.create_block();
    m.builder.CreateBr(block);
    m.builder.SetInsertPoint(block);
  }
}
While::While(SourceLoc sl, Expr condition, Block body)
    : sl(sl), condition(MOVE(condition)), body(MOVE(body)) {
}
void While::emit(Module& m) const {
  auto cond_block = m.create_block();
  auto body_block = m.create_block();
  auto seq = m.create_block();

  m.builder.CreateBr(cond_block);
  m.builder.SetInsertPoint(body_block);
  m.loop_stack.emplace_back(cond_block, seq);
  body.emit(m, true);
  m.loop_stack.pop_back();
  m.builder.CreateBr(cond_block);

  m.builder.SetInsertPoint(cond_block);
  m.cond_br(condition.emit(m), body_block, seq);

  m.builder.SetInsertPoint(seq);
}
For::For(SourceLoc sl, Stmt init, Expr condition, Expr inc, Block body)
    : sl(sl), init{MOVE(init)}, condition{MOVE(condition)}, inc{MOVE(inc)}, body{MOVE(body)} {
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
  body.emit(m, true);
  m.loop_stack.pop_back();
  m.builder.CreateBr(inc_block);
  m.builder.SetInsertPoint(inc_block);
  inc.emit(m);
  m.builder.CreateBr(cond_block);

  m.builder.SetInsertPoint(cond_block);
  m.cond_br(condition.emit(m), body_block, seq);

  m.builder.SetInsertPoint(seq);
}
If::If(SourceLoc sl, Expr condition, Block body)
    : sl(sl), condition{MOVE(condition)}, body{MOVE(body)} {
}
Else::Else(SourceLoc sl, Block body) : sl(sl), body{MOVE(body)} {
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
    if (i + 1 < ifs.size())
      m.cond_br(ifs[i].condition.emit(m), body_blocks[i], cond_blocks[i + 1]);
    else if (else_)
      m.cond_br(ifs[i].condition.emit(m), body_blocks[i], else_block);
    else
      m.cond_br(ifs[i].condition.emit(m), body_blocks[i], seq);

    m.builder.SetInsertPoint(body_blocks[i]);
    ifs[i].body.emit(m, true);
    m.builder.CreateBr(seq);
  }
  if (else_) {
    m.builder.SetInsertPoint(else_block);
    else_->body.emit(m, true);
    m.builder.CreateBr(seq);
  }

  m.builder.SetInsertPoint(seq);
}
void FunctionDefinition::emit(Module& m) const {
  auto ptypes = psl::vector<TypeTag>();
  for (auto& param : params)
    ptypes.push_back(type_tag_from_string(param.type.value));
  auto signature = name + signature_from(type_tag_from_string(return_type.value), ptypes);

m.C;
  auto M = std::make_unique<llvm::Module>((signature + "_module").c_str(), m.C);

  auto F = llvm::Function::Create(
      llvm::FunctionType::get(m.builder.getVoidTy(), m.builder.getPtrTy(), false),
      llvm::Function::ExternalLinkage, signature.c_str(), M.get());

  auto backup_M = psl::exchange(m.M, M.get());
  auto backup_block = m.builder.GetInsertBlock();

  auto entry = llvm::BasicBlock::Create(m.C, "entry", F);
  m.builder.SetInsertPoint(entry);
  body.emit(m);

  m.M = backup_M;
  m.builder.SetInsertPoint(backup_block);

  for (auto& block : *F)
    for (auto it = block.begin(); it != block.end(); ++it)
      if (llvm::ReturnInst::classof(&*it))
        it = block.erase(it, std::next(it));

  F->dump();

  auto EE = psl::unique_ptr<llvm::ExecutionEngine>(llvm::EngineBuilder(MOVE(M)).create());
  EE->setVerifyModules(true);
  m.context(name) = (void (*)(void*))(void*)EE->getFunctionAddress(signature.c_str());
}
void ClassDefinition::emit(Module& m) const {
  (void)m;
}

struct Parser {
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
  Block block_or_single_line() {
    auto elem = block_elem();
    if (elem.is<Block>())
      return elem.as<Block>();
    else
      return Block(psl::vector_of(MOVE(elem)));
  }
  While while_() {
    consume("while");
    consume("(", "to begin condition");
    auto loc = source_loc();
    auto cond = expr();
    consume(")", "to end condition");
    auto body = block_or_single_line();
    return While{loc, MOVE(cond), MOVE(body)};
  }
  For for_() {
    consume("for");

    if (accept("(")) {
      auto init = stmt();
      auto loc = source_loc();
      auto cond = expr();
      consume(";");
      auto inc = expr();
      consume(")");
      auto body = block_or_single_line();
      return For{loc, MOVE(init), MOVE(cond), MOVE(inc), MOVE(body)};
    } else {
      auto loc = source_loc();
      auto id_ = id();
      consume("in");
      auto range_a = expr();
      consume("..", "to specify end point");
      auto range_b = expr();
      auto init = Stmt(Declaration(id_.sl, id_.value, range_a));
      auto cond = Expr(Expr(Expr0(loc, PExpr(id_), Expr0::None)), range_b, Expr::Lt);
      auto body = block_or_single_line();
      return For{loc, MOVE(init), MOVE(cond), Expr(Expr0(loc, PExpr(id_), Expr0::PreInc)),
                 MOVE(body)};
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
    consume("(", "to begin condition");
    auto loc = source_loc();
    auto cond = expr();
    consume(")", "to end condition");
    auto body = block_or_single_line();
    return If{loc, MOVE(cond), MOVE(body)};
  }
  If else_if_() {
    consume("else");
    consume("if");
    consume("(", "to begin condition");
    auto loc = source_loc();
    auto cond = expr();
    consume(")", "to end condition");
    auto body = block_or_single_line();
    return If{loc, MOVE(cond), MOVE(body)};
  }
  Else else_() {
    consume("else");
    auto loc = source_loc();
    return Else{loc, block_or_single_line()};
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
            it, Stmt(Declaration(member.name.sl, member.name.value,
                                 MemberAccess(loc, Id(loc, "self"), member.name), true)));
      }
    for (auto& method : methods)
      for (const auto& member : members) {
        auto loc = method.sl;
        method.body.elems.push_front(
            Stmt(Declaration(member.name.sl, member.name.value,
                             MemberAccess(loc, Id(loc, "self"), member.name), true)));
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
        init_stmts.push_back(Expr(Expr(MemberAccess(loc, Id(loc, "self"), Id(loc, "__" + name))),
                                  expr_, Expr::Assi));
        if (!accept(",")) {
          if (!expect("{")) {
            error("Expect either `,` to continue or '{' to begin function definition");
          }
        }
      }
    }
    auto block_ = block();
    // Create `self` and add the initializer of its members
    auto it = psl::next(block_.elems.insert(
        block_.elems.begin(),
        Stmt(Declaration(loc, "self", FunctionCall(loc, "__" + class_name, {})))));
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
    params.push_front(ParameterDeclaration(Id(loc, "self"), Id(loc, class_name)));
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
        if (accept(":=")) {
          stmt = Stmt(declaration(id_, false));
        } else if (accept("&=")) {
          stmt = Stmt(declaration(id_, true));
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

  Declaration declaration(Id id_, bool create_ref) {
    auto loc = source_loc();
    auto expr_ = expr();
    return Declaration{loc, MOVE(id_.value), MOVE(expr_), create_ref};
  }
  Expr expr() {
    auto exprs = psl::vector<Expr>{};
    auto ops = psl::vector<int>{};
    if (accept("(")) {
      exprs.push_back(expr());
      accept(")");
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
      else if (accept("="))  ops.push_back(0000100101);
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
      backup();
      accept("[");
      auto expect_lambda = expect("&");
      undo();
      if (expect_lambda)
        return PExpr(lambda());

      backup();
      auto vector_ = vector();
      if (expect("(")) {
        undo();
        return PExpr(lambda());
      } else {
        return PExpr(MOVE(vector_));
      }
    } else if (accept("(")) {
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
    consume("[", "to specify capture list");
    auto captures = psl::vector<Id>();
    while (!accept("]")) {
      auto is_ref = accept("&");
      auto id_ = id();
      if (is_ref)
        id_.value = "&" + id_.value;
      captures.push_back(MOVE(id_));
      accept(",");
    }
    consume("(", "to start parameter defintion");
    auto params = param_list();
    consume(")", "to end parameter defintion");
    consume(":", "to specify return type");
    auto return_type = type_name();
    auto body = block();
    return LambdaExpr(loc, captures, FunctionDefinition(loc, "()", return_type, params, body));
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

psl::pair<Block, SourceLines> parse_as_block(psl::string source) {
  auto parser = Parser{source};
  return {parser.block(true), MOVE(parser.sl)};
}

void jit_compile(Context& context, psl::string source, llvm::LLVMContext& C, llvm::Module* M,
                 llvm::Function* F) {
  using namespace llvm;
  auto [block, sl] = parse_as_block(MOVE(source));

  auto entry = BasicBlock::Create(C, "entry", F);

  auto m = Module{.context = context,
                  .source = MOVE(sl),
                  .C = C,
                  .M = M,
                  .F = F,
                  .builder = IRBuilder<>(entry)};

  for (const auto& [name, type] : context.types) {
    if (name == "i32")
      m.add_type(name, Type::getInt32Ty(C), type.byte_size(), type.copy_operator);
    else if (name == "f32")
      m.add_type(name, Type::getFloatTy(C), type.byte_size(), type.copy_operator);
    else
      m.add_type(name, ArrayType::get(Type::getInt8Ty(C), type.byte_size()), type.byte_size(),
                 type.copy_operator);
  }

  for (const auto& f : context.functions)
    m.add_function(f);

  block.emit(m, true);

  m.builder.CreateRetVoid();

  for (auto& block : *m.F)
    for (auto it = block.begin(); it != block.end(); ++it)
      if (BranchInst::classof(&*it) || ReturnInst::classof(&*it))
        block.erase(std::next(it), block.end());
}

void jit_interpret(Context& context, psl::string source) {
  auto shutdown_obj = llvm::llvm_shutdown_obj();
  llvm::InitializeNativeTarget();
  llvm::InitializeNativeTargetAsmPrinter();

  auto C = llvm::LLVMContext();
  auto M = std::make_unique<llvm::Module>("pine_module", C);
  auto main_ = llvm::Function::Create(llvm::FunctionType::get(llvm::Type::getVoidTy(C), {}, false),
                                      llvm::Function::ExternalLinkage, "main", M.get());

  jit_compile(context, MOVE(source), C, M.get(), main_);

  main_->dump();

  auto EE = psl::unique_ptr<llvm::ExecutionEngine>(llvm::EngineBuilder(MOVE(M)).create());
  EE->setVerifyModules(true);
  auto main = (void (*)())(void*)EE->getFunctionAddress("main");
  main();
}

}  // namespace pine