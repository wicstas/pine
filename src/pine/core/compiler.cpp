#include <pine/core/compiler.h>
#include <pine/core/profiler.h>
#include <pine/core/atomic.h>

#include <psl/algorithm.h>
#include <psl/variant.h>

namespace pine {

struct PExpr;
struct Expr0;
struct Block;
struct PBlock;
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

psl::vector<psl::string> split(psl::string_view input, auto pred) {
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

Bytecodes::Bytecodes(SourceLines sl) : sl(MOVE(sl)) {
}

size_t Bytecodes::add(SourceLoc sl, Bytecode::Instruction instruction, size_t value0,
                      size_t value1) {
  codes.emplace_back(sl, instruction, value0, value1);
  return codes.size() - 1;
}
void Bytecodes::placehold_typed(TypeTag type, psl::string name) {
  stack.push_back(MOVE(type));
  name_top_var(MOVE(name));
}
void Bytecodes::add_typed(SourceLoc sl, Bytecode::Instruction instruction, TypeTag type,
                          size_t value, psl::span<uint16_t> args) {
  codes.push_back({sl, instruction, value, 0, args});
  stack.push_back(MOVE(type));
}

void Bytecodes::name_top_var(psl::string name) {
  CHECK(stack.size() != 0);
  variable_map.push_back({MOVE(name), stack.size() - 1});
}
const TypeTag& Bytecodes::var_type(size_t var_index) const {
  CHECK_LE(var_index, top_var_index());
  return stack[var_index];
}
uint16_t Bytecodes::var_index_by_name(psl::string_view name) const {
  if (auto it = psl::find_if(psl::reverse_adapter(variable_map),
                             psl::composite(psl::equal_to(name), psl::first_of_pair));
      it.unwrap() != variable_map.end()) {
    return it->second;
  } else {
    return uint16_t(-1);
  }
}
uint16_t Bytecodes::top_var_index() const {
  CHECK(stack.size() != 0);
  return stack.size() - 1;
}
void Bytecodes::pop_stack() {
  stack.pop_back();
}
size_t Bytecodes::stack_size() const {
  return stack.size();
}

size_t Bytecodes::add_string(psl::string value) {
  string_region.push_back(MOVE(value));
  return string_region.size() - 1;
}
const psl::string& Bytecodes::get_string(size_t i) const {
  CHECK_LT(i, string_region.size());
  return string_region[i];
}

void Bytecodes::enter_scope() {
  scope_stack.push_back({stack.size(), variable_map.size()});
}
void Bytecodes::exit_scope() {
  DCHECK(scope_stack.size() != 0);
  stack.erase_range(stack.begin() + scope_stack.back().type_stack_position, stack.end());
  variable_map.erase_range(variable_map.begin() + scope_stack.back().variable_map_position,
                           variable_map.end());
  scope_stack.pop_back();
}
psl::string Bytecodes::to_string(const Context& context) const {
  auto result = psl::string();
  for (const auto& code : codes) {
    switch (code.instruction) {
      case Bytecode::Break: result += "Break"; break;
      case Bytecode::Continue: result += "Continue"; break;
      case Bytecode::Return: result += "Return"; break;
      case Bytecode::LoadGlobalVar: result += "LoadGlobalVar"; break;
      case Bytecode::LoadFunction: result += "LoadFunction"; break;
      case Bytecode::Copy: result += "Copy"; break;
      case Bytecode::MakeRef: result += "MakeRef"; break;
      case Bytecode::LoadFloatConstant:
        result += "Load " + psl::to_string(psl::bitcast<float>(uint32_t(code.value)));
        break;
      case Bytecode::LoadIntConstant:
        result += "Load " + psl::to_string(psl::bitcast<int>(uint32_t(code.value)));
        break;
      case Bytecode::LoadBoolConstant: result += "Load " + psl::to_string(bool(code.value)); break;
      case Bytecode::LoadStringConstant: result += "Load \"" + get_string(code.value) + "\""; break;
      case Bytecode::Call: result += "Call " + context.functions[code.value].signature(); break;
      case Bytecode::InvokeAsFunction: result += "Invoke"; break;
      case Bytecode::Jump: result += "Jump"; break;
      case Bytecode::JumpIfNot: result += "JumpIfNot"; break;
      case Bytecode::UnwindStack: result += "Unwind"; break;
    }
    result += "\n";
  }
  if (result.size())
    result.pop_back();
  return result;
}

static TypeTag type_tag_from_string(psl::string_view name) {
  if (name.size() && name.back() == '&')
    return TypeTag(psl::string(name.substr(0, name.size() - 1)), true);
  else
    return TypeTag(psl::string(name), false);
}

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
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  Op op;
  psl::Box<Expr0> x;
  psl::Box<Expr> a;
  psl::Box<Expr> b;
};

struct Id {
  Id(SourceLoc sl, psl::string value) : sl(sl), value(MOVE(value)) {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::string value;
};
struct NumberLiteral {
  NumberLiteral(const psl::string& str);
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  SourceLoc sl;
  bool is_float = false;
  float valuef;
  int valuei;
};
struct BooleanLiteral {
  BooleanLiteral(bool value) : value(value) {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  SourceLoc sl;
  bool value;
};
struct StringLiteral {
  StringLiteral(psl::string value) : value(MOVE(value)) {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::string value;
};
struct Vector {
  Vector(SourceLoc sl, psl::vector<Expr> args) : sl(sl), args{MOVE(args)} {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::vector<Expr> args;
};
struct FunctionCall {
  FunctionCall(SourceLoc sl, psl::string name, psl::vector<Expr> args)
      : sl(sl), name{MOVE(name)}, args{MOVE(args)} {
  }
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::string name;
  psl::vector<Expr> args;
};
struct MemberAccess {
  MemberAccess(SourceLoc sl, PExpr pexpr, Id id);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::Box<PExpr> pexpr;
  Id id;
};
struct Subscript {
  Subscript(SourceLoc sl, PExpr pexpr, Expr index);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::Box<PExpr> pexpr;
  Expr index;
};
struct LambdaExpr {
  LambdaExpr(SourceLoc sl, psl::vector<Id> captures, FunctionDefinition body);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::vector<Id> captures;
  psl::Box<FunctionDefinition> body;
};

struct PExpr : psl::variant<Id, NumberLiteral, BooleanLiteral, StringLiteral, Vector, FunctionCall,
                            MemberAccess, Subscript, Expr, LambdaExpr> {
  using variant::variant;
  uint16_t emit(Context& context, Bytecodes& bytecodes) const {
    return dispatch([&](auto&& x) { return x.emit(context, bytecodes); });
  }
};

struct Expr0 {
  enum Op { None, PreInc, PreDec, PostInc, PostDec, Positive, Negate, Invert };
  Expr0(SourceLoc sl, PExpr x, Op op = None) : sl(sl), x{MOVE(x)}, op{op} {};
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  PExpr x;
  Op op;
};
struct Declaration {
  Declaration(SourceLoc sl, psl::string name, Expr expr, bool as_ref = false)
      : sl(sl), name{MOVE(name)}, expr{MOVE(expr)}, as_ref(as_ref) {
  }
  void emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::string name;
  Expr expr;
  bool as_ref;
};
struct Semicolon {
  void emit(Context&, Bytecodes&) const {
  }
};
struct BreakStmt {
  BreakStmt(SourceLoc sl) : sl(sl) {
  }
  void emit(Context&, Bytecodes& bytecodes) const {
    bytecodes.add(sl, Bytecode::Break, 0);
  }

  SourceLoc sl;
};
struct ContinueStmt {
  ContinueStmt(SourceLoc sl) : sl(sl) {
  }
  void emit(Context&, Bytecodes& bytecodes) const {
    bytecodes.add(sl, Bytecode::Continue, 0);
  }
  SourceLoc sl;
};
struct ReturnStmt {
  ReturnStmt(SourceLoc sl, psl::optional<Expr> expr) : sl(sl), expr(MOVE(expr)) {
  }
  void emit(Context& context, Bytecodes& bytecodes) const;

private:
  SourceLoc sl;
  psl::optional<Expr> expr;
};
struct Stmt : psl::variant<Semicolon, Expr, Declaration, BreakStmt, ContinueStmt, ReturnStmt> {
  using variant::variant;
  void emit(Context& context, Bytecodes& bytecodes) const;
};
struct Block {
  Block(psl::vector<PBlock> elems);
  void emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::vector<PBlock> elems;
};
struct While {
  While(SourceLoc sl, Expr condition, PBlock pblock);
  void emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  Expr condition;
  psl::Box<PBlock> pblock;
};
struct For {
  For(SourceLoc sl, Stmt init, Expr condition, Expr inc, PBlock block);
  void emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  Stmt init;
  Expr condition;
  Expr inc;
  psl::Box<PBlock> block;
};
struct If {
  If(SourceLoc sl, Expr condition, PBlock block);

  SourceLoc sl;
  Expr condition;
  psl::Box<PBlock> block;
};
struct ElseIf {
  ElseIf(SourceLoc sl, Expr condition, PBlock block);

  SourceLoc sl;
  Expr condition;
  psl::Box<PBlock> block;
};
struct Else {
  Else(SourceLoc sl, PBlock block);

  SourceLoc sl;
  psl::Box<PBlock> block;
};
struct IfElseChain {
  IfElseChain(If if_, psl::vector<ElseIf> else_ifs, psl::optional<Else> else_)
      : if_{MOVE(if_)}, else_ifs{MOVE(else_ifs)}, else_{MOVE(else_)} {
  }
  void emit(Context& context, Bytecodes& bytecodes) const;

  If if_;
  psl::vector<ElseIf> else_ifs;
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
                     psl::vector<ParameterDeclaration> params, Block block)
      : sl(sl),
        name(MOVE(name)),
        return_type(MOVE(return_type)),
        params(MOVE(params)),
        block(MOVE(block)) {
  }

  void emit(Context& context, Bytecodes& bytecodes) const;

  SourceLoc sl;
  psl::string name;
  Id return_type;
  psl::vector<ParameterDeclaration> params;
  Block block;
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

  void emit(Context& context, Bytecodes& bytecodes) const;

private:
  SourceLoc sl;
  psl::string name;
  psl::vector<FunctionDefinition> ctors;
  psl::vector<FunctionDefinition> methods;
  psl::vector<MemberDefinition> members;
};
struct PBlock
    : psl::variant<Block, Stmt, While, For, IfElseChain, FunctionDefinition, ClassDefinition> {
  using variant::variant;
  void emit(Context& context, Bytecodes& bytecodes) const {
    return dispatch([&](auto&& x) { x.emit(context, bytecodes); });
  }
};

// ================================================
// ================================================
// ================================================
uint16_t Id::emit(Context& context, Bytecodes& bytecodes) const {
  if (auto vi = bytecodes.var_index_by_name(value); vi != uint16_t(-1))
    return vi;

  if (auto vi = context.find_variable(value); vi != uint16_t(-1)) {
    bytecodes.add_typed(sl, Bytecode::LoadGlobalVar, context.variables[vi].first, vi);
    return bytecodes.top_var_index();
  }

  auto fr = context.find_unique_f(value);
  if (fr.status == fr.None) {
    bytecodes.error(sl, "Variable `", value, "` is not found");
  } else if (fr.status == fr.TooMany) {
    bytecodes.error(sl, "Reference to function `", value, "` is ambiguous, candidates:\n",
                    fr.candidates);
  } else {
    bytecodes.add_typed(sl, Bytecode::LoadFunction,
                        TypeTag(context.functions[fr.function_index].signature(), false),
                        fr.function_index);
    return bytecodes.top_var_index();
  }
}
NumberLiteral::NumberLiteral(const psl::string& str) {
  if ((is_float = psl::find(str, '.') != str.end()))
    valuef = psl::stof(str);
  else
    valuei = psl::stoi(str);
}
uint16_t NumberLiteral::emit(Context& context, Bytecodes& bytecodes) const {
  if (is_float)
    bytecodes.add_typed(sl, Bytecode::LoadFloatConstant, context.tag<float>(),
                        psl::bitcast<uint32_t>(valuef));
  else
    bytecodes.add_typed(sl, Bytecode::LoadIntConstant, context.tag<int>(),
                        psl::bitcast<uint32_t>(valuei));
  return bytecodes.top_var_index();
}
uint16_t BooleanLiteral::emit(Context& context, Bytecodes& bytecodes) const {
  bytecodes.add_typed(sl, Bytecode::LoadBoolConstant, context.tag<bool>(), value);
  return bytecodes.top_var_index();
}
uint16_t StringLiteral::emit(Context& context, Bytecodes& bytecodes) const {
  auto si = bytecodes.add_string(value);
  bytecodes.add_typed(sl, Bytecode::LoadStringConstant, context.tag<psl::string>(), si);
  return bytecodes.top_var_index();
}
uint16_t Vector::emit(Context& context, Bytecodes& bytecodes) const {
  auto arg_indices = psl::vector<uint16_t>(args.size());
  if (args.size() >= 2 && args.size() <= 4) {
    auto atypes = psl::vector<TypeTag>(args.size());
    for (size_t i = 0; i < args.size(); i++) {
      arg_indices[i] = args[i].emit(context, bytecodes);
      atypes[i] = bytecodes.var_type(arg_indices[i]);
    }
    auto float_ = false;
    for (auto id : atypes)
      if (id.name == "f32") {
        float_ = true;
        break;
      }
    auto [fi, rtype, converts] =
        context.find_f("vec" + psl::to_string(args.size()) + (float_ ? "" : "i"), atypes);
    for (auto convert : converts) {
      bytecodes.add_typed(sl, Bytecode::Call, convert.rtype, convert.converter_index,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
    }
    bytecodes.add_typed(sl, Bytecode::Call, rtype, fi, arg_indices);
    return bytecodes.top_var_index();
  } else {
    bytecodes.error(sl, "Only 2, 3, or 4 items should exist inside []");
  }

  return bytecodes.top_var_index();
}
static psl::string extract_return_type(psl::string_view type) {
  auto depth = 0;
  for (size_t i = 0; i < type.size(); i++) {
    if (type[i] == '(')
      depth++;
    if (type[i] == ')')
      depth--;
    if (depth == 0 && type[i] == ':')
      return psl::string(type.substr(i + 2));
  }
  return "";
}
uint16_t FunctionCall::emit(Context& context, Bytecodes& bytecodes) const {
  try {
    if (name != "()")
      if (auto vi = bytecodes.var_index_by_name(name); vi != uint16_t(-1)) {
        if (psl::contains(bytecodes.var_type(vi).name, ':')) {
          auto arg_indices = psl::vector<uint16_t>(args.size());

          for (size_t i = 0; i < args.size(); i++) {
            auto vi = args[i].emit(context, bytecodes);
            arg_indices[i] = vi;
          }
          bytecodes.add_typed(sl, Bytecode::InvokeAsFunction,
                              TypeTag(extract_return_type(bytecodes.var_type(vi).name), false), vi,
                              arg_indices);
          return bytecodes.top_var_index();
        } else {
          // If `x` is a non-function variable, then `x(...)` will be transformed to `()(x, ...)`,
          // i.e. call the operator() with x as the first argument
          auto args_ = args;
          args_.push_front(Id(sl, name));
          return FunctionCall(sl, "()", args_).emit(context, bytecodes);
        }
      }
    if (args.size() > 8)
      bytecodes.error(sl, "Functions can accept no more than 8 arguments, got ", args.size());
    auto arg_indices = psl::vector<uint16_t>(args.size());
    auto atypes = psl::vector<TypeTag>(args.size());

    for (size_t i = 0; i < args.size(); i++) {
      auto vi = args[i].emit(context, bytecodes);
      arg_indices[i] = vi;
      atypes[i] = bytecodes.var_type(vi);
    }

    auto [fi, rtype, converts] = context.find_f(name, atypes);
    for (auto convert : converts) {
      bytecodes.add_typed(sl, Bytecode::Call, convert.rtype, convert.converter_index,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
    }
    bytecodes.add_typed(sl, Bytecode::Call, rtype, fi, arg_indices);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
MemberAccess::MemberAccess(SourceLoc sl, PExpr pexpr, Id id)
    : sl(sl), pexpr(MOVE(pexpr)), id(MOVE(id)) {
}
uint16_t MemberAccess::emit(Context& context, Bytecodes& bytecodes) const {
  auto vi = pexpr->emit(context, bytecodes);
  try {
    auto type = bytecodes.var_type(vi);
    auto fi = context.get_type_trait(type.name).find_member_accessor_index(id.value);
    if (fi == size_t(-1))
      bytecodes.error(sl, "Can't find member `", id.value, "` in type `", type.name, '`');
    bytecodes.add_typed(sl, Bytecode::Call, context.functions[fi].rtype(), fi, vi);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
Subscript::Subscript(SourceLoc sl, PExpr pexpr, Expr index)
    : sl(sl), pexpr(MOVE(pexpr)), index(MOVE(index)) {
}
uint16_t Subscript::emit(Context& context, Bytecodes& bytecodes) const {
  uint16_t arg_indices[]{pexpr->emit(context, bytecodes), index.emit(context, bytecodes)};
  TypeTag atypes[]{bytecodes.var_type(arg_indices[0]), bytecodes.var_type(arg_indices[1])};
  try {
    auto [fi, rtype, converts] = context.find_f("[]", atypes);
    for (const auto& convert : converts) {
      bytecodes.add_typed(sl, Bytecode::Call, convert.rtype, convert.converter_index,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
    }
    bytecodes.add_typed(sl, Bytecode::Call, rtype, fi, arg_indices[0], arg_indices[1]);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
LambdaExpr::LambdaExpr(SourceLoc sl, psl::vector<Id> captures, FunctionDefinition body)
    : sl(sl), captures(MOVE(captures)), body(MOVE(body)) {
}
uint16_t LambdaExpr::emit(Context& context, Bytecodes& bytecodes) const {
  try {
    auto name = psl::string("__lambda") + psl::to_string(context.new_internal_class());
    auto fbcodes = Bytecodes(bytecodes.sl);
    auto rtype = type_tag_from_string(body->return_type.value);
    auto ptypes = psl::vector<TypeTag>();
    auto cptypes = psl::vector<TypeTag>();
    auto args = psl::vector<Expr>();

    for (auto capture : captures) {
      auto is_ref = capture.value.front() == '&';
      if (is_ref)
        capture.value.pop_front();
      auto vi = bytecodes.var_index_by_name(capture.value);
      if (vi == uint16_t(-1))
        bytecodes.error(capture.sl, "Variable `", capture.value, "` is not found");
      auto ptype = bytecodes.var_type(vi);
      ptype.is_ref = is_ref;
      fbcodes.placehold_typed(ptype, capture.value);
      cptypes.push_back(ptype);
      args.push_back(capture);
    }
    for (const auto& param : body->params) {
      auto ptype = type_tag_from_string(param.type.value);
      fbcodes.placehold_typed(ptype, param.name.value);
      ptypes.push_back(ptype);
    }
    context.rtype_stack.push_back(rtype);
    body->block.emit(context, fbcodes);
    context.rtype_stack.pop_back();

    context(name) = Function(
        tag<Function, psl::span<const Variable*>>(
            [&context, fbcodes = MOVE(fbcodes), rtype, cptypes,
             ptypes](psl::span<const Variable*> args) mutable {
              auto vm_ = VirtualMachine();
              vm_.stack.reserve(cptypes.size() + ptypes.size());
              for (size_t i = 0; i < args.size(); i++)
                vm_.stack.push(cptypes[i].is_ref ? args[i]->create_ref() : args[i]->copy());
              return Function(
                  lambda<psl::span<const Variable*>>(
                      [&context, fbcodes = MOVE(fbcodes), vm_ = MOVE(vm_),
                       ptypes](psl::span<const Variable*> args) mutable {
                        static thread_local VirtualMachine vm;
                        if (vm.stack.size() == 0) {
                          for (const auto& var : vm_.stack)
                            vm.stack.push(var);
                        } else {
                          vm.stack.unwind(vm_.stack.size());
                        }
                        for (size_t i = 0; i < args.size(); i++)
                          vm.stack.push(ptypes[i].is_ref ? args[i]->create_ref() : args[i]->copy());
                        return execute(context, fbcodes, vm);
                      }),
                  rtype, ptypes);
            }),
        TypeTag(signature_from(rtype, ptypes)), cptypes);

    return FunctionCall(sl, name, args).emit(context, bytecodes);
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
uint16_t Expr0::emit(Context& context, Bytecodes& bytecodes) const {
  if (op == None)
    return x.emit(context, bytecodes);

  try {
    auto vi = x.emit(context, bytecodes);
    uint16_t arg_indices[]{vi};
    TypeTag atypes[]{bytecodes.var_type(vi)};
    auto fr = Context::FindFResult();
    switch (op) {
      case None: break;
      case PreInc: fr = context.find_f("++x", atypes); break;
      case PreDec: fr = context.find_f("--x", atypes); break;
      case PostInc: fr = context.find_f("x++", atypes); break;
      case PostDec: fr = context.find_f("x--", atypes); break;
      case Positive: fr = context.find_f("+x", atypes); break;
      case Negate: fr = context.find_f("-x", atypes); break;
      case Invert: fr = context.find_f("!x", atypes); break;
    }
    CHECK(fr.function_index != size_t(-1));
    for (auto convert : fr.converts) {
      bytecodes.add_typed(sl, Bytecode::Call, convert.rtype, convert.converter_index,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      atypes[convert.position] = convert.rtype;
    }
    bytecodes.add_typed(sl, Bytecode::Call, fr.rtype, fr.function_index, arg_indices);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
template <typename T>
requires requires(T x) { x.sl; }
Expr::Expr(T x) : Expr(Expr0(x.sl, MOVE(x))) {
}
Expr::Expr(Expr0 x) : sl(x.sl), op{None}, x{MOVE(x)} {
}
Expr::Expr(Expr a, Expr b, Op op) : sl(a.sl), op{op}, a{MOVE(a)}, b{MOVE(b)} {
}
uint16_t Expr::emit(Context& context, Bytecodes& bytecodes) const {
  if (op == None)
    return x->emit(context, bytecodes);

  try {
    auto a_index = a->emit(context, bytecodes);
    auto b_index = b->emit(context, bytecodes);
    uint16_t arg_indices[]{a_index, b_index};
    TypeTag atypes[]{bytecodes.var_type(a_index), bytecodes.var_type(b_index)};
    auto fr = Context::FindFResult();
    switch (op) {
      case None: CHECK(false); break;
      case Mul: fr = context.find_f("*", atypes); break;
      case Div: fr = context.find_f("/", atypes); break;
      case Mod: fr = context.find_f("%", atypes); break;
      case Pow: fr = context.find_f("^", atypes); break;
      case Add: fr = context.find_f("+", atypes); break;
      case Sub: fr = context.find_f("-", atypes); break;
      case Lt: fr = context.find_f("<", atypes); break;
      case Gt: fr = context.find_f(">", atypes); break;
      case Le: fr = context.find_f("<=", atypes); break;
      case Ge: fr = context.find_f(">=", atypes); break;
      case Eq: fr = context.find_f("==", atypes); break;
      case Ne: fr = context.find_f("!=", atypes); break;
      case And: fr = context.find_f("&&", atypes); break;
      case Or: fr = context.find_f("||", atypes); break;
      case AddE: fr = context.find_f("+=", atypes); break;
      case SubE: fr = context.find_f("-=", atypes); break;
      case MulE: fr = context.find_f("*=", atypes); break;
      case DivE: fr = context.find_f("/=", atypes); break;
      case ModE: fr = context.find_f("%=", atypes); break;
      case Assi: fr = context.find_f("=", atypes); break;
    }
    CHECK(fr.function_index != size_t(-1));
    for (auto convert : fr.converts) {
      bytecodes.add_typed(sl, Bytecode::Call, convert.rtype, convert.converter_index,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      atypes[convert.position] = convert.rtype;
    }
    bytecodes.add_typed(sl, Bytecode::Call, fr.rtype, fr.function_index, arg_indices);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
void Declaration::emit(Context& context, Bytecodes& bytecodes) const {
  auto vi = expr.emit(context, bytecodes);
  if (as_ref)
    bytecodes.add_typed(sl, Bytecode::MakeRef, bytecodes.var_type(vi), vi);
  else
    bytecodes.add_typed(sl, Bytecode::Copy, bytecodes.var_type(vi), vi);
  bytecodes.name_top_var(name);
}
void ReturnStmt::emit(Context& context, Bytecodes& bytecodes) const {
  CHECK(context.rtype_stack.size() != 0);
  if (expr) {
    auto vi = expr->emit(context, bytecodes);
    if (bytecodes.var_type(vi).name != context.rtype_stack.back().name)
      bytecodes.error(sl, "Expression type `", bytecodes.var_type(vi),
                      "` is inconsistent with function return type `", context.rtype_stack.back(),
                      "`");
    bytecodes.add(sl, Bytecode::Return, vi);
  } else {
    if (TypeTag("void") != context.rtype_stack.back())
      bytecodes.error(sl, "Expression type `void`", " is inconsistent with function return type `",
                      context.rtype_stack.back(), "`");
    bytecodes.add(sl, Bytecode::Return, size_t(-1));
  }
}
void Stmt::emit(Context& context, Bytecodes& bytecodes) const {
  dispatch([&](auto&& x) { x.emit(context, bytecodes); });
}
Block::Block(psl::vector<PBlock> elems) : elems{MOVE(elems)} {
}
void Block::emit(Context& ctx, Bytecodes& bytecodes) const {
  auto sp = bytecodes.stack_size();
  bytecodes.enter_scope();
  for (const auto& pblock : elems)
    pblock.emit(ctx, bytecodes);
  bytecodes.add(sl, Bytecode::UnwindStack, sp);
  bytecodes.exit_scope();
}
While::While(SourceLoc sl, Expr condition, PBlock pblock)
    : sl(sl), condition(MOVE(condition)), pblock(MOVE(pblock)) {
}
void While::emit(Context& context, Bytecodes& bytecodes) const {
  auto sp = bytecodes.stack_size();
  bytecodes.enter_scope();
  auto condition_begin = bytecodes.code_position();
  auto condition_vi = condition.emit(context, bytecodes);
  if (bytecodes.var_type(condition_vi) != context.tag<bool>())
    bytecodes.error(sl, "Expect boolean expression");
  auto ci = bytecodes.add(sl, Bytecode::JumpIfNot, 0 /*placeholder*/, condition_vi);
  pblock->emit(context, bytecodes);
  bytecodes.add(sl, Bytecode::UnwindStack, sp);
  bytecodes.add(sl, Bytecode::Jump, condition_begin);
  auto while_end = bytecodes.code_position();
  bytecodes[ci].value = while_end;
  bytecodes.add(sl, Bytecode::UnwindStack, sp);
  bytecodes.exit_scope();

  for (size_t i = condition_begin; i < bytecodes.length(); i++) {
    auto& code = bytecodes[i];
    if (code.instruction == Bytecode::Break) {
      code.instruction = Bytecode::Jump;
      code.value = while_end;
    } else if (code.instruction == Bytecode::Continue) {
      code.instruction = Bytecode::Jump;
      code.value = condition_begin;
    }
  }
}
For::For(SourceLoc sl, Stmt init, Expr condition, Expr inc, PBlock block)
    : sl(sl),
      init{MOVE(init)},
      condition{MOVE(condition)},
      inc{MOVE(inc)},
      block{MOVE(block)} {
}
void For::emit(Context& context, Bytecodes& bytecodes) const {
  bytecodes.enter_scope();
  auto sp_init = bytecodes.stack_size();
  init.emit(context, bytecodes);
  auto sp = bytecodes.stack_size();
  auto condition_begin = bytecodes.code_position();
  auto condition_vi = condition.emit(context, bytecodes);
  if (bytecodes.var_type(condition_vi) != context.tag<bool>())
    bytecodes.error(sl, "Expect boolean expression");
  auto ci = bytecodes.add(sl, Bytecode::JumpIfNot, 0 /*placeholder*/, condition_vi);
  block->emit(context, bytecodes);
  inc.emit(context, bytecodes);
  bytecodes[bytecodes.code_position() - 1].value1 = size_t(-1);
  bytecodes.pop_stack();

  bytecodes.add(sl, Bytecode::UnwindStack, sp);
  bytecodes.add(sl, Bytecode::Jump, condition_begin);
  auto for_end = bytecodes.code_position();
  bytecodes[ci].value = for_end;
  bytecodes.add(sl, Bytecode::UnwindStack, sp_init);
  bytecodes.exit_scope();

  for (size_t i = condition_begin; i < bytecodes.length(); i++) {
    auto& code = bytecodes[i];
    if (code.instruction == Bytecode::Break) {
      code.instruction = Bytecode::Jump;
      code.value = for_end;
    } else if (code.instruction == Bytecode::Continue) {
      code.instruction = Bytecode::Jump;
      code.value = condition_begin;
    }
  }
}
If::If(SourceLoc sl, Expr condition, PBlock block)
    : sl(sl), condition{MOVE(condition)}, block{MOVE(block)} {
}
ElseIf::ElseIf(SourceLoc sl, Expr condition, PBlock block)
    : sl(sl), condition{MOVE(condition)}, block{MOVE(block)} {
}
Else::Else(SourceLoc sl, PBlock block) : sl(sl), block{MOVE(block)} {
}
void IfElseChain::emit(Context& context, Bytecodes& bytecodes) const {
  auto if_ci_out = size_t(-1);
  auto sp = bytecodes.stack_size();
  bytecodes.enter_scope();
  {
    auto vi = if_.condition.emit(context, bytecodes);
    if (bytecodes.var_type(vi) != context.tag<bool>())
      bytecodes.error(if_.sl, "Expect boolean expression");
    auto ci_cond = bytecodes.add(if_.sl, Bytecode::JumpIfNot, 0 /*placeholder*/, vi);
    bytecodes.enter_scope();
    if_.block->emit(context, bytecodes);
    bytecodes.exit_scope();
    if_ci_out = bytecodes.add(if_.sl, Bytecode::Jump, 0 /*placeholder*/);
    bytecodes[ci_cond].value = bytecodes.code_position();
  }

  auto else_if_ci_outs = psl::vector<size_t>();
  else_if_ci_outs.reserve(else_ifs.size());
  for (const auto& else_if : else_ifs) {
    auto vi = else_if.condition.emit(context, bytecodes);
    if (bytecodes.var_type(vi) != context.tag<bool>())
      bytecodes.error(else_if.sl, "Expect boolean expression");
    auto ci_cond = bytecodes.add(else_if.sl, Bytecode::JumpIfNot, 0 /*placeholder*/, vi);
    bytecodes.enter_scope();
    else_if.block->emit(context, bytecodes);
    bytecodes.exit_scope();
    else_if_ci_outs.push_back(bytecodes.add(else_if.sl, Bytecode::Jump, 0 /*placeholder*/));
    bytecodes[ci_cond].value = bytecodes.code_position();
  }
  if (else_) {
    bytecodes.enter_scope();
    else_->block->emit(context, bytecodes);
    bytecodes.exit_scope();
  }
  bytecodes[if_ci_out].value = bytecodes.code_position();
  for (auto else_if_ci_out : else_if_ci_outs)
    bytecodes[else_if_ci_out].value = bytecodes.code_position();
  bytecodes.add(if_.sl, Bytecode::UnwindStack, sp);
  bytecodes.exit_scope();
}
void FunctionDefinition::emit(Context& context, Bytecodes& bytecodes) const {
  try {
    auto fbcodes = Bytecodes(bytecodes.sl);
    auto rtype = type_tag_from_string(return_type.value);
    auto ptypes = psl::vector<TypeTag>();
    for (const auto& param : this->params) {
      auto ptype = type_tag_from_string(param.type.value);
      fbcodes.placehold_typed(ptype, param.name.value);
      ptypes.push_back(ptype);
    }
    context.rtype_stack.push_back(rtype);
    block.emit(context, fbcodes);
    context.rtype_stack.pop_back();

    // clang-format off
    context(name) = Function(tag<Variable, psl::span<const Variable*>>(
                                 [&context, fbcodes = MOVE(fbcodes), ptypes]
                                 (psl::span<const Variable*> args) mutable {
      DCHECK_EQ(args.size(), ptypes.size());
      static thread_local VirtualMachine vm;
      vm.stack.reserve(args.size() * 2);
      vm.stack.unwind(0);
      for (size_t i = 0; i < args.size(); i++) {
          vm.stack.push(ptypes[i].is_ref ? args[i]->create_ref() : args[i]->copy()) ;
      }
      return execute(context, fbcodes, vm);
    }), rtype, ptypes);
    // clang-format on
  } catch (const Exception& e) {
    bytecodes.error(sl, e.what());
  }
}
void ClassDefinition::emit(Context& context, Bytecodes& bytecodes) const {
  // TODO
  auto& trait = context.types[name] = Context::TypeTrait(name, 1);
  context("__" + name) =
      Function(tag<InternalClass, psl::span<const Variable*>>(
                   [n = members.size()](psl::span<const Variable*>) {
                     return InternalClass{psl::vector_n_of(n, Variable(psl::Any()))};
                   }),
               TypeTag(name, false), context.tags<>());

  for (size_t i = 0; i < members.size(); i++) {
    if (!context.is_registered_type(members[i].type.value))
      bytecodes.error(members[i].type.sl, "Type `", members[i].type.value, "` is not found");
    trait.add_member_accessor(members[i].name.value, context.functions.size());
    context.functions.push_back(
        Function(tag<Variable, psl::span<const Variable*>>([i](psl::span<const Variable*> args) {
                   DCHECK_GT(args[0]->as<InternalClass&>().members.size(), i);
                   return args[0]->as<InternalClass&>().members[i].create_ref();
                 }),
                 TypeTag(members[i].type.value, true), psl::vector_of(TypeTag(name, true))));
  }

  for (size_t i = 0; i < members.size(); i++) {
    trait.add_member_accessor("__" + members[i].name.value, context.functions.size());
    context.functions.push_back(
        Function(tag<Variable, psl::span<const Variable*>>([i](psl::span<const Variable*> args) {
                   DCHECK_GT(args[0]->as<InternalClass&>().members.size(), i);
                   return psl::ref(args[0]->as<InternalClass&>().members[i]);
                 }),
                 context.tag<Variable&>(), psl::vector_of(TypeTag(name, true))));
  }

  for (const auto& ctor : ctors)
    ctor.emit(context, bytecodes);
  for (const auto& method : methods)
    method.emit(context, bytecodes);

  context.get_type_trait<psl::string>().add_from_converter(name, context.functions.size());
  auto to_str = [&context, *this](const InternalClass& x) {
    CHECK_EQ(x.members.size(), members.size());
    auto r = psl::string("{");
    for (size_t i = 0; i < members.size(); i++) {
      auto str = context
                     .call("str", psl::vector_of(TypeTag(members[i].type.value, false)),
                           psl::vector_of(x.members[i]))
                     .as<psl::string>();
      r += members[i].name.value + ": " + str + ", ";
    }
    if (members.size())
      r.pop_back(2);
    return r + "}";
  };
  context("str") = Function(lambda<const InternalClass&>(to_str), context.tag<psl::string>(),
                            psl::vector_of(TypeTag(name, false)));
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

    auto pblocks = psl::vector<PBlock>{};
    while (!expect("}") && next())
      pblocks.push_back(pblock());

    if (top_level)
      accept("}");
    else
      consume("}", "to end block");
    return Block{pblocks};
  }

  PBlock pblock() {
    if (expect("{"))
      return PBlock{block()};
    else if (expect("while"))
      return PBlock{while_()};
    else if (expect("for"))
      return PBlock{for_()};
    else if (expect("if"))
      return PBlock{if_else_chain()};
    else if (expect("fn"))
      return PBlock{function_definition()};
    else if (expect("class"))
      return PBlock{class_definition()};
    else
      return PBlock{stmt()};
  }
  While while_() {
    consume("while");
    consume("(", "to begin condition");
    auto loc = source_loc();
    auto cond = expr();
    consume(")", "to end condition");
    auto body = pblock();
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
      auto body = pblock();
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
      auto body = pblock();
      return For{loc, MOVE(init), MOVE(cond), Expr(Expr0(loc, PExpr(id_), Expr0::PreInc)),
                 MOVE(body)};
    }
  }
  IfElseChain if_else_chain() {
    auto if_clause = if_();
    auto else_ifs = psl::vector<ElseIf>{};
    while (true) {
      backup();
      if (accept("else")) {
        if (expect("if")) {
          undo();
          else_ifs.push_back(else_if_());
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
    return IfElseChain{MOVE(if_clause), MOVE(else_ifs), MOVE(else_clause)};
  }
  If if_() {
    consume("if");
    consume("(", "to begin condition");
    auto loc = source_loc();
    auto cond = expr();
    consume(")", "to end condition");
    auto body = pblock();
    return If{loc, MOVE(cond), MOVE(body)};
  }
  ElseIf else_if_() {
    consume("else");
    consume("if");
    consume("(", "to begin condition");
    auto loc = source_loc();
    auto cond = expr();
    consume(")", "to end condition");
    auto body = pblock();
    return ElseIf{loc, MOVE(cond), MOVE(body)};
  }
  Else else_() {
    consume("else");
    auto loc = source_loc();
    return Else{loc, pblock()};
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
        CHECK_GE(ctor.block.elems.size(), 1);
        auto it = psl::next(ctor.block.elems.begin(), init_size);
        it = ctor.block.elems.insert(
            it, Stmt(Declaration(member.name.sl, member.name.value,
                                 MemberAccess(loc, Id(loc, "self"), member.name), true)));
      }
    for (auto& method : methods)
      for (const auto& member : members) {
        auto loc = method.sl;
        method.block.elems.push_front(
            Stmt(Declaration(member.name.sl, member.name.value,
                             MemberAccess(loc, Id(loc, "self"), member.name), true)));
      }
    return ClassDefinition(loc, MOVE(name), MOVE(ctors), MOVE(methods),
                           MOVE(members));
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
    return {FunctionDefinition(loc, ctor_name, Id(loc, class_name), MOVE(params),
                               MOVE(block_)),
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
    return FunctionDefinition(loc, MOVE(name), MOVE(type), MOVE(params),
                              MOVE(block_));
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
    return FunctionDefinition(loc, MOVE(name), MOVE(return_type), MOVE(params),
                              MOVE(block_));
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
      exprs.insert(exprs.begin() + index,
                   Expr{MOVE(a), MOVE(b), Expr::Op(max_precedence)});
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

Bytecodes compile(Context& context, psl::string source) {
  auto [block, sl] = parse_as_block(MOVE(source));
  auto bytecodes = Bytecodes(MOVE(sl));
  block.emit(context, bytecodes);
  return bytecodes;
}
void compile(Context& context, psl::string source, Bytecodes& bytecodes) {
  auto [block, sl] = parse_as_block(MOVE(source));
  bytecodes.sl = MOVE(sl);
  for (const auto& elem : block.elems)
    elem.emit(context, bytecodes);
}

Variable execute(const Context& context, const Bytecodes& bytecodes, VirtualMachine& vm) {
  auto source_loc = SourceLoc();
  try {
    for (size_t p = 0; p < bytecodes.length();) {
      const auto& code = bytecodes[p];
      source_loc = code.sl;
      auto inc_p = true;
      const auto push = [&](auto x) {
        if (code.value1 != size_t(-1))
          vm.stack.push(MOVE(x));
      };

      switch (code.instruction) {
        case Bytecode::Break: break;
        case Bytecode::Continue: break;
        case Bytecode::Return:
          return code.value == size_t(-1) ? Variable() : vm.stack[code.value].copy();
          break;
        case Bytecode::LoadGlobalVar: vm.stack.push(context.variables[code.value].second); break;
        case Bytecode::LoadFunction: vm.stack.push(context.functions[code.value]); break;
        case Bytecode::Copy: push(vm.stack[code.value].copy()); break;
        case Bytecode::MakeRef: push(vm.stack[code.value].create_ref()); break;
        case Bytecode::LoadFloatConstant: push(psl::bitcast<float>(uint32_t(code.value))); break;
        case Bytecode::LoadIntConstant: push(psl::bitcast<int>(uint32_t(code.value))); break;
        case Bytecode::LoadBoolConstant: push(bool(code.value)); break;
        case Bytecode::LoadStringConstant: push(bytecodes.get_string(code.value)); break;
        case Bytecode::Call: {
          auto f = [&]<int... I>(psl::IntegerSequence<int, I...>) {
            const Variable* arr[]{&vm.stack[code.args[I]]...};
            push(context.functions[code.value].call(arr));
          };
          switch (code.nargs) {
            case 0: push(context.functions[code.value].call({})); break;
            case 1: f(psl::make_integer_sequence<int, 1>()); break;
            case 2: f(psl::make_integer_sequence<int, 2>()); break;
            case 3: f(psl::make_integer_sequence<int, 3>()); break;
            case 4: f(psl::make_integer_sequence<int, 4>()); break;
            case 5: f(psl::make_integer_sequence<int, 5>()); break;
            case 6: f(psl::make_integer_sequence<int, 6>()); break;
            case 7: f(psl::make_integer_sequence<int, 7>()); break;
            case 8: f(psl::make_integer_sequence<int, 8>()); break;
            case 9: CHECK(false); break;
          }
        } break;
        case Bytecode::InvokeAsFunction: {
          auto f = [&]<int... I>(psl::IntegerSequence<int, I...>) {
            const Variable* arr[]{&vm.stack[code.args[I]]...};
            push(vm.stack[code.value].as<const Function&>().call(arr));
          };
          switch (code.nargs) {
            case 0: push(vm.stack[code.value].as<const Function&>().call({})); break;
            case 1: f(psl::make_integer_sequence<int, 1>()); break;
            case 2: f(psl::make_integer_sequence<int, 2>()); break;
            case 3: f(psl::make_integer_sequence<int, 3>()); break;
            case 4: f(psl::make_integer_sequence<int, 4>()); break;
            case 5: f(psl::make_integer_sequence<int, 5>()); break;
            case 6: f(psl::make_integer_sequence<int, 6>()); break;
            case 7: f(psl::make_integer_sequence<int, 7>()); break;
            case 8: f(psl::make_integer_sequence<int, 8>()); break;
            case 9: CHECK(false); break;
          }
        } break;
        case Bytecode::Jump:
          p = code.value;
          inc_p = false;
          break;
        case Bytecode::JumpIfNot:
          if (!vm.stack[code.value1].as<bool>()) {
            p = code.value;
            inc_p = false;
          }
          break;
        case Bytecode::UnwindStack: vm.stack.unwind(code.value); break;
      }

      if (inc_p)
        p++;
    }
  } catch (const Exception& e) {
    bytecodes.error(source_loc, e.what());
  }

  return Variable();
}

struct MemoryPool {
  MemoryPool(size_t pool_byte_size) : pool_byte_size(pool_byte_size) {
  }

  struct Pool {
    Pool(size_t size) {
      bytes.reserve(size);
    }
    Pool(const Pool&) = delete;
    Pool& operator=(const Pool&) = delete;
    Pool(Pool&&) = default;
    Pool& operator=(Pool&&) = default;

    void* take(size_t size) {
      auto ptr = &bytes[occupied];
      occupied += size;
      return ptr;
    }

    size_t free_space() const {
      return bytes.capacity() - occupied;
    }

  private:
    psl::vector<uint8_t> bytes;
    size_t occupied = 0;
  };

  void* alloc(size_t size) {
    prepare(size);
    return pool().take(size);
  }
  void free(void*) {
  }

private:
  void prepare(size_t size) {
    if (pools.size() == 0) {
      pools.push_back(Pool(psl::max(pool_byte_size, size)));
    } else {
      if (size > pool().free_space())
        pools.push_back(Pool(psl::max(pool_byte_size, size)));
    }
  }
  Pool& pool() {
    return pools.back();
  }

  psl::vector<Pool> pools;
  size_t pool_byte_size;
};

Variable execute(const Context& context, const Bytecodes& bytecodes) {
  Profiler _("[Interpreter]Execute");
  //   Debug(bytecodes.to_string(context));

  auto vm = VirtualMachine();
  return execute(context, bytecodes, vm);
}

}  // namespace pine