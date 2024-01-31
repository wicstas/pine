#include <pine/core/compiler.h>
#include <pine/core/profiler.h>

#include <psl/algorithm.h>
#include <psl/variant.h>

namespace pine {
struct PExpr;
struct Expr0;
struct Block;
struct PBlock;
struct FunctionDefinition;
}  // namespace pine

namespace psl {
extern template struct Box<pine::PExpr>;
extern template struct Box<pine::Expr0>;
extern template struct Box<pine::Block>;
extern template struct Box<pine::FunctionDefinition>;
}  // namespace psl

namespace pine {

template <typename T>
struct ExplicitParameter {
  explicit ExplicitParameter(T value) : value(psl::move(value)) {
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
    parts.push_back(psl::string{start, end});
    if (end == input.end())
      break;
    else
      start = psl::next(end);
  }

  return parts;
}

SourceLines::SourceLines(psl::string_view tokens, size_t lines_padding)
    : lines_padding{lines_padding} {
  lines = split(tokens, [](char c) { return c == '\n' || c == '\r' || c == '\f'; });
  for (size_t i = 0; i < lines_padding; i++)
    lines.insert(lines.begin(), "");
  for (size_t i = 0; i < lines_padding; i++)
    lines.push_back("");
}

psl::optional<psl::string_view> SourceLines::next_line(size_t row) const {
  DCHECK_LE(row + lines_padding, lines.size());
  if (row + lines_padding == lines.size())
    return psl::nullopt;
  return lines[row];
}

psl::optional<char> SourceLines::next(size_t row, size_t column) const {
  if (auto line = next_line(row)) {
    DCHECK_LT(column, line->size());
    return (*line)[column];
  }
  return psl::nullopt;
}

[[noreturn]] void SourceLines::error_impl(size_t row, size_t column,
                                          psl::string_view message) const {
  DCHECK(lines_padding != invalid);
  auto vicinity = psl::string();
  for (size_t i = row - lines_padding; i <= row; i++)
    vicinity += " | " + lines[i] + "\n";
  vicinity += "  -" + psl::string_n_of(column, '-') + "^\n";
  for (size_t i = row + 1; i <= row + lines_padding; i++)
    vicinity += " | " + lines[i] + "\n";

  if (vicinity.size())
    vicinity.pop_back();

  Fatal(message, "\n", vicinity);
}

Bytecodes::Bytecodes(const Context& context, SourceLines sl) : sl(psl::move(sl)) {
  for (const auto& var : context.variables)
    stack.push_back(var.type_id());
  for (const auto& var : context.variables_map)
    variable_map.push_back({var.first, var.second});
}

size_t Bytecodes::add(Bytecode::Instruction instruction, size_t value0, size_t value1) {
  codes.push_back({instruction, value0, value1});
  return codes.size() - 1;
}
void Bytecodes::add_typed(psl::string name, size_t type_id) {
  stack.push_back(type_id);
  name_top_var(psl::move(name));
}
void Bytecodes::add_typed(Bytecode::Instruction instruction, size_t value, size_t type_id,
                          psl::span<uint16_t> args) {
  codes.push_back({instruction, value, type_id, args});
  stack.push_back(type_id);
}

void Bytecodes::name_top_var(psl::string name) {
  CHECK(stack.size() != 0);
  variable_map.push_back({psl::move(name), stack.size() - 1});
}
size_t Bytecodes::var_type(size_t var_index) const {
  CHECK_LE(var_index, top_var_index());
  return stack[var_index];
}
uint16_t Bytecodes::var_index_by_name(psl::string_view name) const {
  using psl::pipe::operator|;
  if (auto it = psl::find_if(variable_map | psl::reverse_(),
                             psl::composite(psl::equal_to(name), psl::first_of_pair));
      it.unwrap() != variable_map.end())
    return it->second;
  else
    return uint16_t(-1);
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
  string_region.push_back(psl::move(value));
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
  stack.erase(stack.begin() + scope_stack.back().type_stack_position, stack.end());
  variable_map.erase(variable_map.begin() + scope_stack.back().variable_map_position,
                     variable_map.end());
  scope_stack.pop_back();
}

struct Expr : ASTNode {
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
    Init = 0000100110,
    // clang-format on
  } op;

  Expr() = default;
  template <psl::DerivedFrom<ASTNode> T>
  Expr(T x);
  Expr(Expr0 x);
  Expr(Expr a, Expr b, Op op);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  psl::Box<Expr0> x;
  psl::Box<Expr> a;
  psl::Box<Expr> b;
};

struct Id : ASTNode {
  Id(size_t row, size_t column, psl::string value) : ASTNode{row, column}, value{psl::move(value)} {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  psl::string value;
};
struct NumberLiteral {
  NumberLiteral(const psl::string& str);
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

private:
  bool is_float = false;
  float valuef;
  int valuei;
};
struct BooleanLiteral {
  BooleanLiteral(bool value) : value(value) {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

private:
  bool value;
};
struct StringLiteral {
  StringLiteral(psl::string value) : value(psl::move(value)) {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

private:
  psl::string value;
};
struct Vector : ASTNode {
  Vector(size_t row, size_t column, psl::vector<Expr> args)
      : ASTNode{row, column}, args{psl::move(args)} {
  }
  uint16_t emit(Context&, Bytecodes& bytecodes) const;

  psl::vector<Expr> args;
};
struct FunctionCall : ASTNode {
  FunctionCall(size_t row, size_t column, psl::string name, psl::vector<Expr> args)
      : ASTNode{row, column}, name{psl::move(name)}, args{psl::move(args)} {
  }
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  psl::string name;
  psl::vector<Expr> args;
};
struct MemberAccess : ASTNode {
  MemberAccess(size_t row, size_t column, PExpr pexpr, Id id);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  psl::Box<PExpr> pexpr;
  Id id;
};
struct Subscript : ASTNode {
  Subscript(size_t row, size_t column, PExpr pexpr, Expr index);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  psl::Box<PExpr> pexpr;
  Expr index;
};
struct LambdaExpr : ASTNode {
  LambdaExpr(size_t row, size_t column, psl::vector<Id> captures, FunctionDefinition body);
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  psl::vector<Id> captures;
  psl::Box<FunctionDefinition> body;
};
using Grouped = Expr;
struct PExpr : psl::Variant<Id, NumberLiteral, BooleanLiteral, StringLiteral, Vector, FunctionCall,
                            MemberAccess, Subscript, Grouped, LambdaExpr> {
  using Variant::Variant;
  uint16_t emit(Context& context, Bytecodes& bytecodes) const {
    return dispatch([&](auto&& x) { return x.emit(context, bytecodes); });
  }
};

struct Expr0 : ASTNode {
  enum Op { None, PreInc, PreDec, PostInc, PostDec, Positive, Negate, Invert };
  Expr0(size_t row, size_t column, PExpr x, Op op = None)
      : ASTNode{row, column}, x{psl::move(x)}, op{op} {};
  uint16_t emit(Context& context, Bytecodes& bytecodes) const;

  PExpr x;
  Op op;
};
struct Declaration {
  Declaration(psl::string name, Expr expr, bool as_ref = false)
      : name{psl::move(name)}, expr{psl::move(expr)}, as_ref(as_ref) {
  }
  void emit(Context& context, Bytecodes& bytecodes) const;

  psl::string name;
  Expr expr;
  bool as_ref = false;
};
struct Semicolon {
  void emit(Context&, Bytecodes&) const {
  }
};
struct BreakStmt {
  void emit(Context&, Bytecodes& bytecodes) const {
    bytecodes.add(Bytecode::Break, 0);
  }
};
struct ContinueStmt {
  void emit(Context&, Bytecodes& bytecodes) const {
    bytecodes.add(Bytecode::Continue, 0);
  }
};
struct ReturnStmt {
  ReturnStmt(Expr expr) : expr(psl::move(expr)) {
  }
  void emit(Context& context, Bytecodes& bytecodes) const {
    auto vi = expr.emit(context, bytecodes);
    bytecodes.add(Bytecode::Return, vi, bytecodes.var_type(vi));
  }

  Expr expr;
};
struct Stmt : psl::Variant<Semicolon, Expr, Declaration, BreakStmt, ContinueStmt, ReturnStmt> {
  using Variant::Variant;
  void emit(Context& context, Bytecodes& bytecodes) const;
};
struct Block {
  Block(psl::vector<PBlock> elems);
  void emit(Context& context, Bytecodes& bytecodes) const;

  psl::vector<PBlock> elems;
};
struct While : ASTNode {
  While(size_t row, size_t column, Expr condition, PBlock pblock);
  void emit(Context& context, Bytecodes& bytecodes) const;

  Expr condition;
  psl::Box<PBlock> pblock;
};
struct For : ASTNode {
  For(size_t row, size_t column, Stmt init, Expr condition, Expr inc, PBlock block);
  void emit(Context& context, Bytecodes& bytecodes) const;
  Stmt init;
  Expr condition;
  Expr inc;
  psl::Box<PBlock> block;
};
struct If : ASTNode {
  If(size_t row, size_t column, Expr condition, PBlock block);
  Expr condition;
  psl::Box<PBlock> block;
};
struct ElseIf : ASTNode {
  ElseIf(size_t row, size_t column, Expr condition, PBlock block);
  Expr condition;
  psl::Box<PBlock> block;
};
struct Else {
  Else(PBlock block);
  psl::Box<PBlock> block;
};
struct IfElseChain {
  IfElseChain(If if_, psl::vector<ElseIf> else_ifs, psl::optional<Else> else_)
      : if_{psl::move(if_)}, else_ifs{psl::move(else_ifs)}, else_{psl::move(else_)} {
  }
  void emit(Context& context, Bytecodes& bytecodes) const;

  If if_;
  psl::vector<ElseIf> else_ifs;
  psl::optional<Else> else_;
};
struct ParameterDeclaration {
  psl::string name;
  psl::string type;
};
struct FunctionDefinition : ASTNode {
  FunctionDefinition(size_t row, size_t column, psl::string name, psl::string return_type,
                     psl::vector<ParameterDeclaration> params, Block block)
      : ASTNode(row, column),
        name(psl::move(name)),
        return_type(psl::move(return_type)),
        params(psl::move(params)),
        block(psl::move(block)) {
  }

  void emit(Context& context, Bytecodes& bytecodes) const;

  psl::string name;
  psl::string return_type;
  psl::vector<ParameterDeclaration> params;
  Block block;
};
struct MemberDefinition : ASTNode {
  MemberDefinition(size_t row, size_t column, psl::string name, psl::string type)
      : ASTNode(row, column), name(psl::move(name)), type(psl::move(type)) {
  }

  psl::string name;
  psl::string type;
};
struct InternalClass {
  psl::vector<Variable> members;
};
struct ClassDefinition : ASTNode {
  ClassDefinition(size_t row, size_t column, psl::string name,
                  psl::vector<FunctionDefinition> ctors, psl::vector<FunctionDefinition> methods,
                  psl::vector<MemberDefinition> members)
      : ASTNode(row, column),
        name(psl::move(name)),
        ctors(psl::move(ctors)),
        methods(psl::move(methods)),
        members(psl::move(members)) {
  }

  size_t emit(Context& context, Bytecodes& bytecodes) const;

private:
  psl::string name;
  psl::vector<FunctionDefinition> ctors;
  psl::vector<FunctionDefinition> methods;
  psl::vector<MemberDefinition> members;
};
struct PBlock
    : psl::Variant<Block, Stmt, While, For, IfElseChain, FunctionDefinition, ClassDefinition> {
  using Variant::Variant;
  void emit(Context& context, Bytecodes& bytecodes) const {
    return dispatch([&](auto&& x) { x.emit(context, bytecodes); });
  }
};
}  // namespace pine

namespace psl {
template struct Box<pine::PExpr>;
template struct Box<pine::Expr0>;
template struct Box<pine::Block>;
template struct Box<pine::FunctionDefinition>;
}  // namespace psl

namespace pine {
uint16_t Id::emit(Context&, Bytecodes& bytecodes) const {
  auto var_index = bytecodes.var_index_by_name(value);
  if (var_index == uint16_t(-1))
    bytecodes.error(*this, "Variable `", value, "` is not found");
  return var_index;
}
NumberLiteral::NumberLiteral(const psl::string& str) {
  if ((is_float = psl::find(str, '.') != str.end()))
    valuef = psl::stof(str);
  else
    valuei = psl::stoi(str);
}
uint16_t NumberLiteral::emit(Context&, Bytecodes& bytecodes) const {
  if (is_float)
    bytecodes.add_typed(Bytecode::LoadFloatConstant, psl::bitcast<uint32_t>(valuef),
                        psl::type_id<float>());
  else
    bytecodes.add_typed(Bytecode::LoadIntConstant, psl::bitcast<uint32_t>(valuei),
                        psl::type_id<int>());
  return bytecodes.top_var_index();
}
uint16_t BooleanLiteral::emit(Context&, Bytecodes& bytecodes) const {
  bytecodes.add_typed(Bytecode::LoadBoolConstant, value, psl::type_id<bool>());
  return bytecodes.top_var_index();
}
uint16_t StringLiteral::emit(Context&, Bytecodes& bytecodes) const {
  auto p = bytecodes.add_string(value);
  bytecodes.add_typed(Bytecode::LoadStringConstant, p, psl::type_id<psl::string>());
  return bytecodes.top_var_index();
}
uint16_t Vector::emit(Context& context, Bytecodes& bytecodes) const {
  auto arg_indices = psl::vector<uint16_t>(args.size());
  if (args.size() >= 2 && args.size() <= 4) {
    auto arg_type_ids = psl::vector<size_t>(args.size());
    for (size_t i = 0; i < args.size(); i++) {
      arg_indices[i] = args[i].emit(context, bytecodes);
      arg_type_ids[i] = bytecodes.var_type(arg_indices[i]);
    }
    auto float_ = false;
    for (auto id : arg_type_ids)
      if (id == psl::type_id<float>()) {
        float_ = true;
        break;
      }
    auto [fi, rtid, converts] =
        context.find_f("vec" + psl::to_string(args.size()) + (float_ ? "" : "i"), arg_type_ids);
    for (auto convert : converts) {
      bytecodes.add_typed(Bytecode::Call, convert.converter_id, convert.to_type_id,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      arg_type_ids[convert.position] = convert.to_type_id;
    }
    bytecodes.add_typed(Bytecode::Call, fi, rtid, arg_indices);
    return bytecodes.top_var_index();
  } else {
    bytecodes.error(*this, "Only 2, 3, or 4 items should exist inside []");
  }

  return bytecodes.top_var_index();
}
uint16_t FunctionCall::emit(Context& context, Bytecodes& bytecodes) const {
  try {
    CHECK(name != "()");
    if (auto vi = bytecodes.var_index_by_name(name); vi != uint16_t(-1)) {
      auto args_ = args;
      args_.push_front(Id(row, column, name));
      return FunctionCall(row, column, "()", args_).emit(context, bytecodes);
    }
    if (args.size() > 8)
      bytecodes.error(*this, "Functions can accept no more than 8 arguments, got ", args.size());
    auto arg_indices = psl::vector<uint16_t>(args.size());
    auto arg_type_ids = psl::vector<size_t>(args.size());

    for (size_t i = 0; i < args.size(); i++) {
      auto vi = args[i].emit(context, bytecodes);
      arg_indices[i] = vi;
      arg_type_ids[i] = bytecodes.var_type(vi);
    }

    auto [fi, rtid, converts] = context.find_f(name, arg_type_ids);
    for (auto convert : converts) {
      bytecodes.add_typed(Bytecode::Call, convert.converter_id, convert.to_type_id,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      arg_type_ids[convert.position] = convert.to_type_id;
    }
    bytecodes.add_typed(Bytecode::Call, fi, rtid, arg_indices);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(*this, e.what());
  }
}
MemberAccess::MemberAccess(size_t row, size_t column, PExpr pexpr, Id id)
    : ASTNode(row, column), pexpr(psl::move(pexpr)), id(psl::move(id)) {
}
uint16_t MemberAccess::emit(Context& context, Bytecodes& bytecodes) const {
  auto vi = pexpr->emit(context, bytecodes);
  try {
    auto tid = bytecodes.var_type(vi);
    auto fi = context.find_type(tid).find_member_accessor_index(id.value);
    if (fi == size_t(-1))
      bytecodes.error(*this, "Can't find member `", id.value, "` in type `",
                      context.find_type(tid).alias, '`');
    bytecodes.add_typed(Bytecode::Call, fi, context.functions[fi].return_type_id(), vi);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(*this, e.what());
  }
}
Subscript::Subscript(size_t row, size_t column, PExpr pexpr, Expr index)
    : ASTNode(row, column), pexpr(psl::move(pexpr)), index(psl::move(index)) {
}
uint16_t Subscript::emit(Context& context, Bytecodes& bytecodes) const {
  uint16_t arg_indices[]{pexpr->emit(context, bytecodes), index.emit(context, bytecodes)};
  size_t arg_type_ids[]{bytecodes.var_type(arg_indices[0]), bytecodes.var_type(arg_indices[1])};
  try {
    auto [fi, rtid, converts] = context.find_f("[]", arg_type_ids);
    for (const auto& convert : converts) {
      bytecodes.add_typed(Bytecode::Call, convert.converter_id, convert.to_type_id,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      arg_type_ids[convert.position] = convert.to_type_id;
    }
    bytecodes.add_typed(Bytecode::Call, fi, rtid, arg_indices[0], arg_indices[1]);
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(*this, e.what());
  }
}
LambdaExpr::LambdaExpr(size_t row, size_t column, psl::vector<Id> captures, FunctionDefinition body)
    : ASTNode(row, column), captures(psl::move(captures)), body(psl::move(body)) {
}
uint16_t LambdaExpr::emit(Context& context, Bytecodes& bytecodes) const {
  auto class_name = psl::string("__lambda") + psl::to_string(context.next_internal_class_id());
  auto ctor_params = psl::vector<ParameterDeclaration>();
  auto ctor_args = psl::vector<Expr>();
  auto ctor_body = Block({});
  auto members = psl::vector_of<MemberDefinition>();
  auto body = this->body;
  // In operator(): Add the lambda object itself as the first argument
  body->params.push_front(ParameterDeclaration("self", class_name));
  // In ctor: Initialize `self`
  auto it = psl::next(ctor_body.elems.insert(
      ctor_body.elems.begin(),
      Stmt(Declaration("self", FunctionCall(row, column, "__" + class_name, {})))));
  for (const auto& capture : captures) {
    auto vi = bytecodes.var_index_by_name(capture.value);
    if (vi == uint16_t(-1))
      bytecodes.error(capture, "Variable `", capture.value, "` is not found");
    auto tname = context.find_type(bytecodes.var_type(vi)).alias;
    // Create a member that corresponds to each captured variable
    members.push_back(MemberDefinition(capture.row, capture.column, capture.value, tname));

    // In ctor: Initialize each member with their corresponding captured variable
    it = psl::next(ctor_body.elems.insert(
        it,
        Stmt(Expr(Expr(MemberAccess(capture.row, capture.column, Id(row, column, "self"), capture)),
                  Expr(capture), Expr::Init))));
    ctor_params.push_back(ParameterDeclaration(capture.value, tname));
    ctor_args.push_back(Expr(capture));
  }
  // In ctor: Return self
  ctor_body.elems.push_back(Stmt(ReturnStmt(Id(row, column, "self"))));
  // In operator(): Create references to members so that we can write x instead of self.x
  for (const auto& member : members) {
    body->block.elems.push_front(Stmt(Declaration(
        member.name,
        Expr(MemberAccess(row, column, Id(row, column, "self"), Id(row, column, member.name))),
        true)));
  }
  auto lambda_class =
      ClassDefinition(row, column, class_name,
                      psl::vector_of(FunctionDefinition(row, column, class_name, class_name,
                                                        ctor_params, ctor_body)),
                      psl::vector_of(*body), members);
  auto class_id = lambda_class.emit(context, bytecodes);
  context.find_type(class_id).convert_tos[psl::type_id<Function>()] = context.functions.size();
  auto arg_type_ids = psl::vector<size_t>();
  auto param_type_ids = psl::vector<psl::TypeId>();
  for (size_t i = 0; i < body->params.size(); i++) {
    auto tid = context.get_type_id(body->params[i].type);
    arg_type_ids.push_back(tid);
    if (i != 0)
      param_type_ids.push_back(psl::TypeId(tid, false, false));
  }
  auto fr = context.find_f("()", arg_type_ids);
  auto fid = fr.function_index;
  context.functions.push_back(low_level(
      [f = context.functions[fid], rid = fr.return_type_id,
       param_type_ids](psl::span<const Variable*> args) {
        return Function(low_level(
            [self = *args[0], f](psl::span<const Variable*> args) {
              auto args_ =
                  psl::vector<const Variable*, psl::static_allocator<const Variable*, 10>>();
              args_.push_back(&self);
              args_.insert_range(args_.end(), args);
              return f.call(args_);
            },
            rid, param_type_ids));
      },
      psl::type_id<Function>(), psl::vector_of(psl::TypeId(class_id, false, false))));
  // Create the lambda object
  return Expr(FunctionCall(row, column, class_name, ctor_args)).emit(context, bytecodes);
}

uint16_t Expr0::emit(Context& context, Bytecodes& bytecodes) const {
  if (op == None)
    return x.emit(context, bytecodes);

  try {
    auto vi = x.emit(context, bytecodes);
    uint16_t arg_indices[]{vi};
    size_t arg_type_ids[]{bytecodes.var_type(vi)};
    auto fr = Context::FindFResult();
    switch (op) {
      case None: break;
      case PreInc: fr = context.find_f("++x", arg_type_ids); break;
      case PreDec: fr = context.find_f("--x", arg_type_ids); break;
      case PostInc: fr = context.find_f("x++", arg_type_ids); break;
      case PostDec: fr = context.find_f("x--", arg_type_ids); break;
      case Positive: fr = context.find_f("+x", arg_type_ids); break;
      case Negate: fr = context.find_f("-x", arg_type_ids); break;
      case Invert: fr = context.find_f("!x", arg_type_ids); break;
    }
    CHECK(fr.function_index != size_t(-1));
    for (auto convert : fr.converts) {
      bytecodes.add_typed(Bytecode::Call, convert.converter_id, convert.to_type_id,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      arg_type_ids[convert.position] = convert.to_type_id;
    }
    if (arg_type_ids[0] == psl::type_id<int>() && op != Invert) {
      auto inst = Bytecode::Instruction();
      switch (op) {
        case None: CHECK(false); break;
        case Invert: CHECK(false); break;
        case PreInc: inst = Bytecode::IntPreInc; break;
        case PreDec: inst = Bytecode::IntPreDec; break;
        case PostInc: inst = Bytecode::IntPostInc; break;
        case PostDec: inst = Bytecode::IntPostDec; break;
        case Positive: inst = Bytecode::IntPositive; break;
        case Negate: inst = Bytecode::IntNegate; break;
      }
      bytecodes.add_typed(inst, fr.function_index, fr.return_type_id, arg_indices);
    } else if (arg_type_ids[0] == psl::type_id<float>() && (op == Positive || op == Negate)) {
      auto inst = Bytecode::Instruction();
      switch (op) {
        case None: CHECK(false); break;
        case Invert: CHECK(false); break;
        case PreInc: CHECK(false); break;
        case PreDec: CHECK(false); break;
        case PostInc: CHECK(false); break;
        case PostDec: CHECK(false); break;
        case Positive: inst = Bytecode::FloatPositive; break;
        case Negate: inst = Bytecode::FloatNegate; break;
      }
      bytecodes.add_typed(inst, fr.function_index, fr.return_type_id, arg_indices);
    } else {
      bytecodes.add_typed(Bytecode::Call, fr.function_index, fr.return_type_id, arg_indices);
    }
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(*this, e.what());
  }
}
template <psl::DerivedFrom<ASTNode> T>
Expr::Expr(T x) : Expr(Expr0(x.row, x.column, psl::move(x))) {
}
Expr::Expr(Expr0 x) : ASTNode(x.row, x.column), op{None}, x{psl::move(x)} {
}
Expr::Expr(Expr a, Expr b, Op op) : ASTNode{a}, op{op}, a{psl::move(a)}, b{psl::move(b)} {
}
uint16_t Expr::emit(Context& context, Bytecodes& bytecodes) const {
  if (op == None)
    return x->emit(context, bytecodes);

  try {
    auto a_index = a->emit(context, bytecodes);
    auto b_index = b->emit(context, bytecodes);
    uint16_t arg_indices[]{a_index, b_index};
    size_t arg_type_ids[]{bytecodes.var_type(a_index), bytecodes.var_type(b_index)};
    if (op == Init) {
      bytecodes.add(Bytecode::InitVar, a_index, b_index);
      return uint16_t(-1);
    }
    auto fr = Context::FindFResult();
    switch (op) {
      case None: CHECK(false); break;
      case Mul: fr = context.find_f("*", arg_type_ids); break;
      case Div: fr = context.find_f("/", arg_type_ids); break;
      case Mod: fr = context.find_f("%", arg_type_ids); break;
      case Pow: fr = context.find_f("^", arg_type_ids); break;
      case Add: fr = context.find_f("+", arg_type_ids); break;
      case Sub: fr = context.find_f("-", arg_type_ids); break;
      case Lt: fr = context.find_f("<", arg_type_ids); break;
      case Gt: fr = context.find_f(">", arg_type_ids); break;
      case Le: fr = context.find_f("<=", arg_type_ids); break;
      case Ge: fr = context.find_f(">=", arg_type_ids); break;
      case Eq: fr = context.find_f("==", arg_type_ids); break;
      case Ne: fr = context.find_f("!=", arg_type_ids); break;
      case And: fr = context.find_f("&&", arg_type_ids); break;
      case Or: fr = context.find_f("||", arg_type_ids); break;
      case AddE: fr = context.find_f("+=", arg_type_ids); break;
      case SubE: fr = context.find_f("-=", arg_type_ids); break;
      case MulE: fr = context.find_f("*=", arg_type_ids); break;
      case DivE: fr = context.find_f("/=", arg_type_ids); break;
      case ModE: fr = context.find_f("%=", arg_type_ids); break;
      case Assi: fr = context.find_f("=", arg_type_ids); break;
      case Init: CHECK(false);
    }
    CHECK(fr.function_index != size_t(-1));
    for (auto convert : fr.converts) {
      bytecodes.add_typed(Bytecode::Call, convert.converter_id, convert.to_type_id,
                          arg_indices[convert.position]);
      arg_indices[convert.position] = bytecodes.top_var_index();
      arg_type_ids[convert.position] = convert.to_type_id;
    }
    if (arg_type_ids[0] == psl::type_id<int>() && arg_type_ids[1] == psl::type_id<int>() &&
        op != And && op != Or) {
      auto inst = Bytecode::Instruction();
      switch (op) {
        case None: CHECK(false); break;
        case And: CHECK(false); break;
        case Or: CHECK(false); break;
        case Mul: inst = Bytecode::IntMul; break;
        case Div: inst = Bytecode::IntDiv; break;
        case Mod: inst = Bytecode::IntMod; break;
        case Pow: inst = Bytecode::IntPow; break;
        case Add: inst = Bytecode::IntAdd; break;
        case Sub: inst = Bytecode::IntSub; break;
        case Lt: inst = Bytecode::IntLt; break;
        case Gt: inst = Bytecode::IntGt; break;
        case Le: inst = Bytecode::IntLe; break;
        case Ge: inst = Bytecode::IntGe; break;
        case Eq: inst = Bytecode::IntEq; break;
        case Ne: inst = Bytecode::IntNe; break;
        case AddE: inst = Bytecode::IntAddE; break;
        case SubE: inst = Bytecode::IntSubE; break;
        case MulE: inst = Bytecode::IntMulE; break;
        case DivE: inst = Bytecode::IntDivE; break;
        case ModE: inst = Bytecode::IntModE; break;
        case Assi: inst = Bytecode::IntAssi; break;
        case Init: CHECK(false); break;
      }
      bytecodes.add_typed(inst, fr.function_index, fr.return_type_id, arg_indices);
    } else if (arg_type_ids[0] == psl::type_id<float>() &&
               arg_type_ids[1] == psl::type_id<float>() && op != And && op != Or && op != Mod &&
               op != ModE) {
      auto inst = Bytecode::Instruction();
      switch (op) {
        case None: CHECK(false); break;
        case And: CHECK(false); break;
        case Or: CHECK(false); break;
        case Mul: inst = Bytecode::FloatMul; break;
        case Div: inst = Bytecode::FloatDiv; break;
        case Mod: CHECK(false); break;
        case Pow: inst = Bytecode::FloatPow; break;
        case Add: inst = Bytecode::FloatAdd; break;
        case Sub: inst = Bytecode::FloatSub; break;
        case Lt: inst = Bytecode::FloatLt; break;
        case Gt: inst = Bytecode::FloatGt; break;
        case Le: inst = Bytecode::FloatLe; break;
        case Ge: inst = Bytecode::FloatGe; break;
        case Eq: inst = Bytecode::FloatEq; break;
        case Ne: inst = Bytecode::FloatNe; break;
        case AddE: inst = Bytecode::FloatAddE; break;
        case SubE: inst = Bytecode::FloatSubE; break;
        case MulE: inst = Bytecode::FloatMulE; break;
        case DivE: inst = Bytecode::FloatDivE; break;
        case ModE: CHECK(false); break;
        case Assi: inst = Bytecode::FloatAssi; break;
        case Init: CHECK(false); break;
      }
      bytecodes.add_typed(inst, fr.function_index, fr.return_type_id, arg_indices);
    } else {
      bytecodes.add_typed(Bytecode::Call, fr.function_index, fr.return_type_id, arg_indices);
    }
    return bytecodes.top_var_index();
  } catch (const Exception& e) {
    bytecodes.error(*this, e.what());
  }
}
void Declaration::emit(Context& context, Bytecodes& bytecodes) const {
  auto vi = expr.emit(context, bytecodes);
  if (as_ref)
    bytecodes.add_typed(Bytecode::ShallowCopy, vi, bytecodes.var_type(vi));
  else
    bytecodes.add_typed(Bytecode::Copy, vi, bytecodes.var_type(vi));
  bytecodes.name_top_var(name);
}
void Stmt::emit(Context& context, Bytecodes& bytecodes) const {
  dispatch([&](auto&& x) { x.emit(context, bytecodes); });
  // if (is<Expr>() && as<Expr>().op != Expr::Init) {
  //   CHECK(bytecodes.code_position() != 0);
  //   bytecodes[bytecodes.code_position() - 1].value1 = size_t(-1);
  //   bytecodes.pop_stack();
  // }
}
Block::Block(psl::vector<PBlock> elems) : elems{psl::move(elems)} {
}
void Block::emit(Context& ctx, Bytecodes& bytecodes) const {
  auto sp = bytecodes.stack_size();
  bytecodes.enter_scope();
  for (const auto& pblock : elems)
    pblock.emit(ctx, bytecodes);
  bytecodes.add(Bytecode::UnwindStack, sp);
  bytecodes.exit_scope();
}

While::While(size_t row, size_t column, Expr condition, PBlock pblock)
    : ASTNode{row, column}, condition(psl::move(condition)), pblock(psl::move(pblock)) {
}
void While::emit(Context& context, Bytecodes& bytecodes) const {
  auto sp = bytecodes.stack_size();
  bytecodes.enter_scope();
  auto condition_begin = bytecodes.code_position();
  auto condition_vi = condition.emit(context, bytecodes);
  if (bytecodes.var_type(condition_vi) != psl::type_id<bool>())
    bytecodes.error(condition, "Expect boolean expression");
  auto ci = bytecodes.add(Bytecode::JumpIfNot, 0 /*placeholder*/, condition_vi);
  pblock->emit(context, bytecodes);
  bytecodes.add(Bytecode::UnwindStack, sp);
  bytecodes.add(Bytecode::Jump, condition_begin);
  auto while_end = bytecodes.code_position();
  bytecodes[ci].value = while_end;
  bytecodes.add(Bytecode::UnwindStack, sp);
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

For::For(size_t row, size_t column, Stmt init, Expr condition, Expr inc, PBlock block)
    : ASTNode{row, column},
      init{psl::move(init)},
      condition{psl::move(condition)},
      inc{psl::move(inc)},
      block{psl::move(block)} {
}
void For::emit(Context& context, Bytecodes& bytecodes) const {
  bytecodes.enter_scope();
  auto sp_init = bytecodes.stack_size();
  init.emit(context, bytecodes);
  auto sp = bytecodes.stack_size();
  auto condition_begin = bytecodes.code_position();
  auto condition_vi = condition.emit(context, bytecodes);
  if (bytecodes.var_type(condition_vi) != psl::type_id<bool>())
    bytecodes.error(condition, "Expect boolean expression");
  auto ci = bytecodes.add(Bytecode::JumpIfNot, 0 /*placeholder*/, condition_vi);
  block->emit(context, bytecodes);
  inc.emit(context, bytecodes);
  bytecodes[bytecodes.code_position() - 1].value1 = size_t(-1);
  bytecodes.pop_stack();

  bytecodes.add(Bytecode::UnwindStack, sp);
  bytecodes.add(Bytecode::Jump, condition_begin);
  auto for_end = bytecodes.code_position();
  bytecodes[ci].value = for_end;
  bytecodes.add(Bytecode::UnwindStack, sp_init);
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
If::If(size_t row, size_t column, Expr condition, PBlock block)
    : ASTNode{row, column}, condition{psl::move(condition)}, block{psl::move(block)} {
}
ElseIf::ElseIf(size_t row, size_t column, Expr condition, PBlock block)
    : ASTNode{row, column}, condition{psl::move(condition)}, block{psl::move(block)} {
}
Else::Else(PBlock block) : block{psl::move(block)} {
}
void IfElseChain::emit(Context& context, Bytecodes& bytecodes) const {
  auto if_ci_out = size_t(-1);
  auto sp = bytecodes.stack_size();
  bytecodes.enter_scope();
  {
    auto vi = if_.condition.emit(context, bytecodes);
    CHECK_EQ(bytecodes.var_type(vi), psl::type_id<bool>());
    auto ci_cond = bytecodes.add(Bytecode::JumpIfNot, 0 /*placeholder*/, vi);
    bytecodes.enter_scope();
    if_.block->emit(context, bytecodes);
    bytecodes.exit_scope();
    if_ci_out = bytecodes.add(Bytecode::Jump, 0 /*placeholder*/);
    bytecodes[ci_cond].value = bytecodes.code_position();
  }

  auto else_if_ci_outs = psl::vector<size_t>();
  else_if_ci_outs.reserve(else_ifs.size());
  for (const auto& else_if : else_ifs) {
    auto vi = else_if.condition.emit(context, bytecodes);
    auto ci_cond = bytecodes.add(Bytecode::JumpIfNot, 0 /*placeholder*/, vi);
    bytecodes.enter_scope();
    else_if.block->emit(context, bytecodes);
    bytecodes.exit_scope();
    else_if_ci_outs.push_back(bytecodes.add(Bytecode::Jump, 0 /*placeholder*/));
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
  bytecodes.add(Bytecode::UnwindStack, sp);
  bytecodes.exit_scope();
}

void FunctionDefinition::emit(Context& context, Bytecodes& bytecodes) const {
  try {
    auto f_bytecodes = Bytecodes(context, bytecodes.sl);
    auto param_type_ids = psl::vector<psl::TypeId>();
    for (const auto& param : params) {
      auto tid = context.get_type_id(param.type);
      f_bytecodes.add_typed(param.name, tid);
      param_type_ids.push_back(psl::TypeId(tid, false, false));
    }
    block.emit(context, f_bytecodes);
    context(name) = low_level(
        [&context, f_bytecodes, *this](psl::span<const Variable*> args) mutable {
          CHECK_EQ(args.size(), params.size());
          auto vm = VirtualMachine();
          for (const auto& var : context.variables)
            vm.stack.push(var);
          for (size_t i = 0; i < args.size(); i++)
            vm.stack.push(*args[i]);
          return execute(context, f_bytecodes, vm);
        },
        context.get_type_id(return_type), param_type_ids);
    // bytecodes.add_typed(Bytecode::Load, context.function_variables.size() - 1,
    //                     context.function_variables.back().type_id());
    // bytecodes.name_top_var(name);
  } catch (const Exception& e) {
    bytecodes.error(*this, e.what());
  }
}
size_t ClassDefinition::emit(Context& context, Bytecodes& bytecodes) const {
  auto type_id = context.new_internal_class();
  auto& type_trait = context.types[type_id];
  type_trait.alias = name;
  context("__" + name) = low_level(
      [n = members.size()](psl::span<const Variable*>) {
        return InternalClass{psl::vector_n_of(n, Variable(psl::Empty()))};
      },
      type_id, {});

  for (size_t i = 0; i < members.size(); i++) {
    type_trait.member_accessors[members[i].name] = context.functions.size();
    context.functions.push_back(low_level(
        [i](psl::span<const Variable*> args) {
          DCHECK_GT(args[0]->as<InternalClass&>().members.size(), i);
          return psl::ref(args[0]->as<InternalClass&>().members[i]);
        },
        context.get_type_id(members[i].type), psl::vector_of(psl::TypeId(type_id, false, true))));
  }

  for (const auto& ctor : ctors)
    ctor.emit(context, bytecodes);
  for (const auto& method : methods)
    method.emit(context, bytecodes);

  return type_id;
}

struct Parser {
  Parser(psl::string_view tokens) : sl{tokens, row_padding} {
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
    consume("(");
    auto r = row, c = column;
    auto cond = expr();
    consume(")");
    auto body = pblock();
    return While{r, c, psl::move(cond), psl::move(body)};
  }
  For for_() {
    consume("for");

    if (accept("(")) {
      auto init = stmt();
      auto r = row, c = column;
      auto cond = expr();
      consume(";");
      auto inc = expr();
      consume(")");
      auto body = pblock();
      return For{r, c, psl::move(init), psl::move(cond), psl::move(inc), psl::move(body)};
    } else {
      auto r = row, c = column;
      auto id_ = id();
      consume("in");
      auto range_a = expr();
      consume("..");
      auto range_b = expr();
      auto init = Stmt(Declaration(id_.value, range_a));
      auto cond = Expr(Expr(Expr0(r, c, PExpr(id_), Expr0::None)), range_b, Expr::Lt);
      auto body = pblock();
      return For{r,
                 c,
                 psl::move(init),
                 psl::move(cond),
                 Expr(Expr0(r, c, PExpr(id_), Expr0::PreInc)),
                 psl::move(body)};
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
    return IfElseChain{psl::move(if_clause), psl::move(else_ifs), psl::move(else_clause)};
  }
  If if_() {
    consume("if");
    consume("(");
    auto r = row, c = column;
    auto cond = expr();
    consume(")");
    auto body = pblock();
    return If{r, c, psl::move(cond), psl::move(body)};
  }
  ElseIf else_if_() {
    consume("else");
    consume("if");
    consume("(");
    auto r = row, c = column;
    auto cond = expr();
    consume(")");
    auto body = pblock();
    return ElseIf{r, c, psl::move(cond), psl::move(body)};
  }
  Else else_() {
    consume("else");
    return Else{pblock()};
  }
  ClassDefinition class_definition() {
    auto r = row, c = column;
    consume("class");
    auto name = id().value;
    consume("{");

    auto ctors = psl::vector<FunctionDefinition>();
    auto methods = psl::vector<FunctionDefinition>();
    auto members = psl::vector<MemberDefinition>();

    while (!accept("}")) {
      if (expect("ctor")) {
        ctors.push_back(ctor_definition(name));
      } else if (expect("fn")) {
        methods.push_back(method_definition(name));
      } else {
        members.push_back(member_definition());
        consume(";");
      }
      while (accept(";"))
        ;
    }

    for (auto& ctor : ctors)
      for (const auto& member : members) {
        auto r = ctor.row, c = ctor.column;
        CHECK_GE(ctor.block.elems.size(), 1);
        auto it = psl::next(ctor.block.elems.begin());
        it = ctor.block.elems.insert(
            it,
            Stmt(Declaration(member.name,
                             MemberAccess(r, c, Id(r, c, "self"), Id(r, c, member.name)), true)));
      }
    for (auto& method : methods)
      for (const auto& member : members) {
        auto r = method.row, c = method.column;
        method.block.elems.push_front(Stmt(Declaration(
            member.name, MemberAccess(r, c, Id(r, c, "self"), Id(r, c, member.name)), true)));
      }
    return ClassDefinition(r, c, psl::move(name), psl::move(ctors), psl::move(methods),
                           psl::move(members));
  }
  MemberDefinition member_definition() {
    auto r = row, c = column;
    auto name = id();
    consume(":");
    auto type = id();
    return MemberDefinition(r, c, psl::move(name.value), psl::move(type.value));
  };
  FunctionDefinition ctor_definition(psl::string class_name) {
    auto r = row, c = column;
    consume("ctor");
    auto ctor_name = id().value;
    consume("(");
    auto params = param_list();
    consume(")");
    auto init_stmts = psl::vector<Stmt>();
    if (accept(":")) {
      while (!expect("{")) {
        auto r = row, c = column;
        auto name = id().value;
        auto expr_ = expr();
        init_stmts.push_back(
            Stmt(Expr(Expr(Expr0(r, c, MemberAccess(r, c, Id(r, c, "self"), Id(r, c, name)))),
                      expr_, Expr::Init)));
        if (!accept(",")) {
          if (!expect("{")) {
            error("Expect either `,` to continue or '{' to begin function definition");
          }
        }
      }
    }
    auto block_ = block();
    auto it = psl::next(block_.elems.insert(
        block_.elems.begin(),
        PBlock(Stmt(
            Declaration("self", Expr(Expr0(r, c, FunctionCall(r, c, "__" + class_name, {}))))))));
    for (const auto& init_stmt : init_stmts) {
      it = psl::next(block_.elems.insert(it, PBlock(init_stmt)));
    }
    block_.elems.push_back(PBlock(Stmt(ReturnStmt(Expr(Expr0(r, c, Id(r, c, "self")))))));
    return FunctionDefinition(r, c, ctor_name, class_name, psl::move(params), psl::move(block_));
  }
  FunctionDefinition method_definition(psl::string class_name) {
    auto r = row, c = column;
    consume("fn");
    auto name = id().value;
    consume("(");
    auto params = param_list();
    params.push_front(ParameterDeclaration("self", class_name));
    consume(")");
    consume(":");
    auto type = id().value;
    auto block_ = block();
    return FunctionDefinition(r, c, psl::move(name), psl::move(type), psl::move(params),
                              psl::move(block_));
  }
  FunctionDefinition function_definition() {
    auto r = row, c = column;
    consume("fn");
    auto name = id().value;
    consume("(");
    auto params = param_list();
    consume(")");
    consume(":");
    auto type = id().value;
    auto block_ = block();
    return FunctionDefinition(r, c, psl::move(name), psl::move(type), psl::move(params),
                              psl::move(block_));
  }
  Stmt stmt() {
    auto stmt = Stmt{};
    if (accept(";")) {
      return Stmt(Semicolon());
    }
    if (accept("break")) {
      stmt = Stmt(BreakStmt());
    } else if (accept("continue")) {
      stmt = Stmt(ContinueStmt());
    } else if (accept("return")) {
      stmt = Stmt(ReturnStmt(expr()));
    } else if (accept("var")) {
      auto id_ = id();
      consume("=");
      stmt = Stmt(declaration(id_));
    } else {
      if (auto n = next(); psl::isalpha(*n) || *n == '_') {
        backup();
        auto id_ = id();
        if (accept(":=")) {
          stmt = Stmt(declaration(id_));
        } else {
          undo();
          stmt = Stmt(expr());
        }
      } else {
        stmt = Stmt(expr());
      }
    }
    consume(";");
    return stmt;
  }

  Declaration declaration(Id id_) {
    auto expr_ = expr();
    return Declaration{psl::move(id_.value), psl::move(expr_)};
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
      auto a = psl::move(exprs[index]);
      auto b = psl::move(exprs[index + 1]);
      ops.erase(ops.begin() + index);
      exprs.erase(exprs.begin() + index);
      exprs.erase(exprs.begin() + index);
      exprs.insert(exprs.begin() + index,
                   Expr{psl::move(a), psl::move(b), static_cast<Expr::Op>(max_precedence)});
    }

    return exprs[0];
  }
  Expr0 expr0() {
    auto r = row, c = column;
    if (accept("++")) {
      return Expr0{r, c, pexpr(), Expr0::PreInc};
    } else if (accept("--")) {
      return Expr0{r, c, pexpr(), Expr0::PreDec};
    } else if (accept("+")) {
      return Expr0{r, c, pexpr(), Expr0::Positive};
    } else if (accept("-")) {
      return Expr0{r, c, pexpr(), Expr0::Negate};
    } else if (accept("!")) {
      return Expr0{r, c, pexpr(), Expr0::Invert};
    } else {
      auto pexpr_ = pexpr();
      if (accept("++"))
        return Expr0{r, c, psl::move(pexpr_), Expr0::PostInc};
      else if (accept("--"))
        return Expr0{r, c, psl::move(pexpr_), Expr0::PostDec};
      else
        return Expr0{r, c, psl::move(pexpr_), Expr0::None};
    }
  }
  PExpr pexpr() {
    auto pexpr_ = pexpr_base();
    while (true) {
      if (accept("[")) {
        auto r = row, c = column;
        auto index = expr();
        consume("]");
        pexpr_ = Subscript{r, c, psl::move(pexpr_), psl::move(index)};
      } else if (expect("..")) {
        break;
      } else if (accept(".")) {
        auto r = row, c = column;
        auto id_ = id();
        pexpr_ = MemberAccess{r, c, psl::move(pexpr_), psl::move(id_)};
      } else if (accept("(")) {
        if (pexpr_.is<Id>()) {
          pexpr_ = FunctionCall{pexpr_.as<Id>().row, pexpr_.as<Id>().column, pexpr_.as<Id>().value,
                                arg_list()};
        } else if (pexpr_.is<MemberAccess>()) {
          auto& p = pexpr_.as<MemberAccess>();
          auto args = arg_list();
          auto name = psl::move(p.id).value;
          args.insert(args.begin(), Expr{Expr0{p.row, p.column, *psl::move(p.pexpr), Expr0::None}});
          pexpr_ = FunctionCall{p.id.row, p.id.column, psl::move(name), psl::move(args)};
        } else {
          error("An identifier must precedes function call operator ()");
        }
        consume(")");
      } else {
        break;
      }
    }

    return pexpr_;
  }
  PExpr pexpr_base() {
    if (accept("false"))
      return PExpr{BooleanLiteral{false}};
    else if (accept("true"))
      return PExpr{BooleanLiteral{true}};
    else if (expect("\""))
      return PExpr{string_literal()};
    else if (expect("[")) {
      backup();
      auto vector_ = vector();
      if (expect("(")) {
        undo();
        return PExpr(lambda());
      } else {
        return PExpr(psl::move(vector_));
      }
    } else if (accept("(")) {
      auto pexpr = PExpr{expr()};
      consume(")");
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
    auto r = row, c = column;
    consume("[");
    auto captures = psl::vector<Id>();
    while (!accept("]")) {
      captures.push_back(id());
      accept(",");
    }
    consume("(");
    auto params = param_list();
    consume(")");
    consume(":");
    auto return_type = id().value;
    auto body = block();
    return LambdaExpr(r, c, captures, FunctionDefinition(r, c, "()", return_type, params, body));
  }
  Vector vector() {
    auto r = row, c = column;
    consume("[");
    auto args = psl::vector<Expr>{};
    if (!accept("]"))
      while (true) {
        args.push_back(expr());
        if (accept("]"))
          break;
        else
          consume(",", "to specify more element");
      }
    return Vector{r, c, psl::move(args)};
  }
  psl::vector<ParameterDeclaration> param_list() {
    auto args = psl::vector<ParameterDeclaration>();
    if (!expect(")"))
      while (true) {
        auto param = ParameterDeclaration();
        param.name = id().value;
        consume(":");
        param.type = id().value;
        args.push_back(psl::move(param));
        if (expect(")"))
          break;
        else
          consume(",", "to continue specify parameter");
      }
    return args;
  }
  psl::vector<Expr> arg_list() {
    auto args = psl::vector<Expr>();
    if (!expect(")"))
      while (true) {
        args.push_back(expr());
        if (expect(")"))
          break;
        else
          consume(",", "to continue specify argument");
      }
    return args;
  }
  Id id() {
    auto r = row, c = column;
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
    return Id{r, c, psl::move(str)};
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
    return NumberLiteral{psl::move(str)};
  }
  StringLiteral string_literal() {
    auto str = psl::string();
    consume("\"", ExplicitParameter(false));
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
      if (*n == '"')
        break;
      proceed();
      str.push_back(*n);
      escape = *n == '\\';
    }
    consume("\"");
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
  bool accept(psl::string_view str) {
    if (expect(str)) {
      proceed(str.size());
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
    return sl.next(row, column);
  }
  template <typename... Args>
  [[noreturn]] void error(const Args&... args) {
    sl.error(row, column, args...);
  }

  size_t row = row_padding;
  size_t column = 0;
  psl::vector<psl::pair<size_t, size_t>> backup_stack;
  static constexpr size_t row_padding = 1;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};

psl::pair<Block, SourceLines> parse_as_block(psl::string source) {
  auto parser = Parser{source};
  return {parser.block(true), psl::move(parser.sl)};
}
Bytecodes compile(Context& context, psl::string source) {
  auto [block, sl] = parse_as_block(psl::move(source));
  auto bytecodes = Bytecodes(context, psl::move(sl));
  block.emit(context, bytecodes);
  return bytecodes;
}

Variable execute(const Context& context, const Bytecodes& bytecodes, VirtualMachine& vm) {
  auto return_value = Variable(psl::Empty());
  try {
    for (size_t p = 0; p < bytecodes.length();) {
      const auto& code = bytecodes[p];
      auto inc_p = true;
      const auto push = [&](auto x) {
        if (code.value1 != size_t(-1))
          vm.stack.push(psl::move(x));
      };

      switch (code.instruction) {
        case Bytecode::Break: break;
        case Bytecode::Continue: break;
        case Bytecode::Load: vm.stack.push(context.function_variables[code.value]); break;
        case Bytecode::Copy: push(vm.stack[code.value].clone()); break;
        case Bytecode::ShallowCopy: push(vm.stack[code.value].shallow_clone()); break;
        case Bytecode::InitVar:
          vm.stack[code.value].shallow_as<Variable&>() = vm.stack[code.value1].clone();
          break;
        case Bytecode::LoadFloatConstant:
          push(psl::bitcast<float>(static_cast<uint32_t>(code.value)));
          break;
        case Bytecode::LoadIntConstant:
          push(psl::bitcast<int>(static_cast<uint32_t>(code.value)));
          break;
        case Bytecode::LoadBoolConstant: push(static_cast<bool>(code.value)); break;
        case Bytecode::LoadStringConstant: push(bytecodes.get_string(code.value)); break;
        case Bytecode::IntPreInc: push(++vm.stack[code.args[0]].as<int&>()); break;
        case Bytecode::IntPreDec: push(--vm.stack[code.args[0]].as<int&>()); break;
        case Bytecode::IntPostInc: push(vm.stack[code.args[0]].as<int&>()++); break;
        case Bytecode::IntPostDec: push(vm.stack[code.args[0]].as<int&>()--); break;
        case Bytecode::IntPositive: push(vm.stack[code.args[0]].as<int>()); break;
        case Bytecode::IntNegate: push(-vm.stack[code.args[0]].as<int>()); break;
        case Bytecode::IntEq:
          push(vm.stack[code.args[0]].as<int>() == vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntNe:
          push(vm.stack[code.args[0]].as<int>() != vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntLt:
          push(vm.stack[code.args[0]].as<int>() < vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntGt:
          push(vm.stack[code.args[0]].as<int>() > vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntLe:
          push(vm.stack[code.args[0]].as<int>() <= vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntGe:
          push(vm.stack[code.args[0]].as<int>() >= vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntAdd:
          push(vm.stack[code.args[0]].as<int>() + vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntSub:
          push(vm.stack[code.args[0]].as<int>() - vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntMul:
          push(vm.stack[code.args[0]].as<int>() * vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntDiv:
          push(vm.stack[code.args[0]].as<int>() / vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntPow:
          push(psl::powi(vm.stack[code.args[0]].as<int>(), vm.stack[code.args[1]].as<int>()));
          break;
        case Bytecode::IntMod:
          push(vm.stack[code.args[0]].as<int>() % vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntAddE:
          push(vm.stack[code.args[0]].as<int&>() += vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntSubE:
          push(vm.stack[code.args[0]].as<int&>() -= vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntMulE:
          push(vm.stack[code.args[0]].as<int&>() *= vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntDivE:
          push(vm.stack[code.args[0]].as<int&>() /= vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntModE:
          push(vm.stack[code.args[0]].as<int&>() %= vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::IntAssi:
          push(vm.stack[code.args[0]].as<int&>() = vm.stack[code.args[1]].as<int>());
          break;
        case Bytecode::FloatPositive: push(vm.stack[code.args[0]].as<float>()); break;
        case Bytecode::FloatNegate: push(-vm.stack[code.args[0]].as<float>()); break;
        case Bytecode::FloatEq:
          push(vm.stack[code.args[0]].as<float>() == vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatNe:
          push(vm.stack[code.args[0]].as<float>() != vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatLt:
          push(vm.stack[code.args[0]].as<float>() < vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatGt:
          push(vm.stack[code.args[0]].as<float>() > vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatLe:
          push(vm.stack[code.args[0]].as<float>() <= vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatGe:
          push(vm.stack[code.args[0]].as<float>() >= vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatAdd:
          push(vm.stack[code.args[0]].as<float>() + vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatSub:
          push(vm.stack[code.args[0]].as<float>() - vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatMul:
          push(vm.stack[code.args[0]].as<float>() * vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatDiv:
          push(vm.stack[code.args[0]].as<float>() / vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatPow:
          push(psl::powi(vm.stack[code.args[0]].as<float>(), vm.stack[code.args[1]].as<float>()));
          break;
        case Bytecode::FloatAddE:
          push(vm.stack[code.args[0]].as<float&>() += vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatSubE:
          push(vm.stack[code.args[0]].as<float&>() -= vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatMulE:
          push(vm.stack[code.args[0]].as<float&>() *= vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatDivE:
          push(vm.stack[code.args[0]].as<float&>() /= vm.stack[code.args[1]].as<float>());
          break;
        case Bytecode::FloatAssi:
          push(vm.stack[code.args[0]].as<float&>() = vm.stack[code.args[1]].as<float>());
          break;
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
        case Bytecode::Return: return_value = vm.stack[code.value]; break;
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
    Log(e.what());
  }

  return return_value;
}

Variable execute(const Context& context, const Bytecodes& bytecodes) {
  Profiler _("[Interpreter]Execute");
  auto vm = VirtualMachine();
  for (const auto& var : context.variables)
    vm.stack.push(var);
  return execute(context, bytecodes, vm);
}

}  // namespace pine