#pragma once

#include <pine/core/context.h>
#include <pine/core/vecmath.h>

#include <pine/psl/variant.h>

namespace pine {

struct ASTNode {
  ASTNode() = default;
  ASTNode(size_t row, size_t column) : row{row}, column{column} {
  }

  size_t row = invalid;
  size_t column = invalid;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};

struct PExpr;
struct Expr0;
struct Block;
struct ScopeUnit;
struct FunctionDefinition;

struct Semicolon {
  void eval(Context&) const {
  }
};

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
    // clang-format on
  } op;

  Expr() = default;
  Expr(size_t row, size_t column, Expr0 x);
  Expr(Expr a, Expr b, Op op);
  Variable eval(Context& ctx) const;

private:
  psl::Box<Expr0> x;
  psl::Box<Expr> a;
  psl::Box<Expr> b;
};

struct Id : ASTNode {
  Id() = default;
  Id(size_t row, size_t column, psl::string value) : ASTNode{row, column}, value{psl::move(value)} {
  }
  Variable& eval(Context& ctx) const;

  psl::string value;
};
struct Number {
  Number() = default;
  Number(psl::string str) {
    if (psl::has(str, '.')) {
      is_float = true;
      valuef = psl::stof(str);
    } else {
      valuei = psl::stoi(str);
    }
  }
  Variable eval(Context&) const {
    return is_float ? Variable{valuef} : Variable{valuei};
  }

  bool is_float = false;
  float valuef;
  int valuei;
};
struct BooleanLiteral {
  BooleanLiteral() = default;
  BooleanLiteral(bool value) : value{value} {};
  Variable eval(Context&) const {
    return value;
  }

private:
  bool value = false;
};
struct StringLiteral {
  StringLiteral() = default;
  StringLiteral(psl::string value) : value{psl::move(value)} {
  }
  Variable eval(Context&) const {
    return value;
  }

  psl::string value;
};
struct Vector : ASTNode {
  Vector() = default;
  Vector(size_t row, size_t column, psl::vector<Expr> args)
      : ASTNode{row, column}, args{psl::move(args)} {
  }
  Variable eval(Context& ctx) const;

  psl::vector<Expr> args;
};
struct FunctionCall : ASTNode {
  FunctionCall() = default;
  FunctionCall(size_t row, size_t column, Id id, psl::vector<Expr> args)
      : ASTNode{row, column}, id{psl::move(id)}, args{psl::move(args)} {
  }
  Variable eval(Context& ctx, psl::optional<Variable> receiver = psl::nullopt) const;

  Id id;
  psl::vector<Expr> args;
};
struct MemberAccess : ASTNode {
  MemberAccess() = default;
  MemberAccess(size_t row, size_t column, PExpr pexpr, Id id);
  Variable eval(Context& ctx) const;

  psl::Box<PExpr> pexpr;
  Id id;
};
struct Subscript : ASTNode {
  Subscript() = default;
  Subscript(size_t row, size_t column, PExpr pexpr, Expr index);
  Variable eval(Context& ctx) const;

  psl::Box<PExpr> pexpr;
  Expr index;
};
struct Grouped {
  Grouped() = default;
  Grouped(Expr expr) : expr{psl::move(expr)} {
  }
  Variable eval(Context& ctx) const {
    return expr.eval(ctx);
  }

  Expr expr;
};
struct PExpr : psl::Variant<Id, Number, StringLiteral, BooleanLiteral, Vector, Grouped,
                            FunctionCall, Subscript, MemberAccess> {
  using Variant::Variant;
  Variable eval(Context& ctx) const;
};
struct Expr0 : ASTNode {
  enum Op { None, PreInc, PreDec, PostInc, PostDec, Positive, Negate, Invert };
  Expr0() = default;
  Expr0(size_t row, size_t column, PExpr x, Op op)
      : ASTNode{row, column}, x{psl::move(x)}, op{op} {};
  Variable eval(Context& ctx) const;

private:
  PExpr x;
  Op op;
};
struct Declaration {
  Declaration() = default;
  Declaration(Id id, Expr expr) : id{psl::move(id)}, expr{psl::move(expr)} {
  }
  void eval(Context& ctx) const;

  Id id;
  Expr expr;
};
struct ReturnStmt {
  ReturnStmt() = default;
  ReturnStmt(Expr expr) : expr{psl::move(expr)} {
  }
  void eval(Context& ctx) const;

  Expr expr;
};
struct BreakStmt {
  BreakStmt() = default;
  void eval(Context& ctx) const {
    ctx.set_break_flag();
  }
};
struct ContinueStmt {
  ContinueStmt() = default;
  void eval(Context& ctx) const {
    ctx.set_continue_flag();
  }
};
struct Stmt : psl::Variant<Expr, Declaration, ReturnStmt, BreakStmt, ContinueStmt> {
  Stmt() = default;
  Stmt(Expr x, Semicolon) : Variant(psl::move(x)) {
  }
  Stmt(Declaration x, Semicolon) : Variant(psl::move(x)) {
  }
  Stmt(ReturnStmt x, Semicolon) : Variant(psl::move(x)) {
  }
  Stmt(BreakStmt x, Semicolon) : Variant(psl::move(x)) {
  }
  Stmt(ContinueStmt x, Semicolon) : Variant(psl::move(x)) {
  }
  void eval(Context& ctx) const;
};

struct PBlock {
  PBlock();
  ~PBlock();
  PBlock(const PBlock&);
  PBlock(PBlock&&);
  PBlock& operator=(const PBlock&);
  PBlock& operator=(PBlock&&);
  PBlock(Semicolon x);
  PBlock(Stmt x);
  PBlock(ScopeUnit x);
  PBlock(FunctionDefinition x);

  void eval(Context& ctx) const;

private:
  psl::Box<psl::Variant<Semicolon, Stmt, ScopeUnit, FunctionDefinition>> base;
};
struct While : ASTNode {
  While() = default;
  While(size_t row, size_t column, Expr condition, PBlock block)
      : ASTNode{row, column}, condition{psl::move(condition)}, block{psl::move(block)} {
  }
  void eval(Context& ctx) const;
  Expr condition;
  PBlock block;
};
struct For : ASTNode {
  For() = default;
  For(size_t row, size_t column, Stmt init, Expr condition, Expr inc, PBlock block)
      : ASTNode{row, column},
        init{psl::move(init)},
        condition{psl::move(condition)},
        inc{psl::move(inc)},
        block{psl::move(block)} {
  }
  void eval(Context& ctx) const;
  Stmt init;
  Expr condition;
  Expr inc;
  PBlock block;
};
struct If : ASTNode {
  If() = default;
  If(size_t row, size_t column, Expr condition, PBlock block)
      : ASTNode{row, column}, condition{psl::move(condition)}, block{psl::move(block)} {
  }
  bool eval(Context& ctx) const;
  Expr condition;
  PBlock block;
};
struct ElseIf : ASTNode {
  ElseIf() = default;
  ElseIf(size_t row, size_t column, Expr condition, PBlock block)
      : ASTNode{row, column}, condition{psl::move(condition)}, block{psl::move(block)} {
  }
  bool eval(Context& ctx) const;
  Expr condition;
  PBlock block;
};
struct Else {
  Else() = default;
  Else(PBlock block) : block{psl::move(block)} {
  }
  void eval(Context& ctx) const;
  PBlock block;
};
struct IfElseChain {
  IfElseChain() = default;
  IfElseChain(If if_, psl::vector<ElseIf> else_ifs, psl::optional<Else> else_)
      : if_{psl::move(if_)}, else_ifs{psl::move(else_ifs)}, else_{psl::move(else_)} {
  }
  void eval(Context& ctx) const;
  If if_;
  psl::vector<ElseIf> else_ifs;
  psl::optional<Else> else_;
};
struct Block {
  Block() = default;
  Block(psl::vector<PBlock> elems) : elems{psl::move(elems)} {
  }
  void eval(Context& ctx) const;

  psl::vector<PBlock> elems;
};
struct ScopeUnit : psl::Variant<Block, While, For, IfElseChain> {
  using Variant::Variant;
  void eval(Context& ctx) const {
    ctx.scope_push();
    dispatch([&ctx](auto&& x) { x.eval(ctx); });
    ctx.scope_pop();
  }
};
struct FunctionDefinition {
  FunctionDefinition() = default;
  FunctionDefinition(Id id, psl::vector<psl::string> params, Block block)
      : id{psl::move(id)}, params{psl::move(params)}, block{psl::move(block)} {
  }
  void eval(Context& ctx) const;

  Id id;
  psl::vector<psl::string> params;
  Block block;
};

psl::pair<Block, SourceLines> parse_as_block(psl::string source);

}  // namespace pine