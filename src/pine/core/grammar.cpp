#include <pine/core/profiler.h>
#include <pine/core/grammar.h>

namespace pine {

Variable& Id::eval(Context& ctx) const {
  try {
    return ctx[value];
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }
  PINE_UNREACHABLE;
}
Variable Vector::eval(Context& ctx) const {
  try {
    if (args.size() == 2) {
      auto x = args[0].eval(ctx);
      auto y = args[1].eval(ctx);
      if (x.is<float>() || y.is<float>())
        return vec2(x.as<float>(), y.as<float>());
      else
        return vec2i(x.as<int>(), y.as<int>());
    } else if (args.size() == 3) {
      auto x = args[0].eval(ctx);
      auto y = args[1].eval(ctx);
      auto z = args[2].eval(ctx);
      if (x.is<float>() || y.is<float>() || z.is<float>())
        return vec3(x.as<float>(), y.as<float>(), z.as<float>());
      else
        return vec3i(x.as<int>(), y.as<int>(), z.as<int>());
    } else {
      exception("Only 2 or 3 elements can appear inside [], found ", args.size());
    }
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }

  PINE_UNREACHABLE;
}
Variable FunctionCall::eval(Context& ctx, psl::optional<Variable> receiver) const {
  try {
    auto args_ = psl::vector<Variable>{};
    if (receiver)
      args_.push_back(*receiver);
    args_.insert(psl::end(args_), psl::transform(args, [&ctx](auto&& x) { return x.eval(ctx); }));
    return ctx.call1(id.value, args_);
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }

  PINE_UNREACHABLE;
}
MemberAccess::MemberAccess(size_t row, size_t column, PExpr pexpr, Id id)
    : ASTNode{row, column}, pexpr{psl::move(pexpr)}, id{psl::move(id)} {
}
Variable MemberAccess::eval(Context& ctx) const {
  try {
    return pexpr->eval(ctx)[id.value];
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
    PINE_UNREACHABLE;
  }
}
Subscript::Subscript(size_t row, size_t column, PExpr pexpr, Expr index)
    : ASTNode{row, column}, pexpr{psl::move(pexpr)}, index{psl::move(index)} {
}
Variable Subscript::eval(Context& ctx) const {
  try {
    return ctx.call("[]", pexpr->eval(ctx), index.eval(ctx));
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
    PINE_UNREACHABLE;
  }
}
Variable PExpr::eval(Context& ctx) const {
  return dispatch([&](const auto& x) { return x.eval(ctx); });
}
Expr::Expr(size_t row, size_t column, Expr0 x) : ASTNode{row, column}, op{None}, x{psl::move(x)} {
}
Expr::Expr(Expr a, Expr b, Op op) : ASTNode{a}, op{op}, a{psl::move(a)}, b{psl::move(b)} {
}
Variable Expr::eval(Context& ctx) const {
  if (op == None) {
    CHECK(!!x);
  } else {
    CHECK(a && b);
  }
  try {
    switch (op) {
      case None: return x->eval(ctx);
      case Pow: return ctx.call("^", a->eval(ctx), b->eval(ctx));
      case Mul: return ctx.call("*", a->eval(ctx), b->eval(ctx));
      case Div: return ctx.call("/", a->eval(ctx), b->eval(ctx));
      case Mod: return ctx.call("%", a->eval(ctx), b->eval(ctx));
      case Add: return ctx.call("+", a->eval(ctx), b->eval(ctx));
      case Sub: return ctx.call("-", a->eval(ctx), b->eval(ctx));
      case Lt: return ctx.call("<", a->eval(ctx), b->eval(ctx));
      case Gt: return ctx.call(">", a->eval(ctx), b->eval(ctx));
      case Le: return ctx.call("<=", a->eval(ctx), b->eval(ctx));
      case Ge: return ctx.call(">=", a->eval(ctx), b->eval(ctx));
      case Eq: return ctx.call("==", a->eval(ctx), b->eval(ctx));
      case Ne: return ctx.call("!=", a->eval(ctx), b->eval(ctx));
      case And: return ctx.call("&&", a->eval(ctx), b->eval(ctx));
      case Or: return ctx.call("||", a->eval(ctx), b->eval(ctx));
      case AddE: return ctx.call("+=", a->eval(ctx), b->eval(ctx));
      case SubE: return ctx.call("-=", a->eval(ctx), b->eval(ctx));
      case MulE: return ctx.call("*=", a->eval(ctx), b->eval(ctx));
      case DivE: return ctx.call("/=", a->eval(ctx), b->eval(ctx));
      case ModE: return ctx.call("%=", a->eval(ctx), b->eval(ctx));
      case Assi: return ctx.call("=", a->eval(ctx), b->eval(ctx));
    }
    Fatal("Should never reach here");
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }
  PINE_UNREACHABLE;
}

Variable Expr0::eval(Context& ctx) const {
  try {
    switch (op) {
      case None: return x.eval(ctx);
      case PreInc: return ctx.call("++x", x.eval(ctx));
      case PreDec: return ctx.call("--x", x.eval(ctx));
      case PostInc: return ctx.call("x++", x.eval(ctx));
      case PostDec: return ctx.call("x++", x.eval(ctx));
      case Positive: return ctx.call("+x", x.eval(ctx));
      case Negate: return ctx.call("-x", x.eval(ctx));
      case Invert: return ctx.call("!x", x.eval(ctx));
    }
    Fatal("Should never reach here");
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }
  PINE_UNREACHABLE;
}

void Declaration::eval(Context& ctx) const {
  ctx(id.value) = expr.eval(ctx).make_copy();
}
void ReturnStmt::eval(Context& ctx) const {
  ctx.set_return_value(expr.eval(ctx));
}
void Stmt::eval(Context& ctx) const {
  dispatch([&ctx](auto&& x) { x.eval(ctx); });
}
void While::eval(Context& ctx) const {
  try {
    while (condition.eval(ctx).as<bool>() && !ctx.consume_break_flag())
      block.eval(ctx);
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }
}
void For::eval(Context& ctx) const {
  try {
    for (init.eval(ctx); condition.eval(ctx).as<bool>() && !ctx.consume_break_flag(); inc.eval(ctx))
      block.eval(ctx);
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
  }
}
bool If::eval(Context& ctx) const {
  try {
    if (condition.eval(ctx).as<bool>()) {
      block.eval(ctx);
      return true;
    }
    return false;
  } catch (const Exception& e) {
    ctx.error(row, column, e.what());
    PINE_UNREACHABLE;
  }
}
bool ElseIf::eval(Context& ctx) const {
  if (condition.eval(ctx).as<bool>()) {
    block.eval(ctx);
    return true;
  }
  return false;
}
void Else::eval(Context& ctx) const {
  block.eval(ctx);
}
void IfElseChain::eval(Context& ctx) const {
  if (if_.eval(ctx)) {
  } else {
    for (const auto& else_if : else_ifs)
      if (else_if.eval(ctx))
        return;
    if (else_)
      else_->eval(ctx);
  }
}
void Block::eval(Context& ctx) const {
  for (const auto& elem : elems) {
    elem.eval(ctx);
    if (ctx.returned())
      break;
    if (ctx.consume_continue_flag())
      break;
    if (ctx.consume_break_flag()) {
      ctx.set_break_flag();
      break;
    }
  }
}
void FunctionDefinition::eval(Context& ctx) const {
  ctx(id.value) = tag<Variable, const psl::vector<Variable>&>(
      [&ctx, id = id, params = params, block = block](const psl::vector<Variable>& args) {
        if (args.size() != params.size())
          exception("Function `", id.value, "` is called with incompatible arguments");
        ctx.scope_push();
        for (size_t i = 0; i < params.size(); i++)
          ctx(params[i]) = args[i];
        block.eval(ctx);
        ctx.scope_pop();
        return ctx.consume_return_value();
      });
}
PBlock::PBlock() = default;
PBlock::~PBlock() = default;
PBlock::PBlock(const PBlock&) = default;
PBlock::PBlock(PBlock&&) = default;
PBlock& PBlock::operator=(const PBlock&) = default;
PBlock& PBlock::operator=(PBlock&&) = default;
PBlock::PBlock(Semicolon x) : base(psl::move(x)) {
}
PBlock::PBlock(Stmt x) : base(psl::move(x)) {
}
PBlock::PBlock(ScopeUnit x) : base(psl::move(x)) {
}
PBlock::PBlock(FunctionDefinition x) : base(psl::move(x)) {
}
void PBlock::eval(Context& ctx) const {
  base->dispatch([&ctx](auto&& x) { x.eval(ctx); });
}

template <typename F>
struct ScopeGuard {
  ScopeGuard(F f) : f{psl::move(f)} {
  }
  ~ScopeGuard() {
    f();
  }
  ScopeGuard(ScopeGuard&&) = delete;
  ScopeGuard& operator=(ScopeGuard&&) = delete;

private:
  F f;
};

struct Parser {
  Parser(psl::string_view tokens) : sl{tokens, row_padding} {
    to_valid_pos();
  }

  Block block(bool top_level = false) {
    consume_spaces();

    if (top_level)
      accept("{");
    else
      consume("{");

    auto pblocks = psl::vector<PBlock>{};
    while (!expect("}") && next())
      pblocks.push_back(pblock());

    if (top_level)
      accept("}");
    else
      consume("}");
    return Block{pblocks};
  }

  PBlock pblock() {
    if (accept(";"))
      return PBlock{Semicolon{}};
    else if (expect("fun"))
      return PBlock{function_definition()};
    else if (expect("{") || expect("while") || expect("for") || expect("if"))
      return PBlock{scope_unit()};
    else
      return PBlock{stmt()};
  }

  FunctionDefinition function_definition() {
    consume("fun");
    auto id_ = id();
    consume("(");
    auto args = psl::vector<psl::string>{};
    if (!accept(")"))
      while (true) {
        args.push_back(id().value);
        if (accept(")"))
          break;
        else
          consume(",");
      }
    return FunctionDefinition{psl::move(id_), psl::move(args), block()};
  }

  ScopeUnit scope_unit() {
    if (expect("{"))
      return ScopeUnit{block()};
    else if (expect("while"))
      return ScopeUnit{while_()};
    else if (expect("for"))
      return ScopeUnit{for_()};
    else if (expect("if"))
      return ScopeUnit{if_else_chain()};
    else {
      error("Expect either: `{`, `while`, `for`, or `if`");
      PINE_UNREACHABLE;
    }
  }

  Stmt stmt() {
    auto stmt = Stmt{};
    if (expect("var"))
      stmt = Stmt{declaration(), Semicolon{}};
    else if (expect("break"))
      stmt = Stmt{break_stmt(), Semicolon{}};
    else if (expect("continue"))
      stmt = Stmt{continue_stmt(), Semicolon{}};
    else if (expect("return"))
      stmt = Stmt{return_stmt(), Semicolon{}};
    else
      stmt = Stmt{expr(), Semicolon{}};
    consume(";");
    return stmt;
  }

  Declaration declaration() {
    consume("var");
    auto id_ = id();
    consume("=");
    auto expr_ = expr();
    return Declaration{psl::move(id_), psl::move(expr_)};
  }
  BreakStmt break_stmt() {
    consume("break");
    return BreakStmt{};
  }
  ContinueStmt continue_stmt() {
    consume("continue");
    return ContinueStmt{};
  }
  ReturnStmt return_stmt() {
    consume("return");
    return ReturnStmt{expr()};
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
    return IfElseChain{if_clause, else_ifs, else_clause};
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
    consume("(");
    auto init = stmt();
    auto r = row, c = column;
    auto cond = expr();
    consume(";");
    auto inc = expr();
    consume(")");
    auto body = pblock();
    return For{r, c, psl::move(init), psl::move(cond), psl::move(inc), psl::move(body)};
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
  Expr expr() {
    auto exprs = psl::vector<Expr>{};
    auto ops = psl::vector<int>{};
    if (accept("(")) {
      exprs.push_back(expr());
      accept(")");
    } else {
      exprs.push_back(Expr{row, column, expr0()});
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
        exprs.push_back(Expr{row, column, expr0()});
      }
    }

    while (ops.size() != 0) {
      auto max_precedence = 0;
      auto index = size_t{0};
      for (size_t i = 0; i < ops.size(); i++) {
        if (ops[i] > max_precedence) {
          max_precedence = ops[i];
          index = i;
        }
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
        auto index = expr();
        consume("]");
        pexpr_ = Subscript{row, column, psl::move(pexpr_), psl::move(index)};
      } else if (accept(".")) {
        auto id_ = id();
        pexpr_ = MemberAccess{row, column, psl::move(pexpr_), psl::move(id_)};
      } else if (accept("(")) {
        if (pexpr_.is<Id>()) {
          pexpr_ = FunctionCall{pexpr_.be<Id>().row, pexpr_.be<Id>().column, pexpr_.be<Id>(),
                                arg_list()};
        } else if (pexpr_.is<MemberAccess>()) {
          auto& p = pexpr_.be<MemberAccess>();
          auto args = arg_list();
          auto id_ = psl::move(p.id);
          args.push_front(
              Expr{p.row, p.column, Expr0{p.row, p.column, *psl::move(p.pexpr), Expr0::None}});
          pexpr_ = FunctionCall{p.id.row, p.id.column, psl::move(id_), psl::move(args)};
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
    else if (expect("["))
      return PExpr{vector()};
    else if (expect("("))
      return PExpr{grouped()};
    else if (expect(psl::isdigit, 0) || expect("-") || expect("."))
      return PExpr{number()};
    else if (expect(psl::isalpha, 0) || expect("_"))
      return PExpr{id()};
    else {
      error("Expect a primary expression");
      PINE_UNREACHABLE;
    }
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
          consume(",");
      }
    return Vector{r, c, psl::move(args)};
  }
  psl::vector<Expr> arg_list() {
    auto args = psl::vector<Expr>{};
    if (!expect(")"))
      while (true) {
        args.push_back(expr());
        if (expect(")"))
          break;
        else
          consume(",");
      }
    return args;
  }
  Grouped grouped() {
    consume("(");
    auto expr_ = expr();
    consume(")");
    return Grouped{psl::move(expr_)};
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
  Number number() {
    if (!expect(psl::isdigit, 0) && !expect("-") && !expect("."))
      error("Expect a digit, `-`, or `.` to start a number");
    auto pass_decimal_point = false;
    auto str = psl::string{};
    while (true) {
      str.push_back(*next());
      proceed();
      if (str.back() == '.')
        pass_decimal_point = true;
      auto n = next();
      if (!n || !((psl::isdigit(*n) || (!pass_decimal_point && *n == '.'))))
        break;
    }
    consume_spaces();
    return Number{psl::move(str)};
  }

  StringLiteral string_literal() {
    auto result = StringLiteral{};
    consume("\"", false);
    auto escape = false;
    while (auto n = next()) {
      if (escape) {
        if (*n == 'n')
          result.value.back() = '\n';
        else if (*n == 't')
          result.value.back() = '\t';
        else if (*n == '"')
          result.value.back() = '"';
        else
          error("Unknown escape character");
        escape = false;
        proceed();
        continue;
      }
      if (*n == '"')
        break;
      proceed();
      result.value.push_back(*n);
      escape = *n == '\\';
    }
    consume("\"");
    return result;
  };

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
  void consume(psl::string_view str, bool remove_tail_spaces = true) {
    for (char c : str) {
      if (auto n = next()) {
        if (*n != c)
          error("Expect `", c, "`, get `", *n, "`");
        else
          proceed();
      } else {
        error("Expect `", c, "`, but reach end-of-file");
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
  void error(const Args&... args) {
    sl.error(row, column, args...);
  }

  size_t row = row_padding;
  size_t column = 0;
  psl::vector<psl::pair<size_t, size_t>> backup_stack;
  static constexpr size_t row_padding = 1;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};

psl::pair<Block, SourceLines> parse_as_block(psl::string source) {
  Profiler _("Parse source file");
  auto parser = Parser{source};
  return {parser.block(true), psl::move(parser.sl)};
}

}  // namespace pine