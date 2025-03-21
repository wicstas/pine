#include <pine/core/fileio.h>
#include <pine/core/esl.h>
#include <pine/core/log.h>

#include <psl/variant.h>

namespace pine {

namespace {

// ================================================
// Source
// ================================================
struct SourceLoc {
  SourceLoc() = default;
  SourceLoc(size_t row, size_t column) : row(row), column(column) {}
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
    : lines(split(tokens, psl::isnewline)), paddings(paddings) {}
psl::optional<psl::string_view> SourceLines::next_line(size_t row) const {
  if (row == lines.size()) return psl::nullopt;
  return lines[row];
}
psl::optional<char> SourceLines::next(SourceLoc sl) const {
  if (auto line = next_line(sl.row)) return (*line)[sl.column];
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
  for (size_t i = sl.row + 1; i <= sl.row + paddings; i++) vicinity += " | " + line_at(i) + "\n";
  if (vicinity.size()) vicinity.pop_back();

  SEVERE(message, "\n", vicinity);
}

// ================================================
// AST declaration
// ================================================
struct Pragma {
  Pragma(psl::string value) : value(MOVE(value)) {}
  psl::string emit() const { return value; }

  psl::string value;
};
struct Stmt {
  Stmt(psl::string value) : value(MOVE(value)) {}
  psl::string emit() const { return value; }

  psl::string value;
};
struct BlockElem;
struct Block {
  Block(psl::vector<BlockElem> elems) : elems{MOVE(elems)} {}
  psl::string emit() const;

  psl::vector<BlockElem> elems;
};
struct Parameter {
  psl::string name;
  psl::string type;
};
struct FunctionDefinition {
  FunctionDefinition(psl::string name, psl::string rtype, psl::string params, Block body)
      : name(MOVE(name)), rtype(MOVE(rtype)), params(MOVE(params)), body(MOVE(body)) {}

  psl::string emit() const {
    return rtype + " " + name + "(" + params + ") {\n" + body.emit() + "}";
  }
  psl::string name;
  psl::string rtype;
  psl::string params;
  Block body;
};
struct MemberDefinition {
  MemberDefinition(psl::string name, psl::string type) : name(MOVE(name)), type(MOVE(type)) {}

  psl::string emit() const { return type + " " + name + ";"; }
  psl::string name;
  psl::string type;
};
struct ClassDefinition {
  ClassDefinition(psl::string name, psl::vector<FunctionDefinition> ctors,
                  psl::vector<FunctionDefinition> methods, psl::vector<MemberDefinition> members)
      : name(MOVE(name)), ctors(MOVE(ctors)), methods(MOVE(methods)), members(MOVE(members)) {}
  psl::string emit() const {
    auto str = psl::string("struct " + name + " {\n");
    for (auto&& member : members) str += member.emit() + "\n";
    str += "};\n";
    for (auto&& ctor : ctors) str += ctor.emit() + "\n";
    for (auto&& method : methods) str += method.emit() + "\n";
    return str;
  }

  psl::string name;
  psl::vector<FunctionDefinition> ctors;
  psl::vector<FunctionDefinition> methods;
  psl::vector<MemberDefinition> members;
};
struct BlockElem : psl::variant<Pragma, Block, FunctionDefinition, ClassDefinition, Stmt> {
  using variant::variant;

  psl::string emit() const {
    return dispatch([](auto&& x) { return x.emit(); });
  }
};

psl::string Block::emit() const {
  return psl::sum<psl::string>(psl::transform(elems, [](auto& x) { return x.emit() + "\n"; }));
}

// ================================================
// Parser
// ================================================
struct Parser {
  Parser(psl::string_view tokens) : sl(tokens, row_padding) { to_valid_pos(); }

  Block block(bool top_level = false) {
    consume_spaces();

    auto block_elems = psl::vector<BlockElem>{};

    if (top_level)
      accept("{");
    else
      consume("{", "to begin block");

    while (!expect("}") && next()) block_elems.push_back(block_elem());

    if (top_level)
      accept("}");
    else
      consume("}", "to end block");
    return Block{block_elems};
  }

  BlockElem block_elem() {
    if (expect("return")) return stmt();
    if (expect("#")) return pragma();
    if (expect("{")) return block();
    if (expect("struct")) return class_definition();

    backup();
    if (maybe_id() && maybe_id() && accept("(")) {
      undo();
      return function_definition();
    };
    undo();
    return stmt();
  }
  ClassDefinition class_definition() {
    consume("struct", "to start class definition");
    auto name = id();
    consume("{", "to begin class definition");

    auto ctors = psl::vector<FunctionDefinition>();
    auto methods = psl::vector<FunctionDefinition>();
    auto members = psl::vector<MemberDefinition>();

    while (!accept("}")) {
      if (expect(name)) {
        ctors.push_back(ctor_definition(name));
      } else {
        backup();
        if (maybe_id() && maybe_id() && accept("(")) {
          undo();
          methods.push_back(method_definition(name));
        } else {
          undo();
          members.push_back(member_definition());
          consume(";", "to end the previous member definition");
        }
      }
      while (accept(";"));
    }
    consume(";", "to complete class definition");

    return ClassDefinition(MOVE(name), MOVE(ctors), MOVE(methods), MOVE(members));
  }
  MemberDefinition member_definition() {
    auto type = id();
    auto name = id();
    return MemberDefinition(MOVE(name), MOVE(type));
  };
  FunctionDefinition ctor_definition(psl::string class_name) {
    consume(class_name, "to start ctor definiton");
    auto params = param_list();
    auto block_ = block();

    block_.elems.push_front(Stmt(class_name + " self;"));
    block_.elems.push_back(Stmt("return self;"));
    return FunctionDefinition(class_name + "_", class_name, MOVE(params), MOVE(block_));
  }
  FunctionDefinition method_definition(psl::string class_name) {
    auto rtype = id();
    auto name = id();
    auto params = param_list();
    if (params.size() != 0)
      params = class_name + " self, " + params;
    else
      params = class_name + " self";
    auto block_ = block();
    return FunctionDefinition(MOVE(name), MOVE(rtype), MOVE(params), MOVE(block_));
  }
  FunctionDefinition function_definition() {
    auto rtype = id();
    auto name = id();
    auto params = param_list();
    auto block_ = block();
    return FunctionDefinition(MOVE(name), MOVE(rtype), MOVE(params), MOVE(block_));
  }
  psl::string param_list() {
    consume("(", "to begin parameter list");
    auto str = psl::string{};
    while (true) {
      auto n = next();
      if (!n || *n == ')') break;
      str.push_back(*n);
      proceed();
    }
    consume(")", "to end parameter list");
    consume_spaces();
    return str;
  }
  Stmt stmt() {
    auto str = psl::string{};
    while (true) {
      auto n = next();
      if (!n || *n == ';') break;
      str.push_back(*n);
      proceed();
    }
    consume(";", "to end statement");
    consume_spaces();
    return str + ";";
  }
  Pragma pragma() {
    consume("#", "to start pragma definition");
    auto str = psl::string("#");
    auto row_ = row;
    while (true) {
      auto n = next();
      str.push_back(*n);
      proceed();
      if (row != row_) break;
    }
    return str;
  }
  psl::string id() {
    if (!expect(psl::iswordstart, 0)) error("Expect a letter or `_` to start an identifier");
    auto str = psl::string{};
    while (true) {
      str.push_back(*next());
      proceed();
      auto n = next();
      if (!n || !psl::iswordmiddle(*n)) break;
    }
    consume_spaces();
    return str;
  }
  bool maybe_id() {
    if (!expect(psl::iswordstart, 0)) return false;
    while (true) {
      next();
      proceed();
      auto n = next();
      if (!n || !psl::iswordmiddle(*n)) break;
    }
    consume_spaces();
    return true;
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
        if (*n == '/') {
          if (auto nn = next(); nn && *nn == '/') in_comment = true;
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
        if (*n != str[i]) return false;
      } else {
        return false;
      }
    }

    return true;
  }
  bool accept(psl::string_view str, bool remove_tail_spaces = true) {
    if (expect(str)) {
      proceed(str.size());
      if (remove_tail_spaces) consume_spaces();
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

  void backup() { backup_stack.emplace_back(row, column); }
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
  psl::optional<psl::string_view> next_line() const { return sl.next_line(row); }
  psl::optional<char> next() const { return sl.next(source_loc()); }

  SourceLoc source_loc() const { return {row, column}; }
  template <typename... Args>
  [[noreturn]] void error(const Args&... args) {
    sl.error(source_loc(), args...);
  }

  size_t row = 0;
  size_t column = 0;
  psl::vector<psl::pair<size_t, size_t>> backup_stack;
  static constexpr size_t row_padding = 1;
};

}  // namespace

psl::string load_esl(psl::string filename) {
  auto foldername = filename.substr(filename.begin(), psl::find_last_of(filename, '/'));
  auto source = read_string_file(filename);
//   while (true) {
//     auto it = psl::find_subrange(source, psl::string("#include"));
//     if (it == source.end()) break;
//     auto eit = psl::find_if(psl::range(it, source.end()),
//                             psl::isnewline);  // Ok if `eit` == source.end()
//     auto include_filename = psl::string(it + 9, eit);
//     CHECK_EQ(include_filename, "path.frag");
//     it = source.erase_range(it, eit);
//     source.insert_range(it, read_string_file(foldername + "/" + include_filename));
//   }

  auto block = Parser(source).block(true);
  LOG(block.emit());
  SEVERE("Success");
  return source;
}

}  // namespace pine