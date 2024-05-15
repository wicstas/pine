#pragma once

#include <pine/core/context.h>

#include <psl/optional.h>
#include <psl/memory.h>
#include <psl/array.h>

namespace pine {

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

struct Bytecode {
  enum Instruction {
    Copy,
    MakeRef,
    LoadFunction,
    LoadGlobalVar,
    LoadFloatConstant,
    LoadIntConstant,
    LoadBoolConstant,
    LoadStringConstant,
    Call,
    InvokeAsFunction,
    Return,
    Break,
    Continue,
    Jump,
    JumpIfNot,
    UnwindStack
  };

  Bytecode() = default;
  Bytecode(SourceLoc sl, Instruction inst, size_t value, size_t value1 = 0)
      : sl(sl), instruction(inst), value(value), value1(value1) {
  }
  Bytecode(SourceLoc sl, Instruction inst, size_t value, size_t value1, psl::span<uint16_t> args)
      : sl(sl), instruction(inst), value(value), value1(value1) {
    psl::copy(this->args.begin(), args);
    nargs = int(args.size());
  }

  SourceLoc sl;
  Instruction instruction;
  size_t value = size_t(-1);
  size_t value1 = size_t(-1);
  psl::Array<uint16_t, 8> args;
  int nargs = 0;
};

struct Bytecodes {
  Bytecodes(SourceLines sl);

  size_t add(SourceLoc sl, Bytecode::Instruction instruction, size_t value0, size_t value1 = 0);
  void placehold_typed(TypeTag type, psl::string name);
  void add_typed(SourceLoc sl, Bytecode::Instruction instruction, TypeTag type, size_t value,
                 psl::span<uint16_t> args = {});
  void add_typed(SourceLoc sl, Bytecode::Instruction instruction, TypeTag type, size_t value,
                 uint16_t arg0) {
    add_typed(sl, instruction, type, value, psl::array_of(arg0));
  }
  void add_typed(SourceLoc sl, Bytecode::Instruction instruction, TypeTag type, size_t value,
                 uint16_t arg0, uint16_t arg1) {
    add_typed(sl, instruction, type, value, psl::array_of(arg0, arg1));
  }

  void name_top_var(psl::string name);
  const TypeTag& var_type(size_t var_index) const;
  uint16_t var_index_by_name(psl::string_view name) const;
  uint16_t top_var_index() const;
  void pop_stack();
  size_t stack_size() const;

  size_t add_string(psl::string value);
  const psl::string& get_string(size_t i) const;

  void enter_scope();
  void exit_scope();

  auto begin() const {
    return codes.begin();
  }
  auto end() const {
    return codes.end();
  }
  size_t length() const {
    return codes.size();
  }
  size_t code_position() const {
    return codes.size();
  }

  Bytecode& operator[](size_t i) {
    return codes[i];
  }
  const Bytecode& operator[](size_t i) const {
    return codes[i];
  }

  template <typename... Args>
  [[noreturn]] void error(SourceLoc loc, const Args&... args) const {
    sl.error(loc, args...);
  }

  psl::string to_string(const Context& context) const;

private:
  struct ScopeInfo {
    size_t type_stack_position;
    size_t variable_map_position;
  };
  psl::vector<Bytecode> codes;
  psl::vector<psl::pair<psl::string, size_t>> variable_map;
  psl::vector<TypeTag> stack;
  psl::vector<ScopeInfo> scope_stack;
  psl::vector<psl::string> string_region;

public:
  SourceLines sl;
};

Bytecodes compile(Context& context, psl::string source);
void compile(Context& context, psl::string source, Bytecodes& bytecodes);

template <typename T>
struct Stack {
  void push(T x) {
    storage.push_back(MOVE(x));
  }
  T& top() {
    DCHECK(storage.size() != 0);
    return storage.back();
  }
  T& top(size_t index) {
    DCHECK_LE(index + 1, storage.size());
    return storage[storage.size() - (1 + index)];
  }
  psl::span<T> top_n(int n) {
    DCHECK_GE(n, 0);
    auto span = psl::span<T>(storage);
    return span.subspan(span.size() - n, n);
  }
  void unwind(size_t position) {
    if (position == size_t(-1) || position == storage.size())
      return;
    DCHECK_LE(position, storage.size());
    storage.resize_less(position);
  }
  T& operator[](size_t index) {
    DCHECK_LT(index, storage.size());
    return storage[index];
  }
  void reserve(size_t size) {
    storage.reserve(size);
  }
  size_t size() const {
    return storage.size();
  }
  auto begin() {
    return storage.begin();
  }
  auto begin() const {
    return storage.begin();
  }
  auto end() {
    return storage.end();
  }
  auto end() const {
    return storage.end();
  }

private:
  psl::vector<T> storage;
};

struct VirtualMachine {
  Stack<Variable> stack;
};

Variable execute(const Context& context, const Bytecodes& bytecodes, VirtualMachine& vm);
Variable execute(const Context& context, const Bytecodes& bytecodes);

}  // namespace pine