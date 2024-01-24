#pragma once

#include <pine/core/context.h>

#include <pine/psl/optional.h>
#include <pine/psl/memory.h>
#include <pine/psl/vector.h>
#include <pine/psl/string.h>
#include <pine/psl/array.h>

namespace pine {

struct SourceLines {
  SourceLines() = default;
  SourceLines(psl::string_view tokens, size_t lines_padding);

  psl::optional<psl::string_view> next_line(size_t row) const;

  psl::optional<char> next(size_t row, size_t column) const;

  template <typename... Args>
  [[noreturn]] void error(size_t row, size_t column, const Args&... args) const {
    error_impl(row, column, psl::to_string(args...));
  }

  [[noreturn]] void error_impl(size_t row, size_t column, psl::string_view message) const;

private:
  psl::vector<psl::string> lines;
  size_t lines_padding = invalid;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};

struct ASTNode {
  ASTNode() = default;
  ASTNode(size_t row, size_t column) : row{row}, column{column} {
  }

  size_t row = invalid;
  size_t column = invalid;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};

struct Bytecode {
  enum Instruction {
    Load,
    Copy,
    LoadFloatConstant,
    LoadIntConstant,
    LoadBoolConstant,
    LoadStringConstant,
    Call,
    IntPreInc,
    IntPreDec,
    IntPostInc,
    IntPostDec,
    IntPositive,
    IntNegate,
    IntEq,
    IntNe,
    IntLt,
    IntGt,
    IntLe,
    IntGe,
    IntAdd,
    IntSub,
    IntMul,
    IntDiv,
    IntPow,
    IntMod,
    IntAddE,
    IntSubE,
    IntMulE,
    IntDivE,
    IntModE,
    IntAssi,
    FloatPositive,
    FloatNegate,
    FloatEq,
    FloatNe,
    FloatLt,
    FloatGt,
    FloatLe,
    FloatGe,
    FloatAdd,
    FloatSub,
    FloatMul,
    FloatDiv,
    FloatPow,
    FloatAddE,
    FloatSubE,
    FloatMulE,
    FloatDivE,
    FloatAssi,
    Break,
    Continue,
    Jump,
    JumpIfNot,
    UnwindStack
  };

  Bytecode() = default;
  Bytecode(Instruction inst, size_t value, size_t value1 = 0)
      : instruction(inst), value(value), value1(value1) {
  }
  Bytecode(Instruction inst, size_t value, size_t value1, psl::Array<uint16_t, 8> args)
      : instruction(inst), value(value), value1(value1), args{args} {
  }

  Instruction instruction;
  size_t value = size_t(-1);
  size_t value1 = size_t(-1);
  psl::Array<uint16_t, 8> args;
};

struct Bytecodes {
  Bytecodes(const Context& context, SourceLines sl);

  size_t add(Bytecode::Instruction instruction, size_t value0, size_t value1 = 0);
  void add_typed(Bytecode::Instruction instruction, size_t value, size_t type_id,
                 psl::Array<uint16_t, 8> args = {});

  void name_top_var(psl::string name);
  size_t get_var_type(size_t var_index) const;
  uint16_t get_var_by_name(psl::string_view name) const;
  uint16_t last_var() const;
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
  [[noreturn]] void error(const ASTNode& node, const Args&... args) const {
    sl.error(node.row, node.column, args...);
  }

  psl::vector<Function> functions;
  psl::vector<Variable> variables;

private:
  struct ScopeInfo {
    size_t type_stack_position;
    size_t variable_map_position;
  };
  psl::vector<Bytecode> codes;
  psl::vector<psl::pair<psl::string, size_t>> variable_map;
  psl::vector<size_t> stack;
  psl::vector<ScopeInfo> scope_stack;
  psl::vector<psl::string> string_region;
  SourceLines sl;
};
psl::string to_string(const Context& context, const Bytecodes& bcodes);

Bytecodes compile(Context& context, psl::string source);

void execute(const Bytecodes& bytecodes);

}  // namespace pine