#pragma once
#include <pine/core/context.h>

#include <psl/optional.h>
#include <psl/memory.h>
#include <psl/array.h>

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

void jit_compile(Context& context, psl::string source, llvm::LLVMContext& C, llvm::Module* M,
                 llvm::Function* F);

void jit_interpret(Context& ctx, psl::string source);

}  // namespace pine