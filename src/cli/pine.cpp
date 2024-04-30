#include <pine/core/interpreter.h>
#include <pine/core/integrator.h>
#include <pine/core/profiler.h>
#include <pine/core/context.h>
#include <pine/core/fileio.h>

#include <future>

int runJit();

int main(int argc, char* argv[]) {
  using namespace pine;

  return runJit();

  if (argc != 2) {
    Log("Usage: pine [filename]");
    return 0;
  }

#ifndef NDEBUG
  Warning("[Performance]Debug build");
#endif

  Profiler::Initialize();

  try {
    auto context = get_default_context();

    auto task = std::async(std::launch::async, [&]() {
      try {
        interpret_file(context, argv[1]);
      } catch (const std::exception& e) {
        Log(e.what());
      } catch (const psl::Exception& e) {
        Log(e.what());
      }
    });
    while (true) {
      if (get_progress() != 0)
        Logr(get_progress(), "\r");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (task.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
        break;
    }
    Log("");
  } catch (const std::exception& e) {
    Log(e.what());
  } catch (const psl::Exception& e) {
    Log(e.what());
  }

  Profiler::Finalize();

  return 0;
}

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
#include <algorithm>
#include <cassert>
#include <memory>
#include <vector>

using namespace llvm;

int runJit() {
  InitializeNativeTarget();
  LLVMInitializeNativeAsmPrinter();

  auto C = LLVMContext();
  auto module_ = std::make_unique<Module>("test", C);
  auto M = module_.get();

  //
  auto add = Function::Create(
      FunctionType::get(Type::getInt32Ty(C), {Type::getInt32Ty(C), Type::getInt32Ty(C)}, false),
      Function::ExternalLinkage, "add", M);
  auto BB = BasicBlock::Create(C, "entry", add);
  auto builder = IRBuilder<>(BB);

  auto arg0 = &add->arg_begin()[0];
  auto arg1 = &add->arg_begin()[1];
  auto res = builder.CreateAdd(arg0, arg1);
  builder.CreateRet(res);

  auto myStruct = StructType::create({Type::getInt32Ty(C), Type::getFloatTy(C)}, "myStruct");
  myStruct->dump();

  //
  auto greet = Function::Create(FunctionType::get(Type::getVoidTy(C), {}, false),
                                Function::ExternalLinkage, "greet", M);
  llvm::sys::DynamicLibrary::AddSymbol(
      "greet", (void*)+[]() { pine::Log("Hello!"); });

  //
  Function* foo = Function::Create(FunctionType::get(Type::getInt32Ty(C), {}, false),
                                   Function::ExternalLinkage, "foo", M);
  BB = BasicBlock::Create(C, "EntryBlock", foo);
  builder.SetInsertPoint(BB);

  auto val0 = builder.getInt32(10);
  auto val1 = builder.getInt32(20);

  builder.CreateCall(greet);
  auto val2 = builder.CreateCall(add, {val0, val1});
  val2->setTailCall(true);
  builder.CreateRet(val2);

  //
  auto EE = psl::unique_ptr<ExecutionEngine>(EngineBuilder(std::move(module_)).create());
  auto foo_ = (int (*)())(void*)EE->getFunctionAddress("foo");
  pine::Log(foo_());

  //
  llvm_shutdown();
  return 0;
}