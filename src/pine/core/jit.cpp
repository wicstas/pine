#include <pine/core/jit_compiler.h>
#include <pine/core/jit.h>

namespace pine {

// using namespace llvm;

// struct Module {
//   void add_type(llvm::Module* M, psl::string name, llvm::Type* type, size_t byte_size,
//                 void* copy_operator) {
//     if (copy_operator) {
//       auto c_name = "copy_" + name;
//       llvm::sys::DynamicLibrary::AddSymbol(c_name.c_str(), copy_operator);
//       types[name] = {type, byte_size,
//                      llvm::Function::Create(FunctionType::get(type, {type}, false),
//                                             llvm::Function::ExternalLinkage, c_name.c_str(), M)};
//     } else {
//       types[name] = {type, byte_size, nullptr};
//     }
//   }
//   void add_function(psl::string name, llvm::Function* function, size_t param_byte_size) {
//     functions[name] = {function, param_byte_size};
//   }
//   void add_function_object(LLVMContext& C, llvm::Module* M, const Function& f) {
//     auto byte_type = Type::getInt8Ty(C);
//     auto type = ArrayType::get(byte_type, f.byte_size());

//     auto ptr = (uint8_t*)f.object_ptr();
//     auto values = std::vector<Constant*>(f.byte_size());
//     for (uint32_t i = 0; i < f.byte_size(); i++)
//       values[i] = ConstantInt::get(byte_type, ptr[i]);
//     auto value = ConstantArray::get(type, values);

//     function_objects.push_back(
//         new GlobalVariable(*M, type, true, GlobalValue::PrivateLinkage, value));
//   }

//   llvm::Value* function_object(size_t i) const {
//     return function_objects[i];
//   }
//   llvm::Function* function(psl::string_view name) const {
//     if (auto it = functions.find(name); it != functions.end())
//       return it->second.ptr;
//     else
//       Fatal("Function `", name, "` is not found");
//   }
//   size_t function_param_byte_size(psl::string_view name) const {
//     if (auto it = functions.find(name); it != functions.end())
//       return it->second.param_byte_size;
//     else
//       Fatal("Function `", name, "` is not found");
//   }

//   llvm::Type* type(psl::string_view name) const {
//     if (auto it = types.find(name); it != types.end())
//       return it->second.ptr;
//     else
//       Fatal("Type `", name, "` is not found");
//   }
//   size_t type_size(psl::string_view name) const {
//     if (auto it = types.find(name); it != types.end())
//       return it->second.byte_size;
//     else
//       Fatal("Type `", name, "` is not found");
//   }
//   llvm::Function* copy_operator(psl::string_view name) const {
//     if (auto it = types.find(name); it != types.end()) {
//       if (it->second.copy_operator)
//         return it->second.copy_operator;
//       else
//         Fatal("Type `", name, "` is not copyable");
//     } else {
//       Fatal("Type `", name, "` is not found");
//     }
//   }

// private:
//   psl::vector<llvm::Value*> function_objects;
//   struct FunctionInfo {
//     llvm::Function* ptr = nullptr;
//     size_t param_byte_size = 0;
//   };
//   psl::map<psl::string, FunctionInfo> functions;
//   struct TypeInfo {
//     llvm::Type* ptr = nullptr;
//     size_t byte_size = 0;
//     llvm::Function* copy_operator = nullptr;
//   };
//   psl::map<psl::string, TypeInfo> types;
// };

// PointerType* packed(LLVMContext& C, size_t bytes) {
//   return PointerType::get(ArrayType::get(Type::getInt8Ty(C), bytes), 0);
// }

// void translate_module(const Context& ctx, Module& mod, LLVMContext& C, llvm::Module* M) {
//   for (const auto& [name, type] : ctx.types) {
//     if (name == "i32")
//       mod.add_type(M, name, Type::getInt32Ty(C), type.byte_size(), type.copy_operator);
//     else if (name == "f32")
//       mod.add_type(M, name, Type::getFloatTy(C), type.byte_size(), type.copy_operator);
//     else
//       mod.add_type(M, name, ArrayType::get(Type::getInt8Ty(C), type.byte_size()),
//       type.byte_size(),
//                    type.copy_operator);
//   }

//   for (const auto& f : ctx.functions) {
//     auto rtype = Type::getVoidTy(C);
//     auto param_byte_size = f.byte_size() + mod.type_size(f.rtype().name) +
//                            psl::sum<size_t>(psl::transform(
//                                f.ptypes(), [&](auto&& x) { return mod.type_size(x.name); }));
//     auto ptype = packed(C, param_byte_size);
//     auto function = llvm::Function::Create(FunctionType::get(rtype, ptype, false),
//                                            llvm::Function::ExternalLinkage, f.name().c_str(), M);
//     llvm::sys::DynamicLibrary::AddSymbol(f.name().c_str(), f.ptr());
//     mod.add_function(f.signature(), function, param_byte_size);
//     mod.add_function_object(C, M, f);
//   }
//   M->dump();
// }

// llvm::Function* translate_bytecode(psl::string func_name, const Context& ctx,
//                                    const Bytecodes& bcodes, const Module& mod, LLVMContext& C,
//                                    llvm::Module* M) {
//   auto main = llvm::Function::Create(FunctionType::get(Type::getVoidTy(C), {}, false),
//                                      llvm::Function::ExternalLinkage, func_name.c_str(), M);
//   auto BB = BasicBlock::Create(C, "entry", main);
//   auto builder = IRBuilder<>(BB);

//   struct Var {
//     Value* value;
//     size_t byte_size;
//     psl::string type_name;
//   };
//   auto stack = psl::vector<Var>();

//   for (const auto& code : bcodes) {
//     auto push = [&](Var x) {
//       if (code.value1 != size_t(-1))
//         stack.push_back(MOVE(x));
//     };
//     switch (code.instruction) {
//       case Bytecode::Break: break;
//       case Bytecode::Continue: break;
//       case Bytecode::Return:
//         if (code.value == size_t(-1))
//           builder.CreateRetVoid();
//         else
//           builder.CreateRet(stack[code.value].value);
//         break;
//       case Bytecode::LoadGlobalVar: break;
//       case Bytecode::LoadFunction: break;
//       case Bytecode::Copy: {
//         // auto type_name = stack[code.value].type_name;
//         // push({builder.CreateCall(mod.copy_operator(type_name), stack[code.value].value),
//         //       stack[code.value].byte_size, type_name});
//         push(stack[code.value]);
//       } break;
//       case Bytecode::MakeRef: push(stack[code.value]); break;
//       case Bytecode::LoadFloatConstant: {
//         auto var = builder.CreateAlloca(Type::getFloatTy(C));
//         builder.CreateStore(
//             ConstantFP::get(Type::getFloatTy(C), psl::bitcast<float>(uint32_t(code.value))),
//             var);
//         push({var, 4, "f32"});
//       } break;
//       case Bytecode::LoadIntConstant: {
//         auto var = builder.CreateAlloca(Type::getInt32Ty(C));
//         builder.CreateStore(builder.getInt32(psl::bitcast<int>(uint32_t(code.value))), var);
//         push({var, 4, "i32"});
//       } break;
//       case Bytecode::LoadBoolConstant: {
//         auto var = builder.CreateAlloca(Type::getInt8Ty(C));
//         builder.CreateStore(builder.getInt8(uint8_t(code.value)), var);
//         push({var, 1, "bool"});
//       } break;
//       case Bytecode::LoadStringConstant: break;
//       case Bytecode::Call: {
//         auto&& f = ctx.functions[code.value];
//         auto param_type =
//             ArrayType::get(builder.getInt8Ty(), mod.function_param_byte_size(f.signature()));
//         auto param = builder.CreateAlloca(param_type);
//         builder.CreateMemCpy(param, std::nullopt, mod.function_object(code.value), std::nullopt,
//                              f.byte_size());
//         auto offset = f.byte_size() + mod.type_size(f.rtype().name);
//         for (int i = 0; i < code.nargs; i++) {
//           auto ptr =
//               builder.CreateGEP(param_type, param, {builder.getInt32(0),
//               builder.getInt32(offset)});
//           builder.CreateMemCpy(ptr, std::nullopt, stack[code.args[i]].value, std::nullopt,
//                                stack[code.args[i]].byte_size);
//           offset += stack[code.args[i]].byte_size;
//         }
//         builder.CreateCall(mod.function(f.signature()), {param});
//         auto res = builder.CreateAlloca(mod.type(f.rtype().name));
//         auto ptr = builder.CreateGEP(param_type, param,
//                                      {builder.getInt32(0), builder.getInt32(f.byte_size())});
//         builder.CreateMemCpy(res, std::nullopt, ptr, std::nullopt,
//         mod.type_size(f.rtype().name)); push({res, mod.type_size(f.rtype().name),
//         f.rtype().name});
//       } break;
//       case Bytecode::InvokeAsFunction: break;
//       case Bytecode::Jump: break;
//       case Bytecode::JumpIfNot: break;
//       case Bytecode::UnwindStack: break;
//     }
//   }
//   builder.CreateRetVoid();
//   main->dump();

//   return main;
// }

}  // namespace pine
