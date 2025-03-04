#include <pine/core/integrator.h>
#include <pine/core/compiler.h>
#include <pine/core/context.h>
#include <pine/core/parser.h>

#include <iostream>

int main() {
  using namespace pine;

  auto context = get_default_context();
  auto bytecodes = Bytecodes({});
  auto vm = VirtualMachine();
  while (true) {
    std::string source;
    if (!std::getline(std::cin, source))
      break;
    try {
      compile(context, source.c_str(), bytecodes);
      execute(context, bytecodes, vm);
    } catch (const Exception& e) {
      LOG(e.what());
    } catch (const FallthroughException& e) {
      LOG(e.what());
    } catch (const std::exception& e) {
      LOG(e.what());
    }
  }

  return 0;
}
