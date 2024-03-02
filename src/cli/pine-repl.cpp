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
      Log(e.what());
    } catch (const FallthroughException& e) {
      Log(e.what());
    } catch (const std::exception& e) {
      Log(e.what());
    }
  }

  return 0;
}
