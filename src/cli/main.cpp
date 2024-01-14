#include <pine/core/integrator.h>
#include <pine/core/profiler.h>
#include <pine/core/context.h>
#include <pine/core/parser.h>
#include <pine/core/fileio.h>

#include <future>

int main(int argc, char* argv[]) {
  using namespace pine;
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

    auto task = std::async(std::launch::async, [&]() { interpretFile(context, argv[1]); });
    while (true) {
      Logr(get_progress(), "\r");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (task.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
        break;
    }
    Log("");

  } catch (const Exception& e) {
    Fatal("Pine exception: ", e.what());
  } catch (...) {
    Fatal("Uncaught exception");
  }

  Profiler::Finalize();

  return 0;
}
