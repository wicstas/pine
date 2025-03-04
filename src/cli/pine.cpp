#include <pine/core/context.h>
#include <pine/core/fileio.h>
#include <pine/core/integrator.h>
#include <pine/core/profiler.h>
#include <pine/core/program_context.h>

#include <future>

int main(int argc, char* argv[]) {
  using namespace pine;

  if (argc != 2) {
    LOG("Usage: pine [filename]");
    return 0;
  }

#ifndef NDEBUG
  WARNING("[Performance]Debug build");
#endif

  Profiler::Initialize();

  try {
    setup_program_context();
    auto& context = Context::context;

    auto task = std::async(std::launch::async, [&]() {
      try {
        interpret_file(context, argv[1]);
      } catch (const std::exception& e) {
        LOG(e.what());
      } catch (const psl::Exception& e) {
        LOG(e.what());
      }
    });
    while (true) {
      if (get_progress() != 0) LOGr(get_progress(), "\r");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (task.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) break;
    }
    LOG("");
    // interpret_file(context, argv[1]);
  } catch (const std::exception& e) {
    LOG(e.what());
  } catch (const psl::Exception& e) {
    LOG(e.what());
  }

  Profiler::Finalize();

  return 0;
}