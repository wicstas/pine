#include <pine/core/program_context.h>
#include <pine/core/integrator.h>
#include <pine/core/profiler.h>
#include <pine/core/context.h>
#include <pine/core/fileio.h>

#include <future>

struct F {
  int y0;
  int y1;
};

F make_f() {
  F f;
  f.y0 = 1;
  f.y1 = 2;
  return f;
}

void print2(F f, int v0, int v1, int v2, int v3) {
  pine::Logs(f.y0, f.y1, v0, v1, v2, v3);
};

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
    auto context = get_program_context();

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