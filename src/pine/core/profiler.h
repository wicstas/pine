#pragma once

#include <pine/core/log.h>

#include <psl/memory.h>
#include <psl/map.h>

namespace pine {

struct Profiler {
  static void Initialize() {
    main = psl::make_unique<Profiler>("Main");
  }
  static void Finalize();

  Profiler(psl::string description);
  ~Profiler();

  Profiler(const Profiler&) = delete;
  Profiler(Profiler&&) = delete;
  Profiler& operator=(const Profiler&) = delete;
  Profiler& operator=(Profiler&&) = delete;

  struct Record {
    psl::shared_ptr<Record> parent;
    psl::map<psl::string, psl::shared_ptr<Record>> children;
    psl::string name;
    double time = 0.0f;
    int sampleCount = 0;
  };

  Timer timer;

  static inline psl::unique_ptr<Profiler> main;
};

}  // namespace pine
