#pragma once

#include <pine/core/log.h>

#include <pine/psl/memory.h>
#include <pine/psl/map.h>

namespace pine {

struct Profiler {
  static void Initialize() {
    main = psl::unique_ptr<Profiler>(new Profiler("Main"));
  }
  static void Finalize();

  Profiler(psl::string description);
  ~Profiler();

  Profiler(const Profiler&) = delete;
  Profiler(Profiler&&) = delete;
  Profiler& operator=(const Profiler&) = delete;
  Profiler& operator=(Profiler&&) = delete;

  struct Record {
    friend bool operator==(const Record& lhs, const Record& rhs) {
      return lhs.time == rhs.time && lhs.sampleCount == rhs.sampleCount && lhs.name == rhs.name;
    }

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
