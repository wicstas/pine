#include <pine/core/profiler.h>

#include <pine/psl/map.h>
#include <pine/psl/vector.h>
#include <pine/psl/memory.h>

#include <mutex>
#include <atomic>

#include <signal.h>
#include <sys/time.h>

namespace pine {

static psl::shared_ptr<Profiler::Record> profilerRecord = psl::make_shared<Profiler::Record>();

template <typename T>
psl::string pad(T x, size_t max_len) {
  using psl::to_string;
  auto str = to_string(x);
  const auto len = str.size();
  str = str.substr(0, max_len);
  if (max_len > len)
    str = str + psl::string(max_len - len, ' ');
  return str;
}

void Profiler::Finalize() {
  main.reset();
  Log("[Profiler]========================================>");
  auto ReportRecord = [](auto& me, Record record, size_t indent, double totalTime) -> void {
    if (totalTime != 0.0f && record.time / totalTime < 0.005f)
      return;
    if (record.name != "") {
      Log(psl::string(indent, ' '), pad(record.name, 24 - indent), " ", pad(record.sampleCount, 6),
          pad(record.time, 8), "ms ",
          pad((totalTime == 0.0f) ? 100.0 : 100.0 * record.time / totalTime, 8), "%");
      indent += 2;
    }

    psl::vector<Record> sorted;
    for (auto& rec : record.children)
      sorted.push_back(*rec.second);
    psl::sort(sorted, [](const Record& lhs, const Record& rhs) { return lhs.time > rhs.time; });
    for (auto& rec : sorted)
      me(me, rec, indent, record.time);
  };
  ReportRecord(ReportRecord, *profilerRecord, 0, 0.0f);
}
Profiler::Profiler(psl::string description) {
  psl::shared_ptr<Record>& rec = profilerRecord->children[description];
  if (rec == nullptr)
    rec = psl::make_shared<Record>();

  rec->name = description;
  rec->parent = profilerRecord;
  profilerRecord = rec;
}
Profiler::~Profiler() {
  psl::shared_ptr<Record> rec = profilerRecord;

  rec->time += timer.ElapsedMs();
  rec->sampleCount++;

  profilerRecord = rec->parent;
}

}  // namespace pine