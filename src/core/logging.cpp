#include <core/logging.hpp>

#include <iostream>

namespace pine {

LogSink logSink = OstreamLogSink();

bool OstreamLogSink::push(LogEntry entry) const { std::cout << entry; return true; }

}  // namespace pine
