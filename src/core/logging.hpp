#pragma once
#include <core/string.hpp>

#include <memory>

namespace pine {

using LogEntry = std::string;

struct LogSink {
  template <typename T>
  LogSink(T model) : model(std::make_shared<Model<T>>(model)) {}
  bool push(LogEntry entry) { return model->push(entry); }

 private:
  class Concept {
   public:
    virtual bool push(LogEntry entry) const = 0;
  };
  template <typename T>
  class Model : public Concept {
   public:
    Model(T impl) : impl(impl) {}
    bool push(LogEntry entry) const override { return impl.push(entry); }

   private:
    T impl;
  };

  std::shared_ptr<const Concept> model;
};

extern LogSink logSink;

void log(const auto&... args) { logSink.push(toString(args...)); }

struct OstreamLogSink {
  bool push(LogEntry entry) const;
};

}  // namespace pine
