// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_STREAM_CONTROL_H
#define K4A_STREAM_CONTROL_H

#include <atomic>
#include <exception>
#include <initializer_list>
#include <stdexcept>
#include <thread>

namespace k4a_driver
{
// A missing frame is not a failed device. Bound only consecutive timeouts;
// receiving a real capture starts a new retry budget.
class CaptureTimeoutPolicy
{
 public:
  explicit CaptureTimeoutPolicy(int limit) : limit_(limit)
  {
    if (limit <= 0)
    {
      throw std::invalid_argument("capture_max_consecutive_timeouts must be positive");
    }
  }

  bool timedOut()
  {
    if (count_ < limit_)
    {
      ++count_;
    }
    return count_ >= limit_;
  }

  void received() { count_ = 0; }
  int count() const { return count_; }
  int limit() const { return limit_; }

 private:
  int limit_;
  int count_{0};
};

// Interrupt blocked SDK reads before joining workers. Resource handles must
// remain alive until all workers finish, including partially started streams.
template <typename Interrupt>
void stopAndJoin(std::atomic_bool& running, Interrupt interrupt,
                 std::initializer_list<std::thread*> workers)
{
  running.store(false);
  interrupt();
  for (auto* worker : workers)
  {
    if (worker->joinable())
    {
      worker->join();
    }
  }
}

// Suppress read errors caused by deliberate SDK shutdown, and report unexpected
// worker failures instead of letting an exception terminate the process.
template <typename Work, typename Failure>
void runWorker(std::atomic_bool& running, Work work, Failure failure)
{
  try
  {
    work();
  }
  catch (const std::exception& error)
  {
    if (running.exchange(false))
    {
      failure(error.what());
    }
  }
  catch (...)
  {
    if (running.exchange(false))
    {
      failure("Unknown stream worker exception");
    }
  }
}
}  // namespace k4a_driver

#endif  // K4A_STREAM_CONTROL_H
