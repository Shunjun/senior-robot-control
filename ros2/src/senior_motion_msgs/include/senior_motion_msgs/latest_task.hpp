// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <utility>

namespace senior_motion_msgs::timing
{
// Uses the producer's existing executor: one running task and one replaceable latest.
class LatestTask
{
public:
  ~LatestTask() {close();}
  template<typename Schedule>
  void offer(Schedule schedule, std::function<void()> task)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return;
      pending_ = std::move(task);
      if (running_) return;
      running_ = true;
    }
    try {schedule([this] {run();});} catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false; pending_ = {}; idle_.notify_all();
      throw;
    }
  }

  void close()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    closed_ = true;
    pending_ = {};
    idle_.wait(lock, [this] {return !running_;});
  }
private:
  void run()
  {
    while (true) {
      std::function<void()> job;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        job = std::move(pending_);
        pending_ = {};
        if (!job) {running_ = false; idle_.notify_all(); return;}
      }
      try {job();} catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false; pending_ = {}; idle_.notify_all();
        throw;
      }
    }
  }
  std::mutex mutex_;
  std::condition_variable idle_;
  std::function<void()> pending_;
  bool running_ = false, closed_ = false;
};
}  // namespace senior_motion_msgs::timing
