// Offline tests: no SDK device, ROS node or camera stream is opened.
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>

#include "azure_kinect_ros_driver/stream_control.h"

namespace
{
struct TestTracker
{
  bool accepted = false;
  bool failed = false;
  std::chrono::milliseconds timeout{123};
  bool enqueue_capture(int, std::chrono::milliseconds wait)
  {
    timeout = wait;
    if (failed) { throw std::runtime_error("SDK FAILED"); }
    return accepted;
  }
};
}

TEST(TrackerAdmission, BusyDoesNotInventAnOutstandingResult)
{
  TestTracker tracker;
  std::atomic_int16_t pending{2};
  for (const int wait_ms : {0, 5, 10})
  {
    EXPECT_FALSE(k4a_driver::tryEnqueueTrackerCapture(
        tracker, 1, pending, std::chrono::milliseconds(wait_ms)));
    EXPECT_EQ(tracker.timeout.count(), wait_ms);
    EXPECT_EQ(pending.load(), 2);
  }
}

TEST(TrackerAdmission, ProductionWaitPolicyIsRestrictedToBlockingOrFiveMilliseconds)
{
  EXPECT_NO_THROW(k4a_driver::validateTrackerEnqueueWait(-1));
  EXPECT_NO_THROW(k4a_driver::validateTrackerEnqueueWait(5));
  for (const int wait_ms : {-100, -2, 0, 1, 10, 1000})
  {
    EXPECT_THROW(k4a_driver::validateTrackerEnqueueWait(wait_ms), std::invalid_argument);
  }
  EXPECT_NO_THROW(k4a_driver::validateTrackerEnqueueWait(0, true));
  EXPECT_NO_THROW(k4a_driver::validateTrackerEnqueueWait(10, true));
}

TEST(TrackerAdmission, OnlyAcceptedCapturesIncrementPending)
{
  TestTracker tracker;
  tracker.accepted = true;
  std::atomic_int16_t pending{1};
  EXPECT_TRUE(k4a_driver::tryEnqueueTrackerCapture(
      tracker, 1, pending, std::chrono::milliseconds(5)));
  EXPECT_EQ(tracker.timeout.count(), 5);
  EXPECT_EQ(pending.load(), 2);
}

TEST(TrackerAdmission, BlockingUsesSdkInfiniteWaitAndCountsAcceptedCaptureOnce)
{
  TestTracker tracker;
  tracker.accepted = true;
  std::atomic_int16_t pending{0};
  EXPECT_TRUE(k4a_driver::tryEnqueueTrackerCapture(
      tracker, 1, pending, std::chrono::milliseconds(-1)));
  EXPECT_EQ(tracker.timeout.count(), -1);
  EXPECT_EQ(pending.load(), 1);
}

TEST(TrackerAdmission, SdkFailureStillPropagatesWithoutInventingAResult)
{
  TestTracker tracker;
  tracker.failed = true;
  std::atomic_int16_t pending{1};
  for (const int wait_ms : {-1, 5, 10})
  {
    EXPECT_THROW(k4a_driver::tryEnqueueTrackerCapture(
        tracker, 1, pending, std::chrono::milliseconds(wait_ms)), std::runtime_error);
  }
  EXPECT_EQ(pending.load(), 1);
}

TEST(CaptureTimeout, TemporaryMissingCaptureDoesNotExhaustBudget)
{
  k4a_driver::CaptureTimeoutPolicy policy(5);
  EXPECT_FALSE(policy.timedOut());
  EXPECT_EQ(policy.count(), 1);
  policy.received();
  EXPECT_EQ(policy.count(), 0);
}

TEST(CaptureTimeout, OnlyConsecutiveTimeoutsExhaustBudget)
{
  k4a_driver::CaptureTimeoutPolicy policy(5);
  for (int i = 1; i < 5; ++i)
  {
    EXPECT_FALSE(policy.timedOut());
    EXPECT_EQ(policy.count(), i);
  }
  EXPECT_TRUE(policy.timedOut());
  EXPECT_TRUE(policy.timedOut());
  EXPECT_EQ(policy.count(), 5);
}

TEST(CaptureTimeout, RecoveredCaptureResetsConsecutiveFailures)
{
  k4a_driver::CaptureTimeoutPolicy policy(3);
  EXPECT_FALSE(policy.timedOut());
  EXPECT_FALSE(policy.timedOut());
  policy.received();
  EXPECT_FALSE(policy.timedOut());
  EXPECT_FALSE(policy.timedOut());
  EXPECT_TRUE(policy.timedOut());
}

TEST(CaptureTimeout, SingleAttemptConfigurationIsSupported)
{
  k4a_driver::CaptureTimeoutPolicy policy(1);
  EXPECT_TRUE(policy.timedOut());
}

TEST(CaptureTimeout, InvalidBudgetIsRejected)
{
  EXPECT_THROW(k4a_driver::CaptureTimeoutPolicy(0), std::invalid_argument);
  EXPECT_THROW(k4a_driver::CaptureTimeoutPolicy(-1), std::invalid_argument);
}

TEST(StreamShutdown, UnstartedWorkersCanBeDestroyed)
{
  std::atomic_bool running{false};
  std::thread camera, body, imu;
  bool interrupted = false;
  EXPECT_NO_THROW(k4a_driver::stopAndJoin(running, [&]() { interrupted = true; },
                                        {&camera, &body, &imu}));
  EXPECT_TRUE(interrupted);
}

TEST(StreamShutdown, InterruptReleasesBlockedWorkerBeforeJoin)
{
  std::atomic_bool running{true};
  std::mutex mutex;
  std::condition_variable condition;
  bool sdk_stopped = false;
  bool released_by_stop = false;
  bool saw_running_false = false;
  std::promise<void> ready;
  std::thread camera([&]() {
    std::unique_lock<std::mutex> lock(mutex);
    ready.set_value();
    // Bounded fake wait prevents a broken join-before-interrupt implementation
    // from hanging the test runner indefinitely.
    released_by_stop = condition.wait_for(lock, std::chrono::seconds(2),
                                         [&]() { return sdk_stopped; });
    saw_running_false = !running;
  });
  std::thread unstarted_body, unstarted_imu;
  ready.get_future().wait();
  k4a_driver::stopAndJoin(running, [&]() {
    EXPECT_FALSE(running.load());
    {
      std::lock_guard<std::mutex> lock(mutex);
      sdk_stopped = true;
    }
    condition.notify_all();
  }, {&camera, &unstarted_body, &unstarted_imu});
  EXPECT_TRUE(released_by_stop);
  EXPECT_TRUE(saw_running_false);
  EXPECT_FALSE(camera.joinable());
}

TEST(StreamShutdown, AllStartedWorkersAreJoined)
{
  std::atomic_bool running{true};
  std::atomic_int completed{0};
  auto work = [&]() { ++completed; };
  std::thread camera(work), body(work), imu(work);
  k4a_driver::stopAndJoin(running, []() {}, {&body, &camera, &imu});
  EXPECT_EQ(completed.load(), 3);
  EXPECT_FALSE(camera.joinable());
  EXPECT_FALSE(body.joinable());
  EXPECT_FALSE(imu.joinable());
}

TEST(StreamWorker, StandardExceptionStopsStreamAndReportsCause)
{
  std::atomic_bool running{true};
  std::string error;
  k4a_driver::runWorker(running, []() { throw std::runtime_error("USB read failed"); },
                        [&](const char* message) { error = message; });
  EXPECT_FALSE(running.load());
  EXPECT_EQ(error, "USB read failed");
}

TEST(StreamWorker, DeliberateStopDoesNotReportAnExpectedSDKException)
{
  std::atomic_bool running{false};
  bool reported = false;
  k4a_driver::runWorker(running, []() { throw std::runtime_error("SDK shutdown"); },
                        [&](const char*) { reported = true; });
  EXPECT_FALSE(reported);
}

TEST(StreamWorker, UnknownExceptionIsContained)
{
  std::atomic_bool running{true};
  std::string error;
  k4a_driver::runWorker(running, []() { throw 7; },
                        [&](const char* message) { error = message; });
  EXPECT_FALSE(running.load());
  EXPECT_EQ(error, "Unknown stream worker exception");
}

TEST(StreamWorker, NormalReturnDoesNotDeclareADeviceFailure)
{
  std::atomic_bool running{true};
  bool completed = false;
  bool reported = false;
  k4a_driver::runWorker(running, [&]() { completed = true; },
                        [&](const char*) { reported = true; });
  EXPECT_TRUE(completed);
  EXPECT_FALSE(reported);
  EXPECT_TRUE(running.load());
}
