// Test-only LD_PRELOAD shim. Never installed or used by the production launch.
// Deny all device opens so native startup regressions cannot touch hardware.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <k4a/k4a.h>
#if defined(K4A_BODY_TRACKING)
#include <k4abt.h>
#endif

extern "C" uint32_t k4a_device_get_installed_count(void)
{
  const char* count = std::getenv("CRA_TEST_DEVICE_COUNT");
  return count && std::strcmp(count, "1") == 0 ? 1 : 0;
}

extern "C" k4a_result_t k4a_device_open(uint32_t, k4a_device_t* device)
{
  std::fprintf(stderr, "TEST_ONLY_DEVICE_OPEN_DENIED\n");
  *device = nullptr;
  return K4A_RESULT_FAILED;
}

#if defined(K4A_BODY_TRACKING)
extern "C" k4a_result_t k4abt_tracker_create(const k4a_calibration_t*,
                                             k4abt_tracker_configuration_t,
                                             k4abt_tracker_t* tracker)
{
  std::fprintf(stderr, "TEST_ONLY_UNEXPECTED_TRACKER_CREATE\n");
  *tracker = nullptr;
  return K4A_RESULT_FAILED;
}
#endif
