#include "rclcpp/detail/os_thread.hpp"


#if defined(_WIN32)
#include <windows.h>
#else  // posix and apple
#include <pthread.h>
#endif

#include "rclcpp/logging.hpp"

namespace rclcpp
{
namespace detail
{

void set_thread_name(const std::string & name)
{
    int rc;
#if defined(_WIN32)
    // This will only work on Windows 10 and above.
    auto result = SetThreadDescription(GetCurrentThread(), name.c_str());
    rc = FAILED(result) ? -1 : 0;
#elif defined(__APPLE__)
    rc = pthread_setname_np(name.c_str());
#else  // posix
    // Truncate name to 16 characters as that's the maximum length supported by pthread_setname_np
    std::string truncated_name = name.substr(0, 15);
    rc = pthread_setname_np(pthread_self(), truncated_name.c_str());
#endif
    if (rc != 0) {
        // Don't throw since this is not critical
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set thread name: %s", name.c_str());
    }
}
}
}
