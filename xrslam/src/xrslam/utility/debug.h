#ifndef XRSLAM_DEBUG_H
#define XRSLAM_DEBUG_H

#include <xrslam/common.h>

namespace xrslam {

enum LogLevel {
    XRSLAM_LOG_DEBUG = -1,  /**< debug message                      **/
    XRSLAM_LOG_INFO = 0,    /**< informational message              **/
    XRSLAM_LOG_NOTICE = 1,  /**< normal, but significant, condition **/
    XRSLAM_LOG_WARNING = 2, /**< warning conditions                 **/
    XRSLAM_LOG_ERR = 3,     /**< error conditions                   **/
    XRSLAM_LOG_CRIT = 4,    /**< critical conditions                **/
    XRSLAM_LOG_ALERT = 5,   /**< action must be taken immediately   **/
    XRSLAM_LOG_EMERG = 6    /**< system is unusable                 **/
};

void log_message(LogLevel level, const char *format, ...);

} // namespace xrslam

#define log_emergency(...) log_message(XRSLAM_LOG_EMERG, __VA_ARGS__)
#define log_alert(...) log_message(XRSLAM_LOG_ALERT, __VA_ARGS__)
#define log_critical(...) log_message(XRSLAM_LOG_CRIT, __VA_ARGS__)
#define log_error(...) log_message(XRSLAM_LOG_ERR, __VA_ARGS__)
#define log_warning(...) log_message(XRSLAM_LOG_WARNING, __VA_ARGS__)
#define log_notice(...) log_message(XRSLAM_LOG_NOTICE, __VA_ARGS__)
#define log_info(...) log_message(XRSLAM_LOG_INFO, __VA_ARGS__)

#ifdef XRSLAM_DEBUG
#define log_debug(...) log_message(XRSLAM_LOG_DEBUG, __VA_ARGS__)
#define runtime_assert(condition, message)                                     \
    do {                                                                       \
        if (!(condition)) {                                                    \
            log_error("Assertion failed at " __FILE__                          \
                      ":%d : %s\nWhen testing condition:\n    %s",             \
                      __LINE__, message, #condition);                          \
            abort();                                                           \
        }                                                                      \
    } while (0)
#else
#define log_debug(...)
#define runtime_assert(...)
#endif

#endif // XRSLAM_DEBUG_H
