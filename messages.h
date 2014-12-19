#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdbool.h>
#include <syslog.h>

#include "os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Whether or not to make use of syslog.
 */
void msg_enable_syslog(bool enable_syslog);

/*!
 * Emit error to stderr and syslog.
 *
 * \param error_code The current error code as stored in errno.
 * \param priority A log priority as expected by syslog(3).
 * \param error_format Format string followed by arguments.
 */
void msg_error(int error_code, int priority, const char *error_format, ...)
    __attribute__ ((format (printf, 3, 4)));

/*
 * Emit log informative message to stderr and syslog.
 */
void msg_info(const char *format_string, ...)
    __attribute__ ((format (printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#ifdef NDEBUG
#define log_assert(EXPR) do {} while(0)
#else /* !NDEBUG */
#define log_assert(EXPR) \
    do \
    { \
        if(!(EXPR)) \
        { \
            msg_error(0, LOG_EMERG, "Assertion failed at %s:%d: " #EXPR, \
                      __FILE__, __LINE__); \
            os_abort(); \
        } \
    } \
    while(0)
#endif /* NDEBUG */

#endif /* !MESSAGES_H */
