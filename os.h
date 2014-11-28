#ifndef OS_H
#define OS_H

#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

int os_clock_gettime(clockid_t clk_id, struct timespec *tp);

#ifdef __cplusplus
}
#endif

#endif /* !OS_H */
