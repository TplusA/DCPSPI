#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include "os.h"

int os_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    return clock_gettime(clk_id, tp);
}
