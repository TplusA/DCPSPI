#ifndef NAMED_PIPE_H
#define NAMED_PIPE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int fifo_create_and_open(const char *devname, bool write_not_read);
void fifo_close_and_delete(int fd, const char *devname);

#ifdef __cplusplus
}
#endif

#endif /* !NAMED_PIPE_H */
