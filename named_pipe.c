#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "named_pipe.h"
#include "messages.h"

int fifo_create_and_open(const char *devname, bool write_not_read)
{
    int ret = mkfifo(devname, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

    if(ret < 0)
    {
        if(errno != EEXIST)
        {
            msg_error(errno, LOG_EMERG,
                      "Failed creating named pipe \"%s\"", devname);
            return -1;
        }
    }

    return fifo_open(devname, write_not_read);
}

int fifo_open(const char *devname, bool write_not_read)
{
    int ret = open(devname, write_not_read ? O_WRONLY : (O_RDONLY | O_NONBLOCK));

    if(ret < 0)
        msg_error(errno, LOG_EMERG,
                  "Failed opening named pipe \"%s\"", devname);
    else
        msg_info("Opened %sable pipe \"%s\", fd %d",
                 write_not_read ? "writ" : "read", devname, ret);

    return ret;
}

void fifo_close(int *fd)
{
    int ret;

    while((ret = close(*fd)) < 0 && errno == EINTR)
        ;

    if(ret < 0)
        msg_error(errno, LOG_ERR, "Failed closing named pipe fd %d", *fd);

    *fd = -1;
}

void fifo_close_and_delete(int *fd, const char *devname)
{
    fifo_close(fd);

    if(unlink(devname) < 0)
        msg_error(errno, LOG_ERR,
                  "Failed deleting named pipe \"%s\"", devname);
}
