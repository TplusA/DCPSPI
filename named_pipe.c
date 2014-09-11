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

    ret = open(devname, write_not_read ? O_WRONLY : (O_RDONLY | O_NONBLOCK));

    if(ret < 0)
        msg_error(errno, LOG_EMERG,
                  "Failed opening named pipe \"%s\"", devname);
    else
        msg_info("Pipe \"%s\" fd %d", devname, ret);

    return ret;
}

void fifo_close_and_delete(int fd, const char *devname)
{
    int ret;

    while((ret = close(fd)) < 0 && errno == EINTR)
        ;

    if(ret < 0)
        msg_error(errno, LOG_ERR, "Failed closing named pipe fd %d", fd);

    ret = unlink(devname);

    if(ret < 0)
        msg_error(errno, LOG_ERR,
                  "Failed deleting named pipe \"%s\"", devname);
}
