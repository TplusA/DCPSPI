#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <errno.h>
#include <assert.h>

#include "spi.h"
#include "named_pipe.h"
#include "gpio.h"
#include "dcpdefs.h"
#include "messages.h"

/*!
 * Current state of the DCP transaction.
 */
enum transaction_state
{
    TSTATE_NONE = 0,               /*!< Idle, waiting for activities. */

    TSTATE_MASTER_READ_REQ_HEADER, /*!< Reading header from DCP process. */
    TSTATE_MASTER_READ_REQ_DATA,   /*!< Reading request from DCP process. */
    TSTATE_MASTER_FLUSH_TO_SLAVE,  /*!< Sending request to slave over SPI. */
    TSTATE_MASTER_READ_ANS_HEADER, /*!< Reading header from slave over SPI. */
    TSTATE_MASTER_READ_ANS_DATA,   /*!< Reading answer from slave over SPI. */
    TSTATE_MASTER_FLUSH_TO_DCP,    /*!< Sending answer to DCP process. */

    TSTATE_SLAVE_READ_REQ_HEADER,  /*!< Reading header from slave over SPI. */
    TSTATE_SLAVE_READ_REQ_DATA,    /*!< Reading request from slave over SPI. */
    TSTATE_SLAVE_FLUSH_TO_DCP,     /*!< Sending request to DCP process. */
    TSTATE_SLAVE_READ_ANS_HEADER,  /*!< Reading header from DCP process. */
    TSTATE_SLAVE_READ_ANS_DATA,    /*!< Reading answer from DCP process. */
    TSTATE_SLAVE_FLUSH_TO_SLAVE,   /*!< Sending answer to slave over SPI. */
};

/*!
 * Some buffer and a simple embedded iterator.
 */
struct buffer
{
    uint8_t *const buffer;
    const size_t size;
    size_t pos;
};

/*!
 * State of the DCP transaction in progress.
 */
struct dcp_transaction
{
    enum transaction_state state;

    struct buffer dcp_buffer;
    struct buffer spi_buffer;

    uint16_t pending_size_of_transaction;
    bool pending_escape_sequence_in_spi_buffer;
};

/*!
 * Whether or not the DCP process is allowed to send any data.
 */
static bool expecting_dcp_data(const struct dcp_transaction *transaction)
{
    return (transaction->state == TSTATE_NONE ||
            transaction->state == TSTATE_MASTER_READ_REQ_HEADER ||
            transaction->state == TSTATE_MASTER_READ_REQ_DATA ||
            transaction->state == TSTATE_SLAVE_READ_ANS_HEADER ||
            transaction->state == TSTATE_SLAVE_READ_ANS_DATA);
}

/*!
 * Global flag that gets cleared in the SIGTERM signal handler.
 *
 * For clean shutdown.
 */
static volatile bool keep_running = true;

static void clear_buffer(struct buffer *buffer)
{
    buffer->pos = 0;
}

static void fill_spi_buffer_from_dcp(struct dcp_transaction *transaction)
{
    const struct buffer *const src = &transaction->dcp_buffer;
    struct buffer *const dest = &transaction->spi_buffer;

    clear_buffer(dest);

    for(size_t i = 0; i < src->pos; ++i)
    {
        const uint8_t ch = src->buffer[i];

        if(ch == UINT8_MAX)
        {
            dest->buffer[dest->pos++] = DCP_ESCAPE_CHARACTER;
            dest->buffer[dest->pos++] = 0x01;
        }
        else if(ch == DCP_ESCAPE_CHARACTER)
        {
            dest->buffer[dest->pos++] = DCP_ESCAPE_CHARACTER;
            dest->buffer[dest->pos++] = DCP_ESCAPE_CHARACTER;
        }
        else
            dest->buffer[dest->pos++] = ch;
    }
}

static bool is_buffer_full(const struct buffer *buffer)
{
    return buffer->pos >= buffer->size;
}

static int fill_buffer_from_fd(struct buffer *buffer, size_t count, int fd)
{
    if(count == 0)
        return 0;

    ssize_t len = read(fd, buffer->buffer + buffer->pos, count);

    if(len > 0)
    {
        count -= len;
        buffer->pos += len;
    }
    else if(len == 0)
    {
        msg_error(errno, LOG_NOTICE,
                  "Premature end of input on named pipe %d", fd);
    }
    else if(errno != EINTR && errno != EAGAIN)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed reading %zu bytes from fd %d", count, fd);
        return -1;
    }

    return len;
}

static ssize_t send_buffer_to_fd(struct buffer *buffer,
                                 size_t offset, size_t count, int fd)
{
    ssize_t len = write(fd, buffer->buffer + offset, count);

    if(len >= 0)
    {
        if(len == 0)
            msg_error(errno, LOG_NOTICE,
                      "Written partial buffer to named pipe %d", fd);

        return len;
    }

    msg_error(errno, LOG_EMERG,
              "Failed writing %zu bytes to fd %d", count, fd);
    return -1;
}

static void reset_transaction(struct dcp_transaction *transaction)
{
    printf("## Reset transaction\n");

    transaction->state = TSTATE_NONE;

    clear_buffer(&transaction->dcp_buffer);
    transaction->pending_size_of_transaction = 0;

    clear_buffer(&transaction->spi_buffer);
}

static uint16_t get_dcp_data_size(const uint8_t *dcp_header)
{
    const uint8_t command_type = dcp_header[0] & 0x0f;

    printf("0x%02x 0x%02x\n", dcp_header[0], command_type);

    if(command_type == DCP_COMMAND_MULTI_WRITE_REGISTER ||
       command_type == DCP_COMMAND_MULTI_READ_REGISTER)
        return dcp_header[2] | (dcp_header[3] << 8);
    else
        return 0;
}

static void process_transaction(struct dcp_transaction *transaction,
                                int fifo_in_fd, int fifo_out_fd, int spi_fd,
                                bool request_active)
{
    switch(transaction->state)
    {
      case TSTATE_NONE:
        if(request_active)
        {
            printf("Starting slave transaction\n");
            transaction->state = TSTATE_SLAVE_READ_REQ_HEADER;
            break;
        }

        printf("Starting master transaction\n");
        transaction->state = TSTATE_MASTER_READ_REQ_HEADER;

        /* fall-through */

      case TSTATE_MASTER_READ_REQ_HEADER:
        printf("Master transaction header\n");
        if(fill_buffer_from_fd(&transaction->dcp_buffer,
                               DCP_HEADER_SIZE - transaction->dcp_buffer.pos,
                               fifo_in_fd) < 0)
        {
            printf("Master transaction header comm broken\n");
            reset_transaction(transaction);
            break;
        }

        if(transaction->dcp_buffer.pos != DCP_HEADER_SIZE)
        {
            printf("Master transaction header incomplete, waiting for more input\n");
            break;
        }

        /*
         * Possibly received a DCP header. Note that we explictly do not
         * validate the header content here because this is going to be done by
         * the receiver of the data. We simply assume that we have a header
         * here and that the length can be determined.
         */
        printf("Master transaction header received\n");
        transaction->pending_size_of_transaction =
            get_dcp_data_size(transaction->dcp_buffer.buffer);

        printf("pending %u\n", transaction->pending_size_of_transaction);

        if(transaction->pending_size_of_transaction > 0)
        {
            transaction->state = TSTATE_MASTER_READ_REQ_DATA;
            break;
        }

        transaction->state = TSTATE_MASTER_FLUSH_TO_SLAVE;

        /* fall-through */

      case TSTATE_MASTER_FLUSH_TO_SLAVE:
        assert(transaction->pending_size_of_transaction == 0);

        fill_spi_buffer_from_dcp(transaction);
        printf("Master transaction, send %zu bytes over SPI (were %zu bytes)\n",
               transaction->spi_buffer.pos, transaction->dcp_buffer.pos);
        if(spi_send_buffer(spi_fd, transaction->spi_buffer.buffer,
                           transaction->spi_buffer.pos) < 0)
        {
            reset_transaction(transaction);
            break;
        }

        clear_buffer(&transaction->spi_buffer);
        clear_buffer(&transaction->dcp_buffer);

        if(transaction->pending_size_of_transaction == 0)
            transaction->state = TSTATE_MASTER_READ_ANS_HEADER;
        else
            transaction->state = TSTATE_MASTER_READ_REQ_DATA;

        break;

      case TSTATE_MASTER_READ_REQ_DATA:
        printf("Master transaction, need to receive %u bytes from DCP\n",
               transaction->pending_size_of_transaction);

        size_t read_size =
            transaction->dcp_buffer.size - transaction->dcp_buffer.pos;

        if(read_size > transaction->pending_size_of_transaction)
            read_size = transaction->pending_size_of_transaction;

        read_size =
            fill_buffer_from_fd(&transaction->dcp_buffer, read_size, fifo_in_fd);

        if(read_size < 0)
        {
            printf("Master transaction comm broken\n");
            reset_transaction(transaction);
            break;
        }

        transaction->pending_size_of_transaction -= read_size;
        printf("still pending %u\n", transaction->pending_size_of_transaction);

        if(transaction->pending_size_of_transaction == 0 ||
           is_buffer_full(&transaction->dcp_buffer))
        {
            printf("flush buffer to SPI\n");
            transaction->state = TSTATE_MASTER_FLUSH_TO_SLAVE;
        }

        break;

      case TSTATE_MASTER_READ_ANS_HEADER:
        printf("Master transaction read answer header\n");
        if(spi_read_buffer(spi_fd, transaction->dcp_buffer.buffer,
                           DCP_HEADER_SIZE, 100) < DCP_HEADER_SIZE)
        {
            reset_transaction(transaction);
            break;
        }

        transaction->dcp_buffer.pos = DCP_HEADER_SIZE;

        printf("Master transaction answer header received: "
               "0x%02x 0x%02x 0x%02x 0x%02x\n",
               transaction->dcp_buffer.buffer[0],
               transaction->dcp_buffer.buffer[1],
               transaction->dcp_buffer.buffer[2],
               transaction->dcp_buffer.buffer[3]);

        transaction->pending_size_of_transaction =
            get_dcp_data_size(transaction->dcp_buffer.buffer);

        printf("pending %u\n", transaction->pending_size_of_transaction);

        if(transaction->pending_size_of_transaction > 0)
        {
            transaction->state = TSTATE_MASTER_READ_ANS_DATA;
            break;
        }

        transaction->pending_size_of_transaction = DCP_HEADER_SIZE;
        transaction->state = TSTATE_MASTER_FLUSH_TO_DCP;

        /* fall-through */

      case TSTATE_MASTER_FLUSH_TO_DCP:
        printf("Master transaction, send %zu bytes answer to DCP (%u pending)\n",
               transaction->dcp_buffer.pos,
               transaction->pending_size_of_transaction);

        ssize_t sent_bytes =
            send_buffer_to_fd(&transaction->dcp_buffer,
                              transaction->dcp_buffer.pos - transaction->pending_size_of_transaction,
                              transaction->pending_size_of_transaction,
                              fifo_out_fd);

        if(sent_bytes < 0)
        {
            printf("Master transaction send answer comm broken\n");
            reset_transaction(transaction);
            break;
        }

        transaction->pending_size_of_transaction -= sent_bytes;

        if(transaction->pending_size_of_transaction == 0)
        {
            printf("Master transaction DONE\n");
            reset_transaction(transaction);
        }

        break;

      case TSTATE_MASTER_READ_ANS_DATA:
        /* TODO: not implemented */
        printf("Master transaction read answer data\n");
        break;

      case TSTATE_SLAVE_READ_REQ_HEADER:
        /* TODO: not implemented */
        printf("Slave transaction header\n");
        break;

      case TSTATE_SLAVE_READ_REQ_DATA:
        /* TODO: not implemented */
        printf("Slave transaction, need to receive %u bytes from SPI\n",
               transaction->pending_size_of_transaction);
        break;

      case TSTATE_SLAVE_FLUSH_TO_DCP:
        /* TODO: not implemented */
        printf("Slave transaction, send %zu bytes over named pipe (were %zu bytes)\n",
               transaction->spi_buffer.pos, transaction->dcp_buffer.pos);
        break;

      case TSTATE_SLAVE_READ_ANS_HEADER:
        /* TODO: not implemented */
        printf("Slave transaction read answer header\n");
        break;

      case TSTATE_SLAVE_READ_ANS_DATA:
        /* TODO: not implemented */
        printf("Slave transaction read answer data\n");
        break;

      case TSTATE_SLAVE_FLUSH_TO_SLAVE:
        /* TODO: not implemented */
        printf("Slave transaction send answer data\n");
        break;
    }
}

static void wait_for_dcp_data(struct dcp_transaction *transaction,
                              const int fifo_in_fd, const int fifo_out_fd,
                              const int spi_fd, const int gpio_fd,
                              const struct gpio_handle *const gpio,
                              bool *gpio_active_state)
{
    struct pollfd fds[2] =
    {
        {
            .fd = gpio_fd,
            .events = POLLPRI | POLLERR,
        },
        {
            .fd = fifo_in_fd,
            .events = POLLIN,
        },
    };

    printf("Waiting for activities.\n");

    int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), -1);

    if(ret <= 0)
    {
        if(ret == 0)
            msg_error(errno, LOG_WARNING, "poll() unexpected timeout");
        else if(errno != EINTR)
            msg_error(errno, LOG_CRIT, "poll() failed");

        return;
    }

    if(fds[0].revents & POLLPRI)
    {
        *gpio_active_state = gpio_is_active(gpio);
        process_transaction(transaction, fifo_in_fd, fifo_out_fd, spi_fd,
                            *gpio_active_state);
    }

    if(fds[0].revents & ~(POLLPRI | POLLERR))
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on gpio_fd %d: %04x",
                  gpio_fd, fds[0].revents);

    if(fds[1].revents & POLLHUP)
    {
        msg_error(0, LOG_ERR, "DCP daemon died, terminating");
        keep_running = false;
    }

    if(fds[1].revents & POLLIN)
    {
        if(expecting_dcp_data(transaction))
            process_transaction(transaction, fifo_in_fd, fifo_out_fd, spi_fd,
                                *gpio_active_state);
        else
            /* FIXME: the DCP process needs to know about this */
            msg_info("collision: DCP tries to send command in the middle of a transaction");
    }

    if(fds[1].revents & ~POLLIN)
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on fifo_fd %d: %04x",
                  fifo_in_fd, fds[1].revents);
}

/*!
 * Copy data back and forth.
 *
 * As long as not transaction is in progress, we are waiting on activities on
 * the name pipe and the request pin. A transaction is started if either the
 * request pin is activated or if some process is sending data to the named
 * pipe.
 *
 * Once a transaction has been started, data needs to be copied and
 * transformed. There are two cases:
 * - Transaction initiated by the slave ("slave transaction")
 * - Transaction initiated by the master ("master transaction")
 *
 * Slave transaction:
 * - Read data from SPI
 * - Transform for DCP
 * - Send transformed data to named pipe
 * - Wait for answer from named pipe
 * - Transform for SPI
 * - Send transformed data to SPI, with data rate limiting
 *
 * Master transaction:
 * - Read data from DCP
 * - Transform for SPI
 * - Send transformed data to SPI, with data rate limiting
 * - Wait for answer from SPI
 * - Transform for DCP
 * - Send transformed data to named pipe
 */
static void main_loop(const int fifo_in_fd, const int fifo_out_fd,
                      const int spi_fd, const struct gpio_handle *const gpio)
{
    msg_info("Ready for accepting traffic");

    static uint8_t dcp_backing_buffer[260];
    static uint8_t spi_backing_buffer[260 * 2];

    struct dcp_transaction transaction =
    {
        .dcp_buffer =
        {
            .buffer = dcp_backing_buffer,
            .size = sizeof(dcp_backing_buffer),
        },
        .spi_buffer =
        {
            .buffer = spi_backing_buffer,
            .size = sizeof(spi_backing_buffer),
        },
    };

    reset_transaction(&transaction);

    const int gpio_fd = gpio_get_poll_fd(gpio);
    bool gpio_active_state = gpio_is_active(gpio);

    while(keep_running)
    {
        if(expecting_dcp_data(&transaction))
            wait_for_dcp_data(&transaction, fifo_in_fd, fifo_out_fd, spi_fd,
                              gpio_fd, gpio, &gpio_active_state);
        else
            process_transaction(&transaction, fifo_in_fd, fifo_out_fd, spi_fd,
                                gpio_active_state);
    }
}

struct parameters
{
    const char *fifo_in_name;
    const char *fifo_out_name;
    const char *spidev_name;
    unsigned int gpio_num;
    bool run_in_foreground;
};

/*!
 * Open devices, daemonize.
 */
static int setup(const struct parameters *parameters,
                 int *fifo_in_fd, int *fifo_out_fd,
                 int *spi_fd, struct gpio_handle **gpio)
{
    msg_enable_syslog(!parameters->run_in_foreground);

    if(!parameters->run_in_foreground)
        openlog("dcpspi", LOG_PID, LOG_DAEMON);

    *fifo_in_fd = fifo_create_and_open(parameters->fifo_in_name, false);
    if(*fifo_in_fd < 0)
        return -1;

    *fifo_out_fd = fifo_create_and_open(parameters->fifo_out_name, true);
    if(*fifo_out_fd < 0)
        goto error_fifo_out;

    *spi_fd = spi_open_device(parameters->spidev_name);
    if(*spi_fd < 0)
        goto error_spi_open;

    *gpio = gpio_open(parameters->gpio_num);
    if(*gpio == NULL)
        goto error_gpio_open;

    if(!parameters->run_in_foreground)
    {
        if(daemon(0, 0) < 0)
        {
            msg_error(errno, LOG_EMERG, "Failed to run as daemon");
            return -1;
        }
    }

    return 0;

error_gpio_open:
    spi_close_device(*spi_fd);

error_spi_open:
    fifo_close_and_delete(*fifo_out_fd, parameters->fifo_out_name);

error_fifo_out:
    fifo_close_and_delete(*fifo_in_fd, parameters->fifo_in_name);
    return -1;
}

static void usage(const char *program_name)
{
    printf("Usage: %s --fifo name --spidev name --irq gpio\n"
           "\n"
           "Options:\n"
           "  --ififo name   Name of the named pipe the DCP daemon writes to.\n"
           "  --ofifo name   Name of the named pipe the DCP daemon reads from.\n"
           "  --spidev name  Name of the SPI device.\n"
           "  --irq gpio     Number of the slave request pin.\n",
           program_name);
}

static int process_command_line(int argc, char *argv[],
                                struct parameters *parameters)
{
    parameters->fifo_in_name = "/tmp/dcp_to_spi";
    parameters->fifo_out_name = "/tmp/spi_to_dcp";
    parameters->spidev_name = "/dev/spidev0.0";
    parameters->gpio_num = 4;
    parameters->run_in_foreground = true;

    return 0;
}

static void signal_handler(int signum, siginfo_t *info, void *ucontext)
{
    keep_running = false;
}

int main(int argc, char *argv[])
{
    static struct parameters parameters;

    int ret = process_command_line(argc, argv, &parameters);

    if(ret == -1)
        return EXIT_FAILURE;
    else if(ret == 1)
    {
        usage(argv[0]);
        return EXIT_SUCCESS;
    }

    int fifo_in_fd, fifo_out_fd, spi_fd;
    struct gpio_handle *gpio;

    if(setup(&parameters, &fifo_in_fd, &fifo_out_fd, &spi_fd, &gpio) < 0)
        return EXIT_FAILURE;

    static struct sigaction action =
    {
        .sa_sigaction = signal_handler,
        .sa_flags = SA_SIGINFO | SA_RESETHAND,
    };

    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    main_loop(fifo_in_fd, fifo_out_fd, spi_fd, gpio);

    msg_info("Terminated, shutting down");

    spi_close_device(spi_fd);
    fifo_close_and_delete(fifo_in_fd, parameters.fifo_in_name);
    fifo_close_and_delete(fifo_out_fd, parameters.fifo_out_name);
    gpio_close(gpio);

    return EXIT_SUCCESS;
}
