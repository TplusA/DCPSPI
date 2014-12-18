#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
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
#include "os.h"

/*!
 * Current state of the DCP transaction.
 */
enum transaction_state
{
    TR_IDLE = 0,                                    /*!< Idle, waiting for activities. */

    /* master transactions */
    TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD,  /*!< Reading header from DCP process. */
    TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD,    /*!< Reading data from DCP process. */
    TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE,         /*!< Sending write request to slave over SPI. */

    /* slave transactions */
    TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE,       /*!< Reading header from slave over SPI. */

    /* for slave write commands */
    TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE,    /*!< Reading data from slave over SPI. */
    TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD,           /*!< Sending write request to DCP process. */

    /* for slave read commands */
    TR_SLAVE_READCMD_FORWARDING_TO_DCPD,            /*!< Sending read request to DCP process. */
    TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD,    /*!< Reading header from DCP process. */
    TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD,      /*!< Reading data from DCP process. */
    TR_SLAVE_READCMD_FORWARDING_TO_SLAVE,           /*!< Sending answer to slave over SPI. */
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
    return (transaction->state == TR_IDLE ||
            transaction->state == TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD ||
            transaction->state == TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD ||
            transaction->state == TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD ||
            transaction->state == TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD);
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
    errno = 0;

    ssize_t len = write(fd, buffer->buffer + offset, count);

    if(len > 0)
        return len;

    msg_error(errno, LOG_EMERG,
              "Failed writing %zu bytes to fd %d (write() returned %zd)",
              count, fd, len);
    return -1;
}

static void reset_transaction(struct dcp_transaction *transaction)
{
    msg_info("## Reset transaction");

    transaction->state = TR_IDLE;

    clear_buffer(&transaction->dcp_buffer);
    transaction->pending_size_of_transaction = 0;

    clear_buffer(&transaction->spi_buffer);
}

static uint8_t get_dcp_command_type(uint8_t dcp_header_first_byte)
{
    return dcp_header_first_byte & 0x0f;
}

static bool is_read_command(const uint8_t *dcp_header)
{
    const uint8_t command_type = get_dcp_command_type(dcp_header[0]);

    return (command_type == DCP_COMMAND_READ_REGISTER ||
            command_type == DCP_COMMAND_MULTI_READ_REGISTER);
}

static uint16_t get_dcp_data_size(const uint8_t *dcp_header)
{
    const uint8_t command_type = get_dcp_command_type(dcp_header[0]);

    if(command_type == DCP_COMMAND_MULTI_WRITE_REGISTER ||
       command_type == DCP_COMMAND_MULTI_READ_REGISTER)
        return dcp_header[2] | (dcp_header[3] << 8);
    else
        return 0;
}

static size_t compute_read_size(const struct dcp_transaction *transaction)
{
    size_t read_size =
        transaction->dcp_buffer.size - transaction->dcp_buffer.pos;

    if(read_size > transaction->pending_size_of_transaction)
        read_size = transaction->pending_size_of_transaction;

    return read_size;
}

static const char *tr_log_prefix(enum transaction_state state)
{
    switch(state)
    {
      case TR_IDLE:
        return "No transaction";

      case TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD:
      case TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE:
        return "Master write transaction";

      case TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE:
        return "Slave transaction";

      case TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE:
      case TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD:
        return "Slave write transaction";

      case TR_SLAVE_READCMD_FORWARDING_TO_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_READCMD_FORWARDING_TO_SLAVE:
        return "Slave read transaction";
    }

    return "INVALID transaction";
}

static void process_transaction_receive_data(struct dcp_transaction *transaction,
                                             int fifo_in_fd, int spi_fd,
                                             unsigned int spi_timeout_ms)
{
    const char *read_peer =
        (transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE) ? "slave" : "DCPD";
    const char *write_peer =
        (transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE) ? "DCPD" : "slave";

    msg_info("%s: need to receive %u bytes from %s",
             tr_log_prefix(transaction->state),
             transaction->pending_size_of_transaction, read_peer);

    const size_t read_size = compute_read_size(transaction);
    const int bytes_read =
        (transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE)
        ? spi_read_buffer(spi_fd, transaction->dcp_buffer.buffer, read_size,
                          spi_timeout_ms)
        : fill_buffer_from_fd(&transaction->dcp_buffer, read_size, fifo_in_fd);

    if(bytes_read < 0)
    {
        msg_error(0, LOG_ERR, "%s: communication with %s broken",
                  tr_log_prefix(transaction->state), read_peer);
        reset_transaction(transaction);
        return;
    }

    transaction->pending_size_of_transaction -= (size_t)bytes_read;
    msg_info("%s: still pending %u", tr_log_prefix(transaction->state),
             transaction->pending_size_of_transaction);

    if(transaction->pending_size_of_transaction == 0 ||
       is_buffer_full(&transaction->dcp_buffer))
    {
        msg_info("%s: flush buffer to %s", tr_log_prefix(transaction->state), write_peer);
        if(transaction->state == TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD)
            transaction->state = TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE;
        else if(transaction->state == TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD)
            transaction->state = TR_SLAVE_READCMD_FORWARDING_TO_SLAVE;
        else
            transaction->state = TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD;
    }
}

static void process_transaction(struct dcp_transaction *transaction,
                                int fifo_in_fd, int fifo_out_fd,
                                int spi_fd, unsigned int spi_timeout_ms,
                                bool is_slave_request)
{
    switch(transaction->state)
    {
      case TR_IDLE:
        if(is_slave_request)
        {
            transaction->state = TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE;
            msg_info("%s: begin", tr_log_prefix(transaction->state));
            break;
        }

        transaction->state = TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD;
        msg_info("%s: begin (assuming write command)", tr_log_prefix(transaction->state));

        /* fall-through */

      case TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD:
        msg_info("%s: receiving command header", tr_log_prefix(transaction->state));
        if(fill_buffer_from_fd(&transaction->dcp_buffer,
                               DCP_HEADER_SIZE - transaction->dcp_buffer.pos,
                               fifo_in_fd) < 0)
        {
            msg_error(0, LOG_ERR, "%s: communication with DCPD broken",
                      tr_log_prefix(transaction->state));
            reset_transaction(transaction);
            break;
        }

        if(transaction->dcp_buffer.pos != DCP_HEADER_SIZE)
        {
            msg_info("%s: header incomplete, waiting for more input",
                     tr_log_prefix(transaction->state));
            break;
        }

        /*
         * Possibly received a DCP header. Note that we explictly do not
         * validate the header content here because this is going to be done by
         * the receiver of the data. We simply assume that we have a header
         * here and that the length can be determined.
         */
        msg_info("%s: command header received: 0x%02x 0x%02x 0x%02x 0x%02x",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.buffer[0],
                 transaction->dcp_buffer.buffer[1],
                 transaction->dcp_buffer.buffer[2],
                 transaction->dcp_buffer.buffer[3]);

        transaction->pending_size_of_transaction =
            get_dcp_data_size(transaction->dcp_buffer.buffer);

        if(transaction->pending_size_of_transaction > 0)
        {
            if(transaction->state == TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD)
                transaction->state = TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD;
            else
                transaction->state = TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD;
            break;
        }

        if(transaction->state == TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD)
            transaction->state = TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE;
        else
            transaction->state = TR_SLAVE_READCMD_FORWARDING_TO_SLAVE;

        /* fall-through */

      case TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE:
      case TR_SLAVE_READCMD_FORWARDING_TO_SLAVE:
        assert(transaction->pending_size_of_transaction == 0);

        clear_buffer(&transaction->spi_buffer);
        transaction->spi_buffer.pos =
            spi_fill_buffer_from_raw_data(transaction->spi_buffer.buffer,
                                          transaction->spi_buffer.size,
                                          transaction->dcp_buffer.buffer,
                                          transaction->dcp_buffer.pos);

        msg_info("%s: send %zu bytes over SPI (were %zu bytes)",
                 tr_log_prefix(transaction->state),
                 transaction->spi_buffer.pos, transaction->dcp_buffer.pos);
        if(spi_send_buffer(spi_fd, transaction->spi_buffer.buffer,
                           transaction->spi_buffer.pos, spi_timeout_ms) < 0)
        {
            reset_transaction(transaction);
            break;
        }

        msg_info("%s: DONE", tr_log_prefix(transaction->state));
        reset_transaction(transaction);
        break;

      case TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE:
        msg_info("%s: receiving command header", tr_log_prefix(transaction->state));
        if(spi_read_buffer(spi_fd, transaction->dcp_buffer.buffer,
                           DCP_HEADER_SIZE, spi_timeout_ms) < DCP_HEADER_SIZE)
        {
            reset_transaction(transaction);
            break;
        }

        transaction->dcp_buffer.pos = DCP_HEADER_SIZE;

        msg_info("%s: command header received: 0x%02x 0x%02x 0x%02x 0x%02x",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.buffer[0],
                 transaction->dcp_buffer.buffer[1],
                 transaction->dcp_buffer.buffer[2],
                 transaction->dcp_buffer.buffer[3]);

        transaction->pending_size_of_transaction =
            get_dcp_data_size(transaction->dcp_buffer.buffer);

        if(transaction->pending_size_of_transaction > 0)
        {
            transaction->state = TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE;
            break;
        }

        transaction->pending_size_of_transaction = DCP_HEADER_SIZE;
        transaction->state = (is_read_command(transaction->dcp_buffer.buffer)
                              ? TR_SLAVE_READCMD_FORWARDING_TO_DCPD
                              : TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD);
        break;

      case TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE:
        process_transaction_receive_data(transaction, fifo_in_fd, spi_fd,
                                         spi_timeout_ms);
        break;

      case TR_SLAVE_READCMD_FORWARDING_TO_DCPD:
      case TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD:
        msg_info("%s: send %zu bytes answer to DCPD (%u pending)",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.pos,
                 transaction->pending_size_of_transaction);

        ssize_t sent_bytes =
            send_buffer_to_fd(&transaction->dcp_buffer,
                              transaction->dcp_buffer.pos - transaction->pending_size_of_transaction,
                              transaction->pending_size_of_transaction,
                              fifo_out_fd);

        if(sent_bytes < 0)
        {
            msg_error(0, LOG_ERR, "%s: communication with DCPD broken",
                      tr_log_prefix(transaction->state));
            reset_transaction(transaction);
            break;
        }

        transaction->pending_size_of_transaction -= sent_bytes;

        if(transaction->pending_size_of_transaction == 0)
        {
            if(transaction->state == TR_SLAVE_READCMD_FORWARDING_TO_DCPD)
            {
                clear_buffer(&transaction->dcp_buffer);
                transaction->state = TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD;
            }
            else
            {
                msg_info("%s: DONE", tr_log_prefix(transaction->state));
                reset_transaction(transaction);
            }
        }
        break;
    }
}

static void wait_for_dcp_data(struct dcp_transaction *transaction,
                              const int fifo_in_fd, const int fifo_out_fd,
                              const int spi_fd, unsigned int spi_timeout_ms,
                              const int gpio_fd,
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

    msg_info("Waiting for activities.");

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
        *gpio_active_state = gpio_is_active(gpio);

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
            process_transaction(transaction, fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, *gpio_active_state);
        else
            /* FIXME: the DCP process needs to know about this */
            msg_info("collision: DCP tries to send command in the middle of a transaction");
    }

    if(fds[1].revents & ~POLLIN)
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on fifo_fd %d: %04x",
                  fifo_in_fd, fds[1].revents);
}

static bool process_slave_request(struct dcp_transaction *transaction,
                                  int fifo_in_fd, int fifo_out_fd,
                                  int spi_fd, unsigned int spi_timeout_ms,
                                  bool previous_state, bool new_state)
{
    if(previous_state == new_state)
        return previous_state;

    if(new_state)
    {
        if(transaction->state != TR_IDLE)
        {
            msg_error(0, LOG_WARNING,
                      "Got slave request while processing DCP transaction.");
            return previous_state;
        }

        process_transaction(transaction, fifo_in_fd, fifo_out_fd,
                            spi_fd, spi_timeout_ms, new_state);
    }

    return new_state;
}

/*!
 * Copy data back and forth.
 *
 * As long as no transaction is in progress, we are waiting on activities on
 * the named pipe and the request pin. A transaction is started if either the
 * request pin is activated or if some process is sending data to the named
 * pipe.
 *
 * Once a transaction has been started, data needs to be copied and
 * transformed. There are two cases:
 * - Transaction initiated by the slave ("slave transaction")
 * - Transaction initiated by the master ("master transaction")
 *
 * Slave transaction:
 * - Read four bytes long command from SPI
 * - In case of write command: Read optional data from SPI
 * - Transform for DCPD (unescape raw data)
 * - Send transformed command and data to named pipe
 * - In case of read command: Wait for answer from named pipe
 * - In case of read command: Transform for SPI (insert escape sequences)
 * - In case of read command: Send transformed data to SPI
 *
 * Master transaction (always write commands):
 * - Read four bytes long write command from DCPD
 * - Read optional data from DCPD
 * - Transform for SPI (insert escape sequences)
 * - Send transformed data to SPI
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

    static const unsigned int spi_timeout_ms = 1000;

    const int gpio_fd = gpio_get_poll_fd(gpio);
    bool gpio_active_state = gpio_is_active(gpio);

    if(gpio_active_state)
        process_transaction(&transaction, fifo_in_fd, fifo_out_fd,
                            spi_fd, spi_timeout_ms, gpio_active_state);

    while(keep_running)
    {
        if(expecting_dcp_data(&transaction))
        {
            bool new_gpio_state = gpio_active_state;

            wait_for_dcp_data(&transaction, fifo_in_fd, fifo_out_fd,
                              spi_fd, spi_timeout_ms,
                              gpio_fd, gpio, &new_gpio_state);

            gpio_active_state =
                process_slave_request(&transaction, fifo_in_fd, fifo_out_fd,
                                      spi_fd, spi_timeout_ms,
                                      gpio_active_state, new_gpio_state);
        }
        else
            process_transaction(&transaction, fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, gpio_active_state);
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

    if(!parameters->run_in_foreground)
    {
        if(daemon(0, 0) < 0)
        {
            msg_error(errno, LOG_EMERG, "Failed to run as daemon");
            return -1;
        }
    }

    *fifo_in_fd = fifo_create_and_open(parameters->fifo_in_name, false);
    if(*fifo_in_fd < 0)
        return -1;

    *fifo_out_fd = fifo_create_and_open(parameters->fifo_out_name, true);
    if(*fifo_out_fd < 0)
        goto error_fifo_out;

    *spi_fd = spi_open_device(parameters->spidev_name);
    if(*spi_fd < 0)
        goto error_spi_open;

    *gpio = gpio_open(parameters->gpio_num, false);
    if(*gpio == NULL)
        goto error_gpio_open;

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
           "  --gpio num     Number of the slave request pin.\n",
           program_name);
}

static int process_command_line(int argc, char *argv[],
                                struct parameters *parameters)
{
    parameters->fifo_in_name = "/tmp/dcp_to_spi";
    parameters->fifo_out_name = "/tmp/spi_to_dcp";
    parameters->spidev_name = "/dev/spidev0.0";
    parameters->gpio_num = 4;
    parameters->run_in_foreground = false;

#define CHECK_ARGUMENT() \
    do \
    { \
        if(i + 1 >= argc) \
        { \
            fprintf(stderr, "Option %s requires an argument.\n", argv[i]); \
            return -1; \
        } \
        ++i; \
    } \
    while(0)

    for(int i = 1; i < argc; ++i)
    {
        if(strcmp(argv[i], "--help") == 0)
            return 1;
        else if(strcmp(argv[i], "--fg") == 0)
            parameters->run_in_foreground = true;
        else if(strcmp(argv[i], "--ififo") == 0)
        {
            CHECK_ARGUMENT();
            parameters->fifo_in_name = argv[i];
        }
        else if(strcmp(argv[i], "--ofifo") == 0)
        {
            CHECK_ARGUMENT();
            parameters->fifo_out_name = argv[i];
        }
        else if(strcmp(argv[i], "--spidev") == 0)
        {
            CHECK_ARGUMENT();
            parameters->spidev_name = argv[i];
        }
        else if(strcmp(argv[i], "--gpio") == 0)
        {
            CHECK_ARGUMENT();

            char *endptr;
            unsigned long temp = strtoul(argv[i], &endptr, 10);

            if(*endptr != '\0' || temp > UINT_MAX || (temp == ULONG_MAX && errno == ERANGE))
            {
                fprintf(stderr, "Invalid value \"%s\". Please try --help.\n", argv[i]);
                return -1;
            }

            parameters->gpio_num = temp;
        }
        else
        {
            fprintf(stderr, "Unknown option \"%s\". Please try --help.\n", argv[i]);
            return -1;
        }
    }

#undef CHECK_ARGUMENT

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
