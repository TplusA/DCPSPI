# Filter for receiving and transmitting DCP over SPI

## Copyright and contact

DCPSPI is released under the terms of the GNU General Public License version 2
or (at your option) any later version. See file <tt>COPYING</tt> for licensing
terms of the GNU General Public License version 2, or <tt>COPYING.GPLv3</tt>
for licensing terms of the GNU General Public License version 3.

Contact:

    T+A elektroakustik GmbH & Co. KG
    Planckstrasse 11
    32052 Herford
    Germany

## Short description

_dcpspi_ is a simple daemon that acts as a proxy between a <tt>/dev/spidev</tt>
device and a named pipe. It converts the DCP raw data from SPI to something
more friendly to the named pipe, and the other way around. The program has only
minimal knowledge about the DC protocol.

The program is, more or less, a low-level protocol driver in userspace.

## Details on functionality

The _dcpspi_ program is a filter for DCP over SPI with a few twists.

DCP is a simple request-response protocol. One end is sending a request, the
other is sending back the answer, and that's all.

Since the Linux SPI drivers are master-only, but our slave device needs to send
data at unpredictable times, an interrupt line driven by the slave is
monitored. When that line is activated while no transaction is in progress, a
new DCP transaction is started and data is read from SPI; at other times, the
SPI device remains inactive.

Writing as a master to the slave requires data rate limitation. The slave
device has only so much memory available and also needs to process the data
while receiving it, so the master could easily overflow the slave's buffer if
there were no rate limitation. See below for the implemented solution.

### Named pipe

A named pipe is used to connect the _dcpspi_ daemon with the high-level DCP
implementation. It is used to exchange raw DCP data with escape codes and NOPs
removed, wrapped into a small protocol for synchronization and reliable size
information. DCP reply headers and payloads are expected to be created by the
high-level implementation, not by _dcpspi_.

If the named pipe doesn't exist, it is created by the _dcpspi_ daemon on
startup. If the high-level DCP implementation is running before the named pipe
has been created, then that implementation should wait until the pipe is
available.

#### Protocol

There is a small protocol spoken on the named pipe. It is designed under the
assumption that _dcpspi_ is a mere bridge that can handle only one transaction
at a time, and that the high-level DCP implementation has a queue for incoming
and outgoing DCP packets. Reads and writes issued by the SPI slave are passed
through and assumed to be always accepted by the higher-level implementation,
thus taking precedence over master transactions. Therefore, master transactions
may fail if they collide with a slave transaction, so all pending outgoing
packets are assumed to queue up in and repeated later by the higher-level
implementation.

The protocol prepends six bytes to each DCP packet, consisting of a command
byte followed by a byte for a time-to-live (TTL) counter, a serial number, and
payload size:

    <command byte> <ttl> <serial high> <serial low> <length high> <length low>

Serial numbers 0x0001 through 0x7fff are reserved for slave transactions.
Serial numbers 0x8001 through 0xffff are reserved for master transaction.

Serial numbers 0x0000 and 0x8000 are invalid and must not be used in the
command header. Both sides are required to ignore commands with any such serial
number.

Commands received by _dcpspi_ must always have a size at least of 10,
corresponding to the size of the prepended header and a DCP header. This keeps
the implementation of command reception in _dcpspi_ simple. If necessary, a
dummy DCP header filled with `0xff` bytes must be sent.

| Command  | Description |
|:--------:|:--------|
| `c`      | DCP packet follows: no answer expected if TTL is set to 0, an ACK otherwise. |
| `a`      | DCP packet accepted ("ACK"). |
| `n`      | DCP packet rejected ("NACK"): the write command didn't happen, try again later. |

The TTL is used to limit the number of retries in case of repeated rejections
of master transactions and is currently only relevant in that context. A
positive TTL for a `c` command means that the receiver must return an ACK or a
NACK. In case it returns NACK, it must send a TTL lower than that received with
the `c` command (usually decremented by 1). In case it returns ACK, it must
send a TTL of 0. A TTL of 0 means that the packet must be consumed by the
receiver and that no answer must be returned. The TTL in ACK packets shall be
ignored and be treated as 0.

Since _dcpspi_ will limit the TTL of incoming `c` packets, it is safe for the
higher-level DCP implementation to always pass 0xff for master transactions. A
much lower value will be sent along with NACK packets, but the maximum number
of NACKs can be lowered even further by sending a small value in `c` packets.
Sending a TTL of 1 means that collisions are detectable by the higher-level
implementation, but must be handled by ignoring them (since the returned TTL in
the NACK will be 0). Sending a TTL of 0 means that collisions are going to be
completely undetectable by the higher-level implementation.

If the higher-level DCP implementation receives a NACK, then it may assume that
a slave transaction is going to be performed very soon. It is safe to wait
synchronously for this expected transaction to begin.

#### Protocol examples

##### Slave read request collides with write attempt from master

    S                                                                    M
    |                                                                    |
    |-- ['c' 0x00 0x39 0x6a 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read
    |<- ['c' 0xff 0xef 0x21 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x01)] --|  Master write
    |-- ['n' 0x09 0xef 0x21 0x00 0x00] --------------------------------->|  Master write NACK
    |<- ['c' 0xff 0xef 0x22 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer
    |-- ['a' 0x00 0xef 0x22 0x00 0x00] --------------------------------->|  Master answer ACK
    |<- ['c' 0x09 0xef 0x21 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x01)] --|  Master write retry
    |-- ['a' 0x00 0xef 0x21 0x00 0x00] --------------------------------->|  Master write accepted

##### Fast speculative master writes for high average throughput, but SPI collision causes several transactions to be rejected and retried later

    S                                                                    M
    |                                                                    |
    |-- ['c' 0x00 0x39 0x6c 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read 1
    |<- ['c' 0xff 0x39 0x6c 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 1
    |<- ['c' 0xff 0xef 0x25 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x01)] --|  Master write 1
    |<- ['c' 0xff 0xef 0x26 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x00)] --|  Master write 2
    |<- ['c' 0xff 0xef 0x27 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x00)] --|  Master write 3
    |-- ['a' 0x00 0x39 0x6c 0x00 0x00] --------------------------------->|  Master answer 1 ACK
    |-- ['a' 0x00 0xef 0x25 0x00 0x00] --------------------------------->|  Master write 1 ACK, then SPI collision occurs
    |-- ['c' 0x00 0x39 0x6d 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read 2
    |-- ['n' 0x09 0xef 0x26 0x00 0x00] --------------------------------->|  Master write 2 NACK
    |-- ['n' 0x09 0xef 0x27 0x00 0x00] --------------------------------->|  Master write 3 NACK
    |<- ['c' 0xff 0x39 0x6d 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 2
    |-- ['a' 0x00 0x39 0x6d 0x00 0x00] --------------------------------->|  Master answer 2 ACK
    |<- ['c' 0x09 0xef 0x26 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x00)] --|  Master write 2 retry
    |<- ['c' 0x09 0xef 0x27 0x00 0x06 (0x02 0x32 0x02 0x00 0x02 0x00)] --|  Master write 3 retry
    |-- ['a' 0x00 0xef 0x26 0x00 0x00] --------------------------------->|  Master write 2 accepted
    |-- ['a' 0x00 0xef 0x27 0x00 0x00] --------------------------------->|  Master write 3 accepted

##### Fast slave transactions can cause many NACKs

    S                                                               M
    |                                                               |
    |-- ['c' 0x00 0x3c 0x10 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read 1
    |<- ['c' 0xff 0x91 0x40 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 1
    |-- ['n' 0x09 0x91 0x40 0x00 0x00] --------------------------------->|  Master answer 1 NACK
    |-- ['c' 0x00 0x3c 0x11 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read 2
    |<- ['c' 0x09 0x91 0x40 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 1 retry
    |-- ['n' 0x08 0x91 0x40 0x00 0x00] --------------------------------->|  Master answer 1 NACK
    |-- ['c' 0x00 0x3c 0x12 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read 3
    |<- ['c' 0x08 0x91 0x40 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 1 retry
    |-- ['n' 0x07 0x91 0x40 0x00 0x00] --------------------------------->|  Master answer 1 NACK
    |-- ['c' 0x00 0x3c 0x13 0x00 0x04 (0x01 0x11 0x00 0x00)] ----------->|  SPI slave read 4
    |<- ['c' 0x07 0x91 0x40 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 1 retry
    |-- ['a' 0x00 0x91 0x40 0x00 0x00] --------------------------------->|  Master answer 1 ACK
    |<- ['c' 0xff 0x91 0x41 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 2
    |-- ['a' 0x00 0x91 0x41 0x00 0x00] --------------------------------->|  Master answer 2 ACK
    |<- ['c' 0xff 0x91 0x42 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 3
    |-- ['a' 0x00 0x91 0x42 0x00 0x00] --------------------------------->|  Master answer 3 ACK
    |<- ['c' 0xff 0x91 0x43 0x00 0x06 (0x03 0x11 0x02 0x00 0x24 0x42)] --|  Master answer 4
    |-- ['a' 0x00 0x91 0x43 0x00 0x00] --------------------------------->|  Master answer 4 ACK

### Transactions

An activation of the interrupt line marks the start of a DCP transaction. A
transaction ends when the answer is sent to the slave.

In particular, _dcpspi_ does the following things when the start of a DCP
transaction is requested by the slave:

- read data in small chunks from SPI
- remove leading and trailing 0xff NOP bytes from the data
- handle escaped bytes
- write the filtered data to the named pipe
- wait for data from the named pipe
- escape any 0xff bytes
- send data to SPI, with data rate limitation
- end transaction after the last byte has been sent

Any data that is received from both, the named pipe and SPI, while no DCP
transaction is active is discarded.

### End of transaction

The end of a transaction when reading from SPI is derived from the command code
and length information found in the DCP header. The length is carried over to
the synchronization protocol on the named pipe.

### Data rate limitation

*Not specified yet*

SPI writes from the master to the slave are limited by use of the interrupt
pin. Basically, before sending data, the slave signalizes that it is ready to
receive data by pulsing the interrupt pin. The master then sends a chunk of
data of fixed maximum size, and waits for the next pulse on the interrupt line
or end of transmission.

*Details are to be defined*

## Configuration

The _dcpspi_ daemon requires configuration of the following parameters:

- name of the spidev device;
- name of the named pipe; and
- name of the interrupt GPIO.

These are passed as command line parameters.

## Permissions

The _dcpspi_ daemon reads from and writes to a `/dev/spidev` device and
requires permission for this. It also requires permission to export a GPIO via
`sysfs` and to configure and use it.
