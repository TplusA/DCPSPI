# Filter for receiving and transmitting DCP over SPI

## Copyright and contact

DCPSPI is released under the terms of the GNU General Public License version 3
(GPLv3). See file <tt>COPYING</tt> for licensing terms.

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
implementation. It is used to exchange unescaped, but otherwise raw DCP data.
That is, the DCP headers are expected to be created by the high-level
implementation, not by _dcpspi_.

If the named pipe doesn't exist, it is created by the _dcpspi_ daemon on
startup. If the high-level DCP implementation is running before the named pipe
has been created, then that implementation should wait until the pipe is
available.

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

The end of a transaction on behalf of the named pipe is detected by a minor DCP
awareness inside _dcpspi_. The length of a transaction is known from the DCP
header, particularly its command code and possibly length field, so _dcpspi_
uses it to determine when the end of a transaction has been reached.

An alternative approach would be to <tt>close()</tt> the pipe on the sender
side so that the <tt>read()</tt> would fail in _dcpspi_, but this would be less
efficient and may introduce race conditions. Other approaches like "check if
pipe is empty" or "wait for Unix signal" are bound to be fragile because they
are dependent on timing.

### Data rate limitation

*Not specified yet*

SPI writes from the master to the slave are limited by use of the interrupt
pin. Basically, before sending data, the slave signalizes that it is ready to
receive data by pulsing the interrupt pin. The master then sends a chunk of
data of fixed maximum size, and waits for the next pulse on the interrupt line
or end of transmission.

*Details are to be defined*

## Configuration

The _dcpspi_ requires configuration of the following parameters:

- name of the spidev device;
- name of the named pipe; and
- name of the interrupt GPIO.

These are passed as command line parameters.
