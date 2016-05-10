/*
 * Copyright (C) 2016  T+A elektroakustik GmbH & Co. KG
 *
 * This file is part of DCPSPI.
 *
 * DCPSPI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 3 as
 * published by the Free Software Foundation.
 *
 * DCPSPI is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DCPSPI.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SPI_HW_DATA_HH
#define SPI_HW_DATA_HH

#include <algorithm>
#include <vector>

static constexpr size_t read_from_slave_spi_transfer_size = 32;
static constexpr size_t wait_for_slave_spi_transfer_size = 2;

class spi_rw_data_partial_t
{
  public:
    spi_rw_data_partial_t(const spi_rw_data_partial_t &) = delete;
    spi_rw_data_partial_t &operator=(const spi_rw_data_partial_t &) = delete;

    spi_rw_data_partial_t(spi_rw_data_partial_t &&) = default;

    const std::vector<uint8_t> expected_write_data_;
    const std::vector<uint8_t> read_data_;
    const size_t transfer_size_;
    const int errno_value_;
    const int return_value_;
    std::vector<uint8_t> written_data_;

    explicit spi_rw_data_partial_t(const std::vector<uint8_t> write_data,
                                   const std::vector<uint8_t> read_data,
                                   size_t transfer_size, int err, int ret):
        expected_write_data_(write_data),
        read_data_(read_data),
        transfer_size_(transfer_size),
        errno_value_(err),
        return_value_(ret)
    {}

    explicit spi_rw_data_partial_t(const std::vector<uint8_t> write_data,
                                   size_t transfer_size, int err, int ret):
        expected_write_data_(write_data),
        transfer_size_(transfer_size),
        errno_value_(err),
        return_value_(ret)
    {}
};

class spi_rw_data_t
{
  public:
    enum write_nops
    {
        EXPECT_WRITE_NOPS,
        EXPECT_WRITE_ZEROS,
    };

    enum read_nops
    {
        EXPECT_READ_NOPS,
        EXPECT_READ_ZEROS,
        EXPECT_READ_NON_ZERO,
    };

    spi_rw_data_t(const spi_rw_data_t &) = delete;
    spi_rw_data_t &operator=(const spi_rw_data_t &) = delete;

    size_t fragment_;
    std::vector<spi_rw_data_partial_t> partial_;

    explicit spi_rw_data_t(): fragment_(0) {}

    /*!
     * One big write transfer, don't care about answer (rx buffer is NULL).
     */
    template <size_t N>
    void set(const std::array<uint8_t, N> &write_data)
    {
        set(write_data.data(), N, 0, 0);
    }

  private:
    template <size_t N>
    static void fill_write_data(std::array<uint8_t, N> &data, write_nops nops)
    {
        switch(nops)
        {
          case EXPECT_WRITE_NOPS:
            data.fill(UINT8_MAX);
            break;

          case EXPECT_WRITE_ZEROS:
            data.fill(0);
            break;
        }
    }

    template <size_t N>
    static void fill_read_data(std::array<uint8_t, N> &data, read_nops nops)
    {
        switch(nops)
        {
          case EXPECT_READ_NOPS:
            data.fill(UINT8_MAX);
            break;

          case EXPECT_READ_ZEROS:
            data.fill(0);
            break;

          case EXPECT_READ_NON_ZERO:
            data.fill(0xa5);
            break;
        }
    }

  public:
    /*!
     * Several small write transfers with constant answer from slave.
     */
    template <size_t N>
    void set(const std::array<uint8_t, N> &write_data, enum read_nops nops)
    {
        std::array<uint8_t, N> read_data;

        fill_read_data(read_data, nops);
        set_fragments(write_data.data(), read_data.data(), N);
    }

    /*!
     * Several small read transfers with constant value written to slave.
     */
    template <size_t N>
    void set(enum write_nops nops, const std::array<uint8_t, N> &read_data,
             bool is_slave_ready_probe = false)
    {
        std::array<uint8_t, N> write_data;

        fill_write_data(write_data, nops);

        if(is_slave_ready_probe)
            set_fragments<wait_for_slave_spi_transfer_size>(write_data.data(), read_data.data(), N);
        else
            set_fragments<read_from_slave_spi_transfer_size>(write_data.data(), read_data.data(), N);
    }

    /*!
     * Several small transfers with constant value sent and constant answer
     * from slave.
     */
    template <size_t N>
    void set(enum write_nops wnops, enum read_nops rnops,
             bool is_slave_ready_probe = false)
    {
        std::array<uint8_t, N> write_data;
        std::array<uint8_t, N> read_data;

        fill_write_data(write_data, wnops);
        fill_read_data(read_data, rnops);

        if(is_slave_ready_probe)
            set_fragments<wait_for_slave_spi_transfer_size>(write_data.data(), read_data.data(), N);
        else
            set_fragments<read_from_slave_spi_transfer_size>(write_data.data(), read_data.data(), N);
    }

    /*!
     * Full expected transfer specification.
     */
    template <size_t N>
    void set(const std::array<uint8_t, N> &write_data,
             const std::array<uint8_t, N> &read_data)
    {
        set_fragments(write_data.data(), read_data.data(), N);
    }

  private:
    template <size_t T = read_from_slave_spi_transfer_size>
    void set_fragments(const uint8_t *write_data, const uint8_t *read_data,
                       size_t transfer_size)
    {
        for(size_t offset = 0; offset < transfer_size; offset += T)
        {
            const size_t count =
                std::min(T, transfer_size - offset);

            if(count == T)
                set(write_data + offset, read_data + offset, T, 0, 0);
            else
            {
                std::array<uint8_t, T> last_write_data;
                std::array<uint8_t, T> last_read_data;

                last_write_data.fill(UINT8_MAX);
                last_read_data.fill(UINT8_MAX);

                std::copy_n(write_data + offset, count, last_write_data.begin());
                std::copy_n(read_data + offset,  count, last_read_data.begin());

                set(last_write_data.data(), last_read_data.data(), T, 0, 0);
            }
        }
    }

    void set(const uint8_t *write_data,
             size_t transfer_size, int err, ssize_t ret)
    {
        partial_.push_back(spi_rw_data_partial_t(std::vector<uint8_t>(write_data,
                                                                      write_data + transfer_size),
                                                 transfer_size, err, ret));
    }

    void set(const uint8_t *write_data, const uint8_t *read_data,
             size_t transfer_size, int err, ssize_t ret)
    {
        partial_.push_back(spi_rw_data_partial_t(std::vector<uint8_t>(write_data,
                                                                      write_data + transfer_size),
                                                 std::vector<uint8_t>(read_data,
                                                                      read_data + transfer_size),
                                                 transfer_size, err, ret));
    }
};

static spi_rw_data_t *spi_rw_data;

template <int SPI_FD>
static int mock_spi_tx_template(int fd,
                                const struct spi_ioc_transfer spi_transfer[],
                                size_t number_of_fragments)
{
    cppcut_assert_equal(SPI_FD, fd);
    cppcut_assert_equal(size_t(1), number_of_fragments);
    cppcut_assert_operator(spi_rw_data->fragment_, <, spi_rw_data->partial_.size());

    const spi_rw_data_partial_t &partial(spi_rw_data->partial_[spi_rw_data->fragment_++]);

    if(partial.read_data_.size() > 0)
        cppcut_assert_not_equal(__u64(0), spi_transfer[0].rx_buf);
    else
        cppcut_assert_equal(__u64(0), spi_transfer[0].rx_buf);

    if(partial.expected_write_data_.size() > 0)
        cppcut_assert_not_equal(__u64(0), spi_transfer[0].tx_buf);
    else
        cppcut_assert_equal(__u64(0), spi_transfer[0].tx_buf);

    cppcut_assert_equal(partial.transfer_size_, size_t(spi_transfer[0].len));
    cppcut_assert_equal(partial.transfer_size_, partial.expected_write_data_.size());

    if(partial.read_data_.size() > 0)
    {
        cppcut_assert_equal(partial.transfer_size_, partial.read_data_.size());
        std::copy_n(partial.read_data_.begin(), partial.read_data_.size(),
                    reinterpret_cast<uint8_t *>(spi_transfer[0].rx_buf));
    }

    if(partial.expected_write_data_.size() > 0)
        cut_assert_equal_memory(partial.expected_write_data_.data(),
                                partial.expected_write_data_.size(),
                                reinterpret_cast<uint8_t *>(spi_transfer[0].tx_buf),
                                spi_transfer[0].len);

    errno = partial.errno_value_;

    return partial.return_value_;
}

#endif /* !SPI_HW_DATA_HH */
