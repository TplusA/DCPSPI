#ifndef MOCK_SPI_HW_HH
#define MOCK_SPI_HW_HH

#include "spi_hw.h"
#include "mock_expectation.hh"

class MockSPIHW
{
  public:
    MockSPIHW(const MockSPIHW &) = delete;
    MockSPIHW &operator=(const MockSPIHW &) = delete;

    class Expectation;
    typedef MockExpectationsTemplate<Expectation> MockExpectations;
    MockExpectations *expectations_;

    explicit MockSPIHW();
    ~MockSPIHW();

    void init();
    void check() const;

    void expect_spi_hw_open_device(int ret, const char *devname);
    void expect_spi_hw_close_device(int fd);

    typedef int (*spi_hw_do_transfer_callback_t)(int fd, const struct spi_ioc_transfer spi_transfer[], size_t number_of_fragments);
    void expect_spi_hw_do_transfer(int ret, int fd, size_t number_of_fragments);
    void expect_spi_hw_do_transfer_callback(spi_hw_do_transfer_callback_t fn);
};

extern MockSPIHW *mock_spi_hw_singleton;

#endif /* !MOCK_SPI_HW_HH */
