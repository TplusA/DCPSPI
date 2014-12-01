#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <cppcutter.h>

#include "mock_spi_hw.hh"

enum class SpiHwFn
{
    open_device,
    close_device,
    do_transfer,

    first_valid_spi_hw_fn_id = open_device,
    last_valid_spi_hw_fn_id = do_transfer,
};


static std::ostream &operator<<(std::ostream &os, const SpiHwFn id)
{
    if(id < SpiHwFn::first_valid_spi_hw_fn_id ||
       id > SpiHwFn::last_valid_spi_hw_fn_id)
    {
        os << "INVALID";
        return os;
    }

    switch(id)
    {
      case SpiHwFn::open_device:
        os << "spi_hw_open_device";
        break;

      case SpiHwFn::close_device:
        os << "spi_hw_close_device";
        break;

      case SpiHwFn::do_transfer:
        os << "spi_hw_do_transfer";
        break;
    }

    os << "()";

    return os;
}

class MockSPIHW::Expectation
{
  public:
    const SpiHwFn function_id_;

    const int ret_code_;
    const std::string arg_devname_;
    const int arg_fd_;
    const size_t arg_number_of_fragments_;
    spi_hw_do_transfer_callback_t spi_hw_do_transfer_callback_;

    Expectation(const Expectation &) = delete;
    Expectation &operator=(const Expectation &) = delete;

    explicit Expectation(int ret, const char *devname):
        function_id_(SpiHwFn::open_device),
        ret_code_(ret),
        arg_devname_(devname),
        arg_fd_(-5),
        arg_number_of_fragments_(0),
        spi_hw_do_transfer_callback_(nullptr)
    {}

    explicit Expectation(int fd):
        function_id_(SpiHwFn::close_device),
        ret_code_(-5),
        arg_fd_(fd),
        arg_number_of_fragments_(0),
        spi_hw_do_transfer_callback_(nullptr)
    {}

    explicit Expectation(int ret, int fd, size_t number_of_fragments):
        function_id_(SpiHwFn::do_transfer),
        ret_code_(ret),
        arg_fd_(fd),
        arg_number_of_fragments_(number_of_fragments),
        spi_hw_do_transfer_callback_(nullptr)
    {}

    explicit Expectation(spi_hw_do_transfer_callback_t fn):
        function_id_(SpiHwFn::do_transfer),
        ret_code_(-5),
        arg_fd_(-5),
        arg_number_of_fragments_(0),
        spi_hw_do_transfer_callback_(fn)
    {}

    Expectation(Expectation &&) = default;
};

MockSPIHW::MockSPIHW()
{
    expectations_ = new MockExpectations();
}

MockSPIHW::~MockSPIHW()
{
    delete expectations_;
}

void MockSPIHW::init()
{
    cppcut_assert_not_null(expectations_);
    expectations_->init();
}

void MockSPIHW::check() const
{
    cppcut_assert_not_null(expectations_);
    expectations_->check();
}

void MockSPIHW::expect_spi_hw_open_device(int ret, const char *devname)
{
    expectations_->add(Expectation(ret, devname));
}

void MockSPIHW::expect_spi_hw_close_device(int fd)
{
    expectations_->add(Expectation(fd));
}

void MockSPIHW::expect_spi_hw_do_transfer(int ret, int fd, size_t number_of_fragments)
{
    expectations_->add(Expectation(ret, fd, number_of_fragments));
}

void MockSPIHW::expect_spi_hw_do_transfer_callback(MockSPIHW::spi_hw_do_transfer_callback_t fn)
{
    expectations_->add(Expectation(fn));
}


MockSPIHW *mock_spi_hw_singleton = nullptr;

int spi_hw_open_device(const char *devname)
{
    const auto &expect(mock_spi_hw_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, SpiHwFn::open_device);
    cppcut_assert_equal(expect.arg_devname_, std::string(devname));
    return expect.ret_code_;
}

void spi_hw_close_device(int fd)
{
    const auto &expect(mock_spi_hw_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, SpiHwFn::close_device);
    cppcut_assert_equal(expect.arg_fd_, fd);
}

int spi_hw_do_transfer(int fd, const struct spi_ioc_transfer spi_transfer[], size_t number_of_fragments)
{
    const auto &expect(mock_spi_hw_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, SpiHwFn::do_transfer);

    if(expect.spi_hw_do_transfer_callback_ != nullptr)
        return expect.spi_hw_do_transfer_callback_(fd, spi_transfer, number_of_fragments);

    cppcut_assert_equal(expect.arg_fd_, fd);
    cppcut_assert_equal(expect.arg_number_of_fragments_, number_of_fragments);
    return expect.ret_code_;
}
