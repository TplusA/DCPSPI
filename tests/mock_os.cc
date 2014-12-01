#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <cppcutter.h>

#include "mock_os.hh"

enum class OsFn
{
    os_clock_gettime_fn,

    first_valid_os_fn_id = os_clock_gettime_fn,
    last_valid_os_fn_id = os_clock_gettime_fn,
};


static std::ostream &operator<<(std::ostream &os, const OsFn id)
{
    if(id < OsFn::first_valid_os_fn_id ||
       id > OsFn::last_valid_os_fn_id)
    {
        os << "INVALID";
        return os;
    }

    switch(id)
    {
      case OsFn::os_clock_gettime_fn:
        os << "os_clock_gettime";
        break;
    }

    os << "()";

    return os;
}

class MockOs::Expectation
{
  public:
    const OsFn function_id_;

    const int ret_code_;
    clockid_t arg_clk_id_;
    const struct timespec ret_tp_;
    os_clock_gettime_callback_t os_clock_gettime_callback_;

    Expectation(const Expectation &) = delete;
    Expectation &operator=(const Expectation &) = delete;

    explicit Expectation(int ret, clockid_t clk_id, const struct timespec &ret_tp):
        function_id_(OsFn::os_clock_gettime_fn),
        ret_code_(ret),
        arg_clk_id_(clk_id),
        ret_tp_(ret_tp),
        os_clock_gettime_callback_(nullptr)
    {}

    explicit Expectation(os_clock_gettime_callback_t fn):
        function_id_(OsFn::os_clock_gettime_fn),
        ret_code_(-5),
        arg_clk_id_(CLOCK_REALTIME_COARSE),
        ret_tp_({0}),
        os_clock_gettime_callback_(fn)
    {}

    Expectation(Expectation &&) = default;
};

MockOs::MockOs()
{
    expectations_ = new MockExpectations();
}

MockOs::~MockOs()
{
    delete expectations_;
}

void MockOs::init()
{
    cppcut_assert_not_null(expectations_);
    expectations_->init();
}

void MockOs::check() const
{
    cppcut_assert_not_null(expectations_);
    expectations_->check();
}

void MockOs::expect_os_clock_gettime(int ret, clockid_t clk_id,
                                     const struct timespec &ret_tp)
{
    expectations_->add(Expectation(ret, clk_id, ret_tp));
}

void MockOs::expect_os_clock_gettime_callback(os_clock_gettime_callback_t fn)
{
    expectations_->add(Expectation(fn));
}


MockOs *mock_os_singleton = nullptr;

int os_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    const auto &expect(mock_os_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, OsFn::os_clock_gettime_fn);

    if(expect.os_clock_gettime_callback_ != nullptr)
        return expect.os_clock_gettime_callback_(clk_id, tp);

    cppcut_assert_equal(expect.arg_clk_id_, clk_id);
    *tp = expect.ret_tp_;
    return expect.ret_code_;
}
