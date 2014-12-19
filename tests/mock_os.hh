#ifndef MOCK_OS_HH
#define MOCK_OS_HH

#include "os.h"
#include "mock_expectation.hh"

class MockOs
{
  public:
    MockOs(const MockOs &) = delete;
    MockOs &operator=(const MockOs &) = delete;

    class Expectation;
    typedef MockExpectationsTemplate<Expectation> MockExpectations;
    MockExpectations *expectations_;

    explicit MockOs();
    ~MockOs();

    void init();
    void check() const;

    typedef int (*os_clock_gettime_callback_t)(clockid_t clk_id, struct timespec *tp);
    void expect_os_clock_gettime(int ret, clockid_t clk_id, const struct timespec &ret_tp);
    void expect_os_clock_gettime_callback(os_clock_gettime_callback_t fn);

    void expect_os_abort(void);
};

extern MockOs *mock_os_singleton;

#endif /* !MOCK_OS_HH */
