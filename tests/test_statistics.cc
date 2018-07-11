/*
 * Copyright (C) 2018  T+A elektroakustik GmbH & Co. KG
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

#include <cppcutter.h>
#include <chrono>

#include "statistics.h"

#include "mock_messages.hh"
#include "mock_os.hh"

/*!
 * \addtogroup spi_statistics_tests Unit tests
 * \ingroup spi_statistics
 *
 * SPI statistics gathering unit tests.
 */
/*!@{*/

namespace spi_statistics_tests
{

static MockMessages *mock_messages;
static MockOs *mock_os;

struct stats_context ctx;

void cut_setup()
{
    mock_messages = new MockMessages;
    cppcut_assert_not_null(mock_messages);
    mock_messages->init();
    mock_messages_singleton = mock_messages;

    mock_os = new MockOs;
    cppcut_assert_not_null(mock_os);
    mock_os->init();
    mock_os_singleton = mock_os;

    stats_init();
    stats_context_reset(&ctx);
}

void cut_teardown()
{
    mock_messages->check();
    mock_os->check();

    mock_messages_singleton = nullptr;
    mock_os_singleton = nullptr;

    delete mock_messages;
    delete mock_os;

    mock_messages = nullptr;
    mock_os = nullptr;
}

/*!\test
 * First-time entering any context does not touch the duration in that context.
 *
 * This step occurs only once per program start. It is the first time a
 * non-NULL context is introduced to the context switching machinery.
 */
void test_context_enter_first()
{
    cppcut_assert_equal(uint64_t(0), ctx.t_usec);
    cppcut_assert_equal(uint64_t(0), ctx.ti_usec);

    static const struct timespec t = { .tv_sec = 300, .tv_nsec = 1234567, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_null(stats_context_switch(&ctx));

    cppcut_assert_equal(uint64_t(0), ctx.t_usec);
    cppcut_assert_equal(uint64_t(0), ctx.ti_usec);
}

/*!\test
 * Time consumed is recorded in the context we are switching away from.
 */
void test_context_simple_switch()
{
    cppcut_assert_equal(uint64_t(0), ctx.t_usec);
    cppcut_assert_equal(uint64_t(0), ctx.ti_usec);

    struct timespec t = { .tv_sec = 500, .tv_nsec = 20000, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_null(stats_context_switch(&ctx));

    cppcut_assert_equal(uint64_t(0), ctx.t_usec);
    cppcut_assert_equal(uint64_t(0), ctx.ti_usec);

    struct stats_context next;
    stats_context_reset(&next);

    t.tv_sec = 502;
    t.tv_nsec = 80000;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_equal(&ctx, stats_context_switch(&next));
    cppcut_assert_equal(uint64_t(0), next.t_usec);
    cppcut_assert_equal(uint64_t(2U * 1000U * 1000U + 60U), ctx.t_usec);
    cppcut_assert_equal(ctx.t_usec, ctx.ti_usec);
}

/*!\test
 * Entering the same context twice does not have any effect.
 *
 * Not even the current time is fetched from the system.
 */
void test_enter_same_context_is_filtered()
{
    cppcut_assert_equal(uint64_t(0), ctx.t_usec);

    struct timespec t = { .tv_sec = 82, .tv_nsec = 918374, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_null(stats_context_switch(&ctx));

    mock_os->check();
    cppcut_assert_equal(uint64_t(0), ctx.t_usec);

    cppcut_assert_equal(&ctx, stats_context_switch(&ctx));

    mock_os->check();
    cppcut_assert_equal(uint64_t(0), ctx.t_usec);

    struct stats_context next;
    stats_context_reset(&next);

    t.tv_sec = 85;
    t.tv_nsec = 6000000;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_equal(&ctx, stats_context_switch(&next));
    cppcut_assert_equal(uint64_t(0), next.t_usec);
    cppcut_assert_equal(uint64_t(3U * 1000U * 1000U + 5082U), ctx.t_usec);
}

/*!\test
 * Perform several context switches, check if times are accumulated correctly.
 */
void test_chain_of_context_switches()
{
    struct stats_context a;
    struct stats_context b;
    struct stats_context c;

    stats_context_reset(&a);
    stats_context_reset(&b);
    stats_context_reset(&c);

    struct timespec t = { .tv_sec = 731, .tv_nsec = 4235891, };

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_null(stats_context_switch(&ctx));

    /* ctx: 995764424 ns -> 995764 us */
    t.tv_sec = 732;
    t.tv_nsec = 315;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&ctx, stats_context_switch(&a));
    cppcut_assert_equal(uint64_t(995764), ctx.t_usec);

    /* a: 2059248649 ns -> 2059249 us */
    t.tv_sec = 734;
    t.tv_nsec = 59248964;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&a, stats_context_switch(&b));
    cppcut_assert_equal(uint64_t(2059249), a.t_usec);

    /* b: 20152 ns -> 20 us */
    t.tv_sec = 734;
    t.tv_nsec = 59269116;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&b, stats_context_switch(&c));
    cppcut_assert_equal(uint64_t(20), b.t_usec);

    /* c: 940743229 ns -> 940743 us */
    t.tv_sec = 735;
    t.tv_nsec = 12345;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&c, stats_context_switch(&ctx));
    cppcut_assert_equal(uint64_t(940743), c.t_usec);

    /* ctx: 3000000000 ns -> 3000000 us */
    t.tv_sec = 738;
    t.tv_nsec = 12345;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&ctx, stats_context_switch(&a));
    cppcut_assert_equal(uint64_t(3995764), ctx.t_usec);

    /* a: 999999999 ns -> 1000000 us */
    t.tv_sec = 739;
    t.tv_nsec = 12344;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&a, stats_context_switch(&b));
    cppcut_assert_equal(uint64_t(3059249), a.t_usec);

    /* b: 5008975245 ns -> 5008975 us */
    t.tv_sec = 744;
    t.tv_nsec = 8987589;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&b, stats_context_switch(&a));
    cppcut_assert_equal(uint64_t(5008995), b.t_usec);

    /* a: 991012411 ns -> 991012 us */
    t.tv_sec = 745;
    t.tv_nsec = 0;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&a, stats_context_switch(&c));
    cppcut_assert_equal(uint64_t(4050261), a.t_usec);

    /* c: 18234519 ns -> 18235 us */
    t.tv_sec = 745;
    t.tv_nsec = 18234519;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&c, stats_context_switch(&ctx));
    cppcut_assert_equal(uint64_t(958978), c.t_usec);

    cppcut_assert_equal(a.ti_usec, a.t_usec);
    cppcut_assert_equal(b.ti_usec, b.t_usec);
    cppcut_assert_equal(c.ti_usec, c.t_usec);

    /* this should be 14013998628 ns -> 14013999 us, but we are off by 1 us due
     * to accumulated rounding errors */
    cppcut_assert_equal(uint64_t(14013998), a.t_usec + b.t_usec + c.t_usec + ctx.t_usec);
}

/*!\test
 * Any two contexts may bear a parent/child relation, in which case it makes
 * sense to accumulate time spent in the child in the parent as well.
 *
 * There are two fields for accumulated time: one for the time spent in the
 * parent, and one for the time spent in the parent and its children. The
 * parent/child relation is expressed ad hoc in code; there is no tree of
 * contexts formally defined anywhere.
 */
void test_switch_from_parent_to_child_context_and_back()
{
    struct stats_context child;
    stats_context_reset(&child);

    struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_null(stats_context_switch(&ctx));

    t.tv_sec = 7;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&ctx, stats_context_switch(&child));

    t.tv_sec = 9;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&child, stats_context_switch_to_parent(&ctx));

    cppcut_assert_equal(uint64_t(7000000), ctx.t_usec);
    cppcut_assert_equal(uint64_t(9000000), ctx.ti_usec);
    cppcut_assert_equal(uint64_t(2000000), child.t_usec);
    cppcut_assert_equal(uint64_t(2000000), child.ti_usec);

    t.tv_sec = 10;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&ctx, stats_context_switch(&child));

    t.tv_sec = 13;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    cppcut_assert_equal(&child, stats_context_switch_to_parent(&ctx));

    cppcut_assert_equal(uint64_t(8000000), ctx.t_usec);
    cppcut_assert_equal(uint64_t(13000000), ctx.ti_usec);
    cppcut_assert_equal(uint64_t(5000000), child.t_usec);
    cppcut_assert_equal(uint64_t(5000000), child.ti_usec);
}

/*!
 * Structure for testing computation of differences between times.
 */
class DeltaT
{
    public:
    const struct timespec start_time;
    const struct timespec end_time;
    const uint64_t expected_delta_us;

    DeltaT(const DeltaT &) = delete;
    DeltaT &operator=(const DeltaT &) = delete;
    DeltaT(DeltaT &&) = default;

    constexpr explicit DeltaT(struct timespec &&start, struct timespec &&end,
                              uint64_t delta):
        start_time(std::move(start)),
        end_time(std::move(end)),
        expected_delta_us(delta)
    {}

    constexpr DeltaT(const std::chrono::seconds &start_s,
                     const std::chrono::nanoseconds &start_ns,
                     const std::chrono::seconds &end_s,
                     const std::chrono::nanoseconds &end_ns,
                     const std::chrono::microseconds &delta):
        DeltaT({.tv_sec = start_s.count(), .tv_nsec = start_ns.count()},
               {.tv_sec = end_s.count(),   .tv_nsec = end_ns.count()},
               delta.count())
    {}
};

static void check_delta_time_computation(const DeltaT &d)
{
    stats_init();
    stats_context_reset(&ctx);
    struct stats_context next;
    stats_context_reset(&next);

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, d.start_time);
    cppcut_assert_null(stats_context_switch(&ctx));
    mock_os->check();
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, d.end_time);
    cppcut_assert_equal(&ctx, stats_context_switch(&next));
    mock_os->check();
    cppcut_assert_equal(d.expected_delta_us, ctx.t_usec);
    mock_messages->check();
}

/*!\test
 * Simple cases of duration computation.
 */
void test_delta_time_computation_simple()
{
    using std::chrono::seconds;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;
    using std::chrono::nanoseconds;

    static constexpr uint64_t MAX_SECONDS = UINT64_MAX / 1000U / 1000U / 1000U;

    cppcut_assert_equal(sizeof(uint64_t), sizeof(time_t));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(1), nanoseconds(),
                                        seconds(1)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(5), nanoseconds(),
                                        seconds(5)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(),
                                        seconds(MAX_SECONDS)));
    check_delta_time_computation(DeltaT(seconds(6), milliseconds(600),
                                        seconds(8), nanoseconds(),
                                        milliseconds(1400)));
    check_delta_time_computation(DeltaT(seconds(6), microseconds(999999),
                                        seconds(8), nanoseconds(),
                                        seconds(1) + microseconds(1)));

    mock_messages->expect_msg_error(EOVERFLOW, LOG_WARNING, "Time seconds overflow");
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS + 1), nanoseconds(),
                                        microseconds(UINT64_MAX)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(709551499),
                                        seconds(MAX_SECONDS) + microseconds(709551)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(709551500),
                                        seconds(MAX_SECONDS) + microseconds(709552)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(709551615),
                                        seconds(MAX_SECONDS) + microseconds(709552)));
    mock_messages->expect_msg_error(EOVERFLOW, LOG_WARNING, "Time nanoseconds overflow");
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(709551616),
                                        microseconds(UINT64_MAX)));
    mock_messages->expect_msg_error(EOVERFLOW, LOG_WARNING, "Time nanoseconds overflow");
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(999999999),
                                        microseconds(UINT64_MAX)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(709550500),
                                        seconds(MAX_SECONDS) + microseconds(709551)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(MAX_SECONDS), nanoseconds(709550499),
                                        seconds(MAX_SECONDS) + microseconds(709550)));

    check_delta_time_computation(DeltaT(seconds(), nanoseconds(1),
                                        seconds(MAX_SECONDS), nanoseconds(709551616),
                                        seconds(MAX_SECONDS) + microseconds(709552)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(500),
                                        seconds(MAX_SECONDS), nanoseconds(709552115),
                                        seconds(MAX_SECONDS) + microseconds(709552)));
    mock_messages->expect_msg_error(EOVERFLOW, LOG_WARNING, "Time nanoseconds overflow");
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(1),
                                        seconds(MAX_SECONDS), nanoseconds(709551617),
                                        microseconds(UINT64_MAX)));
    mock_messages->expect_msg_error(EOVERFLOW, LOG_WARNING, "Time nanoseconds overflow");
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(500),
                                        seconds(MAX_SECONDS), nanoseconds(709552116),
                                        microseconds(UINT64_MAX)));
}

/*!\test
 * Check if rounding is done correctly in duration computation.
 */
void test_delta_time_computation_basic_rounding()
{
    using std::chrono::seconds;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;
    using std::chrono::nanoseconds;

    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(), nanoseconds(999999999),
                                        seconds(1)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(4), nanoseconds(999999999),
                                        seconds(5)));

    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(),
                                        seconds(7), nanoseconds(500000000),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(1),
                                        seconds(7), nanoseconds(500000000),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500),
                                        seconds(7), nanoseconds(500000000),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(501),
                                        seconds(7), nanoseconds(500000000),
                                        microseconds(2499999)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(501),
                                        seconds(7), nanoseconds(500000001),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(502),
                                        seconds(7), nanoseconds(500000001),
                                        microseconds(2499999)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(502),
                                        seconds(7), nanoseconds(500000002),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(503),
                                        seconds(7), nanoseconds(500000002),
                                        microseconds(2499999)));

    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(2), nanoseconds(499999499),
                                        microseconds(2499999)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(2), nanoseconds(499999500),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(2), nanoseconds(499999501),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(2), nanoseconds(499999999),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(2), nanoseconds(500000499),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(), nanoseconds(),
                                        seconds(2), nanoseconds(500000500),
                                        microseconds(2500001)));
}

/*!\test
 * Make sure the "problematic" cases for the fractional part are handled
 * correctly.
 */
void test_delta_time_computation_nsec_wraparound()
{
    using std::chrono::seconds;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;
    using std::chrono::nanoseconds;

    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(1),
                                        seconds(7), nanoseconds(),
                                        seconds(2)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(499),
                                        seconds(7), nanoseconds(),
                                        seconds(2)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500),
                                        seconds(7), nanoseconds(),
                                        seconds(2)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(501),
                                        seconds(7), nanoseconds(),
                                        microseconds(1999999)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(999999999),
                                        seconds(7), nanoseconds(498),
                                        seconds(1)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(999999999),
                                        seconds(7), nanoseconds(499),
                                        microseconds(1000001)));

    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000000),
                                        seconds(8), nanoseconds(),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000000),
                                        seconds(8), nanoseconds(1),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000000),
                                        seconds(8), nanoseconds(499),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000000),
                                        seconds(8), nanoseconds(500),
                                        microseconds(2500001)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000001),
                                        seconds(8), nanoseconds(500),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000001),
                                        seconds(8), nanoseconds(501),
                                        microseconds(2500001)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000002),
                                        seconds(8), nanoseconds(501),
                                        milliseconds(2500)));
    check_delta_time_computation(DeltaT(seconds(5), nanoseconds(500000002),
                                        seconds(8), nanoseconds(502),
                                        microseconds(2500001)));

    check_delta_time_computation(DeltaT(seconds(10), nanoseconds(999999500),
                                        seconds(11), nanoseconds(),
                                        microseconds(1)));
    check_delta_time_computation(DeltaT(seconds(10), nanoseconds(999999501),
                                        seconds(11), nanoseconds(1),
                                        microseconds(1)));
    check_delta_time_computation(DeltaT(seconds(10), nanoseconds(999999501),
                                        seconds(11), nanoseconds(),
                                        microseconds()));
    check_delta_time_computation(DeltaT(seconds(10), nanoseconds(999999999),
                                        seconds(11), nanoseconds(499),
                                        microseconds(1)));
    check_delta_time_computation(DeltaT(seconds(10), nanoseconds(999999999),
                                        seconds(11), nanoseconds(498),
                                        microseconds()));
    check_delta_time_computation(DeltaT(seconds(10), nanoseconds(999999999),
                                        seconds(11), nanoseconds(1),
                                        microseconds()));
}

/*!\test
 * Make sure that wraparounds in the struct timespec absolute times are handled
 * correctly.
 */
void test_delta_time_computation_timespec_wraparound()
{
    using std::chrono::seconds;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;
    using std::chrono::nanoseconds;

    check_delta_time_computation(DeltaT(seconds(UINT64_MAX), nanoseconds(),
                                        seconds(), nanoseconds(),
                                        seconds(1)));

    check_delta_time_computation(DeltaT(seconds(UINT64_MAX), nanoseconds(999999999),
                                        seconds(), nanoseconds(498),
                                        seconds(0)));
    check_delta_time_computation(DeltaT(seconds(UINT64_MAX), nanoseconds(999999999),
                                        seconds(), nanoseconds(499),
                                        microseconds(1)));
    check_delta_time_computation(DeltaT(seconds(UINT64_MAX - 4), milliseconds(500),
                                        seconds(9), milliseconds(800),
                                        seconds(14) + milliseconds(300)));
    check_delta_time_computation(DeltaT(seconds(UINT64_MAX - 4), milliseconds(500),
                                        seconds(9), milliseconds(200),
                                        seconds(13) + milliseconds(700)));
}

/*!\test
 * Test if we can, maybe, add 1 to a counter...
 */
void test_event_counter_basics()
{
    struct stats_event_counter ev;
    stats_event_counter_reset(&ev);

    cppcut_assert_equal(uint32_t(0), ev.count);
    stats_event(&ev);
    cppcut_assert_equal(uint32_t(1), ev.count);
    stats_event(&ev);
    cppcut_assert_equal(uint32_t(2), ev.count);
    stats_event_counter_reset(&ev);
    cppcut_assert_equal(uint32_t(0), ev.count);
}

/*!\test
 * Check if I/O statistics are gathered correctly.
 *
 * With \c struct \c stats_io mostly being an aggregate of the other stats
 * structs, testing of gathering I/O statistics can be kept pretty basic.
 */
void test_io_stastistics()
{
    struct timespec t = { .tv_sec = 4190, .tv_nsec = 14589551, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    /* this might be our "idle" context, or any other context; but we must have
     * entered a context before we can gather I/O statistics */
    cppcut_assert_null(stats_context_switch(&ctx));

    struct stats_io io;
    stats_io_reset(&io);

    cppcut_assert_equal(uint32_t(0), io.ops.count);
    cppcut_assert_equal(uint32_t(0), io.failures.count);
    cppcut_assert_equal(uint64_t(0), io.blocked.t_usec);
    cppcut_assert_equal(size_t(0), io.bytes_transferred);

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    struct stats_context *prev_ctx = stats_io_begin(&io);
    cppcut_assert_equal(&ctx, prev_ctx);

    /* do some I/O...*/
    t.tv_sec = 4191;
    t.tv_nsec = 85312377;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    /* let's assume we have transferred 734782 bytes, and have observed 2
     * failures while doing so (whatever "failures" may mean) */
    stats_io_end(&io, prev_ctx, 2, 734782);

    cppcut_assert_equal(uint32_t(1), io.ops.count);
    cppcut_assert_equal(uint32_t(2), io.failures.count);
    cppcut_assert_equal(uint64_t(1070723), io.blocked.t_usec);
    cppcut_assert_equal(size_t(734782), io.bytes_transferred);
}

/*!\test
 * I/O times are also accumulated in the parent context's ti_usec field.
 */
void test_cumulated_times()
{
    struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_null(stats_context_switch(&ctx));

    struct stats_io io;
    stats_io_reset(&io);

    t.tv_sec = 1;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    struct stats_context *prev_ctx = stats_io_begin(&io);
    cppcut_assert_equal(&ctx, prev_ctx);
    cppcut_assert_equal(uint64_t(1000000), ctx.t_usec);
    cppcut_assert_equal(uint64_t(1000000), ctx.ti_usec);
    cppcut_assert_equal(uint64_t(0), io.blocked.t_usec);
    cppcut_assert_equal(io.blocked.t_usec, io.blocked.ti_usec);

    t.tv_sec = 3;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    stats_io_end(&io, prev_ctx, 0, 15);
    cppcut_assert_equal(uint64_t(1000000), ctx.t_usec);
    cppcut_assert_equal(uint64_t(3000000), ctx.ti_usec);
    cppcut_assert_equal(uint64_t(2000000), io.blocked.t_usec);
    cppcut_assert_equal(io.blocked.t_usec, io.blocked.ti_usec);

    t.tv_sec = 7;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    prev_ctx = stats_io_begin(&io);
    cppcut_assert_equal(&ctx, prev_ctx);
    cppcut_assert_equal(uint64_t(5000000), ctx.t_usec);
    cppcut_assert_equal(uint64_t(7000000), ctx.ti_usec);
    cppcut_assert_equal(uint64_t(2000000), io.blocked.t_usec);
    cppcut_assert_equal(io.blocked.t_usec, io.blocked.ti_usec);

    t.tv_sec = 15;
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    stats_io_end(&io, prev_ctx, 0, 20);
    cppcut_assert_equal(uint64_t(5000000), ctx.t_usec);
    cppcut_assert_equal(uint64_t(15000000), ctx.ti_usec);
    cppcut_assert_equal(uint64_t(10000000), io.blocked.t_usec);
    cppcut_assert_equal(io.blocked.t_usec, io.blocked.ti_usec);
}

}

/*!@}*/
