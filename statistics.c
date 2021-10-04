/*
 * Copyright (C) 2018, 2019, 2021  T+A elektroakustik GmbH & Co. KG
 *
 * This file is part of DCPSPI.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <errno.h>

#include "statistics.h"
#include "messages.h"

#define S_IN_NANO   (1000U * 1000U * 1000U)

static bool seconds_to_nanoseconds(const uint64_t s, uint64_t *ns)
{
    // cppcheck-suppress signConversion
    static const uint64_t max = UINT64_MAX / S_IN_NANO;

    if(s <= max)
    {
        *ns = s * (uint64_t)S_IN_NANO;
        return true;
    }
    else
    {
        msg_error(EOVERFLOW, LOG_WARNING, "Time seconds overflow");
        *ns = UINT64_MAX;
        return false;
    }
}

static bool add_saturated(const uint64_t add, const uint64_t overflow_value,
                          bool warn_on_overflow, uint64_t *t)
{
    if(*t <= UINT64_MAX - add)
    {
        *t += add;
        return true;
    }
    else
    {
        if(warn_on_overflow)
            msg_error(EOVERFLOW, LOG_WARNING, "Time nanoseconds overflow");

        *t = overflow_value;

        return false;
    }
}

static uint64_t compute_delta_usec(const struct timespec *past,
                                   const struct timespec *now)
{
    uint64_t past_nsec;
    uint64_t now_nsec;
    bool ok;

    if(past->tv_sec <= now->tv_sec)
    {
        past_nsec = 0;
        ok = seconds_to_nanoseconds(now->tv_sec - past->tv_sec, &now_nsec);
    }
    else
    {
        now_nsec = 0;
        ok = seconds_to_nanoseconds(past->tv_sec - now->tv_sec, &past_nsec);
    }

    if(!ok)
        return UINT64_MAX;

    ok = (past->tv_nsec <= now->tv_nsec)
        ? add_saturated(now->tv_nsec - past->tv_nsec, UINT64_MAX, true, &now_nsec)
        : add_saturated(past->tv_nsec - now->tv_nsec, UINT64_MAX, true, &past_nsec);

    if(!ok)
        return UINT64_MAX;

    if(past_nsec <= now_nsec)
    {
        now_nsec -= past_nsec;

        if(add_saturated(500, now_nsec, false, &now_nsec))
            return now_nsec / 1000U;
        else
            return now_nsec / 1000U + ((now_nsec % 1000U) >= 500);
    }
    else
    {
        past_nsec -= now_nsec;

        if(add_saturated(500, past_nsec, false, &past_nsec))
            return past_nsec / 1000U;
        else
            return past_nsec / 1000U + ((past_nsec % 1000U) >= 500);
    }

    return UINT64_MAX;
}

static bool add_to_time(struct stats_context *ctx, uint64_t usec)
{
    if(ctx->t_usec <= UINT64_MAX - usec)
    {
        ctx->t_usec += usec;
        return true;
    }

    msg_error(EOVERFLOW, LOG_WARNING, "Cumulative time overflow");
    ctx->t_usec = UINT64_MAX;

    return false;
}

static bool add_to_itime(struct stats_context *ctx, uint64_t usec)
{
    if(ctx->ti_usec <= UINT64_MAX - usec)
    {
        ctx->ti_usec += usec;
        return true;
    }

    msg_error(EOVERFLOW, LOG_WARNING, "Cumulative inclusive time overflow");
    ctx->ti_usec = UINT64_MAX;

    return false;
}

static struct stats_context *global_current_content;

void stats_init(void)
{
    global_current_content = NULL;
}

void stats_context_reset(struct stats_context *const ctx)
{
    ctx->t_usec = 0;
    ctx->ti_usec = 0;
}

struct stats_context *stats_context_switch(struct stats_context *const ctx)
{
    if(ctx == NULL)
        return NULL;

    struct stats_context *const prev = global_current_content;

    if(prev == ctx)
        return ctx;

    const int temp =
        os_clock_gettime(CLOCK_MONOTONIC_RAW, &ctx->context_entered);

    global_current_content = ctx;

    if(temp == 0)
    {
        if(prev != NULL)
        {
            const uint64_t delta = compute_delta_usec(&prev->context_entered,
                                                      &ctx->context_entered);
            add_to_time(prev, delta);
            add_to_itime(prev, delta);
        }
    }
    else
    {
        msg_error(errno, LOG_ERR, "Failed to get current time");
        ctx->context_entered.tv_sec = 0;
        ctx->context_entered.tv_nsec = 0;
    }

    return prev;
}

struct stats_context *
stats_context_switch_to_parent(struct stats_context *parent_ctx)
{
    const uint64_t prev_time = global_current_content->t_usec;
    struct stats_context *child_ctx = stats_context_switch(parent_ctx);

    add_to_itime(parent_ctx, child_ctx->t_usec - prev_time);

    return child_ctx;
}

void stats_event_counter_reset(struct stats_event_counter *const cnt)
{
    cnt->count = 0;
}

void stats_event(struct stats_event_counter *cnt)
{
    if(cnt != NULL && cnt->count < UINT32_MAX)
        ++cnt->count;
}

void stats_events(struct stats_event_counter *cnt, uint32_t n)
{
    if(cnt == NULL || n == 0)
        return;

    if(cnt->count <= UINT32_MAX - n)
        cnt->count += n;
    else
        cnt->count = UINT32_MAX;
}

void stats_io_reset(struct stats_io *const io)
{
    stats_event_counter_reset(&io->ops);
    stats_event_counter_reset(&io->failures);
    stats_context_reset(&io->blocked);
    io->bytes_transferred = 0;
}

struct stats_context *stats_io_begin(struct stats_io *const io)
{
    if(io == NULL)
        return NULL;

    int save_errno = errno;
    stats_event(&io->ops);
    struct stats_context *ctx = stats_context_switch(&io->blocked);
    errno = save_errno;
    return ctx;
}

void stats_io_end(struct stats_io *const io,
                  struct stats_context *const previous_ctx,
                  size_t failures, size_t bytes)
{
    if(io == NULL)
        return;

    int save_errno = errno;

    stats_events(&io->failures, failures);

    if(bytes > 0)
    {
        if(io->bytes_transferred <= SIZE_MAX - bytes)
            io->bytes_transferred += bytes;
        else
            io->bytes_transferred = SIZE_MAX;
    }

    stats_context_switch_to_parent(previous_ctx);

    errno = save_errno;
}
