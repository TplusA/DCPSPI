/*
 * Copyright (C) 2018, 2019  T+A elektroakustik GmbH & Co. KG
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

#ifndef STATISTICS_H
#define STATISTICS_H

#include <stdint.h>

#include "os.h"

struct stats_context
{
    uint64_t t_usec;
    uint64_t ti_usec;

    struct timespec context_entered;  /* private */
};

struct stats_event_counter
{
    uint32_t count;
};

struct stats_io
{
    struct stats_event_counter ops;
    struct stats_event_counter failures;
    struct stats_context blocked;
    size_t bytes_transferred;
};

#ifdef __cplusplus
extern "C" {
#endif

void stats_init(void);

void stats_context_reset(struct stats_context *ctx);
struct stats_context *stats_context_switch(struct stats_context *ctx);
struct stats_context *
stats_context_switch_to_parent(struct stats_context *parent_ctx);

void stats_event_counter_reset(struct stats_event_counter *cnt);
void stats_event(struct stats_event_counter *cnt);
void stats_events(struct stats_event_counter *cnt, uint32_t n);

void stats_io_reset(struct stats_io *io);
struct stats_context *stats_io_begin(struct stats_io *io);
void stats_io_end(struct stats_io *io, struct stats_context *previous_ctx,
                  size_t failures, size_t bytes);

#ifdef __cplusplus
}
#endif

#endif /* !STATISTICS_H */
