#
# Copyright (C) 2015, 2016, 2018, 2019  T+A elektroakustik GmbH & Co. KG
#
# This file is part of DCPSPI.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA  02110-1301, USA.
#

SUBDIRS = . tests

ACLOCAL_AMFLAGS = -I m4

bin_PROGRAMS = dcpspi

if WITH_MARKDOWN
html_DATA = README.html
endif

EXTRA_DIST = README.md cppcheck_suppressed.txt

CLEANFILES = README.html

dcpspi_SOURCES = \
    dcpspi.c dcpspi_process.h dcpdefs.h os.c os.h messages.c messages.h \
    messages_signal.c messages_signal.h \
    named_pipe.c named_pipe.h \
    hexdump.c hexdump.h \
    gpio.c gpio.h spi.h spi_hw.c spi_hw.h

AM_CFLAGS = $(CWARNINGS)

noinst_LTLIBRARIES = libspi.la libdcpspi.la libstatistics.la

dcpspi_LDADD = $(noinst_LTLIBRARIES)

libspi_la_SOURCES = spi.c spi.h spi_hw.h dcpdefs.h messages.h os.h
libspi_la_CFLAGS = $(AM_CFLAGS)

libdcpspi_la_SOURCES = \
    dcpspi_process.c dcpspi_process.h dcpdefs.h \
    named_pipe.h gpio.h spi.h os.h messages.h
libdcpspi_la_CFLAGS = $(AM_CFLAGS)

libstatistics_la_SOURCES = statistics.c statistics.h messages.h os.h
libstatistics_la_CFLAGS = $(AM_CFLAGS)

BUILT_SOURCES = versioninfo.h

CLEANFILES += $(BUILT_SOURCES)

versioninfo.h: versioninfo.cache
	(cd $(top_srcdir) && $(AUTOREVISION) -t h -o $(abs_top_builddir)/versioninfo.cache >$(abs_top_builddir)/$@)

.md.html:
	$(MARKDOWN) $< >$@
