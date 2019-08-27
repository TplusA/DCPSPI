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

if WITH_CUTTER
TESTS = run_test.sh

if WITH_VALGRIND
TESTS += valgrind.sh
endif

AM_TESTS_ENVIRONMENT = CUTTER="$(CUTTER)" XSLTPROC="$(XSLTPROC)" VALGRIND="$(VALGRIND)"

AM_LDFLAGS = -module -rpath $(libdir) -avoid-version -no-undefined

AM_CPPFLAGS = $(CPPCUTTER_CFLAGS)
AM_CPPFLAGS += -I$(top_srcdir)

AM_CFLAGS = $(CWARNINGS)

AM_CXXFLAGS = $(CXXWARNINGS)

LIBS += $(CPPCUTTER_LIBS)

check_LTLIBRARIES = test_spi.la test_complete.la test_statistics.la

test_spi_la_SOURCES = \
    test_spi.cc \
    mock_os.hh mock_os.cc \
    mock_messages.hh mock_messages.cc \
    mock_spi_hw.hh mock_spi_hw.cc spi_hw_data.hh  \
    mock_expectation.hh
test_spi_la_LIBADD = ../libspi.la ../libstatistics.la
test_spi_la_CFLAGS = $(AM_CFLAGS)
test_spi_la_CXXFLAGS = $(AM_CXXFLAGS)

test_complete_la_SOURCES = \
    test_complete.cc \
    mock_os.hh mock_os.cc \
    mock_messages.hh mock_messages.cc \
    mock_spi_hw.hh mock_spi_hw.cc spi_hw_data.hh \
    mock_gpio.hh mock_gpio.cc \
    mock_expectation.hh
test_complete_la_LIBADD = ../libdcpspi.la ../libspi.la ../libstatistics.la
test_complete_la_CFLAGS = $(AM_CFLAGS)
test_complete_la_CXXFLAGS = $(AM_CXXFLAGS)

test_statistics_la_SOURCES = \
    test_statistics.cc \
    mock_os.hh mock_os.cc \
    mock_messages.hh mock_messages.cc \
    mock_expectation.hh
test_statistics_la_LIBADD = ../libstatistics.la
test_statistics_la_CFLAGS = $(AM_CFLAGS)
test_statistics_la_CXXFLAGS = $(AM_CXXFLAGS)

CLEANFILES = test_report.xml test_report_junit.xml valgrind.xml

EXTRA_DIST = cutter2junit.xslt
EXTRA_DIST += cutter-1_2_4.supp

check-local: check-TESTS
	@if $(GREP) -w cutter $(TEST_LOGS); then \
	    echo "Unit tests failed (check log)"; \
	    exit 1; \
	fi
endif
