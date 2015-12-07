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

check_LTLIBRARIES = test_spi.la

test_spi_la_SOURCES = \
    test_spi.cc \
    mock_os.hh mock_os.cc \
    mock_messages.hh mock_messages.cc \
    mock_spi_hw.hh mock_spi_hw.cc \
    mock_expectation.hh
test_spi_la_LIBADD = ../libspi.la
test_spi_la_CFLAGS = $(AM_CFLAGS)
test_spi_la_CXXFLAGS = $(AM_CXXFLAGS)

CLEANFILES = test_report.xml test_report_junit.xml valgrind.xml

EXTRA_DIST = cutter2junit.xslt
EXTRA_DIST += cutter-1_2_4.supp

check-local: check-TESTS
	@if $(GREP) -w cutter $(TEST_LOGS); then \
	    echo "Unit tests failed (check log)"; \
	    exit 1; \
	fi
endif
