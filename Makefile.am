SUBDIRS = . tests

ACLOCAL_AMFLAGS = -I m4

bin_PROGRAMS = dcpspi

html_DATA = README.html

EXTRA_DIST = README.md cppcheck_suppressed.txt

CLEANFILES = README.html

dcpspi_SOURCES = \
    dcpspi.c dcpdefs.h os.c os.h messages.c messages.h \
    named_pipe.c named_pipe.h \
    gpio.c gpio.h spi.h spi_hw.c spi_hw.h

AM_CFLAGS = $(CWARNINGS)

noinst_LTLIBRARIES = libspi.la

dcpspi_LDADD = $(noinst_LTLIBRARIES)

libspi_la_SOURCES = spi.c spi.h spi_hw.h dcpdefs.h messages.h os.h
libspi_la_CFLAGS = $(AM_CFLAGS)

BUILT_SOURCES = versioninfo.h

CLEANFILES += $(BUILT_SOURCES)

versioninfo.h: versioninfo.cache
	(cd $(top_srcdir) && $(AUTOREVISION) -t h -o $(abs_top_builddir)/versioninfo.cache >$(abs_top_builddir)/$@)

.md.html:
	$(MARKDOWN) $< >$@

if CAN_BUILD_DEBIAN_PACKAGE
DEBIAN_PACKAGE_FILES = debian/changelog debian/compat debian/control debian/rules

EXTRA_DIST += $(DEBIAN_PACKAGE_FILES)

deb: dist
	rm -rf "$(distdir)"
	GZIP=$(GZIP_ENV) gzip -dc "$(distdir).tar.gz" | $(am__untar)
	$(am__cd) "$(distdir)" && DH_SHLIBDEPS_ARGS="$(DH_SHLIBDEPS_ARGS)" $(DPKG_BUILDPACKAGE) -rfakeroot -D -b -t$(host_alias)
	rm -rf "$(distdir)"
else
deb:
	@echo "Cannot build Debian package, requirements not met."
	exit 1
endif
