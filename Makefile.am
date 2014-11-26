bin_PROGRAMS = dcpspi

html_DATA = README.html

EXTRA_DIST = README.md cppcheck_suppressed.txt

CLEANFILES = README.html

dcpspi_SOURCES = \
    dcpspi.c dcpdefs.h messages.c messages.h \
    named_pipe.c named_pipe.h \
    gpio.c gpio.h spi.c spi.h

AM_CFLAGS = $(CWARNINGS)

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
