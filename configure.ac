AC_PREREQ([2.69])
AC_INIT([DCPSPI], [1.3], [r.tiemann@ta-hifi.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([1.11 -Wall -Werror foreign])
AM_SILENT_RULES([yes])
AC_CONFIG_SRCDIR([dcpspi.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AC_USE_SYSTEM_EXTENSIONS()

AC_ARG_VAR([CWARNINGS], [strict set of C compiler warnings])
AC_ARG_VAR([CXXWARNINGS], [strict set of C++ compiler warnings])
AC_ARG_VAR([CRELAXEDWARNINGS], [options for relaxing some compiler warnings for various files])
AC_ARG_VAR([CXXRELAXEDWARNINGS], [options for relaxing some C++ compiler warnings for various files])
AC_ARG_VAR([DEBIAN_SYSROOT_DIR], [path to Debian system root to use when cross-building a Debian package])
AC_ARG_VAR([DEBIAN_SYSROOT_LIBPATH], [colon-separated list of paths to search for libraries when cross-building a Debian package])

AC_ARG_ENABLE([valgrind],
              [AS_HELP_STRING([--disable-valgrind],
                              [disable an extra run of unit tests with Valgrind on ``make check''])],
              [],
              [enable_valgrind=yes])

# Checks for programs.
AC_PROG_CXX
AC_PROG_AWK
AC_PROG_CC
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
AM_PROG_AR
AC_PROG_LIBTOOL
AC_CHECK_PROGS([MARKDOWN], [markdown])
AC_CHECK_PROGS([DOXYGEN], [doxygen])
AC_CHECK_PROGS([DPKG_BUILDPACKAGE], [dpkg-buildpackage])
AC_CHECK_PROGS([DPKG_ARCHITECTURE], [dpkg-architecture])
AC_CHECK_PROGS([XSLTPROC], [xsltproc])
AC_CHECK_PROGS([VALGRIND], [valgrind])
AC_CHECK_PROGS([AUTOREVISION], [autorevision])

m4_ifdef([AC_CHECK_CUTTER],
[
AC_CHECK_CUTTER
AC_CHECK_CPPCUTTER
AC_CHECK_COVERAGE
],
[ac_cv_use_cutter=no])

# Checks for libraries.
AC_SEARCH_LIBS([clock_gettime], [rt])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h limits.h stdint.h stdlib.h string.h sys/ioctl.h syslog.h unistd.h])

m4_ifdef([AC_CHECK_CUTTER],
[
save_CPPFLAGS="$CPPFLAGS"
CPPFLAGS="$pkg_cv_CUTTER_CFLAGS"
AC_CHECK_HEADER([cutter.h], [], [ac_cv_use_cutter=no])
CPPFLAGS="$save_CPPFLAGS"

AC_LANG_PUSH([C++])
save_CPPFLAGS="$CPPFLAGS"
CPPFLAGS="$pkg_cv_CPPCUTTER_CFLAGS"
AC_CHECK_HEADER([cppcutter.h], [], [ac_cv_use_cutter=no])
CPPFLAGS="$save_CPPFLAGS"
AC_LANG_POP([C++])
])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_OFF_T
AC_C_RESTRICT
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_CHECK_FUNCS([mkfifo strerror])

# Debian architecture name for debian/control.in
AS_IF([test "x$ac_cv_prog_DPKG_ARCHITECTURE" != x],
      [DEBIAN_ARCHITECTURE_NAME=$($ac_cv_prog_DPKG_ARCHITECTURE -qDEB_HOST_ARCH -t$host_alias)],
      [DEBIAN_ARCHITECTURE_NAME=])
AC_SUBST([DEBIAN_ARCHITECTURE_NAME])

# arguments for dh_shlibdeps while building Debian package
AS_IF([test "x$DEBIAN_SYSROOT_DIR" != x],
      [DH_SHLIBDEPS_SYSROOT_ARGS="--dpkg-shlibdeps-params='--admindir=\$(DEBIAN_SYSROOT_DIR)/var/lib/dpkg --ignore-missing-info'"],
      [DH_SHLIBDEPS_SYSROOT_ARGS="--dpkg-shlibdeps-params='--ignore-missing-info'"])
AS_IF([test "x$DEBIAN_SYSROOT_LIBPATH" != x],
      [DH_SHLIBDEPS_LIBPATH_ARGS="-l\$(DEBIAN_SYSROOT_LIBPATH)"],
      [DH_SHLIBDEPS_LIBPATH_ARGS=])
DH_SHLIBDEPS_ARGS="$DH_SHLIBDEPS_SYSROOT_ARGS $DH_SHLIBDEPS_LIBPATH_ARGS"
AC_SUBST([DH_SHLIBDEPS_ARGS])

AM_CONDITIONAL([WITH_CUTTER], [test "x$ac_cv_use_cutter" = "xyes"])
AM_CONDITIONAL([WITH_VALGRIND], [test "x$enable_valgrind" = "xyes"])
AM_CONDITIONAL([CAN_BUILD_DEBIAN_PACKAGE], [test "x$ac_cv_prog_DPKG_BUILDPACKAGE" != x && test "x$DEBIAN_ARCHITECTURE_NAME" != x])

AC_CONFIG_FILES([Makefile tests/Makefile Doxyfile])
AC_CONFIG_FILES([versioninfo.cache])
AC_CONFIG_FILES([debian/control])
AC_CONFIG_FILES([tests/run_test.sh], [chmod +x tests/run_test.sh])
AC_CONFIG_FILES([tests/valgrind.sh], [chmod +x tests/valgrind.sh])
AC_OUTPUT
