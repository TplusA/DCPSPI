AC_PREREQ([2.69])
AC_INIT([DCPSPI], [0.1], [r.tiemann@ta-hifi.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([1.11 -Wall -Werror foreign])
AM_SILENT_RULES([yes])
AC_CONFIG_SRCDIR([dcpspi.c])
AC_CONFIG_HEADERS([config.h])

AC_USE_SYSTEM_EXTENSIONS()

AC_ARG_VAR([CWARNINGS], [strict set of C compiler warnings])
AC_ARG_VAR([CRELAXEDWARNINGS], [options for relaxing some compiler warnings for various files])
AC_ARG_VAR([DEBIAN_SYSROOT_DIR], [path to Debian system root to use when cross-building a Debian package])
AC_ARG_VAR([DEBIAN_SYSROOT_LIBPATH], [colon-separated list of paths to search for libraries when cross-building a Debian package])

# Checks for programs.
AC_PROG_CC
AM_PROG_AR
PKG_PROG_PKG_CONFIG([0.26])
AC_CHECK_PROGS([MARKDOWN], [markdown])
AC_CHECK_PROGS([DOXYGEN], [doxygen])
AC_CHECK_PROGS([DPKG_BUILDPACKAGE], [dpkg-buildpackage])
AC_CHECK_PROGS([DPKG_ARCHITECTURE], [dpkg-architecture])

# Checks for libraries.
AC_SEARCH_LIBS([clock_gettime], [rt])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h limits.h stdint.h stdlib.h string.h sys/ioctl.h syslog.h unistd.h])

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

AM_CONDITIONAL([CAN_BUILD_DEBIAN_PACKAGE], [test "x$ac_cv_prog_DPKG_BUILDPACKAGE" != x && test "x$DEBIAN_ARCHITECTURE_NAME" != x])

AC_CONFIG_FILES([Makefile Doxyfile])
AC_CONFIG_FILES([debian/control])
AC_OUTPUT
