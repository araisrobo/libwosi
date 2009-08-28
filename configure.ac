#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

# the autoconf-2.59 can not process this script correctly
AC_PREREQ(2.61)
AC_INIT(libwou, 0.0.1, yishin.li@gmail.com)
AC_CONFIG_SRCDIR([src/wou.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE

dnl -----------------------------------------------
dnl Checks for programs.
dnl -----------------------------------------------
AC_PROG_CC
AC_PROG_CXX
AC_PROG_MAKE_SET
AC_PROG_LIBTOOL

# copied from configure.ac of UrJTAG ,http://urjtag.org,urjtag)
AC_CHECK_FUNC(clock_gettime, [], [ AC_CHECK_LIB(rt, clock_gettime) ])

dnl Use FTDI ftd2xx library?
AC_ARG_WITH([ftd2xx],
  [AS_HELP_STRING([--with-ftd2xx],
    [use ftd2xx library for FTDI-based cables])],
    [], [with_ftd2xx=check])

FTD2XXLIB=
AS_IF([test "x$with_ftd2xx" = xyes -o "x$with_ftd2xx" = xcheck], [
    case $host in
    *cygwin*|*mingw*)
      HAVELIBFTD2XX=yes
      FTD2XXLIB="-lftd2xx"
      ;;
    *)
      AC_CHECK_LIB([ftd2xx], [FT_OpenEx], [
        HAVELIBFTD2XX=yes
        LIBS="-lftd2xx $LIBS"
      ],[
        AC_MSG_WARN([*** libftd2xx not found. No support for FTDI-based USB JTAG cables via libftd2xx.])
      ])
      ;;
    esac
],[
  AS_IF([test "x$with_ftd2xx" != xno], [
    HAVELIBFTD2XX=yes
    case $host in
    *cygwin*|*mingw*)
      CFLAGS="$CFLAGS -I$with_ftd2xx"
      AS_IF([test -d "$with_ftd2xx/i386"], [
        FTD2XXLIB="$with_ftd2xx/i386/ftd2xx.lib"
      ],[
        FTD2XXLIB="$with_ftd2xx/ftd2xx.lib"
      ])
      # notice: this method failed to build binaries at ./tests/
      #         because FTD2XXLIB got removed by *_LDADD=... macro
      # building library is okay though
      ;;
    *)
      CFLAGS="$CFLAGS -I$with_ftd2xx -L$with_ftd2xx"
      LIBS="-lftd2xx $LIBS"
      ;;
    esac
  ],)
])
AS_IF([test "x$HAVELIBFTD2XX" = xyes], [
  AM_CONDITIONAL(HAVE_LIBFTD2XX, true)
  AC_DEFINE(HAVE_LIBFTD2XX, 1, [define if you have libftd2xx])
],[
  AM_CONDITIONAL(HAVE_LIBFTD2XX, false)
])
AC_SUBST(FTD2XXLIB)


# Macro for driver include check
AC_DEFUN([CHECK_DRIVER], [
    AS_IF([echo "$1" | $GREP -q $3], [
       AC_DEFINE([$4], 1, [define if $3 is enabled])
       AM_CONDITIONAL([$4], true)
       $2="${$2}$3 "
    ], [
       AM_CONDITIONAL([$4], false)
    ])
])


# Enable lowlevel drivers
AC_DEFUN([DEF_ENABLE_LOWLEVELDRIVERS], [\
ftdi ftd2xx])
AC_ARG_ENABLE(lowlevel,
[AS_HELP_STRING([--enable-lowlevel], [Enable default set or specific lowlevel drivers:])]
[AS_HELP_STRING([],                  ['default' enables:])]
[AS_HELP_STRING([],DEF_ENABLE_LOWLEVELDRIVERS)]
,
[AS_CASE([${enableval}],
    [yes],  [lowleveldrivers=default],
    [no],   [lowleveldrivers=none],
    [none], [lowleveldrivers=none],
            [lowleveldrivers=`echo ${enableval} | $SED -e 's/,/ /g'`])]
,
[lowleveldrivers=default])
# expand 'default' to default enabled lowlevel drivers
lowleveldrivers=`echo ${lowleveldrivers} | $SED -e "s/default/DEF_ENABLE_LOWLEVELDRIVERS/"`
# automatically disable lowlevel drivers when a required feature is not available
AS_IF([test "x$HAVELIBFTDI" != "xyes"], [
    lowleveldrivers=`echo ${lowleveldrivers} | $SED -e "s/ftdi//"`
])
AS_IF([test "x$HAVELIBFTD2XX" != "xyes"], [
    lowleveldrivers=`echo ${lowleveldrivers} | $SED -e "s/ftd2xx//"`
])
#
enabled_lowlevel_drivers=''
CHECK_DRIVER([$lowleveldrivers], [enabled_lowlevel_drivers], [ftd2xx], [ENABLE_LOWLEVEL_FTD2XX])
CHECK_DRIVER([$lowleveldrivers], [enabled_lowlevel_drivers], [ftdi],   [ENABLE_LOWLEVEL_FTDI])


dnl # Checks for library functions.
dnl AC_FUNC_FORK
dnl AC_FUNC_MALLOC
dnl AC_FUNC_SELECT_ARGTYPES
dnl AC_TYPE_SIGNAL
dnl AC_CHECK_FUNCS([gettimeofday inet_ntoa memset select socket strerror])

dnl -----------------------------------------------
dnl Generates Makefile's, configuration files and scripts
dnl -----------------------------------------------
AC_OUTPUT([
        Makefile
        src/Makefile
        src/wou/Makefile
        tests/Makefile
        wou.pc
])

dnl
dnl Configuration summary
dnl
# AS_IF([test ${HAVELIBUSB:-no} != no], [
#    FLAG_HAVELIBUSB=yes
# ], [
#    FLAG_HAVELIBUSB=no
# ])
AS_IF([test ${HAVELIBFTDI:-no} != no], [
   FLAG_HAVELIBFTDI=yes
], [
   FLAG_HAVELIBFTDI=no
])
AS_IF([test ${HAVELIBFTD2XX:-no} != no], [
   FLAG_HAVELIBFTD2XX=yes
], [
   FLAG_HAVELIBFTD2XX=no
])
AC_MSG_NOTICE([

wou is now configured for

dnl  Detected libusb      : $FLAG_HAVELIBUSB
dnl  Detected libftdi     : $FLAG_HAVELIBFTDI
  Detected libftd2xx   : $FLAG_HAVELIBFTD2XX
dnl  Bus drivers          : $enabled_bus_drivers
dnl  Cable drivers        : $enabled_cable_drivers
  Lowlevel drivers     : $enabled_lowlevel_drivers
])

