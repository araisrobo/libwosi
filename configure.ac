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

dnl check for libusb-1.0
AS_IF([test "x$with_libusb" != xno], [
  save_LIBS=$LIBS
  save_CPPFLAGS=$CPPFLAGS
  PKG_CHECK_MODULES(LIBUSB, libusb-1.0, have_libusb=yes, have_libusb=no)
  AS_IF([test "$have_libusb" = "yes"],[
    LIBUSB_CFLAGS="$CFLAGS $LIBUSB_CFLAGS"
    LIBUSB_LIBS="$USB_LIBS $LIBUSB_LIBS"
  ],)
  LIBS="$LIBS $LIBUSB_LIBS"
  CPPFLAGS="$CPPFLAGS $LIBUSB_CFLAGS"
  AC_CHECK_FUNC([libusb_get_device_list], [
    AC_DEFINE(HAVE_LIBUSB1, 1, [Define if you have libusb-1.0])
    HAVELIBUSB=1.0
  ],[
    AS_IF([test "x$with_libusb" = x1.0], [
      AC_MSG_ERROR([*** libusb-1.0 not detected.])
    ],[
      AC_MSG_WARN([*** libusb-1.0 not detected. No support for async mode for FTDI cables.])
    ])
    LIBS=$save_LIBS
    CPPFLAGS=$save_CPPFLAGS
  ])
],)
if test "x$HAVELIBUSB" != "xno"; then
  AC_DEFINE(HAVE_LIBUSB, 1, [Define if you have libusb])
fi
AM_CONDITIONAL(HAVE_LIBUSB, [test "x$HAVELIBUSB" != "xno"])


dnl Use FTDI library?
AC_ARG_WITH([libftdi],
  [AS_HELP_STRING([--with-libftdi],
    [use libftdi for FTDI-based cables])],
    [], [with_libftdi=check])

HAVELIBFTDI=no
AS_IF([test "x$with_libftdi" != xno], [
  save_LIBS=$LIBS
  save_CPPFLAGS=$CPPFLAGS
  PKG_CHECK_MODULES(LIBFTDI, libftdi, [:;], [dnl
    AS_IF([test "x$with_libftdi" != xyes -a "x$with_libftdi" != xcheck], [
      LIBFTDI_LIBS="-L$with_libftdi -lftdi"
      LIBFTDI_CFLAGS="-I$with_libftdi"
    ],[
      AC_PATH_TOOL(LIBFTDI_CONFIG, libftdi-config, no)
      AS_IF([test "$LIBFTDI_CONFIG" != "no"],[
        LIBFTDI_LIBS=`$LIBFTDI_CONFIG --libs`
        LIBFTDI_CFLAGS=`$LIBFTDI_CONFIG --cflags`
      ],)
    ])
  ])
  LIBS="$LIBFTDI_LIBS $LIBS"
  CPPFLAGS="$CPPFLAGS $LIBFTDI_CFLAGS"
  AC_CHECK_FUNC([ftdi_usb_open], [
    AC_DEFINE(HAVE_LIBFTDI, 1, [Define if you have libftdi])
    HAVELIBFTDI=yes
  ],[
    AC_MSG_WARN([*** libftdi not detected. No support for FTDI-based USB JTAG cables via libftdi.])
    LIBS=$save_LIBS
    CPPFLAGS=$save_CPPFLAGS
  ])
  AC_CHECK_FUNC([ftdi_read_data_submit], [
    AS_IF([test "x$HAVELIBUSB" = "x0.1"], [
      AC_MSG_ERROR([this libftdi cannot be used with libusb-0.1, libusb-1.0 is needed])
    ],)
    AC_DEFINE(HAVE_LIBFTDI_ASYNC_MODE, 1, [Define if libftdi support async mode])
    HAVELIBFTDI_ASYNCMODE=yes
  ], [
    AS_IF([test "x$HAVELIBUSB" = "x1.0"], [
      AC_MSG_ERROR([this libftdi cannot be used with libusb-1.0, libusb-0.1 is needed.
                    did you configure libftdi with --with-async-mode ?])
    ],)
  ])
],)
AM_CONDITIONAL(HAVE_LIBFTDI, [test "x$HAVELIBFTDI" = "xyes"])
AM_CONDITIONAL(HAVE_LIBFTDI_ASYNC_MODE, [test "x$HAVELIBFTDI_ASYNCMODE" = "xyes"])


dnl dnl Use FTDI ftd2xx library?
dnl AC_ARG_WITH([ftd2xx],
dnl   [AS_HELP_STRING([--with-ftd2xx],
dnl     [use ftd2xx library for FTDI-based cables])],
dnl     [], [with_ftd2xx=check])
dnl 
dnl FTD2XXLIB=
dnl AS_IF([test "x$with_ftd2xx" = xyes -o "x$with_ftd2xx" = xcheck], [
dnl     case $host in
dnl     *cygwin*|*mingw*)
dnl       HAVELIBFTD2XX=yes
dnl       FTD2XXLIB="-lftd2xx"
dnl       ;;
dnl     *)
dnl       AC_CHECK_LIB([ftd2xx], [FT_OpenEx], [
dnl         HAVELIBFTD2XX=yes
dnl         LIBS="-lftd2xx $LIBS"
dnl       ],[
dnl         AC_MSG_WARN([*** libftd2xx not found. No support for FTDI-based USB JTAG cables via libftd2xx.])
dnl       ])
dnl       ;;
dnl     esac
dnl ],[
dnl   AS_IF([test "x$with_ftd2xx" != xno], [
dnl     HAVELIBFTD2XX=yes
dnl     case $host in
dnl     *cygwin*|*mingw*)
dnl       CFLAGS="$CFLAGS -I$with_ftd2xx"
dnl       AS_IF([test -d "$with_ftd2xx/i386"], [
dnl         FTD2XXLIB="$with_ftd2xx/i386/ftd2xx.lib"
dnl       ],[
dnl         FTD2XXLIB="$with_ftd2xx/ftd2xx.lib"
dnl       ])
dnl       # notice: this method failed to build binaries at ./tests/
dnl       #         because FTD2XXLIB got removed by *_LDADD=... macro
dnl       # building library is okay though
dnl       ;;
dnl     *)
dnl       CFLAGS="$CFLAGS -I$with_ftd2xx -L$with_ftd2xx"
dnl       LIBS="-lftd2xx $LIBS"
dnl       ;;
dnl     esac
dnl   ],)
dnl ])
dnl AS_IF([test "x$HAVELIBFTD2XX" = xyes], [
dnl   AM_CONDITIONAL(HAVE_LIBFTD2XX, true)
dnl   AC_DEFINE(HAVE_LIBFTD2XX, 1, [define if you have libftd2xx])
dnl ],[
dnl   AM_CONDITIONAL(HAVE_LIBFTD2XX, false)
dnl ])
dnl AC_SUBST(FTD2XXLIB)


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
dnl orig: # AS_IF([test ${HAVELIBUSB:-no} != no], [
dnl orig: #    FLAG_HAVELIBUSB=yes
dnl orig: # ], [
dnl orig: #    FLAG_HAVELIBUSB=no
dnl orig: # ])
dnl orig: AS_IF([test ${HAVELIBFTDI:-no} != no], [
dnl orig:    FLAG_HAVELIBFTDI=yes
dnl orig: ], [
dnl orig:    FLAG_HAVELIBFTDI=no
dnl orig: ])
dnl orig: AS_IF([test ${HAVELIBFTD2XX:-no} != no], [
dnl orig:    FLAG_HAVELIBFTD2XX=yes
dnl orig: ], [
dnl orig:    FLAG_HAVELIBFTD2XX=no
dnl orig: ])
dnl orig: AC_MSG_NOTICE([
dnl orig: 
dnl orig: wou is now configured for
dnl orig: 
dnl orig: dnl  Detected libusb      : $FLAG_HAVELIBUSB
dnl orig: dnl  Detected libftdi     : $FLAG_HAVELIBFTDI
dnl orig:   Detected libftd2xx   : $FLAG_HAVELIBFTD2XX
dnl orig: dnl  Bus drivers          : $enabled_bus_drivers
dnl orig: dnl  Cable drivers        : $enabled_cable_drivers
dnl orig:   Lowlevel drivers     : $enabled_lowlevel_drivers
dnl orig: ])
FLAG_HAVELIBUSB=$HAVELIBUSB
AS_IF([test "x$HAVELIBFTDI_ASYNCMODE" = "xyes"], [
  FLAG_HAVELIBFTDI_ASYNCMODE="(have async mode)"
],[
  FLAG_HAVELIBFTDI_ASYNCMODE="(no async mode)"
])
AC_DEFUN([MAKE_YESNO_VAR],[dnl
  AS_IF([test ${$1:-$2} != $2], [dnl
     FLAG_$1=yes
  ], [dnl
     FLAG_$1=no
  ])
])
MAKE_YESNO_VAR([HAVELIBFTDI], [no])
MAKE_YESNO_VAR([HAVELIBFTD2XX], [no])
AC_MSG_NOTICE([

wou is now configured for

  Detected libusb      : $FLAG_HAVELIBUSB
  Detected libftdi     : $FLAG_HAVELIBFTDI $FLAG_HAVELIBFTDI_ASYNCMODE
  Detected libftd2xx   : $FLAG_HAVELIBFTD2XX
  Lowlevel drivers     : $enabled_lowlevel_drivers
])

