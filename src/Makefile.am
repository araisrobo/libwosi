SUBDIRS = wou

h_sources = wou.h wb_regs.h mailtag.h sync_cmd.h
c_sources = wou.c

lib_LTLIBRARIES = libwou.la
libwou_la_SOURCES = $(h_sources) $(c_sources)
# libwou_la_LIBADD = wou/libwou.la @FTD2XXLIB@
libwou_la_LIBADD = wou/libwou.la
libwou_la_LDFLAGS = -version-info 2:0:0

INCLUDES = -I$(top_srcdir)

# Include files to install
libwouincludedir = $(includedir)/wou
libwouinclude_HEADERS = $(h_sources)

CLEANFILES = *~
