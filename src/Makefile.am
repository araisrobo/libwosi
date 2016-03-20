SUBDIRS = wosi

h_sources = wosi.h wb_regs.h mailtag.h
c_sources = wosi.c

lib_LTLIBRARIES = libwosi.la
libwosi_la_SOURCES = $(h_sources) $(c_sources)
libwosi_la_LIBADD = wosi/libwosi.la
libwosi_la_LDFLAGS = -version-info 2:0:0

# INCLUDES = -I$(top_srcdir)
AM_CFLAGS = -I$(top_srcdir)

# Include files to install
libwosiincludedir = $(includedir)/wosi
libwosiinclude_HEADERS = $(h_sources)

CLEANFILES = *~
