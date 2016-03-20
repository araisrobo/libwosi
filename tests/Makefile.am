EXTRA_DIST = README

noinst_PROGRAMS = \
	check_ar11-bbb

common_ldflags = \
	$(top_builddir)/src/libwosi.la \
	@CHECK_LIBS@ \
	-lm

# for profile and valgrind
# PROFILE_CFLAGS = -p -Wall
PROFILE_CFLAGS =

# for debugging, static linking of test programs:
AM_LDFLAGS = -static

check_ar11_bbb_SOURCES = check_ar11-bbb.c
check_ar11_bbb_CFLAGS = @CHECK_CFLAGS@ $(PROFILE_CFLAGS)
check_ar11_bbb_LDADD = $(common_ldflags)

INCLUDES = -I$(top_srcdir) -I$(top_srcdir)/src
CLEANFILES = *~
