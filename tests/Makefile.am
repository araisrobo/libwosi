EXTRA_DIST = README

noinst_PROGRAMS = \
	wou-unit-test-gpio \
	wou-unit-test-jcmd \
	wou-unit-test-axis \
	wou-unit-test-bp_reg_update \
	wou-unit-test-home_pos \
	wou-unit-test-ustep


common_ldflags = \
	$(top_builddir)/src/libwou.la \
 	@FTD2XXLIB@

#random_test_slave_LDFLAGS = -all-static
# for debugging, static linking of test programs:
AM_LDFLAGS = -static

wou_unit_test_gpio_SOURCES = wou-unit-test-gpio.c
wou_unit_test_gpio_LDADD = $(common_ldflags)

wou_unit_test_jcmd_SOURCES = wou-unit-test-jcmd.c
wou_unit_test_jcmd_LDADD = $(common_ldflags)

wou_unit_test_axis_SOURCES = wou-unit-test-axis.c
wou_unit_test_axis_LDADD = $(common_ldflags)

wou_unit_test_bp_reg_update_SOURCES = wou-unit-test-bp_reg_update.c
wou_unit_test_bp_reg_update_LDADD = $(common_ldflags)

wou_unit_test_home_pos_SOURCES = wou-unit-test-home_pos.c
wou_unit_test_home_pos_LDADD = $(common_ldflags)

wou_unit_test_ustep_SOURCES = wou-unit-test-ustep.c
wou_unit_test_ustep_LDADD = $(common_ldflags)

INCLUDES = -I$(top_srcdir) -I$(top_srcdir)/src
CLEANFILES = *~
