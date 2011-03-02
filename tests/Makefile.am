EXTRA_DIST = README

#TODO: noinst_PROGRAMS = \
#TODO: 	wou-unit-test-gpio \
#TODO: 	wou-unit-test-axis \
#TODO: 	wou-unit-test-bp_reg_update \
#TODO: 	wou-unit-test-home_pos \
#TODO: 	wou-unit-test-ustep

noinst_PROGRAMS = \
	wou-unit-test-spi \
	wou-unit-test-jcmd \
  	wou-unit-test-ustep


# common_ldflags = \
# 	$(top_builddir)/src/libwou.la \
#  	@FTD2XXLIB@

common_ldflags = \
	$(top_builddir)/src/libwou.la

#random_test_slave_LDFLAGS = -all-static
# for debugging, static linking of test programs:
AM_LDFLAGS = -static

wou_unit_test_spi_SOURCES = wou-unit-test-spi.c
wou_unit_test_spi_LDADD = $(common_ldflags)

wou_unit_test_jcmd_SOURCES = wou-unit-test-jcmd.c
wou_unit_test_jcmd_LDADD = $(common_ldflags)

#TODO: wou_unit_test_axis_SOURCES = wou-unit-test-axis.c
#TODO: wou_unit_test_axis_LDADD = $(common_ldflags)
#TODO: 
#TODO: wou_unit_test_bp_reg_update_SOURCES = wou-unit-test-bp_reg_update.c
#TODO: wou_unit_test_bp_reg_update_LDADD = $(common_ldflags)
#TODO: 
#TODO: wou_unit_test_home_pos_SOURCES = wou-unit-test-home_pos.c
#TODO: wou_unit_test_home_pos_LDADD = $(common_ldflags)

wou_unit_test_ustep_SOURCES = wou-unit-test-ustep.c
wou_unit_test_ustep_LDADD = $(common_ldflags)

INCLUDES = -I$(top_srcdir) -I$(top_srcdir)/src
CLEANFILES = *~
