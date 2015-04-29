EXTRA_DIST = README

# TESTS = \
# 	check_ar11-bbb

#TODO: noinst_PROGRAMS = \
#TODO:	wosi-unit-test-spi \
#TODO: 	wosi-unit-test-gpio \
#TODO: 	wosi-unit-test-axis \
#TODO: 	wosi-unit-test-bp_reg_update \
#TODO: 	wosi-unit-test-home_pos \
#TODO: 	wosi-unit-test-ustep

# wosi-unit-test-spi: ysli 2015-02-08

noinst_PROGRAMS = \
	check_ar11-bbb

common_ldflags = \
	$(top_builddir)/src/libwosi.la \
	@CHECK_LIBS@

# for debugging, static linking of test programs:
AM_LDFLAGS = -static

#TODO: wosi_unit_test_spi_SOURCES = wosi-unit-test-spi.c
#TODO: wosi_unit_test_spi_CFLAGS = @CHECK_CFLAGS@
#TODO: wosi_unit_test_spi_LDADD = $(common_ldflags)

check_ar11_bbb_SOURCES = check_ar11-bbb.c
check_ar11_bbb_CFLAGS = @CHECK_CFLAGS@
check_ar11_bbb_LDADD = $(common_ldflags)

#TODO: wosi_unit_test_jcmd_SOURCES = wosi-unit-test-jcmd.c
#TODO: wosi_unit_test_jcmd_LDADD = $(common_ldflags)

#TODO: wosi_unit_test_axis_SOURCES = wosi-unit-test-axis.c
#TODO: wosi_unit_test_axis_LDADD = $(common_ldflags)
#TODO: 
#TODO: wosi_unit_test_bp_reg_update_SOURCES = wosi-unit-test-bp_reg_update.c
#TODO: wosi_unit_test_bp_reg_update_LDADD = $(common_ldflags)
#TODO: 
#TODO: wosi_unit_test_home_pos_SOURCES = wosi-unit-test-home_pos.c
#TODO: wosi_unit_test_home_pos_LDADD = $(common_ldflags)

#TODO: wosi_unit_test_ustep_SOURCES = wosi-unit-test-ustep.c
#TODO: wosi_unit_test_ustep_LDADD = $(common_ldflags)

INCLUDES = -I$(top_srcdir) -I$(top_srcdir)/src
CLEANFILES = *~
