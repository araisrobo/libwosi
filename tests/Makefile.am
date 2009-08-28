EXTRA_DIST = README

noinst_PROGRAMS = \
	wou-unit-test-gpio \
	wou-unit-test-jcmd

# mou-unit-test-slave
# random-test-slave \
# random-test-master \
# unit-test-slave \
# unit-test-master \
# bandwidth-slave-one \
# bandwidth-slave-many-up \
# bandwidth-master

common_ldflags = \
	$(top_builddir)/src/libwou.la \
 	@FTD2XXLIB@

#random_test_slave_LDFLAGS = -all-static
# for debugging, static linking of test programs:
AM_LDFLAGS = -static

# random_test_slave_SOURCES = random-test-slave.c
# random_test_slave_LDADD = $(common_ldflags)
# 
# random_test_master_SOURCES = random-test-master.c
# random_test_master_LDADD = $(common_ldflags)
# 
# unit_test_slave_SOURCES = unit-test-slave.c unit-test.h
# unit_test_slave_LDADD = $(common_ldflags)
# 
# unit_test_master_SOURCES = unit-test-master.c unit-test.h
# unit_test_master_LDADD = $(common_ldflags)
# 
# bandwidth_slave_one_SOURCES = bandwidth-slave-one.c
# bandwidth_slave_one_LDADD = $(common_ldflags)
# 
# bandwidth_slave_many_up_SOURCES = bandwidth-slave-many-up.c
# bandwidth_slave_many_up_LDADD = $(common_ldflags)
# 
# bandwidth_master_SOURCES = bandwidth-master.c
# bandwidth_master_LDADD = $(common_ldflags)

# mou_unit_test_slave_SOURCES = mou-unit-test-slave.c unit-test.h
# mou_unit_test_slave_LDADD = $(common_ldflags)

wou_unit_test_gpio_SOURCES = wou-unit-test-gpio.c
wou_unit_test_gpio_LDADD = $(common_ldflags)

wou_unit_test_jcmd_SOURCES = wou-unit-test-jcmd.c
wou_unit_test_jcmd_LDADD = $(common_ldflags)

INCLUDES = -I$(top_srcdir) -I$(top_srcdir)/src
CLEANFILES = *~
