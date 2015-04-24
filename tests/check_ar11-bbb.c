#include <check.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#include "wosi.h"
#include "wb_regs.h"
#include "sync_cmd.h"
#include "mailtag.h"

// include files to check board.c:
#include <libusb.h>
#include <ftdi.h>       // from FTDI
#include "../src/wosi/board.h"

static wosi_param_t w_param;

START_TEST (test_wosi_init)
{
    wosi_init(&w_param, "ar11-bbb", 0, "ar11_top.bit");
    ck_assert_str_eq (w_param.board->board_type, "ar11-bbb");
    ck_assert_str_eq (w_param.board->fpga_bit_file, "ar11_top.bit");
}
END_TEST

START_TEST (test_wosi_connect)
{
    int ret;

    // wosi_connect(): open io-device, and programe fpga with given "<fpga>.bit"
    ret = wosi_connect(&w_param);
    ck_assert_int_eq (ret, 0);
    ck_assert_str_eq (w_param.board->io.spi.device_wr, "/dev/spidev1.0");
    ck_assert_str_eq (w_param.board->io.spi.device_rd, "/dev/spidev1.1");
    ck_assert_int_eq (w_param.board->io.spi.mode_wr, 0);
    ck_assert_int_eq (w_param.board->io.spi.mode_rd, 0);
    ck_assert_int_eq (w_param.board->io.spi.bits, 8);
    ck_assert_int_eq (w_param.board->io.spi.speed, 25000000UL);
    ck_assert_int_ne (w_param.board->io.spi.fd_wr, 0);
    ck_assert_int_ne (w_param.board->io.spi.fd_rd, 0);
    ck_assert_int_ne (w_param.board->io.spi.fd_burst_rd_rdy, 0);
    ck_assert_int_eq (w_param.board->wosi->Sm, (NR_OF_WIN - 1));
}
END_TEST

START_TEST (test_wosi_cmd)
{
    int ret, i;
    uint8_t value;

    value = (uint8_t) GPIO_ALARM_EN;
    for (i=0; i<100; i++)
    {
        wosi_cmd (
                &w_param,
                WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                1,
                &value);
        wosi_update(&w_param);
        ret = wosi_flush(&w_param);
    }

//    printf ("debug: value(%.2X)\n", value);
//    getchar();

//    ret = wosi_prog_risc(&w_param, "css.bin");
//    ck_assert_int_eq (ret, 0);
//    ck_assert_int_eq (w_param.board->ready, 1); // set board->ready as 1 after programming RISC
//    ck_assert_str_eq (w_param.board->io.spi.device_wr, "/dev/spidev1.0");
//    ck_assert_str_eq (w_param.board->io.spi.device_rd, "/dev/spidev1.1");
//    ck_assert_int_eq (w_param.board->io.spi.mode_wr, 0);
//    ck_assert_int_eq (w_param.board->io.spi.mode_rd, 0);
//    ck_assert_int_eq (w_param.board->io.spi.bits, 8);
//    ck_assert_int_eq (w_param.board->io.spi.speed, 25000000UL);
//    ck_assert_int_eq (w_param.board->wosi->Sm, (NR_OF_WIN - 1));
}
END_TEST

START_TEST (test_wosi_prog_risc)
{
    int ret;

    ret = wosi_prog_risc(&w_param, "css.bin");
    ck_assert_int_eq (ret, 0);
    ck_assert_int_eq (w_param.board->ready, 1); // set board->ready as 1 after programming RISC
//    ck_assert_str_eq (w_param.board->io.spi.device_wr, "/dev/spidev1.0");
//    ck_assert_str_eq (w_param.board->io.spi.device_rd, "/dev/spidev1.1");
//    ck_assert_int_eq (w_param.board->io.spi.mode_wr, 0);
//    ck_assert_int_eq (w_param.board->io.spi.mode_rd, 0);
//    ck_assert_int_eq (w_param.board->io.spi.bits, 8);
//    ck_assert_int_eq (w_param.board->io.spi.speed, 25000000UL);
//    ck_assert_int_eq (w_param.board->wosi->Sm, (NR_OF_WIN - 1));
}
END_TEST

START_TEST (test_wosi_close)
{
    int ret;

    wosi_close(&w_param);
    printf ("how to check if memory is freed?\n");
}
END_TEST

Suite *
wosi_suite (void)
{
  Suite *s = suite_create ("WOSI");

  /* Core test case */
  TCase *tc_core = tcase_create ("Core");
  tcase_add_test (tc_core, test_wosi_init);
  tcase_add_test (tc_core, test_wosi_connect);
  tcase_add_test (tc_core, test_wosi_cmd);
//  tcase_add_test (tc_core, test_wosi_prog_risc);
  tcase_add_test (tc_core, test_wosi_close);
  suite_add_tcase (s, tc_core);

  return s;
}

int
main (void)
{
  int number_failed;
  Suite *s = wosi_suite ();
  SRunner *sr = srunner_create (s);
  // set CK_NOFORK to share static variable, w_param, between tests
  srunner_set_fork_status(sr, CK_NOFORK);
  srunner_run_all (sr, CK_NORMAL);
  number_failed = srunner_ntests_failed (sr);
  srunner_free (sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
