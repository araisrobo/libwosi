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

// to disable DP(): #define TRACE 0
// to enable DP(): #define TRACE 1
// to dump more info: #define TRACE 2
#define TRACE 1
#include "../src/wosi/dptrace.h"
#if (TRACE!=0)
FILE *dptrace; // dptrace = fopen("dptrace.log","w");
#endif

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
    ck_assert_int_eq (w_param.board->io.spi.mode_rd, 1);        // mode is 1 for 25MHz SPI-READ
    ck_assert_int_eq (w_param.board->io.spi.bits, 8);
    ck_assert_int_eq (w_param.board->io.spi.speed, 25000000UL);
    ck_assert_int_ne (w_param.board->io.spi.fd_wr, 0);
    ck_assert_int_ne (w_param.board->io.spi.fd_rd, 0);
    ck_assert_int_ne (w_param.board->io.spi.fd_burst_rd_rdy, 0);
    ck_assert_int_eq (w_param.board->wosi->Sm, (NR_OF_WIN - 1));
    ck_assert_int_eq (w_param.board->ready, 0); // board->ready is 0 before programming RISC
}
END_TEST

START_TEST (test_tx_timeout)
{
    int ret, i;
    uint8_t value;
    uint8_t cur_tid;
    uint8_t free_wosif;

    // check TX_TIMEOUT
    value = (uint8_t) GPIO_ALARM_EN;
    i = 0;
    do {
        wosi_cmd (
                &w_param,
                WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                1,
                &value);

        // calculated tid after wosi_eof()
        cur_tid = w_param.board->wosi->tid;
        cur_tid += 1;

        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
        ck_assert_int_eq (ret, 0);                      // TODO: wosi_eof always return 0

        free_wosif = w_param.board->wosi->Sm - (w_param.board->wosi->Sn - 1);
        DP ("%5d tx_timeout(%d) free_wosif(%u) Sm(%u) Sn(%u)\n",
                i,
                w_param.board->wosi->tx_timeout,
                free_wosif,
                w_param.board->wosi->Sm,
                w_param.board->wosi->Sn);
        wosi_send(w_param.board);                       // send but not receive;

        _ck_assert_int (free_wosif, <=, NR_OF_WIN);
        if ((free_wosif == 0) && (w_param.board->wosi->tx_timeout == 0)) {
            // no more free WOSIF, and (not tx_timeout yet)
            ck_assert_int_eq (w_param.board->wosi->tx_size, 0); // stop sending
        } else {
            ck_assert_int_ne (w_param.board->wosi->tx_size, 0); // send at least a WOSIF
        }

        if (w_param.board->wosi->tx_timeout)
        {
            DP ("%5d tx_timeout(%d) free_wosif(%u) Sm(%u) Sn(%u)\n",
                    i,
                    w_param.board->wosi->tx_timeout,
                    free_wosif,
                    w_param.board->wosi->Sm,
                    w_param.board->wosi->Sn);
        }

        // check TID always increase by 1 after wosi_flush
        ck_assert_int_eq (w_param.board->wosi->tid, cur_tid);
        i++;
    } while ((w_param.board->wosi->tx_timeout == 0) && (i < 1000));
    _ck_assert_int (i, <, 1000);        // must hit TX_TIMEOUT within 100 WOSI-FRAMES

}
END_TEST

START_TEST (test_wosi_recv)
{
    int ret, i;
    uint8_t value;
    uint8_t cur_tid;
    uint8_t free_wosif;

    return; /* WIP */
    // check TX_TIMEOUT
    value = (uint8_t) GPIO_ALARM_EN;
    i = 0;
    do {
        wosi_cmd (
                &w_param,
                WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                1,
                &value);

        // calculated tid after wosi_eof()
        cur_tid = w_param.board->wosi->tid;
        cur_tid += 1;

        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
        ck_assert_int_eq (ret, 0);                      // TODO: wosi_eof always return 0

        free_wosif = w_param.board->wosi->Sm - (w_param.board->wosi->Sn - 1);
//        printf ("%5d tx_timeout(%d) free_wosif(%u) Sm(%u) Sn(%u)\n",
//                i,
//                w_param.board->wosi->tx_timeout,
//                free_wosif,
//                w_param.board->wosi->Sm,
//                w_param.board->wosi->Sn);
        wosi_send(w_param.board);                       // send
        wosi_recv(w_param.board);                       // receive

        _ck_assert_int (free_wosif, <=, NR_OF_WIN);
        if ((free_wosif == 0) && (w_param.board->wosi->tx_timeout == 0)) {
            // no more free WOSIF, and (not tx_timeout yet)
            ck_assert_int_eq (w_param.board->wosi->tx_size, 0); // stop sending
        } else {
            ck_assert_int_ne (w_param.board->wosi->tx_size, 0); // send at least a WOSIF
        }

//        if (w_param.board->wosi->tx_timeout)
//        {
//            printf ("%5d tx_timeout(%d) free_wosif(%u) Sm(%u) Sn(%u)\n",
//                    i,
//                    w_param.board->wosi->tx_timeout,
//                    free_wosif,
//                    w_param.board->wosi->Sm,
//                    w_param.board->wosi->Sn);
//        }

        // check TID always increase by 1 after wosi_flush
        ck_assert_int_eq (w_param.board->wosi->tid, cur_tid);
        i++;
    } while ((w_param.board->wosi->tx_timeout == 0) && (i < 1000));
    _ck_assert_int (i, <=, 1000);        // must hit TX_TIMEOUT within 100 WOSI-FRAMES

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
  tcase_add_test (tc_core, test_tx_timeout);
  tcase_add_test (tc_core, test_wosi_recv);

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
