#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys/logging/logging.h"

// The flush must reach USB only through stdio (which holds its USB mutex),
// never through tinyusb directly - see tests/include/tusb.h

static bool usb_connected;
static uint32_t fifo_free; // tud_cdc_write_available()
static char output[32768];
static size_t output_len;
static int put_string_calls;

bool stdio_usb_connected(void) {
    return usb_connected;
}

uint32_t tud_cdc_write_available(void) {
    return fifo_free;
}

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation) {
    // Raw bytes, and never more than fits, so stdio doesn't wait for the host
    assert_false(newline);
    assert_false(cr_translation);
    assert_true(len > 0);
    assert_true((uint32_t)len <= fifo_free);
    assert_true(output_len + (size_t)len <= sizeof(output));

    memcpy(&output[output_len], s, (size_t)len);
    output_len += (size_t)len;
    fifo_free -= (uint32_t)len;
    put_string_calls++;
    return len;
}

static void reset_output(void) {
    output_len = 0;
    put_string_calls = 0;
}

static int setup(void **state) {
    (void)state;
    usb_connected = true;
    fifo_free = 0;
    // Drain anything a previous test left in the (static) ring buffer
    fifo_free = sizeof(output);
    logging_flush_to_stdout();
    fifo_free = 0;
    reset_output();
    return 0;
}

// Log a recognisable line of exactly 'len' bytes (including the newline)
static void log_line(char *expected, size_t *expected_len, int n, size_t len) {
    char line[200];
    assert_true(len >= 8 && len < sizeof(line));
    int head = snprintf(line, sizeof(line), "L%05d ", n);
    memset(&line[head], 'a' + (n % 26), len - (size_t)head - 1);
    line[len - 1] = '\n';
    line[len] = '\0';
    debug_printf("%s", line);
    memcpy(&expected[*expected_len], line, len);
    *expected_len += len;
}

static void test_flush_writes_only_what_fits(void **state) {
    (void)state;
    char expected[1024];
    size_t expected_len = 0;
    for(int i=0; i<5; i++) {
        log_line(expected, &expected_len, i, 60); // 300 bytes
    }

    // The CDC TX FIFO has 64 bytes free
    fifo_free = 64;
    logging_flush_to_stdout();
    assert_int_equal(output_len, 64);
    assert_memory_equal(output, expected, 64);

    // Nothing free: nothing written, nothing lost
    logging_flush_to_stdout();
    assert_int_equal(output_len, 64);

    fifo_free = 100;
    logging_flush_to_stdout();
    assert_int_equal(output_len, 164);

    fifo_free = 1000;
    logging_flush_to_stdout();
    assert_int_equal(output_len, expected_len);
    assert_memory_equal(output, expected, expected_len);
    assert_int_equal(fifo_free, 1000 - (expected_len - 164));
}

static void test_flush_splits_large_writes(void **state) {
    (void)state;
    char expected[4096];
    size_t expected_len = 0;
    for(int i=0; i<20; i++) {
        log_line(expected, &expected_len, i, 150); // 3000 bytes
    }

    fifo_free = sizeof(output);
    logging_flush_to_stdout();
    assert_int_equal(output_len, expected_len);
    assert_memory_equal(output, expected, expected_len);
    // In bounded chunks (the flush copies through a stack buffer)
    assert_true(put_string_calls > 1);
}

static void test_flush_waits_for_connection(void **state) {
    (void)state;
    char expected[256];
    size_t expected_len = 0;
    log_line(expected, &expected_len, 1, 40);

    usb_connected = false;
    fifo_free = 1000;
    logging_flush_to_stdout();
    assert_int_equal(put_string_calls, 0);

    usb_connected = true;
    logging_flush_to_stdout();
    assert_int_equal(output_len, expected_len);
    assert_memory_equal(output, expected, expected_len);
}

static void test_flush_with_nothing_logged(void **state) {
    (void)state;
    fifo_free = 1000;
    logging_flush_to_stdout();
    assert_int_equal(put_string_calls, 0);
    assert_int_equal(fifo_free, 1000);
}

static void test_flush_after_overrun_keeps_newest(void **state) {
    (void)state;
    // More than the 16 KB ring buffer while nothing could be flushed: the
    // oldest is overwritten and the flush resumes from the oldest still held
    static char expected[24000];
    size_t expected_len = 0;
    for(int i=0; i<200; i++) {
        log_line(expected, &expected_len, i, 100); // 20000 bytes
    }

    fifo_free = sizeof(output);
    logging_flush_to_stdout();
    assert_int_equal(output_len, 16384);
    assert_memory_equal(output, &expected[expected_len - 16384], 16384);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_flush_writes_only_what_fits, setup),
        cmocka_unit_test_setup(test_flush_splits_large_writes, setup),
        cmocka_unit_test_setup(test_flush_waits_for_connection, setup),
        cmocka_unit_test_setup(test_flush_with_nothing_logged, setup),
        cmocka_unit_test_setup(test_flush_after_overrun_keeps_newest, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
