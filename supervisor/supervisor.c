#include "pins.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "tusb.h"

#ifdef BMS_USB_EXTERNAL_PULLUP
#include "hardware/structs/usb.h"

// The external pull-up keeps D+ high through a reboot, so the host never sees
// this device go away and keeps talking to the old one. Drive SE0 for a moment
// so the host registers a disconnect and enumerates again.
static void usb_force_reenumerate(void) {
    // SIE direct bus drive: DP and DM both low = SE0
    hw_clear_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_DIRECT_DP_BITS | USB_SIE_CTRL_DIRECT_DM_BITS);
    hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_DIRECT_EN_BITS);
    sleep_ms(50);
    hw_clear_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_DIRECT_EN_BITS);
}
#endif

#ifdef BMS_USB_VBUS_SENSE
// USB VBUS reaches PIN_USB_VBUS_SENSE through a 3.3k/5.1k divider. The chip's
// D+ pull-up is only switched on while the host supplies VBUS, so a switched-off
// host is never back-powered through D+. A reboot drops the pull-up with the
// USB controller, so the host also sees every reboot.
#define USB_VBUS_DEBOUNCE_US 50000

static bool usb_vbus_connected;
static bool usb_vbus_pending;
static absolute_time_t usb_vbus_changed_at;

static void usb_vbus_sense_init(void) {
    gpio_init(PIN_USB_VBUS_SENSE);
    gpio_set_dir(PIN_USB_VBUS_SENSE, GPIO_IN);
    gpio_disable_pulls(PIN_USB_VBUS_SENSE);
    sleep_us(100);

    // The USB stack switches the pull-up on at init
    usb_vbus_connected = gpio_get(PIN_USB_VBUS_SENSE);
    if(!usb_vbus_connected) {
        tud_disconnect();
    }
}

static void usb_vbus_sense_poll(void) {
    bool vbus = gpio_get(PIN_USB_VBUS_SENSE);
    if(vbus == usb_vbus_connected) {
        usb_vbus_pending = false;
        return;
    }
    if(!usb_vbus_pending) {
        usb_vbus_pending = true;
        usb_vbus_changed_at = get_absolute_time();
        return;
    }
    if(absolute_time_diff_us(usb_vbus_changed_at, get_absolute_time()) < USB_VBUS_DEBOUNCE_US) {
        return;
    }

    usb_vbus_pending = false;
    usb_vbus_connected = vbus;
    if(vbus) {
        tud_connect();
    } else {
        tud_disconnect();
    }
}
#endif

#include "cell_snoop.h"
#include "supervisor_logic.h"
#include "isospi/isosnoop.h"

#include "config/limits.h"
#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"

#include "hardware/irq.h"
#include "hardware/uart.h"

#include <stdio.h>

#define MCU_CHECK_PIN 0

#define INTERNAL_UART uart0
#define INTERNAL_UART_IRQ UART0_IRQ
#define INTERNAL_UART_BAUDRATE 115200
#define INTERNAL_UART_RX_PIN 1
#define INTERNAL_UART_TX_PIN 2

#define HMI_UART uart1
#define HMI_UART_BAUDRATE 460800
#define HMI_UART_TX_PIN 38
#define HMI_UART_RX_PIN 39

// Gates the contactor MOSFET gate drivers (CTR_EN)
#define OUTPUT_ENABLE_PIN 14
#define LED_PIN 25

// How often to print a status line over USB
#define STATUS_PRINT_PERIOD_MS 10000

// supervisor serial: 675DE5D4CCFD90CA

static cell_snoop_t snoop;
static sup_logic_t logic;
static sup_frame_parser_t frame_parser;

// Bytes received from the BMS, filled by the UART interrupt so a stalled USB
// printf can't overflow the 32-byte hardware FIFO. The indices wrap at 256.
static volatile uint8_t rx_ring[256];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

static void on_internal_uart_rx(void) {
    while(uart_is_readable(INTERNAL_UART)) {
        uint8_t c = uart_getc(INTERNAL_UART);
        uint8_t next = rx_head + 1;
        if(next != rx_tail) {
            rx_ring[rx_head] = c;
            rx_head = next;
        }
    }
}

void init_serial() {
    uart_init(INTERNAL_UART, INTERNAL_UART_BAUDRATE);
    gpio_set_function(INTERNAL_UART_TX_PIN, UART_FUNCSEL_NUM(INTERNAL_UART, INTERNAL_UART_TX_PIN));
    gpio_set_function(INTERNAL_UART_RX_PIN, UART_FUNCSEL_NUM(INTERNAL_UART, INTERNAL_UART_RX_PIN));
    uart_set_hw_flow(INTERNAL_UART, false, false);
    uart_set_format(INTERNAL_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(INTERNAL_UART, true);

    irq_set_exclusive_handler(INTERNAL_UART_IRQ, on_internal_uart_rx);
    irq_set_enabled(INTERNAL_UART_IRQ, true);
    uart_set_irq_enables(INTERNAL_UART, true, false);

    gpio_init(HMI_UART_TX_PIN);
    gpio_init(HMI_UART_RX_PIN);
    gpio_set_dir(HMI_UART_TX_PIN, GPIO_IN);
    gpio_set_dir(HMI_UART_RX_PIN, GPIO_IN);
}

void init_watchdog() {
    if (watchdog_enable_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    } else {
        printf("Clean boot\n");
    }

    watchdog_enable(5000, 0); // second arg enables pause on debug
}

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static const char *state_name(sup_state_t state) {
    switch(state) {
        case SUP_STATE_OFF: return "OFF";
        case SUP_STATE_ON: return "ON";
        case SUP_STATE_LATCHED: return "LATCHED";
    }
    return "?";
}

static void print_reasons(uint16_t reasons) {
    static const struct { uint16_t bit; const char *name; } names[] = {
        { SUP_REASON_NO_HEARTBEAT, "no-heartbeat" },
        { SUP_REASON_VERSION, "version" },
        { SUP_REASON_MAP_MISMATCH, "map-mismatch" },
        { SUP_REASON_NO_SCAN, "no-scan" },
        { SUP_REASON_CELL_HIGH, "cell-high" },
        { SUP_REASON_CELL_LOW, "cell-low" },
        { SUP_REASON_CELL_MISSING, "cell-missing" },
        { SUP_REASON_BMS_DISAGREE, "bms-disagree" },
        { SUP_REASON_SETTLING, "settling" },
        { SUP_REASON_ABSENT_READS, "absent-channel-reads" },
    };
    if(reasons == 0) {
        printf(" none");
    }
    for(size_t i=0; i<sizeof(names)/sizeof(names[0]); i++) {
        if(reasons & names[i].bit) {
            printf(" %s", names[i].name);
        }
    }
}

static void print_status(void) {
    uint32_t now = now_ms();
    printf("Supervisor %s | blocking:", state_name(logic.state));
    print_reasons(logic.reasons);
    if(logic.state == SUP_STATE_LATCHED) {
        printf(" | latched:");
        print_reasons(logic.latch_reasons);
    }
    printf("\n  cells %d-%d mV, %u/%d ok, %u good scans | scans %lu, bank reads %lu, CRC errors %lu, incomplete %lu\n",
        logic.cell_min_mV, logic.cell_max_mV, logic.cells_ok, NUM_CELLS, logic.good_scans,
        (unsigned long)snoop.scans, (unsigned long)snoop.bank_reads,
        (unsigned long)snoop.crc_errors, (unsigned long)snoop.incomplete_scans);
    if(logic.have_status) {
        printf("  BMS v%u up %llu ms, map %08lx (ours %08lx), cells %d-%d mV, flags 0x%02x, age %lu ms\n",
            logic.status.version, (unsigned long long)logic.status.uptime_ms,
            (unsigned long)logic.status.map_hash, (unsigned long)cell_map_hash(),
            logic.status.cell_min_mV, logic.status.cell_max_mV, logic.status.flags,
            (unsigned long)(now - logic.status_millis));
    } else {
        printf("  no BMS status yet (link frames with bad CRC: %lu)\n", (unsigned long)frame_parser.crc_errors);
    }
}

// Every channel of the last complete scan; * marks the channels the cell map
// says are populated
static void print_scan(void) {
    for(int module=0; module<CELL_MAP_MODULES; module++) {
        printf("BMB %d:", module);
        for(int cell=0; cell<CELL_MAP_CELLS_PER_MODULE; cell++) {
            int physical_index = module * CELL_MAP_CELLS_PER_MODULE + cell;
            int16_t mV = snoop.scan_mV[physical_index];
            char mark = cell_map_is_present(physical_index) ? '*' : ' ';
            if(mV == CELL_MV_NOT_READ) {
                printf(" %c----", mark);
            } else {
                printf(" %c%4d", mark, mV);
            }
        }
        printf("\n");
    }
}

static void send_reply(void) {
    sup_reply_t reply;
    sup_logic_reply(&logic, &reply);

    uint8_t payload[SUP_REPLY_LEN];
    size_t len = sup_pack_reply(&reply, payload, sizeof(payload));
    uint8_t frame[SUP_MAX_FRAME_LEN];
    size_t frame_len = sup_frame_encode(payload, len, frame, sizeof(frame));
    uart_write_blocking(INTERNAL_UART, frame, frame_len);
}

// Returns whether a BMS status arrived
static bool poll_internal_serial(uint32_t now) {
    bool got_status = false;
    while(rx_tail != rx_head) {
        uint8_t c = rx_ring[rx_tail];
        rx_tail = rx_tail + 1;

        const uint8_t *payload;
        size_t len = sup_frame_feed(&frame_parser, c, &payload);
        sup_bms_status_t status;
        if(len > 0 && sup_unpack_bms_status(payload, len, &status)) {
            sup_logic_on_status(&logic, &status, now);
            got_status = true;
        }
    }
    return got_status;
}

static void poll_snoop(uint32_t now) {
    uint8_t buf[256];
    size_t len;
    while((len = isosnoop_read(buf, sizeof(buf))) > 0) {
        for(size_t i=0; i<len; i++) {
            cell_snoop_feed(&snoop, buf[i]);
        }
        if(cell_snoop_take_scan(&snoop)) {
            sup_logic_on_scan(&logic, snoop.scan_mV, now);
        }
    }
}

static void poll_usb_commands(void) {
    int c = getchar_timeout_us(0);
    if(c == 'r') {
        print_scan();
    } else if(c == 's') {
        print_status();
    }
}

int main() {
    // Keep the contactor drivers disabled until everything has been checked
    gpio_init(OUTPUT_ENABLE_PIN);
    gpio_set_dir(OUTPUT_ENABLE_PIN, GPIO_OUT);
    gpio_put(OUTPUT_ENABLE_PIN, false);

    stdio_usb_init();

#ifdef BMS_USB_EXTERNAL_PULLUP
    // The external D+ pull-up is powered from VBUS; switch the chip's own one
    // (enabled by the USB stack at init) off so it can't back-power the host.
    tud_disconnect();
    usb_force_reenumerate();
#endif
#ifdef BMS_USB_VBUS_SENSE
    usb_vbus_sense_init();
#endif

    sleep_ms(4000);

    init_watchdog();
    printf("Supervisor starting... (profile %d, %d cells, map %08lx)\n",
        BMS_PROFILE, NUM_CELLS, (unsigned long)cell_map_hash());
    printf("USB commands: s = status, r = last snooped scan\n");

    isosnoop_setup(
        PIN_ISOSPI_RX_HIGH,
        PIN_ISOSNOOP_RX_DEBUG,
        PIN_ISOSPI_RX_AND,
        PIN_ISOSNOOP_TIMER_DISABLE
    );

    gpio_init(MCU_CHECK_PIN);
    gpio_set_dir(MCU_CHECK_PIN, GPIO_IN);
    bool mcu_check = gpio_get(MCU_CHECK_PIN);
    if(!mcu_check) {
        printf("Error: Supervisor firmware is running on the main MCU!\n");
        while(1);
    }

    cell_snoop_init(&snoop);
    sup_logic_init(&logic);
    sup_frame_parser_init(&frame_parser, SUP_BMS_STATUS_LEN);

    init_serial();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    sup_state_t last_state = logic.state;
    uint16_t last_reasons = logic.reasons;
    uint32_t last_print = now_ms();

    while(1) {
        watchdog_update();
        uint32_t now = now_ms();

        poll_snoop(now);
        bool got_status = poll_internal_serial(now);

        gpio_put(OUTPUT_ENABLE_PIN, sup_logic_tick(&logic, now));

        if(got_status) {
            // Answer with the state as of this status
            send_reply();
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
        }

        if(logic.state != last_state || logic.reasons != last_reasons) {
            if(logic.state != last_state) {
                printf("Supervisor %s -> %s\n", state_name(last_state), state_name(logic.state));
            }
            print_status();
            if(logic.state == SUP_STATE_LATCHED && last_state != SUP_STATE_LATCHED) {
                print_scan();
            }
            last_state = logic.state;
            last_reasons = logic.reasons;
            last_print = now;
        } else if(now - last_print >= STATUS_PRINT_PERIOD_MS) {
            print_status();
            last_print = now;
        }

        poll_usb_commands();
#ifdef BMS_USB_VBUS_SENSE
        usb_vbus_sense_poll();
#endif

        sleep_ms(1);
    }

    return 0;
}
