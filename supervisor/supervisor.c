// Supervisor MCU firmware: holds the contactor gate drivers disabled (CTR_EN)
// unless its own checks of the snooped cell voltages pass (supervisor_logic.h).
//
// The main loop never waits on anything: the BMS link is read from an interrupt
// fed ring and answered only when the UART can take the whole reply at once,
// the snooper is read from its DMA ring, and USB output waits at most 5ms
// before it is dropped (PICO_STDIO_USB_STDOUT_TIMEOUT_US). CTR_EN is updated
// every loop, before any printing.

#include "allocations.h"
#include "cell_snoop.h"
#include "pins.h"
#include "supervisor_config.h"
#include "supervisor_hmi.h"
#include "supervisor_logic.h"
#include "isospi/isosnoop.h"

#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"

#include BMS_BOARD_HEADER

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include <stdio.h>

#if USB_VBUS_SENSE && USB_EXTERNAL_PULLUP
#error "USB_VBUS_SENSE and USB_EXTERNAL_PULLUP are alternatives"
#endif

#if USB_EXTERNAL_PULLUP
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

#if USB_VBUS_SENSE
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

#define INTERNAL_UART uart0
#define INTERNAL_UART_IRQ UART0_IRQ
#define INTERNAL_UART_BAUDRATE 115200

// The HMI lines are shared with the BMS MCU (full duplex): we hear the HMI's
// requests, and only drive TX while answering one meant for us
// (supervisor_hmi.h). The HMI decides who talks when.
#define HMI_UART uart1
#define HMI_UART_IRQ UART1_IRQ
#define HMI_UART_BAUDRATE 460800

// How often to print a status line over USB
#define STATUS_PRINT_PERIOD_MS 10000

static cell_snoop_t snoop;
static sup_logic_t logic;
static sup_frame_parser_t frame_parser;
static uint32_t replies_sent;
static uint32_t replies_skipped;

// Bytes received from the BMS, filled by the UART interrupt so the 32-byte
// hardware FIFO never overflows. The indices wrap at 256.
static volatile uint8_t rx_ring[256];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint32_t rx_overflows;

static void on_internal_uart_rx(void) {
    while(uart_is_readable(INTERNAL_UART)) {
        uint8_t c = uart_getc(INTERNAL_UART);
        uint8_t next = rx_head + 1;
        if(next != rx_tail) {
            rx_ring[rx_head] = c;
            rx_head = next;
        } else {
            rx_overflows++;
        }
    }
}

// HMI requests, filled by the UART interrupt. Larger than the internal link's:
// the HMI runs at 460800 and some of its requests to the BMS are 260 bytes.
static volatile uint8_t hmi_rx_ring[1024];
static volatile uint16_t hmi_rx_head;
static volatile uint16_t hmi_rx_tail;
static volatile uint32_t hmi_rx_overflows;
static sup_frame_parser_t hmi_parser;
static volatile bool hmi_tx_active;
static uint32_t hmi_replies_sent;
static uint32_t hmi_replies_skipped;

static void on_hmi_uart_rx(void) {
    while(uart_is_readable(HMI_UART)) {
        uint8_t c = uart_getc(HMI_UART);
        uint16_t next = (hmi_rx_head + 1) % sizeof(hmi_rx_ring);
        if(next != hmi_rx_tail) {
            hmi_rx_ring[hmi_rx_head] = c;
            hmi_rx_head = next;
        } else {
            hmi_rx_overflows++;
        }
    }
}

// Our HMI TX pin is an input with a pull-up while we aren't sending, like the
// BMS's: the line idles high, and whoever is answering drives it
static void hmi_release_tx(void) {
    gpio_set_function(PIN_HMI_SERIAL_TX, GPIO_FUNC_SIO);
}

static int64_t hmi_tx_done(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;
    if(uart_get_hw(HMI_UART)->fr & UART_UARTFR_BUSY_BITS) {
        // Still shifting out; look again shortly
        return -50;
    }
    hmi_release_tx();
    hmi_tx_active = false;
    return 0;
}

static void init_hmi_serial(void) {
    uart_init(HMI_UART, HMI_UART_BAUDRATE);
    uart_set_hw_flow(HMI_UART, false, false);
    uart_set_format(HMI_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(HMI_UART, true);

    // RX is driven by the HMI and read by the BMS too: add no pull of ours
    gpio_set_function(PIN_HMI_SERIAL_RX, UART_FUNCSEL_NUM(HMI_UART, PIN_HMI_SERIAL_RX));
    gpio_disable_pulls(PIN_HMI_SERIAL_RX);

    gpio_init(PIN_HMI_SERIAL_TX);
    gpio_set_dir(PIN_HMI_SERIAL_TX, GPIO_IN);
    gpio_pull_up(PIN_HMI_SERIAL_TX);

    sup_frame_parser_init(&hmi_parser, 256);

    irq_set_exclusive_handler(HMI_UART_IRQ, on_hmi_uart_rx);
    irq_set_enabled(HMI_UART_IRQ, true);
    uart_set_irq_enables(HMI_UART, true, false);
}

static void hmi_send(const uint8_t *payload, size_t len) {
    uint8_t frame[SUP_HMI_RESPONSE_LEN + 4];
    size_t frame_len = duart_frame_encode(payload, len, frame, sizeof(frame));

    // The frame fits the empty 32-byte FIFO, so this never waits. If the last
    // one is somehow still going out, skip this: the HMI will ask again.
    _Static_assert(SUP_HMI_RESPONSE_LEN + 4 <= 32, "HMI response must fit the UART FIFO");
    if(frame_len == 0 || hmi_tx_active || !(uart_get_hw(HMI_UART)->fr & UART_UARTFR_TXFE_BITS)) {
        hmi_replies_skipped++;
        return;
    }

    hmi_tx_active = true;
    gpio_set_function(PIN_HMI_SERIAL_TX, UART_FUNCSEL_NUM(HMI_UART, PIN_HMI_SERIAL_TX));
    for(size_t i=0; i<frame_len; i++) {
        uart_get_hw(HMI_UART)->dr = frame[i];
    }
    // Release the line as soon as the last bit is out (10 bits per byte)
    uint32_t frame_us = (uint32_t)(frame_len * 10 * 1000000ULL / HMI_UART_BAUDRATE);
    if(add_alarm_in_us(frame_us + 20, hmi_tx_done, NULL, true) < 0) {
        // No alarm slot: wait it out rather than hold the line
        uart_tx_wait_blocking(HMI_UART);
        hmi_release_tx();
        hmi_tx_active = false;
    }
    hmi_replies_sent++;
}

static void poll_hmi_serial(uint32_t now) {
    while(hmi_rx_tail != hmi_rx_head) {
        uint8_t c = hmi_rx_ring[hmi_rx_tail];
        hmi_rx_tail = (hmi_rx_tail + 1) % sizeof(hmi_rx_ring);

        sup_frame_feed(&hmi_parser, c);
        const uint8_t *payload;
        size_t len;
        while((len = sup_frame_next(&hmi_parser, &payload)) > 0) {
            uint8_t response[SUP_HMI_RESPONSE_LEN];
            size_t response_len = sup_hmi_handle(&logic, payload, len, now, response, sizeof(response));
            if(response_len > 0) {
                hmi_send(response, response_len);
            }
        }
    }
}

static void init_serial(void) {
    uart_init(INTERNAL_UART, INTERNAL_UART_BAUDRATE);
    gpio_set_function(PIN_INTERNAL_SERIAL_TX, UART_FUNCSEL_NUM(INTERNAL_UART, PIN_INTERNAL_SERIAL_TX));
    gpio_set_function(PIN_INTERNAL_SERIAL_RX, UART_FUNCSEL_NUM(INTERNAL_UART, PIN_INTERNAL_SERIAL_RX));
    uart_set_hw_flow(INTERNAL_UART, false, false);
    uart_set_format(INTERNAL_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(INTERNAL_UART, true);

    irq_set_exclusive_handler(INTERNAL_UART_IRQ, on_internal_uart_rx);
    irq_set_enabled(INTERNAL_UART_IRQ, true);
    uart_set_irq_enables(INTERNAL_UART, true, false);
}

static void init_watchdog(void) {
    if(watchdog_enable_caused_reboot()) {
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
        { SUP_REASON_MAP_INVALID, "map-invalid" },
        { SUP_REASON_CHEMISTRY, "chemistry" },
        { SUP_REASON_MAP_CHANGED, "map-changed" },
        { SUP_REASON_NO_SCAN, "no-scan" },
        { SUP_REASON_CELL_HIGH, "cell-high" },
        { SUP_REASON_CELL_LOW, "cell-low" },
        { SUP_REASON_CELL_MISSING, "cell-missing" },
        { SUP_REASON_ABSENT_READS, "absent-channel-reads" },
        { SUP_REASON_BMS_DISAGREE, "bms-disagree" },
        { SUP_REASON_SETTLING, "settling" },
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

static void print_map(const cell_map_t *map) {
    printf("%s, %u cells on %u modules, mask",
        map->chemistry == LFP ? "LFP" : map->chemistry == NMC ? "NMC" : "?",
        map->num_cells, map->num_modules);
    for(int word=0; word<CELL_MAP_MASK_WORDS; word++) {
        printf(" %08lx", (unsigned long)map->presence[word]);
    }
    printf(", hash %08lx", (unsigned long)cell_map_hash(map));
}

static void print_status(void) {
    uint32_t now = now_ms();
    printf("Supervisor %s (%s) | blocking:", state_name(logic.state), SUPERVISOR_CHEMISTRY_NAME);
    print_reasons(logic.reasons);
    if(logic.state == SUP_STATE_LATCHED) {
        printf(" | latched:");
        print_reasons(logic.latch_reasons);
    }

    uint32_t crc_errors = 0;
    int modules = logic.have_map ? logic.map.num_modules : CELL_MAP_MODULES;
    for(int module=0; module<modules; module++) {
        crc_errors += snoop.module_crc_errors[module];
    }
    printf("\n  cells %d-%d mV, %u/%u ok, %u good scans | scans %lu checked, %lu with CRC errors, %lu out of order | module CRC errors %lu\n",
        logic.cell_min_mV, logic.cell_max_mV, logic.cells_ok, logic.have_map ? logic.map.num_cells : 0,
        logic.good_scans, (unsigned long)logic.scans, (unsigned long)logic.incomplete_scans,
        (unsigned long)snoop.incomplete_scans, (unsigned long)crc_errors);

    if(logic.have_map) {
        printf("  map: ");
        print_map(&logic.map);
        printf("\n");
    }
    if(logic.have_status) {
        printf("  BMS v%u (ours v%u)", logic.status_version, SUP_PROTOCOL_VERSION);
        if(logic.status_version == SUP_PROTOCOL_VERSION) {
            printf(" up %llu ms, cells %d-%d mV, flags 0x%02x",
                (unsigned long long)logic.status.uptime_ms,
                logic.status.cell_min_mV, logic.status.cell_max_mV, logic.status.flags);
        } else {
            // The rest of a status in another version can't be read
            printf(", status unreadable");
        }
        printf(", age %lu ms | link: %lu replies, %lu skipped, %lu bad frames, %lu overflows\n",
            (unsigned long)(now - logic.status_millis),
            (unsigned long)replies_sent, (unsigned long)replies_skipped,
            (unsigned long)frame_parser.crc_errors, (unsigned long)rx_overflows);
        printf("  HMI: answering at address %u | %lu replies, %lu skipped, %lu bad frames, %lu overflows\n",
            logic.hmi_address, (unsigned long)hmi_replies_sent, (unsigned long)hmi_replies_skipped,
            (unsigned long)hmi_parser.crc_errors, (unsigned long)hmi_rx_overflows);
        if((logic.status_reasons & ~SUP_REASON_VERSION) || logic.map_changed) {
            // (A status in another version has no map we can read)
            printf("  BMS map: ");
            print_map(&logic.status.map);
            printf(" (check 0x%02x)\n", cell_map_check(&logic.status.map));
        }
    } else {
        printf("  no BMS status yet (link frames with bad CRC: %lu)\n", (unsigned long)frame_parser.crc_errors);
    }
}

// Every channel of the last complete scan; * marks the channels the cell map
// says are populated
static void print_scan(void) {
    // Modules past the map's (and past the chain) only matter if they answered
    int modules = logic.have_map ? logic.map.num_modules : CELL_MAP_MODULES;
    for(int module=0; module<CELL_MAP_MODULES; module++) {
        bool read = false;
        for(int cell=0; cell<CELL_MAP_CELLS_PER_MODULE; cell++) {
            read |= snoop.scan_mV[module * CELL_MAP_CELLS_PER_MODULE + cell] != CELL_MV_NOT_READ;
        }
        if(module >= modules && !read) {
            continue;
        }
        printf("BMB %d:", module);
        for(int cell=0; cell<CELL_MAP_CELLS_PER_MODULE; cell++) {
            int physical_index = module * CELL_MAP_CELLS_PER_MODULE + cell;
            int16_t mV = snoop.scan_mV[physical_index];
            char mark = (logic.have_map && cell_map_is_present(&logic.map, physical_index)) ? '*' : ' ';
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
    uint8_t frame[SUP_REPLY_LEN + 4];
    size_t frame_len = duart_frame_encode(payload, len, frame, sizeof(frame));

    // The whole frame fits the empty 32-byte FIFO, so this never waits. If
    // the last reply is somehow still going out, skip this one: the BMS only
    // needs a recent reply, not every one.
    _Static_assert(SUP_REPLY_LEN + 4 <= 32, "reply must fit the UART FIFO");
    if(!(uart_get_hw(INTERNAL_UART)->fr & UART_UARTFR_TXFE_BITS)) {
        replies_skipped++;
        return;
    }
    for(size_t i=0; i<frame_len; i++) {
        uart_get_hw(INTERNAL_UART)->dr = frame[i];
    }
    replies_sent++;
}

// Returns whether a BMS status arrived
static bool poll_internal_serial(uint32_t now) {
    bool got_status = false;
    while(rx_tail != rx_head) {
        uint8_t c = rx_ring[rx_tail];
        rx_tail = rx_tail + 1;

        sup_frame_feed(&frame_parser, c);
        const uint8_t *payload;
        size_t len;
        while((len = sup_frame_next(&frame_parser, &payload)) > 0) {
            sup_bms_status_t status;
            if(sup_unpack_bms_status(payload, len, &status)) {
                sup_logic_on_status(&logic, &status, now);
                got_status = true;
            }
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
            if(cell_snoop_take_scan(&snoop)) {
                sup_logic_on_scan(&logic, snoop.scan_mV, snoop.scan_module_ok, now);
            }
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
    gpio_init(PIN_CTR_EN);
    gpio_set_dir(PIN_CTR_EN, GPIO_OUT);
    gpio_put(PIN_CTR_EN, false);

    stdio_usb_init();

#if USB_EXTERNAL_PULLUP
    // The external D+ pull-up is powered from VBUS; switch the chip's own one
    // (enabled by the USB stack at init) off so it can't back-power the host.
    tud_disconnect();
    usb_force_reenumerate();
#endif
#if USB_VBUS_SENSE
    usb_vbus_sense_init();
#endif

    // Time to open a terminal before the banner (CTR_EN stays low meanwhile)
    sleep_ms(4000);

    gpio_init(PIN_MCU_CHECK);
    gpio_set_dir(PIN_MCU_CHECK, GPIO_IN);
    if(!gpio_get(PIN_MCU_CHECK)) {
        printf("Error: Supervisor firmware is running on the main MCU!\n");
        while(1) {
            tight_loop_contents();
        }
    }

    init_watchdog();
    printf("Supervisor starting: %s chemistry (cells %d-%d mV), board %s, protocol v%u\n",
        SUPERVISOR_CHEMISTRY_NAME, CELL_VOLTAGE_HARD_MIN_mV, CELL_VOLTAGE_HARD_MAX_mV,
        BMS_BOARD_NAME, SUP_PROTOCOL_VERSION);
    printf("The cell map comes from the BMS. USB commands: s = status, r = last snooped scan\n");

    isosnoop_setup(
        PIN_ISOSPI_RX_HIGH,
        PIN_ISOSNOOP_RX_DEBUG,
        PIN_ISOSNOOP_TIMER_DISABLE
    );

    cell_snoop_init(&snoop);
    sup_logic_init(&logic);
    sup_frame_parser_init(&frame_parser, SUP_BMS_STATUS_LEN);

    init_serial();
    init_hmi_serial();
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, true);

    sup_state_t last_state = logic.state;
    uint16_t last_reasons = logic.reasons;
    bool had_map = false;
    uint32_t last_print = now_ms();

    while(1) {
        watchdog_update();
        uint32_t now = now_ms();

        poll_snoop(now);
        bool got_status = poll_internal_serial(now);

        gpio_put(PIN_CTR_EN, sup_logic_tick(&logic, now));

        if(got_status) {
            // Answer with the state as of this status
            send_reply();
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
        }

        poll_hmi_serial(now);

        if(logic.have_map && !had_map) {
            printf("Supervisor: cell map from the BMS: ");
            print_map(&logic.map);
            printf("\n");
        }
        had_map = logic.have_map;

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
#if USB_VBUS_SENSE
        usb_vbus_sense_poll();
#endif

        sleep_ms(1);
    }

    return 0;
}
