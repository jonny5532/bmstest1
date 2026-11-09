#pragma once

// Supervisor MCU pins

#define PIN_ISOSPI_TX_EN 26
#define PIN_ISOSPI_TX_DATA 27
#define PIN_ISOSPI_RX_HIGH 28
#define PIN_ISOSPI_RX_LOW 29
#define PIN_ISOSPI_RX_DEBUG 31
#define PIN_ISOSNOOP_RX_DEBUG 32
// The isosnoop state machines pass a flag through this pin (one sets it, another
// tests it with jmp pin), so it must be in the PIO's GPIO window and driven by
// nothing else. GPIO30 isn't bonded to anything on the rev1 PCB (GPIO5 was on
// the J22 header).
#define PIN_ISOSNOOP_TIMER_DISABLE 30

// USB VBUS through a 3.3k/5.1k divider (J22 pin 2), on USB_VBUS_SENSE boards
#define PIN_USB_VBUS_SENSE 3

#define PIN_HMI_SERIAL_RX 39  // was 38
#define PIN_HMI_SERIAL_TX 38  // was 39

// Strapped high on the supervisor, low on the BMS MCU
#define PIN_MCU_CHECK 0

// Internal serial link to the BMS (uart0)
#define PIN_INTERNAL_SERIAL_RX 1
#define PIN_INTERNAL_SERIAL_TX 2

// Gates the contactor MOSFET gate drivers (CTR_EN): high enables them
#define PIN_CTR_EN 14
#define PIN_LED 25
