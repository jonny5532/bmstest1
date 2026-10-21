#define PIN_ISOSPI_TX_EN 26
#define PIN_ISOSPI_TX_DATA 27
#define PIN_ISOSPI_RX_HIGH 28
#define PIN_ISOSPI_RX_LOW 29
#define PIN_ISOSPI_RX_AND 30
#define PIN_ISOSPI_RX_DEBUG 31
#define PIN_ISOSNOOP_RX_DEBUG 32
// The isosnoop state machines pass a flag through this pin (one sets it, another
// tests it with jmp pin), so it must be in the PIO's GPIO window and driven by
// nothing else. GPIO30 isn't bonded to anything on the rev1 PCB (GPIO5 goes to
// the J22 header there). Rev0 needs GPIO5: on ck1 the snooper decoded only
// garbage with the flag on GPIO30, and every scan with it on GPIO5.
#include "config/limits.h"
#if BMS_BOARD == BMS_BOARD_REV0
#define PIN_ISOSNOOP_TIMER_DISABLE 5
#else
#define PIN_ISOSNOOP_TIMER_DISABLE 30
#endif

// USB VBUS through a 3.3k/5.1k divider (J22 pin 2), for BMS_USB_VBUS_SENSE
#define PIN_USB_VBUS_SENSE 3

#define PIN_HMI_SERIAL_RX 39  // was 38
#define PIN_HMI_SERIAL_TX 38  // was 39
