#include <pico/stdlib.h>

#define INA228_I2C i2c0

#define ADS1115_I2C i2c1

// CAN1: inverter interface (PIN_CAN_TX/RX)
#define CAN2040_PIO_NUM 0
#define CAN2040_PIO_IRQ0 PIO0_IRQ_0

// CAN2: inter-BMS fleet bus (PIN_INTERCAN_TX/RX). can2040 needs a whole PIO
// block, so this takes PIO2 from isosnoop, which is compiled out when
// BMS_INTERCAN is enabled (see CMakeLists.txt).
#define INTERCAN_PIO_NUM 2
#define INTERCAN_PIO_IRQ0 PIO2_IRQ_0

#define ISOSPI_MASTER_PIO pio1
#define ISOSPI_MASTER_PIO_IRQ0 PIO1_IRQ_0
#define ISOSNOOP_PIO pio2

#define INTERNAL_SERIAL_DUART duart0
#define HMI_SERIAL_DUART duart1
