#define PIN_3V3_SENSE 40
#define PIN_5V_SENSE 41
#define PIN_12V_SENSE 42
#define PIN_CONTACTOR_SENSE 43

#define PIN_CONTACTOR_POS 18
#define PIN_CONTACTOR_NEG 17
#define PIN_CONTACTOR_PRE 16
#define PIN_CONTACTOR_FC_NEG 15  // not used

#define PIN_AUX_CONTACTOR_PRE 9

#define PIN_INA228_I2C_SDA 24
#define PIN_INA228_I2C_SCL 21

#define PIN_ADS1115_I2C_SDA 22
#define PIN_ADS1115_I2C_SCL 23

#define PIN_CAN_TX 19
#define PIN_CAN_RX 20

#define PIN_INTERCAN_TX 4
#define PIN_INTERCAN_RX 5

#define PIN_MCU_CHECK 0
#define PIN_ESTOP 11
#define PIN_EXTRA_INPUT 12
#define PIN_EXTRA_OUTPUT_1 13
#define PIN_EXTRA_OUTPUT_2 14
#define PIN_LED 25

#define PIN_INTERNAL_SERIAL_RX 1
#define PIN_INTERNAL_SERIAL_TX 2

#define PIN_HMI_SERIAL_RX 39  // was 38
#define PIN_HMI_SERIAL_TX 38  // was 39

#define PIN_ISOSPI_TX_EN 26
#define PIN_ISOSPI_TX_DATA 27
#define PIN_ISOSPI_RX_HIGH 28
#define PIN_ISOSPI_RX_LOW 29
#define PIN_ISOSPI_RX_AND 30
#define PIN_ISOSPI_RX_DEBUG 31
#define PIN_ISOSNOOP_RX_DEBUG 32
#define PIN_ISOSNOOP_TIMER_DISABLE 3 // spare GPIO that isn't used for anything else

// Each PIO SM can only see 32 GPIOs (0-31 by default? but can be remapped to
// 16-47 via the GPIOBASE register)

/* ADS1115 inputs:
AIN0 = bat pos
AIN1 = bat neg
AIN2 = FC pos
AIN3 = FC neg

bat voltage: AIN0 - AIN1
fc voltage: AIN2 - AIN3


*/