#define PIN_12V_SENSE 27

#define PIN_CONTACTOR_POS 1
#define PIN_CONTACTOR_NEG 25
#define PIN_CONTACTOR_PRE 2

#define PIN_INA228_I2C_SDA 24
#define PIN_INA228_I2C_SCL 23

// TODO - these need swapping around
#define PIN_ADS1115_I2C_SDA 21
#define PIN_ADS1115_I2C_SCL 22

#define PIN_CAN_TX 19
#define PIN_CAN_RX 20

/* ADS1115 inputs:
AIN0 = bat pos
AIN1 = bat neg
AIN2 = FC pos
AIN3 = FC neg

bat voltage: AIN0 - AIN1
fc voltage: AIN2 - AIN3


*/