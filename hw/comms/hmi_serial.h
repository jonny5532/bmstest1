#define HMI_MSG_REGISTER_BROADCAST 0x01

#define HMI_TYPE_UINT8  0x11
#define HMI_TYPE_INT8   0x21
#define HMI_TYPE_UINT16 0x12
#define HMI_TYPE_INT16  0x22
#define HMI_TYPE_UINT32 0x14
#define HMI_TYPE_INT32  0x24
#define HMI_TYPE_UINT64 0x18
#define HMI_TYPE_INT64  0x28

#define HMI_TYPE_FLOAT  0x34
#define HMI_TYPE_DOUBLE 0x38

#define HMI_REG_SERIAL                  1 // uint64
#define HMI_REG_MILLIS                  2 // uint64
#define HMI_REG_SOC                     3 // uint32
#define HMI_REG_CURRENT                 4 // int32 (mA)
#define HMI_REG_CHARGE                  5 // int64 (mC)
#define HMI_REG_BATTERY_VOLTAGE         6 // int32 (mV)
#define HMI_REG_OUTPUT_VOLTAGE          7 // int32 (mV)
#define HMI_REG_POS_CONTACTOR_VOLTAGE   8 // int32 (mV)
#define HMI_REG_NEG_CONTACTOR_VOLTAGE   9 // int32 (mV)
#define HMI_REG_TEMPERATURE_MIN        10 // int16 (0.1C)
#define HMI_REG_TEMPERATURE_MAX        11 // int16 (0.1C)
#define HMI_REG_CELL_VOLTAGE_MIN       12 // int16 (mV)
#define HMI_REG_CELL_VOLTAGE_MAX       13 // int16 (mV)
#define HMI_REG_CELL_VOLTAGES_START 0x100
#define HMI_REG_CELL_VOLTAGES_END   0x1FF

/* 

HMI serial format

1. Packet framing

Serial messages are sent in packets. Each packet consists of:

0xff (sync byte)
<length-1 byte> (not counting the sync, CRC bytes or length byte itself)
<payload bytes> (min 1 byte)
<crc16 bytes>

2. HMI message format

The payload of each packet consists of one or more HMI messages concatenated
together. Each message starts with a message type byte, followed by message-specific
data.

2.1. Register broadcast message

The register broadcast message is used to send the values of one or more registers
from the BMS to the HMI. The format is:

<message type byte = HMI_MSG_REGISTER_BROADCAST (0x01)>
<register 1 id (2 bytes)>
<register 1 type (1 byte)>
<register 1 value (N bytes)>
<register 2 id (2 bytes)>
<register 2 type (1 byte)>
<register 2 value (N bytes)>
...

where the value length N depends on the type.

*/