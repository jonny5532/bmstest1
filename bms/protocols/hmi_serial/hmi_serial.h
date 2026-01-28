#define HMI_MSG_ANNOUNCE_DEVICE      0x81
#define HMI_MSG_SET_DEVICE_ADDRESS   0x01

#define HMI_MSG_WRITE_REGISTERS      0x03

#define HMI_MSG_READ_REGISTERS       0x04
#define HMI_MSG_READ_REGISTERS_RESPONSE 0x84

#define HMI_MSG_READ_CELL_VOLTAGES   0x05
#define HMI_MSG_READ_CELL_VOLTAGES_RESPONSE 0x85

#define HMI_MSG_READ_EVENTS          0x06
#define HMI_MSG_READ_EVENTS_RESPONSE 0x86

#define HMI_ANNOUNCE_DEVICE_TYPE_BMS 0x01

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
#define HMI_REG_SOC                     3 // uint16
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
#define HMI_REG_SYSTEM_REQUEST         14 // uint8
#define HMI_REG_SYSTEM_STATE           15 // uint8
#define HMI_REG_CONTACTORS_STATE       16 // uint8
#define HMI_REG_SOC_VOLTAGE_BASED      17 // uint16
#define HMI_REG_SOC_BASIC_COUNT        18 // uint16

#define HMI_REG_CAPACITY               20 // uint32 (mC)
#define HMI_REG_SUPPLY_VOLTAGE_3V3     21 // uint16 (mV)
#define HMI_REG_SUPPLY_VOLTAGE_5V      22 // uint16 (mV)
#define HMI_REG_SUPPLY_VOLTAGE_12V     23 // uint16 (mV)
#define HMI_REG_SUPPLY_VOLTAGE_CTR     24 // uint16 (mV)
#define HMI_REG_CELL_VOLTAGE_LIMIT_MIN 25 // uint16 (mV)
#define HMI_REG_CELL_VOLTAGE_LIMIT_MAX 26 // uint16 (mV)
#define HMI_REG_SOC_SCALING_MIN        27 // int16 (0.01%)
#define HMI_REG_SOC_SCALING_MAX        28 // int16 (0.01%)
#define HMI_REG_VOLTAGE_LIMIT_OFFSET_LOWER 29 // int16 (0.1V)
#define HMI_REG_VOLTAGE_LIMIT_OFFSET_UPPER 30 // int16 (0.1V)

#define HMI_REG_CELL_VOLTAGES_START 0x100
#define HMI_REG_CELL_VOLTAGES_END   0x1FF

#define HMI_REG_MODULE_TEMPS_START    0x200
#define HMI_REG_MODULE_TEMPS_END      0x207
#define HMI_REG_RAW_TEMPS_START       0x208
#define HMI_REG_RAW_TEMPS_END         0x238

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

2.1. Amnounce device message (from BMS to HMI)

The announce device message is used by a BMS to announce its presence to the HMI.
The format is:

<message type byte = HMI_MSG_ANNOUNCE_DEVICE (0x01)>
<device type byte> (0x01 = BMS)
<device address (1 byte)> (initially 0 for all devices, to be assigned by HMI later)
<unique serial number (8 bytes)>

2.2. Set device address (from HMI to BMS)

The set device address message is used by the HMI to assign a unique address
to each BMS on the network. The format is:

<message type byte = HMI_MSG_SET_DEVICE_ADDRESS (0x01)>
<current device address (1 byte)>
<new device address (1 byte)>
<unique serial number (8 bytes)>

Following a successful address assignment, the device will respond with an
ANNOUNCE_DEVICE.

2.3. Write registers (from HMI to BMS)

The write registers message is used by the HMI to write one or more registers
on the BMS. The format is:

<message type byte = HMI_MSG_WRITE_REGISTERS (0x03)>
<device address (1 byte)>
<register 1 id (2 bytes)>
<register 1 type (1 byte)>
<register 1 value (N bytes)>
<register 2 id (2 bytes)>
<register 2 type (1 byte)>
<register 2 value (N bytes)>
...

Following a successfully processed message, the BMS will respond with a
READ_REGISTERS_RESPONSE containing the new values of the written registers.

2.4. Read registers (from HMI to BMS)

The read registers message is used by the HMI to request the values of one or more
registers from the BMS. The format is:

<message type byte = HMI_MSG_READ_REGISTERS (0x04)>
<device address (1 byte)>
<register 1 id (2 bytes)>
<register 2 id (2 bytes)>
...

Following a successfully processed message, the BMS will respond with a
READ_REGISTERS_RESPONSE containing the requested register values.

2.5. Read register response (from BMS to HMI)

The read register response message is used by the BMS to respond to a read registers
request from the HMI. The format is:

<message type byte = HMI_MSG_READ_REGISTERS_RESPONSE (0x84)>
<device address (1 byte)>
<register 1 id (2 bytes)>
<register 1 type (1 byte)>
<register 1 value (N bytes)>
<register 2 id (2 bytes)>
<register 2 type (1 byte)>
<register 2 value (N bytes)>
...

2.6. Read events (from HMI to BMS)

The read events message is used by the HMI to request active or previous events
from the BMS. The format is:

<message type byte = HMI_MSG_READ_EVENTS (0x06)>
<device address (1 byte)>
<start index (2 bytes)>

2.7. Read events response (from BMS to HMI)

The read events response is sent by the BMS in response to a read events request.
The format is:

<message type byte = HMI_MSG_READ_EVENTS_RESPONSE (0x86)>
<device address (1 byte)>
<next index (2 bytes)>
<event count in packet (2 bytes)>
<event 1 type (2 bytes)>
<event 1 level (2 bytes)>
<event 1 count (2 bytes)>
<event 1 timestamp (8 bytes)>
<event 1 data (8 bytes)>
...

*/

typedef struct bms_model bms_model_t;

void hmi_serial_tick(bms_model_t *model);
