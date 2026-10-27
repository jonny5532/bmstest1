#pragma once

#define HMI_MSG_ANNOUNCE_DEVICE      0x81
#define HMI_MSG_SET_DEVICE_ADDRESS   0x01

#define HMI_MSG_WRITE_REGISTERS      0x03

#define HMI_MSG_READ_REGISTERS       0x04
#define HMI_MSG_READ_REGISTERS_RESPONSE 0x84

#define HMI_MSG_READ_CELL_VOLTAGES   0x05
#define HMI_MSG_READ_CELL_VOLTAGES_RESPONSE 0x85

#define HMI_MSG_READ_EVENTS          0x06
#define HMI_MSG_READ_EVENTS_RESPONSE 0x86

#define HMI_MSG_READ_BALANCING_TIMES 0x07
#define HMI_MSG_READ_BALANCING_TIMES_RESPONSE 0x87

#define HMI_MSG_READ_LOG             0x08
#define HMI_MSG_READ_LOG_RESPONSE    0x88

#define HMI_MSG_READ_ARRAY           0x09
#define HMI_MSG_READ_ARRAY_RESPONSE  0x89

#define HMI_MSG_WRITE_ARRAY          0x0A

// Version of this protocol, in HMI_REG_PROTOCOL_VERSION:
//   1 = READ_ARRAY
//   2 = WRITE_ARRAY, the EKF and OCV learner arrays, HMI_REG_OCV_LEARN_*
//   3 = HMI_REG_NAMEPLATE_CAPACITY; HMI_REG_CAPACITY is the actual capacity, saved to NVM;
//       the learned capacity, HMI_REG_OCV_LEARN_CAPACITY*
#define HMI_PROTOCOL_VERSION 3

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

#define HMI_REG_CAPACITY               20 // uint32 (mC), actual capacity the SOC estimator uses. Writes are
                                          // saved to NVM; 0 goes back to the profile's ACTUAL_CAPACITY_AH
#define HMI_REG_SUPPLY_VOLTAGE_3V3     21 // uint16 (mV)
#define HMI_REG_SUPPLY_VOLTAGE_5V      22 // uint16 (mV)
#define HMI_REG_SUPPLY_VOLTAGE_12V     23 // uint16 (mV)
#define HMI_REG_SUPPLY_VOLTAGE_CTR     24 // uint16 (mV)
#define HMI_REG_CELL_VOLTAGE_WORKING_MIN 25 // uint16 (mV)
#define HMI_REG_CELL_VOLTAGE_WORKING_MAX 26 // uint16 (mV)
#define HMI_REG_SOC_SCALING_MIN        27 // int16 (0.01%)
#define HMI_REG_SOC_SCALING_MAX        28 // int16 (0.01%)
#define HMI_REG_VOLTAGE_LIMIT_OFFSET_LOWER 29 // int16 (0.1V)
#define HMI_REG_VOLTAGE_LIMIT_OFFSET_UPPER 30 // int16 (0.1V)
#define HMI_REG_CELL_VOLTAGE_SOFT_MIN 31 // uint16 (mV)
#define HMI_REG_CELL_VOLTAGE_SOFT_MAX 32 // uint16 (mV)
#define HMI_REG_AUTO_BALANCING_PERIOD_MS     33 // uint32
#define HMI_REG_BALANCING_PERIODS_PER_MV     34 // uint16
#define HMI_REG_BALANCE_MIN_OFFSET_MV        35 // uint16
#define HMI_REG_MINIMUM_BALANCING_VOLTAGE_MV 36 // uint16
#define HMI_REG_INVERTER_SOC                  37 // int16 (0.01%)
#define HMI_REG_INVERTER_FULL_CAPACITY_DAH    38 // int16 (0.1Ah)
#define HMI_REG_INVERTER_REMAINING_CAPACITY_DAH 39 // int16 (0.1Ah)
#define HMI_REG_INVERTER_MIN_VOLTAGE_LIMIT_DV 40 // int16 (0.1V)
#define HMI_REG_INVERTER_MAX_VOLTAGE_LIMIT_DV 41 // int16 (0.1V)
#define HMI_REG_BALANCING_START_VOLTAGE_MV    42 // uint16

#define HMI_REG_USER_CHARGE_CURRENT_LIMIT_DA            43 // uint16 (0.1A)
#define HMI_REG_USER_DISCHARGE_CURRENT_LIMIT_DA         44 // uint16 (0.1A)
#define HMI_REG_CHARGE_CURRENT_LIMIT_DA                 45 // uint16 (0.1A)
#define HMI_REG_DISCHARGE_CURRENT_LIMIT_DA              46 // uint16 (0.1A)
#define HMI_REG_TEMP_CHARGE_CURRENT_LIMIT_DA            47 // uint16 (0.1A)
#define HMI_REG_TEMP_DISCHARGE_CURRENT_LIMIT_DA         48 // uint16 (0.1A)
#define HMI_REG_CELL_VOLTAGE_CHARGE_CURRENT_LIMIT_DA    49 // uint16 (0.1A)
#define HMI_REG_CELL_VOLTAGE_DISCHARGE_CURRENT_LIMIT_DA 50 // uint16 (0.1A)
#define HMI_REG_WORKING_CHARGE_CURRENT_LIMIT_DA         51 // uint16 (0.1A)
#define HMI_REG_WORKING_DISCHARGE_CURRENT_LIMIT_DA      52 // uint16 (0.1A)
#define HMI_REG_DELTA_CHARGE_CURRENT_LIMIT_DA           53 // uint16 (0.1A)
#define HMI_REG_HARD_CHARGE_CURRENT_LIMIT_DA            54 // uint16 (0.1A)
#define HMI_REG_HARD_DISCHARGE_CURRENT_LIMIT_DA         55 // uint16 (0.1A)

#define HMI_REG_LINK_VOLTAGE           56 // int32 (mV), rev1 boards only

#define HMI_REG_PROTOCOL_VERSION       71 // uint16, HMI_PROTOCOL_VERSION; absent before READ_ARRAY

// The OCV learner (bms/app/estimators/ocv_learn.h), when the profile has OCV_LEARN
#define HMI_REG_OCV_LEARN_S_FULL       72 // float, factory-curve SOC (0-1] of full charge; absent until learning
                                          // starts. Writing it starts learning (to restore a saved learner).
                                          // More than 1% from the charge voltage's, it's re-anchored there.
#define HMI_REG_OCV_LEARN_CAPACITY_REF 73 // float (Ah), from full to OCV_LEARN_V_REF; NaN until trusted

#define HMI_REG_NAMEPLATE_CAPACITY     74 // uint32 (mC), the profile's NAMEPLATE_CAPACITY_AH (the label's); read only

// Whole-pack capacity learned from every trusted learner bin (see ocv_learn_t), when the
// profile has OCV_LEARN. Read only. The floats are NaN while the state is below PRELIMINARY.
#define HMI_REG_OCV_LEARN_CAPACITY        75 // float (Ah)
#define HMI_REG_OCV_LEARN_CAPACITY_SD     76 // float (Ah), 1-sigma uncertainty
#define HMI_REG_OCV_LEARN_CAPACITY_STATE  77 // uint8, OCV_LEARN_CAPACITY_* (not learning, no data,
                                             // preliminary, good)
#define HMI_REG_OCV_LEARN_CAPACITY_DEPTH  78 // float (0-1), SOC below full of the deepest trusted bin
#define HMI_REG_OCV_LEARN_CAPACITY_DEPTH_V 79 // float (V/cell), learned OCV at that bin
#define HMI_REG_OCV_LEARN_CAPACITY_DEPTH_CYCLES 80 // float, that bin's decaying count of full charge cycles

// Health and diagnostics, for a long-term record (the HMI firmware reads these from 96)
#define HMI_REG_EKF_CAPACITY           96 // float (Ah), the EKF's estimate of the pack's capacity; absent until it runs
#define HMI_REG_SOC_FANCY_COUNT        97 // uint16 (0.01%), the third SOC estimate (coulomb count with corrections)
#define HMI_REG_OVERCURRENT_CHARGE_BUFFER    98 // float (%), how close charging is to an overcurrent trip (100 = trips)
#define HMI_REG_OVERCURRENT_DISCHARGE_BUFFER 99 // float (%), the same for discharging
#define HMI_REG_SOFT_CHARGE_BUFFER    100 // float (%), charge past the soft cell limits, of what trips it (+ charge, - discharge)
#define HMI_REG_UART0_PACKETS_RECEIVED 101 // uint32, since boot (each serial port's counters wrap)
#define HMI_REG_UART0_PACKETS_SENT     102 // uint32
#define HMI_REG_UART0_CRC_ERRORS       103 // uint32
#define HMI_REG_UART1_PACKETS_RECEIVED 104 // uint32
#define HMI_REG_UART1_PACKETS_SENT     105 // uint32
#define HMI_REG_UART1_CRC_ERRORS       106 // uint32
#define HMI_REG_CAN_FRAMES_SENT        107 // uint32
#define HMI_REG_CAN_FRAMES_RECEIVED    108 // uint32
#define HMI_REG_BALANCING_STATE        109 // uint8, balancing_states (0 idle, 1 active...)
#define HMI_REG_CALIBRATION_STATE      110 // uint8, calibration_state_t (0 idle)

// Deprecated: these ranges and READ_CELL_VOLTAGES / READ_BALANCING_TIMES stay
// for older HMIs, and are fixed at the first 120 cells and 8 modules. Use
// READ_ARRAY instead.

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

2.8. Read array (from HMI to BMS)

The read array message is used by the HMI to read part of one of the BMS's
arrays (HMI_ARRAY_* in hmi_arrays.h). The format is:

<message type byte = HMI_MSG_READ_ARRAY (0x09)>
<device address (1 byte)>
<array id (1 byte)>
<start index (2 bytes)>

BMS firmware before HMI_PROTOCOL_VERSION 1 ignores this message; the HMI can
check for HMI_REG_PROTOCOL_VERSION first (it is left out of the read register
response by older firmware).

2.9. Read array response (from BMS to HMI)

The read array response is sent by the BMS in response to a read array request,
holding as many elements from the start index as fit in one packet. The HMI
reads the rest by asking again from start index + element count. The format is:

<message type byte = HMI_MSG_READ_ARRAY_RESPONSE (0x89)>
<device address (1 byte)>
<array id (1 byte)>
<encoding (1 byte)> (element HMI_TYPE_*, 0 for an unknown array)
<start index (2 bytes)>
<total elements in the array (2 bytes)>
<element count in packet (1 byte)>
<sample time (4 bytes)> (millis when the values were read, 0 if not tracked)
<elements>

An unknown array id gives a total of 0 and encoding HMI_ARRAY_ENCODING_NONE.
An array with a sample time and no reading yet gives an element count of 0.
Pages read at different sample times may come from different readings.

The encoding is the elements' HMI_TYPE_*, and each element is that type's
usual little-endian value. The top bit (0x80) is reserved for a packed (for
example delta coded) form of the type in future; an HMI should treat an
encoding it doesn't know as an error rather than guess its size.

READ_CELL_VOLTAGES_RESPONSE and READ_BALANCING_TIMES_RESPONSE use an older
delta coding: absolute values are two bytes, big-endian, with the MSB set
(keeping only the low 15 bits of the value); deltas from the previous value in
-64 to +63 are one byte with the MSB clear. Balancing times of 16384 or more
don't survive it.

2.10. Write array (from HMI to BMS)

The write array message is used by the HMI to write part of one of the BMS's
writable arrays (see hmi_arrays.h). The format is:

<message type byte = HMI_MSG_WRITE_ARRAY (0x0A)>
<device address (1 byte)>
<array id (1 byte)>
<encoding (1 byte)> (must be the array's, as given by READ_ARRAY_RESPONSE)
<start index (2 bytes)>
<element count (1 byte)>
<elements> (as in READ_ARRAY_RESPONSE)

The BMS responds with a READ_ARRAY_RESPONSE from the start index, holding the
array's values after the write. A write is all or nothing: if it is refused
(read-only or unknown array, no elements yet, wrong encoding, past the end of
the array, or a value out of range, including any non-finite float), nothing
changes, which the HMI sees as the response not matching what it wrote. A
large array is written a page at a time, so between pages it holds a mix of
old and new values.

*/

typedef struct bms_model bms_model_t;

void init_hmi_serial(void);
void hmi_serial_tick(bms_model_t *model);
