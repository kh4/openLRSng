#ifndef _DEFINES_H_
#define _DEFINES_H_

// ########### FUNCTIONS

// string expander
#define STR(S) #S
#define XSTR(S) STR(S)

// helper to access data structure into uint8_t array
#define AS_U8ARRAY(x) ((uint8_t *)(x))

#define MIN256(x)  (((x)<256) ? 256 : (x))
#define ROUNDUP(x) (((x) + 15) & 0xFFF0)
#define SIGNIT(x) ((int16_t)(((x & 0x200) ? 0xFC00U:0) | (x & 0x3FF)))
#define NONZERO(x) ((x != 0) ? x : 1)

// for bit-banged SPI
#define NOP() __asm__ __volatile__("nop")

// minimum required input channel count
// 0 - no PPM needed, 1=2ch ... 0x0F=16ch
#define TX_CONFIG_GETMINCH() (tx_config.flags >> 28)
#define TX_CONFIG_SETMINCH(x) (tx_config.flags = (tx_config.flags & 0x0FFFFFFF) | (((uint32_t)(x) & 0x0F) << 28))

// Sending an 'x' byte packet at 'y' bps takes approx. (empirical):
// usec = (x + 15 {20 w/ diversity}) * 8200000 / bps
#define BYTES_AT_BAUD_TO_USEC(bytes, bps, div) ((uint32_t)((bytes) + (div?20:15)) * 8200000L / (uint32_t)(bps))

#define DATARATE_COUNT (sizeof(modem_params) / sizeof(modem_params[0]))

#define BIND_MAGIC (0xDEC1BE15 + (OPENLRSNG_VERSION & 0xFFF0))
#define BINDING_VERSION ((OPENLRSNG_VERSION & 0x0FF0)>>4)

#if (COMPILE_TX == 1)
#define EEPROM_DATASIZE MIN256(ROUNDUP((sizeof(tx_config) + sizeof(bind_data) + 4) * 4 + 3))
#else
#define EEPROM_DATASIZE MIN256(ROUNDUP(sizeof(rx_config) + sizeof(bind_data) + sizeof(failsafePPM) + 6))
#endif

// helper macro for European PMR channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch1-ch16 (Jan 2016  ECC update)

// helper macro for US FRS channels 1-7
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch1-ch7


// ######## CONSTANTS ########

// EEPROM is 1k on 328p and 32u4
#define EEPROM_SIZE 1024

// RX board types
#define RX_FLYTRON8CH 0x01
#define RX_OLRSNG4CH  0x02
#define RX_OLRSNG12CH 0x03
#define RX_DTFUHF10CH 0x04
#define RX_PTOWER     0x05
#define RX_MICRO      0x06
#define RX_FLYTRONM3  0x07
#define RX_BRORX      0x08

#define PINMAP_PPM    0x20
#define PINMAP_RSSI   0x21
#define PINMAP_SDA    0x22
#define PINMAP_SCL    0x23
#define PINMAP_RXD    0x24
#define PINMAP_TXD    0x25
#define PINMAP_ANALOG 0x26
#define PINMAP_LBEEP  0x27 // packet loss beeper
#define PINMAP_SPKTRM 0x28 // Spektrum satellite output
#define PINMAP_SBUS   0x29 // SBUS output
#define PINMAP_SUMD   0x2A // SUMD output
#define PINMAP_LLIND  0x2B // Link-Loss indication (digital output)


// RX PPM / PWM constants
#define PWM_MULTIPLIER 2
#define PWM_DEJITTER   32
#define PPM_PULSELEN   600
#define PPM_FRAMELEN   40000

// Serial buffer size
#define SERIAL_BUFSIZE 32

// TX serial telemetry states
#define SERIAL_WAIT0    0
#define SERIAL_WAIT1    1
#define SERIAL_SEND      2
#define SERIAL_ACK        3
#define SERIAL_RESEND  4

// Serial RC protocol symbols
#define SBUS_SYNC 0x0F
#define SBUS_TAIL 0x00
#define SPKTRM_SYNC1 0x03
#define SPKTRM_SYNC2 0x01
#define SUMD_HEAD 0xA8


// PSP binary configuration protocol symbols
#define PSP_SYNC1 0xB5
#define PSP_SYNC2 0x62

#define PSP_REQ_BIND_DATA               1
#define PSP_REQ_RX_CONFIG               2
#define PSP_REQ_RX_JOIN_CONFIGURATION   3
#define PSP_REQ_SCANNER_MODE            4
#define PSP_REQ_SPECIAL_PINS            5
#define PSP_REQ_FW_VERSION              6
#define PSP_REQ_NUMBER_OF_RX_OUTPUTS    7
#define PSP_REQ_ACTIVE_PROFILE          8
#define PSP_REQ_RX_FAILSAFE             9
#define PSP_REQ_TX_CONFIG               10
#define PSP_REQ_PPM_IN                  11
#define PSP_REQ_DEFAULT_PROFILE         12

#define PSP_SET_BIND_DATA               101
#define PSP_SET_RX_CONFIG               102
#define PSP_SET_TX_SAVE_EEPROM          103
#define PSP_SET_RX_SAVE_EEPROM          104
#define PSP_SET_TX_RESTORE_DEFAULT      105
#define PSP_SET_RX_RESTORE_DEFAULT      106
#define PSP_SET_ACTIVE_PROFILE          107
#define PSP_SET_RX_FAILSAFE             108
#define PSP_SET_TX_CONFIG               109
#define PSP_SET_DEFAULT_PROFILE         110

#define PSP_SET_EXIT                    199

#define PSP_INF_ACK                     201
#define PSP_INF_REFUSED                 202
#define PSP_INF_CRC_FAIL                203
#define PSP_INF_DATA_TOO_LONG           204

// RFM interrupt states
#define AVAILABLE    0
#define TRANSMIT    1
#define TRANSMITTED  2
#define RECEIVE    3
#define RECEIVED  4

// TX_CONFIG flag masks
#define SW_POWER            0x04 // enable powertoggle via switch (JR dTX)
#define ALT_POWER           0x08
#define MUTE_TX             0x10 // do not beep on telemetry loss
#define MICROPPM            0x20
#define INVERTED_PPMIN      0x40
#define WATCHDOG_USED       0x80 // read only flag, only sent to configurator

// RX_CONFIG flag masks
#define PPM_MAX_8CH         0x01
#define ALWAYS_BIND         0x02
#define SLAVE_MODE          0x04
#define IMMEDIATE_OUTPUT    0x08
#define STATIC_BEACON       0x10
#define INVERTED_PPMOUT      0x40
#define WATCHDOG_USED       0x80 // read only flag, only sent to configurator

// BIND_DATA flag masks
#define TELEMETRY_OFF       0x00
#define TELEMETRY_PASSTHRU  0x08
#define TELEMETRY_FRSKY     0x10 // covers smartport if used with &
#define TELEMETRY_SMARTPORT 0x18
#define TELEMETRY_MASK      0x18
#define CHANNELS_4_4        0x01
#define CHANNELS_8          0x02
#define CHANNELS_8_4        0x03
#define CHANNELS_12         0x04
#define CHANNELS_12_4       0x05
#define CHANNELS_16         0x06
#define DIVERSITY_ENABLED   0x80
#define DEFAULT_FLAGS       (CHANNELS_8 | TELEMETRY_PASSTHRU)

// HW frequency limits
#if (RFMTYPE == 868)
#  define MIN_RFM_FREQUENCY 848000000
#  define MAX_RFM_FREQUENCY 888000000
#  define DEFAULT_CARRIER_FREQUENCY 868000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 868000000 // Hz
#elif (RFMTYPE == 915)
#  define MIN_RFM_FREQUENCY 895000000
#  define MAX_RFM_FREQUENCY 935000000
#  define DEFAULT_CARRIER_FREQUENCY 915000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 915000000 // Hz
#else
#  define MIN_RFM_FREQUENCY 413000000
#  define MAX_RFM_FREQUENCY 463000000
#  define DEFAULT_CARRIER_FREQUENCY 435000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 435000000 // Hz
#endif


//####### RADIOLINK RF POWER (beacon is always 100/13/1.3mW) #######
// 7 == 100mW (or 1000mW with M3)
// 6 == 50mW (use this when using booster amp), (800mW with M3)
// 5 == 25mW
// 4 == 13mW
// 3 == 6mW
// 2 == 3mW
// 1 == 1.6mW
// 0 == 1.3mW
#define DEFAULT_RF_POWER 7

#define DEFAULT_CHANNEL_SPACING 5 // 50kHz
#define DEFAULT_HOPLIST 22,10,19,34,49,41
#define DEFAULT_RF_MAGIC 0xDEADFEED

//  0 -- 4800bps, best range
//  1 -- 9600bps, medium range
//  2 -- 19200bps, medium range
#define DEFAULT_DATARATE 2

#define DEFAULT_BAUDRATE 115200

#define DEFAULT_BEACON_FREQUENCY 0 // disable beacon
#define DEFAULT_BEACON_DEADTIME 30 // time to wait until go into beacon mode (30s)
#define DEFAULT_BEACON_INTERVAL 10 // interval between beacon transmits (10s)

#define MIN_DEADTIME 0
#define MAX_DEADTIME 255

#define MIN_INTERVAL 1
#define MAX_INTERVAL 255

// max number of hop channels
#define MAXHOPS      24

// max number of PPM channels
#define PPM_CHANNELS 16

// RFM power level used in BIND mode
// not lowest since may result fail with RFM23BP
#define BINDING_POWER     0x06

// Telemetry sequence control bits
#define UPLINK_SEQBIT 0x80
#define DOWNLINK_SEQBIT 0x40

#define DOWNLINK_PACKETSIZE 9
#define UPLINK_MAX_PACKETSIZE 21

#ifndef BZ_FREQ
#define BZ_FREQ 2000
#endif

#define TX_PROFILE_COUNT  4


// Following table is used by the dialog code to
// determine possible extra functions for each output.
typedef struct pinMask {
  uint8_t B;
  uint8_t C;
  uint8_t D;
} pinMask_t;

typedef struct rxSpecialPinMap {
  uint8_t output;
  uint8_t type;
} rxSpecialPinMap_t;

// Serial protocol modes
typedef enum {
  SERIAL_MODE_NONE = 0,
  SERIAL_MODE_SPEKTRUM1024,
  SERIAL_MODE_SPEKTRUM2048,
  SERIAL_MODE_SBUS,
  SERIAL_MODE_SUMD,
  SERIAL_MODE_MULTI,
  SERIAL_MODE_MAX = SERIAL_MODE_MULTI
} serialMode_e;

struct sbus_help {
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
} __attribute__ ((__packed__));

struct sbus {
  struct sbus_help ch[2];
  uint8_t status;
} __attribute__ ((__packed__));

// This is common temporary buffer
// used by all PPM input methods
typedef union {
  uint8_t  bytes[32];
  uint16_t words[16];
  struct sbus sbus;
} ppm_msg;

#endif
