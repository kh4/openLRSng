#ifndef _BINDING_H_
#define _BINDING_H_

#if (COMPILE_TX != 1)
extern uint16_t failsafePPM[PPM_CHANNELS];
#endif

static uint8_t default_hop_list[] = {DEFAULT_HOPLIST};
uint8_t activeProfile = 0;
uint8_t defaultProfile = 0;

struct tx_config {
  uint8_t  rfm_type;
  uint32_t max_frequency;
  uint32_t console_baud_rate;
  uint32_t flags;
  uint8_t  chmap[16];
} tx_config;

// 27 bytes
struct RX_config {
  uint8_t  rx_type; // RX type fillled in by RX, do not change
  uint8_t  pinMapping[13];
  uint8_t  flags;
  uint8_t  RSSIpwm; //0-15 inject composite, 16-31 inject quality, 32-47 inject RSSI, 48-63 inject quality & RSSI on two separate channels
  uint32_t beacon_frequency;
  uint8_t  beacon_deadtime;
  uint8_t  beacon_interval;
  uint16_t minsync;
  uint8_t  failsafeDelay;
  uint8_t  ppmStopDelay;
  uint8_t  pwmStopDelay;
} rx_config;

// 18 bytes
struct bind_data {
  uint8_t version;
  uint32_t serial_baudrate;
  uint32_t rf_frequency;
  uint32_t rf_magic;
  uint8_t rf_power;
  uint8_t rf_channel_spacing;
  uint8_t hopchannel[MAXHOPS];
  uint8_t modem_params;
  uint8_t flags;
} bind_data;

struct rfm22_modem_regs {
  uint32_t bps;
  uint8_t  r_1c, r_1d, r_1e, r_20, r_21, r_22, r_23, r_24, r_25, r_2a, r_6e, r_6f, r_70, r_71, r_72;
} modem_params[] = {
  { 4800, 0x1A, 0x40, 0x0A, 0xA1, 0x20, 0x4E, 0xA5, 0x00, 0x1B, 0x1E, 0x27, 0x52, 0x2C, 0x23, 0x30 }, // 50000 0x00
  { 9600, 0x05, 0x40, 0x0A, 0xA1, 0x20, 0x4E, 0xA5, 0x00, 0x20, 0x24, 0x4E, 0xA5, 0x2C, 0x23, 0x30 }, // 25000 0x00
  { 19200, 0x06, 0x40, 0x0A, 0xD0, 0x00, 0x9D, 0x49, 0x00, 0x7B, 0x28, 0x9D, 0x49, 0x2C, 0x23, 0x30 }, // 25000 0x01
  { 57600, 0x05, 0x40, 0x0A, 0x45, 0x01, 0xD7, 0xDC, 0x03, 0xB8, 0x1E, 0x0E, 0xBF, 0x00, 0x23, 0x2E },
  { 125000, 0x8A, 0x40, 0x0A, 0x60, 0x01, 0x55, 0x55, 0x02, 0xAD, 0x1E, 0x20, 0x00, 0x00, 0x23, 0xC8 },
};

struct rfm22_modem_regs bind_params =
{ 9600, 0x05, 0x40, 0x0A, 0xA1, 0x20, 0x4E, 0xA5, 0x00, 0x20, 0x24, 0x4E, 0xA5, 0x2C, 0x23, 0x30 };


uint16_t CRC16_value;

// prototypes
void fatalBlink(uint8_t blinks);
void myEEPROMwrite(int16_t addr, uint8_t data);
void CRC16_reset(void);
void CRC16_add(uint8_t c);
bool accessEEPROM(uint8_t dataType, bool write);
bool bindReadEeprom(void);
void bindWriteEeprom(void);
void bindInitDefaults(void);
#if (COMPILE_TX == 1)
void profileSet(void);
void profileInit(void);
void setDefaultProfile(uint8_t profile);
void txInitDefaults(void);
void bindRandomize(bool randomChannels);
void txWriteEeprom(void);
void txReadEeprom(void);
#else
void rxInitHWConfig(void);
void failsafeSave(void);
void failsafeLoad(void);
void rxInitDefaults(bool save);
void rxReadEeprom(void);
#endif


// Save EEPROM by writing just changed data
void myEEPROMwrite(int16_t addr, uint8_t data)
{
  uint8_t retries = 5;
  while ((--retries) && (data != eeprom_read_byte((uint8_t *)addr))) {
    eeprom_write_byte((uint8_t *)addr, data);
  }
  if (!retries) {
    fatalBlink(2);
  }
}

void CRC16_reset(void)
{
  CRC16_value = 0;
}

void CRC16_add(uint8_t c) // CCITT polynome
{
  uint8_t i;
  CRC16_value ^= (uint16_t)c << 8;
  for (i = 0; i < 8; i++) {
    if (CRC16_value & 0x8000) {
      CRC16_value = (CRC16_value << 1) ^ 0x1021;
    } else {
      CRC16_value = (CRC16_value << 1);
    }
  }
}

bool accessEEPROM(uint8_t dataType, bool write)
{
  void *dataAddress = NULL;
  uint16_t dataSize = 0;

  uint16_t addressNeedle = 0;
  uint16_t addressBase = 0;
  uint16_t CRC = 0;

  do {
start:
#if (COMPILE_TX == 1)
    if (dataType == 0) {
      dataAddress = &tx_config;
      dataSize = sizeof(tx_config);
      addressNeedle = (sizeof(tx_config) + sizeof(bind_data) + 4) * activeProfile;
    } else if (dataType == 1) {
      dataAddress = &bind_data;
      dataSize = sizeof(bind_data);
      addressNeedle = sizeof(tx_config) + 2;
      addressNeedle += (sizeof(tx_config) + sizeof(bind_data) + 4) * activeProfile;
    } else if (dataType == 2) {
      dataAddress = &defaultProfile;
      dataSize = 1;
      addressNeedle = (sizeof(tx_config) + sizeof(bind_data) + 4) * 4; // defaultProfile is stored behind all 4 profiles
    }
#else
    if (dataType == 0) {
      dataAddress = &rx_config;
      dataSize = sizeof(rx_config);
      addressNeedle = 0;
    } else if (dataType == 1) {
      dataAddress = &bind_data;
      dataSize = sizeof(bind_data);
      addressNeedle = sizeof(rx_config) + 2;
    } else if (dataType == 2) {
      dataAddress = &failsafePPM;
      dataSize = sizeof(failsafePPM);
      addressNeedle = sizeof(rx_config) + sizeof(bind_data) + 4;
    }
#endif
    addressNeedle += addressBase;
    CRC16_reset();

    for (uint8_t i = 0; i < dataSize; i++, addressNeedle++) {
      if (!write) {
        *((uint8_t*)dataAddress + i) = eeprom_read_byte((uint8_t *)(addressNeedle));
      } else {
        myEEPROMwrite(addressNeedle, *((uint8_t*)dataAddress + i));
      }

      CRC16_add(*((uint8_t*)dataAddress + i));
    }

    if (!write) {
      CRC = eeprom_read_byte((uint8_t *)addressNeedle) << 8 | eeprom_read_byte((uint8_t *)(addressNeedle + 1));

      if (CRC16_value == CRC) {
        // recover corrupted data
        // write operation is performed after every successful read operation, this will keep all cells valid
        write = true;
        addressBase = 0;
        goto start;
      } else {
        // try next block
      }
    } else {
      myEEPROMwrite(addressNeedle++, CRC16_value >> 8);
      myEEPROMwrite(addressNeedle, CRC16_value & 0x00FF);
    }
    addressBase += EEPROM_DATASIZE;
  } while (addressBase <= (EEPROM_SIZE - EEPROM_DATASIZE));
  return (write); // success on write, failure on read
}

bool bindReadEeprom(void)
{
  if (accessEEPROM(1, false) && (bind_data.version == BINDING_VERSION)) {
    return true;
  }
  return false;
}

void bindWriteEeprom(void)
{
  accessEEPROM(1, true);
}

void bindInitDefaults(void)
{
  bind_data.version = BINDING_VERSION;
  bind_data.serial_baudrate = DEFAULT_BAUDRATE;
  bind_data.rf_power = DEFAULT_RF_POWER;
  bind_data.rf_frequency = DEFAULT_CARRIER_FREQUENCY;
  bind_data.rf_channel_spacing = DEFAULT_CHANNEL_SPACING;

  bind_data.rf_magic = DEFAULT_RF_MAGIC;

  memset(bind_data.hopchannel, 0, sizeof(bind_data.hopchannel));
  memcpy(bind_data.hopchannel, default_hop_list, sizeof(default_hop_list));

  bind_data.modem_params = DEFAULT_DATARATE;
  bind_data.flags = DEFAULT_FLAGS;
}

#if (COMPILE_TX == 1)
void profileSet(void)
{
  accessEEPROM(2, true);
}

void profileInit(void)
{
  accessEEPROM(2, false);
  if (defaultProfile > TX_PROFILE_COUNT) {
    defaultProfile = 0;
    profileSet();
  }
  activeProfile = defaultProfile;
}

void setDefaultProfile(uint8_t profile)
{
  defaultProfile = profile;
  profileSet();
}

void txInitDefaults(void)
{
  tx_config.max_frequency = MAX_RFM_FREQUENCY;
  tx_config.console_baud_rate = DEFAULT_BAUDRATE;
  tx_config.flags = 0x00;
  TX_CONFIG_SETMINCH(5); // 6ch
  for (uint8_t i = 0; i < 16; i++) {
    tx_config.chmap[i] = i;
  }
}

void bindRandomize(bool randomChannels)
{
  uint8_t emergency_counter = 0;
  uint8_t c;
  uint32_t t = 0;
  while (t == 0) {
    t = micros();
  }
  srandom(t);

  bind_data.rf_magic = 0;
  for (c = 0; c < 4; c++) {
    bind_data.rf_magic = (bind_data.rf_magic << 8) + (random() % 255);
  }

  if (randomChannels) {
    // TODO: verify if this works properly
    for (c = 0; (c < MAXHOPS) && (bind_data.hopchannel[c] != 0); c++) {
again:
      if (emergency_counter++ == 255) {
        bindInitDefaults();
        return;
      }

      uint8_t ch = (random() % 50) + 1;

      // don't allow same channel twice
      for (uint8_t i = 0; i < c; i++) {
        if (bind_data.hopchannel[i] == ch) {
          goto again;
        }
      }

      // don't allow frequencies higher then tx_config.max_frequency
      uint32_t real_frequency = bind_data.rf_frequency + (uint32_t)ch * (uint32_t)bind_data.rf_channel_spacing * 10000UL;
      if (real_frequency > tx_config.max_frequency) {
        goto again;
      }

      bind_data.hopchannel[c] = ch;
    }
  }
}

void txWriteEeprom(void)
{
  accessEEPROM(0,true);
  accessEEPROM(1,true);
}

void txReadEeprom(void)
{
  if ((!accessEEPROM(0, false)) || (!accessEEPROM(1, false)) || (bind_data.version != BINDING_VERSION)) {
    txInitDefaults();
    bindInitDefaults();
    bindRandomize(true);
    txWriteEeprom();
  }
}
#endif

#if (COMPILE_TX != 1)
// following is only needed on receiver
void failsafeSave(void)
{
  accessEEPROM(2, true);
}

void failsafeLoad(void)
{
  if (!accessEEPROM(2, false)) {
    memset(failsafePPM, 0, sizeof(failsafePPM));
  }
}

void rxInitDefaults(bool save)
{
  rxInitHWConfig();
  rx_config.flags = ALWAYS_BIND;
  rx_config.RSSIpwm = 255; // off
  rx_config.failsafeDelay = 10; //1s
  rx_config.ppmStopDelay = 0;
  rx_config.pwmStopDelay = 0;
  rx_config.beacon_frequency = DEFAULT_BEACON_FREQUENCY;
  rx_config.beacon_deadtime = DEFAULT_BEACON_DEADTIME;
  rx_config.beacon_interval = DEFAULT_BEACON_INTERVAL;
  rx_config.minsync = 3000;

  if (save) {
    accessEEPROM(0, true);
    memset(failsafePPM, 0, sizeof(failsafePPM));
    failsafeSave();
  }
}

void rxReadEeprom(void)
{
  if (!accessEEPROM(0, false)) {
    rxInitDefaults(1);
  }
}

#endif

#endif
