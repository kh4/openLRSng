#ifndef _MULTI_H
#define _MULTI_H_

#define MULTI_HEADER 0x55
#define MULTI_PROTOCOL_OPENLRS 0x1B
#define MULTI_OPERATION_TIMEOUT_MS 5000

#define MULTI_TELEMETRY_FRSKY_HUB    0x03
#define MULTI_FLAGS_INPUT_DETECTED   0x01
#define MULTI_FLAGS_SERIAL_ENABLED   0X02
#define MULTI_FLAGS_PROTOCOL_VALID   0x04
#define MULTI_FLAGS_BINDING          0x08
#define MULTI_FLAGS_WAITING_FOR_BIND 0x08

extern uint8_t serialFrameIndex;
extern ppm_msg ppmWork;
extern volatile uint8_t ppmAge;

#ifdef DEBUG_DUMP_PPM
extern uint8_t ppmDump;
#endif

uint8_t multiWork[3];
bool multiBind = false;
bool multiAutoBind = false;
bool multiRangeCheck = false;
bool multiLowPower = false;
uint8_t multiProfile = 0;
int8_t multiOption = 0;
uint32_t multiLastProfileChange = 0;

// prototypes
bool newMultiProfileSelected(bool useTimeout);
uint16_t MULTItoPPM(uint16_t input);
void processMULTI(uint8_t c);
void checkMultiState(void);
uint8_t checkMultiPower(uint8_t power);
uint8_t multiBindActivated(void);
uint8_t getMultiProfile(void);
void sendMultiTelemetry(bool inputDetected, bool protocolValid, bool isBinding);

void sendMultiTelemetry(bool inputDetected, bool protocolValid, bool isBinding)
{
  rcSerial->write('M');
  rcSerial->write('P');
  rcSerial->write(MULTI_TELEMETRY_FRSKY_HUB);
  rcSerial->write(5);

  uint8_t flags = MULTI_FLAGS_SERIAL_ENABLED;
  if (inputDetected) {
    flags = flags | MULTI_FLAGS_INPUT_DETECTED;
  }
  if (protocolValid) {
    flags = flags | MULTI_FLAGS_PROTOCOL_VALID;
  }
  if (isBinding) {
    flags = flags | MULTI_FLAGS_BINDING;
  }
  rcSerial->write(flags);
  rcSerial->write(version >> 8);
  rcSerial->write((version >> 4) & 0x0f);
  rcSerial->write(version & 0x0f);
  rcSerial->write(0);
}

uint8_t checkMultiPower(uint8_t power)
{
  uint8_t p = power;

  if (multiOption >= 0) {
    p = multiOption & 0x07;
  }

  if (!multiLowPower) {
    p = 7;
  }

  if (multiRangeCheck) {
    if (p > 3) {
      p -= 3;
    } else {
      p= 0;
    }
  }
  return p;
}

bool newMultiProfileSelected(bool useTimeout)
{
  return ((multiProfile > 0) &&
          (multiProfile <= TX_PROFILE_COUNT) &&
          ((multiProfile - 1) != activeProfile) &&
          (!useTimeout ||
           ((millis() - multiLastProfileChange) >= MULTI_OPERATION_TIMEOUT_MS)));
}

uint16_t MULTItoPPM(uint16_t input)
{
  int32_t value = input;
  // Taken from
  // https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Multiprotocol/Multiprotocol.h#L478-L483
  value = (((value - 204) * 1000) / 1639) + 1000;
  return servoUs2Bits((uint16_t)value);
}

void processMULTI(uint8_t c)
{
  if (serialFrameIndex == 0) {
    if (c == MULTI_HEADER) {
      serialFrameIndex++;
    }
  } else if (serialFrameIndex == 1) {
    if ((c & 0x1F) == MULTI_PROTOCOL_OPENLRS) {
      multiWork[0] = c;
      serialFrameIndex++;
    } else {
      serialFrameIndex = 0;
    }
  } else if (serialFrameIndex <= 3) {
    multiWork[serialFrameIndex - 1] = c;
    serialFrameIndex++;
  } else if (serialFrameIndex <= 25) {
    ppmWork.bytes[serialFrameIndex - 4] = c;
    if (serialFrameIndex == 25) {
      uint32_t timestamp = millis();

      multiBind = multiWork[0] & 0x80;
      multiAutoBind = multiWork[0] & 0x40;
      multiRangeCheck = multiWork[0] & 0x20;

      uint8_t oldMultiProfile = multiProfile;
      multiProfile = multiWork[1] & 0x0F;

      if (multiProfile != oldMultiProfile) {
        multiLastProfileChange = timestamp;
      }

      multiLowPower = multiWork[1] & 0x80;
      multiOption = multiWork[2];

      for (uint8_t set = 0; set < 2; set++) {
        PPM[(set << 3)] = MULTItoPPM(ppmWork.sbus.ch[set].ch0);
        PPM[(set << 3) + 1] = MULTItoPPM(ppmWork.sbus.ch[set].ch1);
        PPM[(set << 3) + 2] = MULTItoPPM(ppmWork.sbus.ch[set].ch2);
        PPM[(set << 3) + 3] = MULTItoPPM(ppmWork.sbus.ch[set].ch3);
        PPM[(set << 3) + 4] = MULTItoPPM(ppmWork.sbus.ch[set].ch4);
        PPM[(set << 3) + 5] = MULTItoPPM(ppmWork.sbus.ch[set].ch5);
        PPM[(set << 3) + 6] = MULTItoPPM(ppmWork.sbus.ch[set].ch6);
        PPM[(set << 3) + 7] = MULTItoPPM(ppmWork.sbus.ch[set].ch7);
      }
#ifdef DEBUG_DUMP_PPM
      ppmDump = 1;
#endif
      ppmAge = 0;
      serialFrameIndex = 0;
    } else {
      serialFrameIndex++;
    }
  } else {
    sendMultiTelemetry(true, false, false);
    serialFrameIndex = 0;
  }
}

uint8_t getMultiProfile(void)
{
  return multiProfile;
}

uint8_t multiBindActivated(void)
{
  return multiBind;
}

#endif
