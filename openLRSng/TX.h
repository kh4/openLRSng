#ifndef _TX_H_
#define _TX_H_

uint8_t RF_channel = 0;
bool bndMode = true;

uint8_t failsafeState = 0; // 1 = waiting timer, 2 = send FS, 3 sent waiting btn release
uint32_t failsafeTime = 0;  // loopTime when button went down...

uint32_t hopInterval = 0;
uint32_t loopTime = 0;
uint32_t lastSent = 0;
uint32_t lastTelemetry = 0;
uint32_t lastSerialPPM = 0;

uint8_t RSSI_rx = 0;
uint8_t RSSI_tx = 0;
uint8_t RX_ain0 = 0;
uint8_t RX_ain1 = 0;
uint32_t sampleRSSI = 0;

uint16_t linkQualityTX = 0;
uint16_t linkQualityRX = 0;

uint8_t tx_buf[UPLINK_MAX_PACKETSIZE];
uint8_t rx_buf[DOWNLINK_PACKETSIZE];

volatile uint8_t ppmAge = 255; // age of PPM data
volatile uint8_t ppmCounter = 255; // ignore data until first sync pulse
bool ppmIsEnabled = false;

uint8_t serial_buffer[SERIAL_BUFSIZE];
uint8_t serial_resend[9];
uint8_t serial_head = 0;
uint8_t serial_tail = 0;
uint8_t serialSendState = SERIAL_WAIT0; // 2 if it is ok to send serial instead of servo

uint8_t serialFrameIndex = 0;
uint32_t srxLast = 0;
uint8_t srxFlags = 0;
uint8_t srxChannels = 0;

serialMode_e serialMode = SERIAL_MODE_NONE;
ppm_msg ppmWork;

#ifdef DEBUG_DUMP_PPM
uint8_t ppmDump   = 0;
uint32_t lastDump = 0;
#endif


// ########## prototypes

void loop(void);
void setup(void);
void hardwareInit(void);
void loopStateMachine(void);
void checkTxPacketSent(void);
void handlePacketTelem(void);
void handlePacketTX(void);

void bindMode(void);
void checkBND(void);
void checkButton(void);
void checkFS(void);
void doBeeps(uint8_t numBeeps);
void configureProfile(void);
void setupProfile(void);

void checkTelemLinkState(void);
void checkFrSkyTelem(void);
void checkRSSI(void);
uint8_t compositeRSSI(uint8_t rssi, uint8_t linkq);

void checkSetupPPM(void);
void setupInputPPM(void);
void processPulsePPM(uint16_t pulse);

void checkSerial(void);
void checkMultiState(void);
void setupSerialRC(void);
void processSBUS(uint8_t c);
void processSpektrum(uint8_t c);
void processSUMD(uint8_t c);
void processSerial(void);
void processChannelsFromSerial(uint8_t c);
uint16_t SBUStoPPM(uint16_t input);
uint16_t getChannel(uint8_t ch);

size_t debugPrint(const char str[]);
void consoleFlush(void);
int consoleDataAvailable(void);
int consoleRead(void);
size_t consolePrint(const char* str);
#ifdef USE_CONSOLE_SERIAL
Serial_ *getConsoleSerial(void);
#else
HardwareSerial *getConsoleSerial(void);
#endif


// ####### core setup & data loop routines

void setup(void)
{
  hardwareInit();
  setupProfile();
  txReadEeprom();

#ifdef __AVR_ATmega32U4__
  Serial.begin(0); // Suppress warning on overflow on Leonardo
#else
  Serial.begin(tx_config.console_baud_rate);
#endif

  buzzerOn(BZ_FREQ);
  Red_LED_ON ;

  consoleFlush();
  consolePrint("OpenLRSng TX starting ");
  printVersion(version, getConsoleSerial());
  consolePrint(" on HW " XSTR(BOARD_TYPE) "\n");
  delay(100);

  checkSetupPPM();
  checkBND();
  checkButton();
  Red_LED_OFF;
  buzzerOff();
  setupSerialRC();

  uint32_t timeout = millis() + (serialMode == SERIAL_MODE_MULTI ? 20000 : 2000);

  while (ppmAge == 255 && millis() < timeout) {
    processSerial();
  }

  if (newMultiProfileSelected(false)) {
    activeProfile = (multiProfile - 1);
  }

  configureProfile();
  lastPktTimeTX = micros();
  lastPktTimeRX = micros();
}

void loop(void)
{
#ifdef DEBUG_DUMP_PPM
  if (ppmDump) {
    watchdogReset();
    dumpPPM();
  }
#endif
  watchdogReset();
  checkModule();
  checkSerial();
  checkFS();
  loopStateMachine();
}

void hardwareInit(void)
{
#ifdef __AVR_ATmega32U4__
  // mimic standard bootloader start-up delay
  // necessary for Configurator connection stability
  delay(850);
#endif

  watchdogConfig(WATCHDOG_OFF);
  setupSPI();

#ifdef SDN_pin
  pinMode(SDN_pin, OUTPUT); //SDN
  digitalWrite(SDN_pin, 0);
#endif

  //LED and other interfaces
  pinMode(Red_LED, OUTPUT); //RED LED
  pinMode(Green_LED, OUTPUT); //GREEN LED

#ifdef Red_LED2
  pinMode(Red_LED2, OUTPUT); //RED LED
  pinMode(Green_LED2, OUTPUT); //GREEN LED
#endif

  pinMode(BTN, INPUT); //Button
  digitalWrite(BTN, HIGH);

#ifdef TX_MODE1
  pinMode(TX_MODE1, INPUT);
  digitalWrite(TX_MODE1, HIGH);
#endif

#ifdef TX_MODE2
  pinMode(TX_MODE2, INPUT);
  digitalWrite(TX_MODE2, HIGH);
#endif

  pinMode(PPM_IN, INPUT); //PPM from TX
  digitalWrite(PPM_IN, HIGH); // enable pullup for TX:s with open collector output

#if defined (RF_OUT_INDICATOR)
  pinMode(RF_OUT_INDICATOR, OUTPUT);
  digitalWrite(RF_OUT_INDICATOR, LOW);
#endif

  buzzerInit();
  cli();
  setupRfmInterrupt();
  sei();
}

void loopStateMachine(void)
{
  static uint8_t state = 0;

  loopTime = micros();
  switch (state++) {
  case 0:
    handlePacketTelem();
    break;
  case 1:
    checkRSSI();
    break;
  case 2:
    handlePacketTX();
    break;
  case 3:
    checkFrSkyTelem();
    break;
  case 4:
    checkTxPacketSent();
    break;
  case 5:
    checkMultiState();
    state = 0;
    break;
  }
}

void handlePacketTX(void)
{
  if ((loopTime - lastSent) < hopInterval) {
    return;
  }

  lastSent = loopTime;
  //lastSent = lastPktTimeTX;

  watchdogReset();

  if (TX_CONFIG_GETMINCH() && (ppmAge > 8)) {
    if (ppmAge == 8) {
      Red_LED_ON;
    }
    ppmAge = 9;
    // PPM data outdated - do not send packets
    return;
  }

  uint8_t power;
  ppmAge++;

  checkTelemLinkState();

  // Construct packet to be sent
  tx_buf[0] &= 0xC0; //preserve seq. bits

  if ((serial_tail != serial_head) && (serialSendState == SERIAL_SEND)) {
    tx_buf[0] ^= UPLINK_SEQBIT; // signal new data on line
    uint8_t bytes = 0;
    uint8_t maxbytes = 8;

    if (getPacketSize(&bind_data) < 9) {
      maxbytes = getPacketSize(&bind_data) - 1;
    }

    while ((bytes < maxbytes) && (serial_head != serial_tail)) {
      tx_buf[++bytes] = serial_buffer[serial_head];
      serial_resend[bytes] = serial_buffer[serial_head];
      serial_head = (serial_head + 1) % SERIAL_BUFSIZE;
    }

    tx_buf[0] |= (0x37 + bytes);
    serial_resend[0] = bytes;
    serialSendState = SERIAL_ACK; // sent but not acked

  } else if (serialSendState == SERIAL_RESEND) {
    for (uint8_t i = 0; i < serial_resend[0]; i++) {
      tx_buf[i + 1] = serial_resend[i + 1];
    }
    tx_buf[0] |= (0x37 + serial_resend[0]);
    serialSendState = SERIAL_ACK; // sent but not acked

  } else {
    uint16_t PPMout[16];
    if (failsafeState == 2) {
      tx_buf[0] |= 0x01; // save failsafe
      Red_LED_ON
    } else {
      tx_buf[0] |= 0x00; // servo positions
      Red_LED_OFF
      if (serialSendState == SERIAL_WAIT0) {
        serialSendState = SERIAL_WAIT1;
      }
      if (serialSendState == SERIAL_ACK) {
        serialSendState = SERIAL_RESEND;  // resend
      }
    }
    for (uint8_t i = 0; i < PPM_CHANNELS; i++) {
      PPMout[i] = getChannel(i);
    }
    packChannels((bind_data.flags & 0x07), PPMout, (tx_buf + 1));
  }

  //Green LED will be on during transmission
  Green_LED_ON;
  power = bind_data.rf_power;

#ifdef TX_MODE1
  if (tx_config.flags & SW_POWER) {
    if (!digitalRead(TX_MODE1)) {
      power = 7;
    }
  }
#endif

  if (serialMode == SERIAL_MODE_MULTI) {
    power = checkMultiPower(power);
  }

  rfmSetPower(power);

  //Hop to the next frequency
  RF_channel++;
  if ((RF_channel == MAXHOPS) || (bind_data.hopchannel[RF_channel] == 0)) {
    RF_channel = 0;
  }
  setHopChannel(RF_channel);

  // Send the data over RF
  tx_packet_async(tx_buf, getPacketSize(&bind_data));
  Green_LED_OFF;
}

void handlePacketTelem(void)
{
  if (RF_Mode != RECEIVED) {
    return;
  }

  // got telemetry packet
  lastTelemetry = lastPktTimeRX;
  linkQualityTX |= 1;

  rfmGetPacket(rx_buf, DOWNLINK_PACKETSIZE);

  if ((tx_buf[0] ^ rx_buf[0]) & DOWNLINK_SEQBIT) {
    // swap sequence to ACK
    // received data from RX
    tx_buf[0] ^= DOWNLINK_SEQBIT;

    if ((rx_buf[0] & 0x38) == 0x38) {
      // transparent serial data...
      for (uint8_t i = 0; i <= (rx_buf[0] & 0x07);) {
        if (bind_data.flags & TELEMETRY_FRSKY) {
          frskyUserData(rx_buf[++i]);
          if (serialMode == SERIAL_MODE_MULTI) {
            sendMultiTelemetry(true, true, false);
          }
        } else {
          rcSerial->write(rx_buf[++i]);
        }
      }
    } else if ((rx_buf[0] & 0x3F) == 0) {
      RSSI_rx = rx_buf[1];
      RX_ain0 = rx_buf[2];
      RX_ain1 = rx_buf[3];
      linkQualityRX = rx_buf[6];
    }
  }

  if (serialSendState == SERIAL_WAIT1) {
    serialSendState = SERIAL_SEND;
  } else if (serialSendState == SERIAL_ACK) {
    serialSendState = SERIAL_WAIT0;
  }
}

void checkTelemLinkState(void)
{
  if (!lastTelemetry) {
    return;
  }

  if ((loopTime - lastTelemetry) > hopInterval) {
    // telemetry lost
    if (!(tx_config.flags & MUTE_TX)) {
      buzzerOn(BZ_FREQ);
    }
    lastTelemetry = 0;
  } else {
    // telemetry link re-established
    buzzerOff();
  }
}

void checkFrSkyTelem(void)
{
  if (lastTelemetry && (bind_data.flags & TELEMETRY_FRSKY)) {
    uint8_t linkQualityTX = countSetBits(linkQualityTX & 0xFFfe);
    uint8_t compRX = compositeRSSI(RSSI_rx, linkQualityRX);
    uint8_t compTX = compositeRSSI(RSSI_tx, linkQualityTX);

    frskyUpdate(RX_ain0, RX_ain1, compRX, compTX, activeProfile);
    //frskyUpdate(RX_ain0,RX_ain1,lastTelemetry?RSSI_rx:0,lastTelemetry?RSSI_tx:0);
  }
}

void checkTxPacketSent(void)
{
  if ((tx_done() == 1) && (bind_data.flags & TELEMETRY_MASK)) {
    linkQualityTX <<= 1;
    rx_reset();

    // tell loop to sample down-link RSSI
    sampleRSSI = micros() | 1;
  }
}

void checkRSSI(void)
{
  if ((sampleRSSI) && ((loopTime - sampleRSSI) >= 3000)) {
    RSSI_tx = rfmGetRSSI();
    sampleRSSI = 0;
  }
}

uint8_t compositeRSSI(uint8_t rssi, uint8_t linkq)
{
  if (linkq >= 15) {
    // RSSI 0 - 255 mapped to 128 - ((255>>2)+192) == 128-255
    return (rssi >> 1) + 128;
  } else {
    // linkQualityTX gives 0 to 14*0 == 126
    return linkq * 9;
  }
}


// ####### PPM routines

void checkSetupPPM(void)
{
  if (serialMode == SERIAL_MODE_NONE && !ppmIsEnabled) {
    setupInputPPM();
    ppmIsEnabled = true;
  }
}

void processPulsePPM(uint16_t pulse)
{
  if (serialMode != SERIAL_MODE_NONE) {
    return;
  }

  if (!(tx_config.flags & MICROPPM)) {
    pulse >>= 1; // divide by 2 to get servo value on normal PPM
  }

  if (pulse > 2500) {      // Verify if this is the sync pulse (2.5ms)
    if ((ppmCounter>(TX_CONFIG_GETMINCH()?TX_CONFIG_GETMINCH():1)) && (ppmCounter!=255)) {
      uint8_t i;
      for (i=0; i < ppmCounter; i++) {
        PPM[i] = ppmWork.words[i];
      }
      ppmAge = 0;                 // brand new PPM data received
#ifdef DEBUG_DUMP_PPM
      ppmDump = 1;
#endif
    }
    ppmCounter = 0;             // -> restart the channel counter
  } else if ((pulse > 700) && (ppmCounter < PPM_CHANNELS)) { // extra channels will get ignored here
    ppmWork.words[ppmCounter++] = servoUs2Bits(pulse);   // Store measured pulse length (converted)
  } else {
    ppmCounter = 255; // glitch ignore rest of data
  }
}

#ifdef USE_ICP1 // Use ICP1 in input capture mode
volatile uint16_t startPulse = 0;
ISR(TIMER1_CAPT_vect)
{
  uint16_t stopPulse = ICR1;
  processPulsePPM(stopPulse - startPulse); // as top is 65535 uint16 math will take care of rollover
  startPulse = stopPulse;         // Save time at pulse start
}

void setupInputPPM(void)
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 <<ICNC1));
  // normally capture on rising edge, allow invertting via SW flag
  if (!(tx_config.flags & INVERTED_PPMIN)) {
    TCCR1B |= (1 << ICES1);
  }
  OCR1A = 65535;
  TIMSK1 |= (1 << ICIE1);   // Enable timer1 input capture interrupt
}

#else // sample PPM using pin change interrupt
ISR(PPM_Signal_Interrupt)
{
  uint16_t pulseWidth;
  if ( (tx_config.flags & INVERTED_PPMIN) ^ PPM_Signal_Edge_Check) {
    pulseWidth = TCNT1; // read the timer1 value
    TCNT1 = 0; // reset the timer1 value for next
    processPulsePPM(pulseWidth);
  }
}

void setupInputPPM(void)
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11));
  OCR1A = 65535;
  TIMSK1 = 0;
  PPM_Pin_Interrupt_Setup
}
#endif


// ####### bind / button / profile routines

void bindMode(void)
{
  uint32_t prevsend = millis();
  uint8_t  tx_buf[sizeof(bind_data) + 1];
  bool  sendBinds = 1;

  tx_buf[0] = 'b';
  memcpy((tx_buf + 1), &bind_data, sizeof(bind_data));

  init_rfm(1);

  consoleFlush();

  Red_LED_OFF;

  while (bndMode || !(serialMode == SERIAL_MODE_MULTI && !multiBind)) {
    if (sendBinds & (millis() - prevsend > 200)) {
      prevsend = millis();
      Green_LED_ON;
      buzzerOn(BZ_FREQ);
      tx_packet(tx_buf, (sizeof(bind_data) + 1));
      Green_LED_OFF;
      buzzerOff();
      rx_reset();
      delay(50);
      if (RF_Mode == RECEIVED) {
        rfmGetPacket(tx_buf, 1);
        if (tx_buf[0] == 'B') {
          sendBinds = 0;
        }
      }
    }

    if (!digitalRead(BTN)) {
      sendBinds = 1;
    }

    while (consoleDataAvailable()) {
      Red_LED_ON;
      Green_LED_ON;
      switch (consoleRead()) {
#ifdef CLI
      case '\n':
      case '\r':
#ifdef CLI_ENABLED
        consolePrint("Enter menu...\n");
        handleCLI();
#else
        consolePrint("CLI not available, use configurator!\n");
#endif
        break;
#endif
      case '#':
        scannerMode();
        break;
#ifdef CONFIGURATOR
      case 'B':
        binaryMode();
        break;
#endif
      default:
        break;
      }
      Red_LED_OFF;
      Green_LED_OFF;
    }

    if (!bndMode) {
      processSerial();
      if (serialMode == SERIAL_MODE_MULTI) {
        sendMultiTelemetry(true, true, true);
      }
    }

    watchdogReset();
  }
}

void checkButton(void)
{
  uint32_t time, loop_time;

  if (digitalRead(BTN) == 0) {     // Check the button
    delay(200);   // wait for 200mS with buzzer ON
    buzzerOff();

    time = millis();  //set the current time
    loop_time = time;

    while (millis() < (time + 4800)) {
      if (digitalRead(BTN)) {
        goto just_bind;
      }
    }

    // Check the button again, If it is still down reinitialize
    if (0 == digitalRead(BTN)) {
      int8_t bzstate = HIGH;
      uint8_t swapProfile = 0;

      buzzerOn(bzstate?BZ_FREQ:0);
      loop_time = millis();

      while (0 == digitalRead(BTN)) {     // wait for button to release
        if (loop_time > (time + 9800)) {
          buzzerOn(BZ_FREQ);
          swapProfile = 1;
        } else {
          if ((millis() - loop_time) > 200) {
            loop_time = millis();
            bzstate = !bzstate;
            buzzerOn(bzstate ? BZ_FREQ : 0);
          }
        }
      }

      buzzerOff();
      if (swapProfile) {
        setDefaultProfile((defaultProfile + 1) % (TX_PROFILE_COUNT+1));
        setupProfile();
        txReadEeprom();
        delay(500);
        return;
      }
      bindRandomize(false);
      chooseChannelsPerRSSI();
      txWriteEeprom();
    }
just_bind:
    // Enter binding mode, automatically after recoding or when pressed for shorter loopTime.
    bindMode();
  }
}

void checkBND(void)
{
  if ((consoleDataAvailable() > 3) &&
      (consoleRead() == 'B') && (consoleRead() == 'N') &&
      (consoleRead() == 'D') && (consoleRead() == '!')) {
    buzzerOff();
    bindMode();
  } else {
    bndMode = false;
  }
}

void checkFS(void)
{

  switch (failsafeState) {
  case 0:
    if (!digitalRead(BTN)) {
      failsafeState = 1;
      failsafeTime = millis();
    }

    break;

  case 1:
    if (!digitalRead(BTN)) {
      if ((millis() - failsafeTime) > 1000) {
        failsafeState = 2;
        buzzerOn(BZ_FREQ);
      }
    } else {
      failsafeState = 0;
    }

    break;

  case 2:
    if (digitalRead(BTN)) {
      buzzerOff();
      failsafeState = 0;
    }

    break;
  }
}

void doBeeps(uint8_t numBeeps)
{
  for (uint8_t i = 0; i <= numBeeps; i++) {
    delay(50);
    buzzerOn(BZ_FREQ);
    delay(50);
    buzzerOff();
  }
}

void setupProfile(void)
{
  profileInit();
  if (activeProfile==TX_PROFILE_COUNT) {
#if defined(TX_MODE2)
    switch ((digitalRead(TX_MODE1)?1:0) | (digitalRead(TX_MODE2)?2:0)) {
    case 2:
      activeProfile = 0; // MODE1 grounded
      break;
    case 1:
      activeProfile = 1; // MODE2 grounded
      break;
    case 3:
      activeProfile = 2; // both high
      break;
    case 0:
      activeProfile = 3; // both ground
      break;
    }
#elif defined(TX_MODE1)
    activeProfile = digitalRead(TX_MODE1) ? 0 : 1;
#else
    activeProfile = 0;
#endif
  }
}

void configureProfile(void)
{
  txReadEeprom();

  doBeeps(activeProfile);

  setupSerialRC();

  checkSetupPPM();

  if ((bind_data.flags & TELEMETRY_MASK) && (bind_data.flags & TELEMETRY_FRSKY)) {
    frskyInit(rcSerial, (bind_data.flags & TELEMETRY_MASK) == TELEMETRY_SMARTPORT, serialMode != SERIAL_MODE_NONE);
  }

  hopInterval = getInterval(&bind_data);

  init_rfm(0);
  setHopChannel(RF_channel);
  rx_reset();

  watchdogConfig(WATCHDOG_2S);
}

// ####### serial-related routines

void setupSerialRC(void)
{
  if (bind_data.serial_baudrate && (bind_data.serial_baudrate <= SERIAL_MODE_MAX)) {
    serialMode = (serialMode_e)bind_data.serial_baudrate;
  } else {
    serialMode = SERIAL_MODE_NONE;
  }

  if (serialMode == SERIAL_MODE_SBUS || serialMode == SERIAL_MODE_MULTI) {
    rcSerial->begin(100000, SERIAL_8E2);
  } else if (serialMode == SERIAL_MODE_SPEKTRUM1024 || serialMode == SERIAL_MODE_SPEKTRUM2048 || serialMode == SERIAL_MODE_SUMD) {
    rcSerial->begin(115200);
  } else { // SERIAL_MODE_NONE
    // switch to user-defined baudrate here
    rcSerial->begin(bind_data.serial_baudrate);
  }
}

void processSerial(void)
{
  if (serialMode != SERIAL_MODE_NONE) {
    while (rcSerial->available()) {
      processChannelsFromSerial(rcSerial->read());
    }
  }
}

void checkSerial(void)
{
  while (rcSerial->available()) {
    uint8_t ch = rcSerial->read();
    if (serialMode != SERIAL_MODE_NONE) {
      processChannelsFromSerial(ch);
    } else if (((serial_tail + 1) % SERIAL_BUFSIZE) != serial_head) {
      serial_buffer[serial_tail] = ch;
      serial_tail = (serial_tail + 1) % SERIAL_BUFSIZE;
    }
  }
#ifdef __AVR_ATmega32U4__
  if (serialMode != SERIAL_MODE_NONE) {
    while (Serial.available()) {
      processChannelsFromSerial(Serial.read());
    }
  }
#endif
}

void processSBUS(uint8_t c)
{
  if (serialFrameIndex == 0) {
    if ((c == SBUS_SYNC) && ((millis() - lastSerialPPM) > 1)) { // prevent locking onto wrong byte in frame
      serialFrameIndex++;
    }
  } else if (serialFrameIndex < 24) {
    ppmWork.bytes[(serialFrameIndex++)-1] = c;
  } else {
    if ((serialFrameIndex == 24) && (c == SBUS_TAIL)) {
      for (uint8_t set = 0; set < 2; set++) {
        PPM[(set<<3)] = SBUStoPPM(ppmWork.sbus.ch[set].ch0);
        PPM[(set<<3)+1] = SBUStoPPM(ppmWork.sbus.ch[set].ch1);
        PPM[(set<<3)+2] = SBUStoPPM(ppmWork.sbus.ch[set].ch2);
        PPM[(set<<3)+3] = SBUStoPPM(ppmWork.sbus.ch[set].ch3);
        PPM[(set<<3)+4] = SBUStoPPM(ppmWork.sbus.ch[set].ch4);
        PPM[(set<<3)+5] = SBUStoPPM(ppmWork.sbus.ch[set].ch5);
        PPM[(set<<3)+6] = SBUStoPPM(ppmWork.sbus.ch[set].ch6);
        PPM[(set<<3)+7] = SBUStoPPM(ppmWork.sbus.ch[set].ch7);
      }
      if ((ppmWork.sbus.status & 0x08)==0) {
#ifdef DEBUG_DUMP_PPM
        ppmDump = 1;
#endif
        ppmAge = 0;
      }
    }
    serialFrameIndex = 0;
  }
  lastSerialPPM = millis();
}

void processSpektrum(uint8_t c)
{
  if (serialFrameIndex == 0) {
    serialFrameIndex++;
  } else if (serialFrameIndex == 1) {
    serialFrameIndex++;
  } else if (serialFrameIndex < 16) {
    ppmWork.bytes[serialFrameIndex++] = c;
    if (serialFrameIndex==16) { // frameComplete
      for (uint8_t i=1; i<8; i++) {
        uint8_t ch,v;
        if (serialMode == SERIAL_MODE_SPEKTRUM1024) {
          ch = ppmWork.words[i] >> 10;
          v = ppmWork.words[i] & 0x3FF;
        } else {
          ch = ppmWork.words[i] >> 11;
          v = (ppmWork.words[i] & 0x7FF)>>1;
        }
        if (ch < 16) {
          PPM[ch] = v;
        }

#ifdef DEBUG_DUMP_PPM
        ppmDump = 1;
#endif
        ppmAge = 0;
      }
    }
  } else {
    serialFrameIndex = 0;
  }
}

void processSUMD(uint8_t c)
{
  if ((serialFrameIndex == 0) && (c == SUMD_HEAD)) {
    CRC16_reset();
    CRC16_add(c);
    serialFrameIndex=1;
  } else {
    if (serialFrameIndex == 1) {
      srxFlags = c;
      CRC16_add(c);
    } else if (serialFrameIndex == 2) {
      srxChannels = c;
      CRC16_add(c);
    } else if (serialFrameIndex < (3 + (srxChannels << 1))) {
      if (serialFrameIndex < 35) {
        ppmWork.bytes[serialFrameIndex-3] = c;
      }
      CRC16_add(c);
    } else if (serialFrameIndex == (3 + (srxChannels << 1))) {
      CRC16_value ^= (uint16_t)c << 8;
    } else {
      if ((CRC16_value == c) && (srxFlags == 0x01)) {
        if (srxChannels > 16) {
          srxChannels = 16;
        }
        for (uint8_t ch = 0; ch < srxChannels; ch++) {
          uint16_t val = (uint16_t)ppmWork.bytes[ch*2]<<8 | (uint16_t)ppmWork.bytes[ch*2+1];
          PPM[ch] = servoUs2Bits(val >> 3);
        }
#ifdef DEBUG_DUMP_PPM
        ppmDump = 1;
#endif
        ppmAge = 0;
      }
      serialFrameIndex = 0;
    }
    if (serialFrameIndex > 0) {
      serialFrameIndex++;
    }
  }
}

void processChannelsFromSerial(uint8_t c)
{
  uint32_t now = micros();
  if ((now - srxLast) > 5000) {
    serialFrameIndex = 0;
  }
  srxLast = now;

  if (serialMode == SERIAL_MODE_SPEKTRUM1024 || serialMode == SERIAL_MODE_SPEKTRUM2048) {
    processSpektrum(c);
  } else if (serialMode == SERIAL_MODE_SBUS) {
    processSBUS(c);
  } else if (serialMode == SERIAL_MODE_SUMD) {
    processSUMD(c);
  } else if (serialMode == SERIAL_MODE_MULTI) {
    processMULTI(c);
  }
}

uint16_t SBUStoPPM(uint16_t input)
{
  uint16_t value = ((input * 5) >> 3) + 880;
  return servoUs2Bits(value);
}

uint16_t getChannel(uint8_t ch)
{
  uint16_t v=512;
  ch = tx_config.chmap[ch];
  if (ch < 16) {
    if (serialMode == SERIAL_MODE_NONE) {
      // disable interrupts when copying servo positions,
      // to avoid race on 2 byte variable written by ISR
      cli();
      v = PPM[ch];
      sei();
    } else {
      v = PPM[ch];
    }
  } else if ((ch > 0xF1) && (ch < 0xFD)) {
    v = 12 + (ch - 0xF2) * 100;
  } else {
    switch (ch) {
#ifdef TX_AIN0
#ifdef TX_AIN_IS_DIGITAL
    case 16:
      v = digitalRead(TX_AIN0) ? 1012 : 12;
      break;
    case 17:
      v = digitalRead(TX_AIN1) ? 1012 : 12;
      break;
#else
    case 16:
      v = analogRead(TX_AIN0);
      break;
    case 17:
      v = analogRead(TX_AIN1);
      break;
#endif
#endif
    case 18: // mode switch
#if defined(TX_MODE2)
      switch ((digitalRead(TX_MODE1)?1:0) | (digitalRead(TX_MODE2)?2:0)) {
      case 2:
        v = 12;
        break;
      case 1:
        v =  1012;
        break;
      case 3:
        v =  345;
        break;
      case 0:
        v = 678;
        break;
      }
#elif defined(TX_MODE1)
      v = (digitalRead(TX_MODE1) ? 12 : 1012);
#endif
      break;
    case 0xF0:
      v = 0;
      break;
    case 0xF1:
      v = 6;
      break;
    case 0xFD:
      v = 1018;
      break;
    case 0xFE:
      v = 1023;
      break;
    }
  }
  return v;
}

void checkMultiState(void)
{
  // make sure that serial error does not accidentally
  // cause the TX to drop the link while linked to an RX
  if (serialMode == SERIAL_MODE_MULTI &&
      (lastPktTimeRX == 0 || ((micros() - lastPktTimeRX) >= MULTI_OPERATION_TIMEOUT_MS))) {
    if (newMultiProfileSelected(true)) {
      activeProfile = (getMultiProfile() - 1);
      configureProfile();
    }

    if (multiBindActivated()) {
      bindMode();
      init_rfm(0);
      rx_reset();
    }
  }
}

// ####### serial console / debug routines

size_t debugPrint(const char str[])
{
  size_t result = 0;
  if (!(bind_data.flags & TELEMETRY_MASK)) {
    result = rcSerial->print(str);
  } else {
#ifdef USE_CONSOLE_SERIAL
    result = consoleSerial->print(str);
#endif
  }
  return result;
}

void consoleFlush(void)
{
#ifdef USE_CONSOLE_SERIAL
  while (consoleSerial->available()) {
    consoleSerial->read();
  }
#else
  if (bndMode || serialMode == SERIAL_MODE_NONE) {
    while (rcSerial->available()) {
      rcSerial->read();
    }
  }
#endif
}

int consoleDataAvailable(void)
{
  int result = 0;
#ifdef USE_CONSOLE_SERIAL
  result = consoleSerial->available();
#else
  if (bndMode || serialMode == SERIAL_MODE_NONE) {
    result = rcSerial->available();
  }
#endif
  return result;
}

int consoleRead(void)
{
  int result = -1;
#ifdef USE_CONSOLE_SERIAL
  result = consoleSerial->read();
#else
  if (bndMode || serialMode == SERIAL_MODE_NONE) {
    result = rcSerial->read();
  }
#endif
  return result;
}

size_t consolePrint(const char* str)
{
  size_t result = 0;
#ifdef USE_CONSOLE_SERIAL
  result = consoleSerial->print(str);
#else
  if (bndMode || !(bind_data.flags & TELEMETRY_MASK)) {
    result = rcSerial->print(str);
  }
#endif
  return result;
}

#ifdef USE_CONSOLE_SERIAL
Serial_ *getConsoleSerial(void)
{
  return consoleSerial;
}
#else
HardwareSerial *getConsoleSerial(void)
{
  HardwareSerial *result = NULL;
  if (bndMode || !(bind_data.flags & TELEMETRY_MASK)) {
    result = rcSerial;
  }
  return result;
}
#endif

#ifdef DEBUG_DUMP_PPM
void dumpPPM(void)
{
  uint32_t timeTMP = millis();
  debugPrint((timeTMP - lastDump));
  lastDump = timeTMP;
  debugPrint(":");
  for (uint8_t i = 0; i < 16; i++) {
    debugPrint(PPM[i]);
    debugPrint(",");
  }
  debugPrint("\n");
  ppmDump = 0;
}
#endif

#endif
