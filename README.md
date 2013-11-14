openLRSng (with extended telemetry support)
===========================================
	This is a fork of openLRSng code by kah (which in turn is based on thUndeadMod of openLRS).
	The project is geared towards the ArduPilot platform.
	This software enables you to use your openLRS hardware compatible RC receiver and transmitter as a telemetry module. The 3DR telemetry module becomes redundant.
	The telemetry link is pretty much the same as the one found in original openLRSng project except for:
		- Variable (selectable through CLI) downlink telemetry packet size.
		- Injection of mavlink radio modem status packets. Lets ArduPilot adjust packet send rate automatically in order to avoid lost packets due to overload.
		
	All features from openLRSng v3.2.1 are still supported, except for:
		- FRSKY emulation (will not work).
		
	A note to those wanting to use a joystick to control the mav: It could possible be done, but the focus on this project

Useful links:
	http://diydrones.com/forum/topics/telemetry-openlrs-and-apm?id=705844%3ATopic%3A711570&page=3#comments


USING THE SOFTWARE WITH MISSION PLANNER
=======================================

	Connect the telemetry port to the OpenLRS Rx.
		- The wires that need connection between the ardupilot and the Rx is the following:
			APM TX -> OpenLRS RX
			APM RX -> OpenLRS TX
			APM GND -> OpenLRS GND
			Do NOT connect the 5V wire to the OpenLRS Rx since this may damage the 3.3V radio module. So you need a total of three wires.
			
			

	Compile and upload the code to your Rx and Tx module.
		- Set correct sketchbook location in arduino IDE: Should point to the folder containing the openLRSng.ino file.
		- Follow the directions under section 'Upload'.

	Configure your TX via CLI (done in the same fashion as with original openLRSng):
		- Set telemetry baud rate (Default for ArduPilot is 57600)
		- Set RFM module datarate, recommended to be set to 2 (19200), inorder to be able to have decent update frequency (packet interval) and telemetry data size.
		- Set wanted telemetry packet size:
		  Value of this param will vary depending on your application.
		  Higher values will reduce your update frequency. Also since packets sent from Rx (downlink) will be
		  larger the risk of introducing bit errors within the packets will increase, which can decrease downlink quality and range.
		  However a high value will increase the number of bytes possible to transfer per second, which can be useful for some applications when you 
		  want good response on the artificial horizon etc.
		  There is currently no changes made to the uplink (Tx to Rx) communication, so the quality of the RC control will remain unaffected,
		  except for packet interval (update frequency).
		  Note that when update frequency is decreased it will also affect the link speed negatively. So this parameter needs some tweaking in-order to get good performance.
		  As a guideline you should stay within a value of 6-30 (maximum allowed is 64).
		  
	Configure Rx:
		- Bind to your Rx as usual, and configure wanted parameters.
		- Note about RSSI: the rssi is currently not sent from the Rx to Tx. This is because ardupilot can send the RSSI information within the mavlink protocol.
		  To be able to see your Rx RSSI you must connect the receiver RSSI pin to your configured RSSI analog input
		  on the ardupilot and set this up in mission planner, see: http://plane.ardupilot.com/wiki/arduplane-parameters/#Receiver-RSSI-sensing-pin-(ArduPlane-RSSI_PIN)
	
	Configure Mission Planner		
		- Set correct baud rate for telemetry port on your APM (do this while connected via USB to your ardupilot). Param: SERIAL3_BAUD 
		- Configure your data rates:
		  Under section 'Configure->Planner->Telemetry rates' you can set the rate for each type of telemetry. Note that since the link speed will be adjusted 
		  automatically you will not loose any information sent from the Rx by setting a rate too high but the rates are important inorder to adjust the portion of each 
		  telemetry type going through the telemetry link.
		  The data rate needed for each type will vary depending on your application. So for example if you only need position updates, set all other data rates to 0 and you'll receive 
		  position updates more frequently, since the telemetry link will only be used to send that type of data.
		  
		- Uncheck the checkbox 'Reset apm on USB connect' if you do not want your Tx to reset each time you connect.
		- Press connect.
		- Tip: you can click on the 'link status' text close to the connect button to see current link quality and status.
		
	
	Connection bug fix in ArduPlane software (ArduCopter works ok)
		- see 'fix connection with lower bps telemetry, honour slowdown of telemetry': https://github.com/gitsly/ardupilot/commit/cfd2c77dda02ec308da6c0ace5077e38add9a75e

	Connect your groundstation computer to the serial port of the Tx. Use this port when connecting in MissionPlanner (also rememeber to use correct baud rate when connecting).


CONFIGURATOR UTILY / BINARY FIRMWARE NOTE:
==========================================
  This software should be used in source form only by expert users, for normal use 'binary' firmwares can be uploaded and configured by free configuration software available for Windows/Mac/Linux from Chrome web store.

  http://goo.gl/iX7dJx

  Binary firmware images are also available from

  https://github.com/openLRSng/openlrsng.github.com/tree/master/binaries

TRANSMITTER HW:
===============
  - Flytron openLRS M2/M3 TX unit -- set TX_BOARD_TYPE 2

  - OrangeRX UHF TX unit -- set TX_BOARD_TYPE 2
  
  - Flytron openLRS RX v2 / OrangeRX UHF RX / HawkEye openLRS RX as TX -- set TX_BOARD_TYPE 3
    - connect PPM input to 5th slot from left (channel 4)
    - button between ground and 4th slot from left (ch3)
    - buzzer via transistor on 3rd slot (ch2) (active high)
    
  - openLRSngTX -- set TX_BOARD_TYPE 4

  - DTFUHF 4ch RX as TX -- set TX_BOARD_TYPE 5

  - HawkEye OpenLRSng TX -- set TX_BOARD_TYPE 4

RECEIVER HW:  
============
  - Flytron openLRS RX 
  - OrangeRX UHF RX (NOTE both LEDs are RED!!)
  - HawkEye OpenLRS RX
  - DTF UHF 4ch RX
  
  Flytron / OrangeRX / HawkEye (RX_BOARD_TYPE 3) default settings:
    RSSI output at 'first' port (marked as RSSI on OrangeRX / ch1 on Flytron) 32kHz PWM signal. To make this analog you can use a simple RC filter (R=10kOhm C=100nF).
    Ports 2-9 are parallel PWM outputs for channels 1-8 (50Hz).
  
  DTF UHF 4ch (RX_BOARD_TYPE 5) default settings:
    CH1-CH4 outputted as PWM (50Hz).

  Receiver pin functiontions can be changed by using the TX CLI system and powering up RX when asked (so it enters config mode).

SOFTWARE CONFIGURATION:
=======================
  Only hardware related selections are done in the openLRSng.ino.

  Run time configuration is done by connecting to the TX module (which is put into binding mode) with serial terminal. For best restults use real terminal program like Putty, TeraTerm, minicom(Linux) but it is possible to use Arduino Serial Monitor too.
  Sending '<CR>' (enter) will enter the menu which will guide further. It should be noted that doing edits will automatically change 'RF magic' to force rebinding, if you want to use a specific magic set that using the command (and further automatic changes are ceased for the edit session). 

  Datarates are: 0==4800bps 1==9600bps 2==19200 bps
  
UPLOADING:
==========
Use a 3v3 FTDI (or other USB to TTL serial adapter) and Arduino >= 1.0. 

  o set board to "Arduino Pro or Pro Mini (5V, 16MHz) w/ atmega328" (yes it runs really on 3v3 but arduino does not need to know that)

  o define COMPILE_TX and upload to TX module

  o comment out COMPILE_TX and upload to RX


USERS GUIDE
===========

TX:
  - Enter binding mode
    - power up while keeping button down and release button after ~1 second.
      Buzzer should emit short beep ~5 times/s in sync with led.
    - To exit bindmode powercycle TX.
  - Randomize channels and 'magic'
    - power up while keeping button down for ~5 seconds (buzzer starts to emit beeps) and release button
    - binding mode is entered automatically
  - Reset settings and randomize channels and 'magic'
    - power up the TX and keep button down for >~10 seconds (buzzer beeps continously).
    - binding mode is entered automatically
  - Setting failsafe
    - Press and hold button for ~1s during normal operation until red LED lights and buzzer beeps, release button.
  - LEDs
    - Green(or blue) LED is lit when module is transmitting
    - Red LED indicates setting of failsafe, or problem with radio module.
  - Link/RX settings are changed via menu, connect using serial terminal while the TX is in binding mode.

RX:
  - Binding
    - If enabled in the .ino RX always binds at boot (and times out after 0.5s) so it is enough to put TX to bind mode and power up RX.
      On successful bind both red and blue (or a second red on OrangeRX RX) leds light up (and remain lit until TX is put on normal mode)
    - RX will also enter bind mode forcibly (without timeout) if EEPROM data is incorrect or a jumpper is placed between two first outputs (RSSI&CH1 on orange/flytron)
  - Failsafe:
    - Failsafe activates after 1s of no link by default, can be changed via menu (0.1 - 25s)
  - LEDs
    - Blue LED lights when packet is received (losing a single packet shows as no pulse on LED)
    - Red LED indicates trouble, it blinks when two consequent packets are lost, and lights up when more than 2 packets are lost
  - Beacon (if enabled) automatically starts after 'deadtime' with no data from TX, the beacon will send three tone 'FM' modulated signal hearable on PMR channel 1. The signal starts with 500Hz @ 100mW and continues with 250Hz @ 15mW and 166Hz @ 1mW. The degrading signal allows to estimate distance.
    - you can use cheap PMR walkie to listen to this signal and using your body as shield determine the direction of it. Alternatively use a directional 433Mhz antenna.

SPECIAL FUNCTIONS
=================

Both TX and RX can be used as spectrum analysers with the "openLRS spectrum analyser GUI). See http://www.rcgroups.com/forums/showpost.php?p=24549162&postcount=551 http://www.rcgroups.com/forums/showthread.php?t=1617297

TX: Put TX into binding mode and connect with GUI (may need to press update once). 

RX: put jumper on output pins 3 and 4 (CH2&3 by default). This will force the RX to act as spectrum scanner, both LEDs will be off in this mode.
