// Sensor Repeater with CC110L
//
//  https://gitlab.com/Andy4495/Sensor-Repeater
//
// 1.0.0  03/10/2018  A.T.     Original
// 1.1.0  04/05/2018  A.T.     Add support for external OLED display
// 1.2.0  04/06/2018  A.T.     Add support for averaging on-chip Temp and Vcc measurements
// 1.3.0  05/08/2018  A.T.     Change OLED info to include RSSI, LQI, # valid messages and CRC errors
// 1.4.0  05/23/2018  A.T.     Add support for 2-digit 7-segment LED, using LED744511 library with 74HC164 serial interface
// 1.5.0  06/26/2018  A.T.     Update 7-segment: use 74HC164 to control DPs and button to enable/disable
//                             Note that 7-segment pin I/O changed
// 1.6.0  11/27/2018  A.T.     Default 7-segment LED ON after reset
// 1.7.0  02/21/2018  A.T.     Send a message any time door status changes, instead of just at 5 minute intervals.


/* Pin Summary
  1 - 3V3             (LaunchPad)
  2 - GDO2            (CC110L)
  3 - RX              (G2, FR2433)
  4 - TX              (G2, FR2433)
  5 - PUSH2           (G2553)
  6 - LED_LSB_LE      (7SEG)
  7 - SPI SCK         (CC110L)
  8 - LED_MSB_LE      (7SEG)        // Also PUSH2 on FR2433, but unused
  9 - SWI2C SCL       (ZX)
  10 - SWI2C SDA      (ZX)
  11 - CS             (OLED)
  12 - SWSPI MOSI/SDA (7SEG, OLED)
  13 - SWSPI SCK      (7SEG, OLED)
  14 - SPI MISO       (CC110L)
  15 - SPI MOSI       (CC110L)
  16 - RESET          (LaunchPad)
  17 - LED_BL         (7SEG)
  18 - CS             (CC110L)
  19 - GDO0           (CC110L)
  20 - GND            (LaunchPad)
*/

#include <SPI.h>
#include <AIR430BoostFCC.h>
#include "MspTandV.h"

///
///#define SERIAL_ENABLED        Disable serial to save program memory
#define ZX_SENSOR_ENABLED
#define OLED_ENABLED
#define RADIO_ENABLED
#define LED_7SEG_ENABLED

// Time between Garage Sensor transmissions
#define GARAGE_MAX_TX_DELAY  (1000UL * 60UL * 3UL)

#if defined(__MSP430FR4133__)
#define LCD_ENABLED
#define BOARD_LED LED2
#define PUSHBUTTON PUSH2
#define BUTTON7SEG PUSH1
#endif

#if defined(__MSP430FR6989__)
#define LCD_ENABLED
#define BOARD_LED GREEN_LED
#define PUSHBUTTON PUSH2
#define BUTTON7SEG PUSH1
#endif

#if defined(__MSP430G2553__)
#define LED_DISABLED         // LEDs conflict with CC110L pins
#define PUSHBUTTON PUSH2     // Uses Pin 5, only button available
#define BUTTON7SEG PUSH2     // Use same button for both functions. 
#endif

#if defined(__MSP430F5529__)
#define BOARD_LED GREEN_LED
#define PUSHBUTTON PUSH2
#define BUTTON7SEG PUSH1
#endif

#if defined(__MSP430FR2433__)
#define LED_DISABLED         // LEDs conflict with CC110L pins
#define PUSHBUTTON PUSH1     // Both functions are on PUSH1, so pressing PUSH1 will also momentarily turn on OLED
#define BUTTON7SEG PUSH1
#endif

#if defined(__MSP430FR5969__)
#define BOARD_LED LED2
#define PUSHBUTTON PUSH2
#define BUTTON7SEG PUSH1
#endif

#ifdef LED_7SEG_ENABLED
#include "LED744511.h"
#define LED_MSB_LE   8
#define LED_LSB_LE   6
#define LED_BL      17
// Next two pins are shared with OLED. This works because they
// have separate chip select pins.
#define LED_SCK    13   // Shared with OLED SCK
#define LED_DIN    12   // Shared with OLED SDIN
LED744511_Serial myLED(LED_SCK, LED_DIN, LED_MSB_LE, LED_LSB_LE, LED744511_Serial::NO_PIN, LED744511_Serial::NO_PIN, LED_BL );
#endif

#ifdef LCD_ENABLED
#include "LCD_Launchpad.h"
LCD_LAUNCHPAD myLCD;
#endif

#ifdef OLED_ENABLED
#include "NewhavenOLED.h"
const byte OLED_ROWS = 2;                 // Number of display rows
const byte OLED_COLS = 16;             // Number of display columns
const byte CS_PIN = 11;
const byte RES_PIN = NO_PIN;          // Hardwire RESET pin HIGH
const byte SCLK_PIN = 13;
const byte SDIN_PIN = 12;
const byte row_address[2] = {0x80, 0xC0};   // DDRAM addresses for rows (2-row models)
NewhavenOLED oled(OLED_ROWS, OLED_COLS, SDIN_PIN, SCLK_PIN, CS_PIN, RES_PIN);
byte oled_text[OLED_ROWS][OLED_COLS + 1] =
{ "Sensor Repeater ",
  " Initializing..."
};
#define OLED_ON_TIME (1000UL * 15UL)
unsigned long oledStartTime;
int oledStatus = 0;
#endif

#ifdef ZX_SENSOR_ENABLED
#include "SWI2C.h"
#define SCL_PIN 9
#define SDA_PIN 10
const uint8_t ZX_ADDR = 0x10;  // ZX Sensor I2C address
const uint8_t ZPOS_REG = 0x0A;
uint8_t z_pos = 0, prev_z_pos = 0;
SWI2C myZX = SWI2C(SDA_PIN, SCL_PIN, ZX_ADDR);
#endif
#define Z_POS_DIFF_THRESHOLD 30

#define CC110L_CS  18
#define RF_GDO0    19


#define ADDRESS_REMOTE   0x01
#define ADDRESS_WEATHER  0x02
#define ADDRESS_G2       0x03
#define ADDRESS_SENSOR4  0x04
#define ADDRESS_SENSOR5  0x05
#define ADDRESS_REPEATER 0x06
#define LAST_ADDRESS     0x06


struct WeatherData {
  int             BME280_T;  // Tenth degrees F
  unsigned int    BME280_P;  // Pressure in inches of Hg * 100
  int             BME280_H;  // % Relative Humidity
  int             TMP107_Ti; // Tenth degrees F
  int             TMP107_Te; // Tenth degrees F
  unsigned long   LUX;       // Lux units
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
};

struct TempSensor {
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
  unsigned int    Light_Sensor;
  unsigned int    Door_Sensor;
};

// Check "struct_type" member for the type of structure to
// decode in the sPacket union.
enum {WEATHER_STRUCT, TEMP_STRUCT};

struct sPacket  // CC110L packet structure
{
  uint8_t from;              // Local node address that message originated from
  uint8_t struct_type;       // Filler byte to keep rest of struct on word boundary
  // Also used to indicate type of struct in union
  union {
    uint8_t message[58];     // Local node message keep even word boundary
    WeatherData weatherdata;
    TempSensor  sensordata;
  };
};

// -----------------------------------------------------------------------------
/**
    Global data
*/

struct sPacket Packet;
int crcFailed = 0; // 1 is bad CRC, 0 is good CRC
MspTemp myTemp;
MspVcc  myVcc;
int loopCount = 0;
int messageReceived = 0;
int lastRSSI = 0, lastLQI = 0;
long totalValid = 0, totalCRC = 0;
int  sevenSegEnabled = ~0;           // Enabled by default

unsigned int    last_BME280_P = 0;     // Pressure in inches of Hg * 100

int             last_BME280_H = 0;     // % Relative Humidity
int             last_TMP107_Ti = 0;    // Tenth degrees F
unsigned int    last_weather_mV = 0;   // milliVolts
int             last_garage_T = 0;     // Tenth degrees F

unsigned long   prevGarageMillis = 0;        // Last time Garage sensor transmitted data
unsigned long   prevWeatherMillis = 0;       // Last time Weather sensor transmitted data

int current_display = ADDRESS_REPEATER;

// Averaging on-chip readings
#define NUMBER_OF_SAMPLES 10
int samplesTaken = 0;
int currentSample = 0;
int mspTsamples[NUMBER_OF_SAMPLES];
int mspTavg;
unsigned int mspmVsamples[NUMBER_OF_SAMPLES];
unsigned long mspmVavg;


void setup() {
#ifdef RADIO_ENABLED
  // Set the CC110L Chip Select High (inactive) inactive for now
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);
#endif

#ifdef SERIAL_ENABLED
  // Setup serial for status printing.
  Serial.begin(9600);
  Serial.println(F("Repeater"));
#endif

#ifndef LED_DISABLED
  pinMode(BOARD_LED, OUTPUT);       // Flash LED to indicate ready to receive
  digitalWrite(BOARD_LED, HIGH);
#endif

#ifdef LED_7SEG_ENABLED
  pinMode(BUTTON7SEG, INPUT_PULLUP);
  if (sevenSegEnabled)
  {
    myLED.blankDisplay(HIGH);
    displayOnLED(0);
  }
  else
    myLED.blankDisplay(LOW);
#endif

#ifdef OLED_ENABLED
  oled.begin();
  oledDisplay();        // Print the welcome message
#ifdef SERIAL_ENABLED
  Serial.println("OLED Enabled");
#endif
#endif

  // Set up button to monitor for external OLED display
  pinMode(PUSHBUTTON, INPUT_PULLUP);

#ifdef LCD_ENABLED
  myLCD.init();
#ifdef SERIAL_ENABLED
  Serial.println(F("LCD Enabled"));
#endif
#endif

#ifdef ZX_SENSOR_ENABLED
  myZX.begin();
#ifdef SERIAL_ENABLED
  Serial.println("SWI2C - ZX Sensor Enabled.");
#endif
#endif

  // Setup CC110L data structure
  Packet.from = 0;         // "from" and "struct_type" filled in by received message
  Packet.struct_type = 0;  // Zero them out here just for completeness
  memset(Packet.message, 0, sizeof(Packet.message));

  SPI.begin(CC110L_CS);

#ifdef LCD_ENABLED
  myLCD.clear();
  myLCD.displayText(F("RX ON "));
#endif

  delay(1000);   // Pause for the status messages and displays

#ifndef LED_DISABLED
  digitalWrite(BOARD_LED, LOW);
#endif

#ifdef OLED_ENABLED
  oled.command(0x08);              // Turn off the welcome message
#endif
}

void loop() {
  int packetSize;
  loopCount++;

  // Turn on the receiver and listen for incoming data. Timeout after 1 seconds.
  // The receiverOn() method returns the number of bytes copied to rxData.
  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
#ifdef RADIO_ENABLED
  Radio.begin(ADDRESS_REMOTE, CHANNEL_4, POWER_MAX);
  packetSize = Radio.receiverOn((unsigned char*)&Packet, sizeof(Packet), 1000);
  // Simulate a Radio.end() call, but don't want to disable Chip Select pin
  // Radio.end();
  while (Radio.busy()) ; // Empty loop statement
  detachInterrupt(RF_GDO0);
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);    // Need to pull radio CS high to keep it off the SPI bus
#endif

  if (packetSize > 0) {
    lastRSSI = Radio.getRssi();
    lastLQI = Radio.getLqi();
#ifndef LED_DISABLED
    digitalWrite(BOARD_LED, HIGH);
#endif
#ifdef SERIAL_ENABLED
    Serial.print(F("RX from: "));
    Serial.print(Packet.from);
    Serial.print(F(", bytes: "));
    Serial.println(packetSize);
    Serial.print(F("RSSI: "));
    Serial.println(lastRSSI);
    Serial.print(F("LQI: "));
    Serial.println(lastLQI);
#endif
    if (Radio.getCrcBit() == 0) {
      totalCRC++;
      crcFailed = 1;
#ifdef SERIAL_ENABLED
      Serial.println(F("*** CRC check failed! ***"));
#endif
    } else {
      totalValid++;
      crcFailed = 0;
    }

    switch (Packet.from) {
      case (ADDRESS_WEATHER):
        if (crcFailed == 0) {
          process_weatherdata();
          messageReceived = 1;     // Flag to display Antenna symbol on LCD
        }
        break;
      case (ADDRESS_G2):
      case (ADDRESS_SENSOR4):
      case (ADDRESS_SENSOR5):
#ifdef SERIAL_ENABLED
        Serial.println(F("Message received from untracked sensor."));
#endif
        break;
      default:
#ifdef SERIAL_ENABLED
        Serial.println(F("Message received from unknown sensor."));
#endif
        break;
    }
#ifdef SERIAL_ENABLED
    Serial.println(F("--"));
#endif
#ifndef LED_DISABLED
    digitalWrite(BOARD_LED, LOW);
#endif
    // Clear the buffer
    Packet.from = 0;         // "from" and "struct_type" filled in by received message
    Packet.struct_type = 0;  // Zero them out here just for completeness
    memset(Packet.message, 0, sizeof(Packet.message));
  }
  else {
#ifdef SERIAL_ENABLED
    Serial.print(F("Nothing received: "));
    Serial.println(millis());
#endif
    // Check if time to send garage sensosr data
#ifdef ZX_SENSOR_ENABLED
    myZX.read1bFromRegister(ZPOS_REG, &z_pos);
#endif 
    if ((millis() - prevGarageMillis) > GARAGE_MAX_TX_DELAY  ||
         abs( (z_pos - prev_z_pos) > Z_POS_DIFF_THRESHOLD) )
    {
      prev_z_pos = z_pos;
      process_localdata();
    }
#ifdef SERIAL_ENABLED
    Serial.print("Door: ");
    Serial.println(z_pos);
#endif
#ifdef LCD_ENABLED
    display_on_LCD();
    messageReceived = 0;   // Clear the flag so LCD antenna only stays on for one display cycle
    if (current_display == ADDRESS_WEATHER) current_display = ADDRESS_REPEATER;
    else current_display = ADDRESS_WEATHER;
#endif
#ifdef OLED_ENABLED
    showStatusOnOled();
#endif
  }
#ifdef LED_7SEG_ENABLED
  if (digitalRead(BUTTON7SEG) == LOW) {
    myLCD.clear();
    sevenSegEnabled = ~sevenSegEnabled;
    if (sevenSegEnabled)
    {
      myLED.blankDisplay(HIGH);
      displayOnLED((last_TMP107_Ti + 5) / 10);
    }
    else
    {
      myLED.blankDisplay(LOW);
      myLED.setDP(LOW, LOW);      // Clear the decimal points if necessary
      myLED.writeBCD(-1);         // Update the decimal points
    }
#ifdef LCD_ENABLED
    myLCD.clear();
    myLCD.showSymbol(LCD_SEG_DOT3, 1);
    if (sevenSegEnabled) myLCD.displayText("LEDON", 0);
    else myLCD.displayText("LEDOFF", 0);
#endif
  }
#endif
}

void process_weatherdata() {
#ifdef RADIO_ENABLED
  // Repeat the weather transmission to the hub
  Radio.begin(ADDRESS_WEATHER, CHANNEL_1, POWER_MAX);
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&Packet, sizeof(Packet));
  // Simulate a Radio.end() call, but don't want to disable Chip Select pin
  // Radio.end();
  while (Radio.busy()) ; // Empty loop statement
  detachInterrupt(RF_GDO0);
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);    // Need to pull radio CS high to keep it off the SPI bus
#endif
  prevWeatherMillis = millis();

#ifdef SERIAL_ENABLED
  Serial.println("Received packet from WEATHER.");
  Serial.println(F("Temperature (F): "));
  Serial.print(F("    BME280:  "));
  Serial.print(Packet.weatherdata.BME280_T / 10);
  Serial.print(F("."));
  Serial.println(Packet.weatherdata.BME280_T % 10);
  Serial.print(F("    TMP106 (Die):  "));
  Serial.print(Packet.weatherdata.TMP107_Ti / 10);
  Serial.print(F("."));
  Serial.println(Packet.weatherdata.TMP107_Ti % 10);
  Serial.print(F("    TMP106 (Ext):  "));
  Serial.print(Packet.weatherdata.TMP107_Te / 10);
  Serial.print(F("."));
  Serial.println(Packet.weatherdata.TMP107_Te % 10);
  Serial.print(F("    MSP Die: "));
  Serial.print(Packet.weatherdata.MSP_T / 10);
  Serial.print(F("."));
  Serial.println(Packet.weatherdata.MSP_T % 10);
  Serial.print(F("Pressure (inHg): "));
  Serial.print(Packet.weatherdata.BME280_P / 100);
  Serial.print(F("."));
  Serial.print((Packet.weatherdata.BME280_P / 10) % 10);
  Serial.println(Packet.weatherdata.BME280_P % 10);
  Serial.print(F("%RH: "));
  Serial.print(Packet.weatherdata.BME280_H / 10);
  Serial.print(F("."));
  Serial.println(Packet.weatherdata.BME280_H % 10);
  Serial.print(F("Lux: "));
  Serial.println(Packet.weatherdata.LUX);
  Serial.print(F("Battery V: "));
  Serial.print(Packet.weatherdata.Batt_mV / 1000);
  Serial.print(F("."));
  Serial.print((Packet.weatherdata.Batt_mV / 100) % 10);
  Serial.print((Packet.weatherdata.Batt_mV / 10) % 10);
  Serial.print(Packet.weatherdata.Batt_mV % 10);
  if (Packet.weatherdata.Batt_mV < 2200) {
    Serial.print(F("   *** Out of Spec ***"));
  }
  Serial.println(F(" "));
  Serial.print(("Loops: "));
  Serial.println(Packet.weatherdata.Loops);
  Serial.print(("Millis: "));
  Serial.println(Packet.weatherdata.Millis);
#endif
  last_BME280_P = Packet.weatherdata.BME280_P;
  last_BME280_H = Packet.weatherdata.BME280_H;
  last_TMP107_Ti = Packet.weatherdata.TMP107_Ti;
  last_weather_mV = Packet.weatherdata.Batt_mV;
#ifdef LED_7SEG_ENABLED
  if (sevenSegEnabled) {
    displayOnLED((last_TMP107_Ti + 5) / 10);
  }
#endif
}

void process_localdata() {
  myTemp.read(CAL_ONLY);
  myVcc.read(CAL_ONLY);

  // Calculate averages for Temp and mV
  mspTsamples[currentSample] = myTemp.getTempCalibratedF();
  mspmVsamples[currentSample] = myVcc.getVccCalibrated();
  currentSample++;
  if (currentSample >= NUMBER_OF_SAMPLES) currentSample = 0;
  samplesTaken++;
  if (samplesTaken > NUMBER_OF_SAMPLES) samplesTaken = NUMBER_OF_SAMPLES;
  mspTavg = 0;
  mspmVavg = 0;
  for (int i = 0; i < samplesTaken; i++) {
    mspTavg += mspTsamples[i];
    mspmVavg += mspmVsamples[i];
  }
  mspTavg = mspTavg / samplesTaken;
  mspmVavg = mspmVavg / samplesTaken;

  Packet.sensordata.MSP_T = mspTavg;
  last_garage_T = Packet.sensordata.MSP_T;
  Packet.sensordata.Batt_mV = mspmVavg;

  Packet.sensordata.Loops = loopCount;
  Packet.sensordata.Millis = millis();
  Packet.sensordata.Door_Sensor = z_pos;
  Packet.from = ADDRESS_REPEATER;

#ifdef RADIO_ENABLED
  // Send the data over-the-air
  Radio.begin(ADDRESS_REPEATER, CHANNEL_1, POWER_MAX);
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&Packet, sizeof(Packet));
  // Simulate a Radio.end() call, but don't want to disable Chip Select pin
  // Radio.end();
  while (Radio.busy()) ; // Empty loop statement
  detachInterrupt(RF_GDO0);
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);    // Need to pull radio CS high to keep it off the SPI bus
#endif
  prevGarageMillis = millis();

#ifdef SERIAL_ENABLED
  Serial.print("Local temp: ");
  Serial.println(Packet.sensordata.MSP_T);
  Serial.print("Local Vcc: ");
  Serial.println(Packet.sensordata.Batt_mV);
#ifdef ZX_SENSOR_ENABLED
  Serial.print("Door z_pos: ");
  Serial.println(Packet.sensordata.Door_Sensor);
#endif
#endif
}

#ifdef LCD_ENABLED
void display_on_LCD() {
  myLCD.clear();
  switch (current_display) {
    case ADDRESS_REPEATER:
      displayTempOnLCD(last_garage_T);
      myLCD.showSymbol(LCD_SEG_HEART, true);
      break;
    case ADDRESS_WEATHER:
      displayTempOnLCD(last_TMP107_Ti);
      displayBattOnLCD(last_weather_mV);
      break;
    default:
      break;
  }
  myLCD.showSymbol(LCD_SEG_RADIO, messageReceived);
}
#endif

#ifdef LCD_ENABLED
void displayBattOnLCD(int mV) {
  if (mV > 3200) myLCD.showSymbol(LCD_SEG_BAT5, 1);
  if (mV > 3000) myLCD.showSymbol(LCD_SEG_BAT4, 1);
  if (mV > 2800) myLCD.showSymbol(LCD_SEG_BAT3, 1);
  if (mV > 2600) myLCD.showSymbol(LCD_SEG_BAT2, 1);
  if (mV > 2500) myLCD.showSymbol(LCD_SEG_BAT1, 1);
  if (mV > 2400) myLCD.showSymbol(LCD_SEG_BAT0, 1);
  myLCD.showSymbol(LCD_SEG_BAT_ENDS, 1);
  myLCD.showSymbol(LCD_SEG_BAT_POL, 1);
}
#endif

#ifdef OLED_ENABLED
void showStatusOnOled() {
  if (digitalRead(PUSHBUTTON) == LOW) {
    oledStartTime = millis();
    oledStatus = 1;              // Waiting-to-turn-on state
  }
  switch (oledStatus) {
    case 1:
      buildStatusString();
      oled.command(0x0C);         // Turn on display
      oledDisplay();
      oledStatus = 2;             // Display-is-on state
#ifdef LED_7SEG_ENABLED           // Clean up decimal points since CLK/DATA lines shared
      if (sevenSegEnabled) {
        displayOnLED((last_TMP107_Ti + 5) / 10);
      }
      else
      {
        myLED.setDP(LOW, LOW);      // Clear the decimal points if necessary
        myLED.writeBCD(-1);         // Update the decimal points
      }
#endif
      break;
    case 2:
      if ((millis() - oledStartTime) > OLED_ON_TIME) {
        oled.command(0x08);         // Turns off OLED
        oledStatus = 0;             // Display is off state
#ifdef LED_7SEG_ENABLED             // Clean up decimal points since CLK/DATA lines shared
        if (sevenSegEnabled) {
          displayOnLED((last_TMP107_Ti + 5) / 10);
        }
        else
        {
          myLED.setDP(LOW, LOW);    // Clear the decimal points if necessary
          myLED.writeBCD(-1);       // Update the decimal points
        }
#endif
      }
      break;
    default:
      break;
  }
}

void oledDisplay() {
  byte r = 0;
  byte c = 0;

  oled.command(0x01); // Clear display and cursor home
  delay(2);           // Need a pause after clearing display
  for (r = 0; r < OLED_ROWS; r++)        // One row at a time
  {
    oled.command(row_address[r]);        //  moves the cursor to the first column of that line
    for (c = 0; c < OLED_COLS; c++)      //  One character at a time
    {
      oled.data(oled_text[r][c]);         //  displays the corresponding string
    }
  }
}

void buildStatusString() {
  int splen;
  unsigned long lastminutes;

  lastminutes = (millis() - prevWeatherMillis) / 1000 / 60;
  if (lastminutes > 99) lastminutes = 99;

  // Row 0: +xxxxxxx,-xxxxxx
  // Row 1: SS-xxx,Qxx,dt:xx
  // "+" = Valid messages received, "-" = CRC errors received
  // "SS" = RSSI (Signal Strength), "Q"=LQI (Quality), "dt" = minutes since last message received (delta time)
  if (lastRSSI < -999) lastRSSI = -999;
  if (lastRSSI > 0) lastRSSI = 0;
  if (lastLQI > 99) lastLQI = 99;
  if (lastLQI < 0) lastLQI = 0;
  snprintf((char*)oled_text[0], OLED_COLS + 1, "+%7ld,-%6ld", totalValid, totalCRC);
  snprintf((char*)oled_text[1], OLED_COLS + 1, "SS-%3d,Q%2d,dt:%2d", -lastRSSI, lastLQI, lastminutes);
  oled_text[1][11] = 0x15;   // Replace 'd' with delta character (from ROM C character set)

#ifdef SERIAL_ENABLED
  Serial.print("OLED Row 0: ");
  Serial.println((char*)oled_text[0]);
  Serial.print("OLED ROW 1: ");
  Serial.println((char*)oled_text[1]);
#endif
}
#endif

#ifdef LCD_ENABLED
void displayTempOnLCD(int temp) {
  char tempChar[12];
  int tempLen;
  int tempSign;

  memset(tempChar, 0, 12);
  if (temp < 0) {
    tempSign = 1;
    temp = -temp;
  } else tempSign = 0;
  sprintf(tempChar, "%d", temp);
  tempLen = strlen(tempChar);
  // Need to add a leading zero if len == 1
  if (tempLen == 1) {
    char x = tempChar[0];
    tempChar[0] = '0';
    tempChar[1] = x;
    tempChar[2] = '\0';
    tempLen = 2;
  }
  for (int i = 0; i < tempLen; i++) {
    myLCD.showChar(tempChar[i], 5 - tempLen + i);
  }
  myLCD.showSymbol(LCD_SEG_DOT4, 1);
  myLCD.showSymbol(LCD_SEG_DEG5, 1);
  myLCD.showSymbol(LCD_SEG_MINUS1, tempSign);
}
#endif

#ifdef LED_7SEG_ENABLED
void displayOnLED(int value) {
  // Negative values -- turn on DP on right (LSB) digit
  if (value < 0) {
    myLED.setDP(0, 1);
    if (value < -99) value = -99; // Limit to 2 digits
    myLED.writeBCD(-value);
  }
  // Values 100 and above -- turn on both DPs
  else if (value > 99) {
    myLED.setDP(1, 1);
    // Note that library displays a leading zero for values from 100 - 109
    // and automatically truncates to 2 digits if value > 99
    myLED.writeBCD(value);
  }
  else // Values from 0 - 99 -- no DPs
  {
    myLED.setDP(0, 0);
    myLED.writeBCD(value);
  }
}
#endif
