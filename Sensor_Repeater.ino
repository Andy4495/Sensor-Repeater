// Sensor Repeater with CC110L
// 1.0.0  03/10/2018  A.T.     Original
// 1.1.0  04/05/2018  A.T.     Add support for external OLED display
//


#include <SPI.h>
#include <AIR430BoostFCC.h>
#include "MspTandV.h"
#define ZX_SENSOR_ENABLED
#define OLED_ENABLED
#define RADIO_ENABLED
//#define

// Normally, the repeater code sends the Garage data whenever it receives weather data
// In case no weather data is received, define a time that garage data will be sent
// even without weather data
#define GARAGE_MAX_TX_DELAY  (1000UL * 60UL * 3UL)

#if defined(__MSP430FR4133__)
#define LCD_ENABLED
#define BOARD_LED LED2
#define PUSHBUTTON PUSH2
#endif

#if defined(__MSP430FR6989__)
#define LCD_ENABLED
#define BOARD_LED GREEN_LED
#define PUSHBUTTON PUSH2
#endif

#if defined(__MSP430G2553__)
#define LED_DISABLED         // LEDs conflict with CC110L pins
#define PUSHBUTTON PUSH2     // Uses Pin 5, only button available
#endif

#if defined(__MSP430F5529__)
#define BOARD_LED GREEN_LED
#define PUSHBUTTON PUSH2
#endif

#if defined(__MSP430FR2433__)
#define BOARD_LED LED2
#define PUSHBUTTON PUSH1
#endif

#if defined(__MSP430FR5969__)
#define BOARD_LED LED2
#define PUSHBUTTON PUSH2
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
uint8_t z_pos;
SWI2C myZX = SWI2C(SDA_PIN, SCL_PIN, ZX_ADDR);
#endif

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
long           msp430T;
unsigned long  msp430mV;
int loopCount = 0;

unsigned int    last_BME280_P = 0;     // Pressure in inches of Hg * 100

int             last_BME280_H = 0;     // % Relative Humidity
int             last_TMP107_Ti = 0;    // Tenth degrees F
unsigned int    last_weather_mV = 0;   // milliVolts
int             last_garage_T = 0;     // Tenth degrees F

unsigned long   prevGarageMillis = 0;        // Last time Garage sensor transmitted data
unsigned long   prevWeatherMillis = 0;       // Last time Weather sensor transmitted data

int current_display = ADDRESS_REPEATER;

void setup() {
#ifdef RADIO_ENABLED
  // Set the CC110L Chip Select High (inactive) inactive for now
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);
#endif

  // Setup serial for status printing.
  Serial.begin(9600);
  Serial.println(F("Repeater"));

#ifndef LED_DISABLED
  pinMode(BOARD_LED, OUTPUT);       // Flash LED to indicate ready to receive
  digitalWrite(BOARD_LED, HIGH);
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
#ifndef LED_DISABLED
    digitalWrite(BOARD_LED, HIGH);
#endif
    Serial.print(F("RX from: "));
    Serial.print(Packet.from);
    Serial.print(F(", bytes: "));
    Serial.println(packetSize);
    Serial.print(F("RSSI: "));
    Serial.println(Radio.getRssi());
    Serial.print(F("LQI: "));
    Serial.println(Radio.getLqi());
    if (Radio.getCrcBit() == 0) {
      crcFailed = 1;
#ifdef SERIAL_ENABLED
      Serial.println(F("*** CRC check failed! ***"));
#endif
    } else crcFailed = 0;

    switch (Packet.from) {
      case (ADDRESS_WEATHER):
        if (crcFailed == 0) {
          process_weatherdata();
          sleep(5000);             // Give the Rx Hub time to process the weather data
        }
        process_localdata();
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
    // If we haven't gotten any weather data in a while, then send the garage data anyway
    if ((millis() - prevGarageMillis) > GARAGE_MAX_TX_DELAY) process_localdata();
#ifdef ZX_SENSOR_ENABLED
    myZX.read1bFromRegister(ZPOS_REG, &z_pos);
    Serial.print("Door: ");
    Serial.println(z_pos);
#endif
#ifdef LCD_ENABLED
    display_on_LCD();
    if (current_display == ADDRESS_WEATHER) current_display = ADDRESS_REPEATER;
    else current_display = ADDRESS_WEATHER;
#endif
#ifdef OLED_ENABLED
    showStatusOnOled();
#endif
  }
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
}

void process_localdata() {
  myTemp.read(CAL_ONLY);
  myVcc.read(CAL_ONLY);
  Packet.sensordata.MSP_T = myTemp.getTempCalibratedF();
  last_garage_T = Packet.sensordata.MSP_T;
  Packet.sensordata.Batt_mV = myVcc.getVccCalibrated();
  Packet.sensordata.Loops = loopCount;
  Packet.sensordata.Millis = millis();
#ifdef ZX_SENSOR_ENABLED
  myZX.read1bFromRegister(ZPOS_REG, &z_pos);
  Packet.sensordata.Door_Sensor = z_pos;
#endif
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
      myLCD.showSymbol(LCD_SEG_CLOCK, true);
      myLCD.showSymbol(LCD_SEG_HEART, true);
      break;
    case ADDRESS_WEATHER:
      displayTempOnLCD(last_TMP107_Ti);
      myLCD.showSymbol(LCD_SEG_CLOCK, true);
      displayBattOnLCD(last_weather_mV);
      break;
    default:
      break;
  }
}
#endif

#ifdef LCD_ENABLED
void displayBattOnLCD(int mV) {
  if (mV > 3200) myLCD.showSymbol(LCD_SEG_BAT5, 1);
  if (mV > 3000) myLCD.showSymbol(LCD_SEG_BAT4, 1);
  if (mV > 2800) myLCD.showSymbol(LCD_SEG_BAT3, 1);
  if (mV > 2600) myLCD.showSymbol(LCD_SEG_BAT2, 1);
  if (mV > 2400) myLCD.showSymbol(LCD_SEG_BAT1, 1);
  if (mV > 2200) myLCD.showSymbol(LCD_SEG_BAT0, 1);
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
      break;
    case 2:
      if ((millis() - oledStartTime) > OLED_ON_TIME) {
        oled.command(0x08);         // Turns off OLED
        oledStatus = 0;             // Display is off state
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


  snprintf((char*)oled_text[0], OLED_COLS + 1, "To=%4d, Tg=%4d", (last_TMP107_Ti +5) / 10, (last_garage_T + 5) / 10);
  snprintf((char*)oled_text[1], OLED_COLS + 1, "P=%3d,H=%2d,dt=%2d", last_BME280_P / 10, last_BME280_H / 10, lastminutes);
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
