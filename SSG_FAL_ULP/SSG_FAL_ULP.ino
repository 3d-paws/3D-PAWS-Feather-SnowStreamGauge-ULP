#define COPYRIGHT "Copyright [2024] [University Corporation for Atmospheric Research]"
#define VERSION_INFO "SSGFALMO-240719"

/*
 *======================================================================================================================
 * SnowStreamGauge(SSG) Feather Ada Logger (FAL) - Logs to SD card. Not Wireless support.
 *   Board Type : Adafruit Feather M0
 *   Description: 
 *   Author: Robert Bubon
 *   Date:   2022-09-20 RJB Initial
 *           2022-09-28 RJB Remove divide by 4 on gauge read, M0 has 10-bit analog pins, not 12-bit
 *                          Snow gauges read double
 *           2023-08-27 RJB Adding 1-Wire Dallas Temperature Sensor 
 *                          Adding mcp support
 *           2024-07-19 RJB Split in to multiple files            
 *                          Added Copyright
 *                          Updated oled code
 *                          Updated enable serial console code
 * SEE https://learn.adafruit.com/adafruit-feather-m0-adalogger/
 * SEE https://www.microchip.com/wwwproducts/en/MCP73831 - Battery Charger
 * 
 * The RTC PCF8523 is simple and inexpensive but not a high precision device. It may lose or gain up to 2 seconds a day.
 * Use RTC DS3231 https://www.adafruit.com/product/3013
 * 
 * Dallas OneWire for Particle
 * https://github.com/Hotaman/OneWireSpark
 * ======================================================================================================================
 */
#include <SPI.h>
#include <Wire.h>
#include <ArduinoLowPower.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MCP9808.h>

#include <RTClib.h>

#define RTC_I2C_ADDRESS 0x68       // I2C address for PCF8523 and DS3231

#define SSB_PWRON           0x1     // Set at power on, but cleared after first observation
#define SSB_SD              0x2     // Set if SD missing at boot or other SD related issues
#define SSB_RTC             0x4     // Set if RTC missing at boot
#define SSB_OLED            0x8     // Set if OLED missing at boot, but cleared after first observation
#define SSB_N2S             0x10    // Set when Need to Send observations exist
#define SSB_FROM_N2S        0x20    // Set in transmitted N2S observation when finally transmitted
#define SSB_AS5600          0x40    // Set if wind direction sensor AS5600 has issues
#define SSB_BMX_1           0x80    // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_BMX_2           0x100   // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_HTU21DF         0x200   // Set if Humidity & Temp Sensor missing
#define SSB_SI1145          0x400   // Set if UV index & IR & Visible Sensor missing
#define SSB_MCP_1           0x800   // Set if Precision I2C Temperature Sensor missing
#define SSB_DS_1           0x1000   // Set if Dallas One WireSensor missing at startup


unsigned int SystemStatusBits = SSB_PWRON; // Set bit 0 for initial value power on. Bit 0 is cleared after first obs
bool JustPoweredOn = true;         // Used to clear SystemStatusBits set during power on device discovery

/*
 * =======================================================================================================================
 *  Globals
 * =======================================================================================================================
 */
char msgbuf[256];
char *msgp;                               // Pointer to message text
char Buffer32Bytes[32];                   // General storage
int countdown = 1800;        // Exit calibration mode when reaches 0 - protects against burnt out pin or forgotten jumper
unsigned int SendSensorMsgCount=0;        // Counter for Sensor messages transmitted
unsigned int SendType2MsgCount=0;         // Counter for Powerup and Heartbeat messages transmitted
int  LED_PIN = LED_BUILTIN;               // Built in LED

/*
 * ======================================================================================================================
 *  SD Card
 * ======================================================================================================================
 */
#define SD_ChipSelect 4    // D4
// SD;                      // File system object defined by the SD.h include file.
File SD_fp;
char SD_obsdir[] = "/OBS";  // Store our obs in this directory. At Power on, it is created if does not exist
bool SD_exists = false;     // Set to true if SD card found at boot

/*
 * ======================================================================================================================
 *  Local Code Includes - Do not change the order of the below 
 * ======================================================================================================================
 */
#include "QC.h"                   // Quality Control Min and Max Sensor Values on Surface of the Earth
#include "SF.h"                   // Support Functions
#include "OP.h"                   // OutPut support for OLED and Serial Console
#include "CF.h"                   // Configuration File Variables
#include "TM.h"                   // Time Management
#include "DS.h"                   // Dallas Sensor - One Wire
#include "Sensors.h"              // I2C Based Sensors
#include "SDC.h"                  // SD Card
#include "SG.h"                   // Stream/Snow Gauge
#include "OBS.h"                  // Do Observation Processing
#include "SM.h"                   // Station Monitor


/* 
 *=======================================================================================================================
 * seconds_to_next_obs() - do observations on 0, 15, 30, or 45 minute window
 *=======================================================================================================================
 */
int seconds_to_next_obs() {
  now = rtc.now(); //get the current date-time
  return (900 - (now.unixtime() % 900)); // 900 = 60s * 15m,  The mod operation gives us seconds passed in this 15m window
}

/*
 * =======================================================================================================================
 * setup()
 * =======================================================================================================================
 */
void setup() 
{
  // Put initialization like pinMode and begin functions here.
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Output_Initialize();
  delay(2000); // Prevents usb driver crash on startup

  Serial_writeln(COPYRIGHT);
  Output (VERSION_INFO);

  // Set up gauge pin for reading 
  pinMode(SGAUGE_PIN, INPUT);

  // Initialize SD card if we have one.
  SD_initialize();

  if (SD_exists && SD.exists(CF_NAME)) {
    SD_ReadConfigFile();
  }
  else {
    sprintf(msgbuf, "CF:NO %s", CF_NAME); Output (msgbuf);
  }

  // Read RTC and set system clock if RTC clock valid
  rtc_initialize();

  if (RTC_valid) {
    Output("RTC: Valid");
  }
  else {
    Output("RTC: Not Valid");
  }

  rtc_timestamp();
  sprintf (msgbuf, "%s", timestamp);
  Output(msgbuf);
  delay (2000);

  // Dallas Sensor
  dallas_sensor_init();

  // Adafruit i2c Sensors
  bmx_initialize();
  mcp9808_initialize();
}

/*
 * =======================================================================================================================
 * loop()
 * =======================================================================================================================
 */
void loop()
{
  // RTC not set, Get Time for User
  if (!RTC_valid) {
    static bool first = true;

    delay (1000);
      
    if (first) {
      if (digitalRead(SCE_PIN) != LOW) {
        Serial.begin(9600);
        SerialConsoleEnabled = true;
      }  
    
      Output("SET RTC ENTER:");
      Output("YYYY:MM:DD:HH:MM:SS");
      first = false;
    }
    
    if (rtc_readserial()) { // check for serial input, validate for rtc, set rtc, report result
      Output("!!!!!!!!!!!!!!!!!!!");
      Output("!!! Press Reset !!!");
      Output("!!!!!!!!!!!!!!!!!!!");

      while (true) {
        delay (1000);
      }
    }
  }

  //Calibration mode, You can also reset the RTC here
  else if (countdown && digitalRead(SCE_PIN) == LOW) { 
    // Every minute, Do observation (don't save to SD) and transmit - So we can test LoRa
    I2C_Check_Sensors();
    
    if ( (countdown%60) == 0) { 
      OBS_Do(false);
      sprintf (Buffer32Bytes, "NO:%ds", seconds_to_next_obs());
      Output (Buffer32Bytes);
    }
    
    if (BMX_1_exists || BMX_2_exists) {
      StationMonitor();
    }
    else {
      float batt = vbat_get();
      if (ds_found) {
        getDSTemp();
      }
      sprintf (Buffer32Bytes, "S:%3d T:%d.%02d %d.%02d %04X", 
        (int) analogRead(SGAUGE_PIN),    // Pins are 10bit resolution (0-1023)
        (int)ds_reading, (int)(ds_reading*100)%100,
        (int)batt, (int)(batt*100)%100,
        SystemStatusBits); 
      Output (Buffer32Bytes);
    }
    
    // check for input sting, validate for rtc, set rtc, report result
    if (Serial.available() > 0) {
      rtc_readserial(); // check for serial input, validate for rtc, set rtc, report result
    }
    
    countdown--;
    delay (1000);
  }

  // Normal Operation
  else {
    I2C_Check_Sensors();
    OBS_Do(true);

    // Shutoff System Status Bits related to initialization after we have logged first observation
    JPO_ClearBits();
    
    Output("Going to Sleep");
    
    delay(2000);    
    OLED_sleepDisplay();

    // At this point we need to determine seconds to next 0, 15, 30, or 45 minute window
    
    LowPower.sleep(seconds_to_next_obs()*1000); // uses milliseconds
 
    OLED_wakeDisplay();   // May need to toggle the Display reset pin.
    delay(2000);
    OLED_ClearDisplayBuffer(); 
    Output("Wakeup");
  }
}
