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
 *                    
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
#define VERSION_INFO "SSGFALMO-230827"
#define W4SC false   // Set true to Wait for Serial Console to be connected

#include <OneWire.h>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoLowPower.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
 * ======================================================================================================================
 *  Quality Control - Min and Max Sensor Values on Surface of the Earth
 * ======================================================================================================================
 */
// Temperature
#define QC_MIN_T       -40.0     // deg C - Min Recorded -89.2°C (-128.6°F), -40.0 is sensor limit
#define QC_MAX_T       60.0      // deg C - Max Recorded 56.7°C (134°F)
#define QC_ERR_T       -999.9    // deg C Error

// Preasure - We are not adjusting for altitude, record min/max values are adjusted.
#define QC_MIN_P       300.0     // hPa - Min Recorded 652.5 mmHg 869.93hPa
#define QC_MAX_P       1100.0    // hPa - Max Recorded 1083.8mb aka 1083.8hPa
#define QC_ERR_P       -999.9    // hPa Error

// Relative Humidity
#define QC_MIN_RH      0.0       // %
#define QC_MAX_RH      100.0     // %
#define QC_ERR_RH      -999.9    // Relative Humidity Error

// Sensor SI1145
#define QC_MIN_IR      0.0       // unitless
#define QC_MAX_IR      16000.0   // unitless - based on the maximum readings from 43 stations located in tropics
#define QC_ERR_IR      -999.9    // Infrared Light Error
#define QC_MIN_VI      0.0       // unitless
#define QC_MAX_VI      2000.0    // unitless
#define QC_ERR_VI      -999.9    // Visual Light Error
#define QC_MIN_UV      0.0       // unitless
#define QC_MAX_UV      1000.0    // unitless
#define QC_ERR_UV      -999.9    // UV Light Error

// Sensor VEML7700 - Ambient Light Sensor
#define QC_MIN_LX      0         // lx
#define QC_MAX_LX      120000    // lx - based on sensor spec
#define QC_ERR_LX      -999      // Ambient Light Error

// Wind Speed  - world-record surface wind speed measured on Mt. Washington on April 12, 1934 (231 mph or 103 mps)
#define QC_MIN_WS      0.0       // m/s
#define QC_MAX_WS      103.0     // m/s
#define QC_ERR_WS      -999.9    // Relative Humidity Error

// Wind Direction
#define QC_MIN_WD      0         // deg
#define QC_MAX_WD      360       // deg
#define QC_ERR_WD      -999      // deg Error

// Rain Gauge 1 minute measurement 
#define QC_MIN_RG      0         // mm
#define QC_MAX_RG      30.0      // mm based on the world-record 1-minute rainfall in Maryland in 1956 (31.24 mm or 1.23")
#define QC_ERR_RG      -999.9    // Rain Gauge Error

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
 *  Serial Console Enable
 * ======================================================================================================================
 */
int  SCE_PIN = A4;
bool SerialConsoleEnabled = false;  // Variable for serial monitor control

/*
 * =======================================================================================================================
 *  Measuring Battery - SEE https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/power-management
 * =======================================================================================================================
 */
#define VBATPIN      A7

/*
 * =======================================================================================================================
 *  Stream / Snow Gauge
 * =======================================================================================================================
 */
#define SGAUGE_PIN     A3
#define SG_BUCKETS      60

unsigned int sg_bucket = 0;
unsigned int sg_buckets[SG_BUCKETS];

/*
 * ======================================================================================================================
 *  MCP9808 - I2C - Temperature sensor
 * 
 * I2C Address is:  0011,A2,A1,A0
 *                  0011000 = 0x18  where A2,1,0 = 0 MCP9808_I2CADDR_DEFAULT  
 *                  0011001 = 0x19  where A0 = 1
 * ======================================================================================================================
 */
#define MCP_ADDRESS_1     0x18
#define MCP_ADDRESS_2     0x19        // A0 set high, VDD
Adafruit_MCP9808 mcp1;
Adafruit_MCP9808 mcp2;
bool MCP_1_exists = false;
bool MCP_2_exists = false;

/*
 * =======================================================================================================================
 *  One Wire
 * =======================================================================================================================
 */
#define DS0_PIN A2

// Allociation for Dallas Sensors attached
OneWire  ds = OneWire(DS0_PIN);

// Dallas Sensor Address
byte ds_addr[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

bool  ds_found = false;
float ds_reading = 0.0;
bool  ds_valid = false;

/*
 * ======================================================================================================================
 *  RTC Setup
 * ======================================================================================================================
 */
RTC_DS3231 rtc;
DateTime now;
char timestamp[32];
bool RTC_valid = false;
bool RTC_exists = false;

/*
 * ======================================================================================================================                         
 *  OLED Display 
 * ======================================================================================================================
 */
bool DisplayEnabled = true;
#define DISPLAY_TYPE 32  // Set Display Type: 32 = 4 lines, 64 = 8 lines

#define SCREEN_WIDTH 128 // OLED display width, in pixels

#if (DISPLAY_TYPE == 32)
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_I2C_ADDRESS 0x3C // 128x32
#define DISPLAY_LINES 4 
#else
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_I2C_ADDRESS 0x3D // 128x64
#define DISPLAY_LINES 8
#endif

char oled_lines[DISPLAY_LINES][23];
#define OLED_RESET A5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)
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
 *  BMX280 humidity - I2C - Temperature, pressure sensor & altitude - Support 2 of any combination
 * 
 *  https://www.asknumbers.com/PressureConversion.aspx
 *  Pressure is returned in the SI units of Pascals. 100 Pascals = 1 hPa = 1 millibar. 
 *  Often times barometric pressure is reported in millibar or inches-mercury. 
 *  For future reference 1 pascal = 0.000295333727 inches of mercury, or 1 inch Hg = 3386.39 Pascal. 
 *
 *  Looks like you divide by 100 and you get millibars which matches NWS page
 * 
 *  Surface Observations and Station Elevation 
 *  https://forecast.weather.gov/product.php?issuedby=BOU&product=OSO&site=bou 
 * ======================================================================================================================
 */
#define BMX_STATION_ELEVATION 1017.272  // default 1013.25
#define BMX_ADDRESS_1         0x77        // BMP Default Address - Connecting SDO to GND will change BMP to 0x76
#define BMX_ADDRESS_2         0x76        // BME Default Address - Connecting SDO to GND will change BME to 0x77
#define BMP280_CHIP_ID        0x58
#define BME280_BMP390_CHIP_ID 0x60
#define BMP388_CHIP_ID        0x50
#define BMX_TYPE_UNKNOWN      0
#define BMX_TYPE_BMP280       1
#define BMX_TYPE_BME280       2
#define BMX_TYPE_BMP388       3
#define BMX_TYPE_BMP390       4
Adafruit_BMP280 bmp1;
Adafruit_BMP280 bmp2;
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
Adafruit_BMP3XX bm31;
Adafruit_BMP3XX bm32;
byte BMX_1_chip_id = 0x00;
byte BMX_2_chip_id = 0x00;
bool BMX_1_exists = false;
bool BMX_2_exists = false;
byte BMX_1_type=BMX_TYPE_UNKNOWN;
byte BMX_2_type=BMX_TYPE_UNKNOWN;

/*
 *======================================================================================================================
 * myswap()
 *======================================================================================================================
 */
void myswap(unsigned int *p, unsigned int *q) {
  int t;
  
  t=*p;
  *p=*q;
  *q=t;
}

/*
 *======================================================================================================================
 * mysort()
 *======================================================================================================================
 */
void mysort(unsigned int a[], int n)
{
  unsigned int i,j,temp;

  for(i = 0;i < n-1;i++) {
    for(j = 0;j < n-i-1;j++) {
      if(a[j] > a[j+1])
        myswap(&a[j],&a[j+1]);
    }
  }
}

/*
 * =======================================================================================================================
 * isnumeric() - check if string contains all digits
 * =======================================================================================================================
 */
bool isnumeric(char *s) {
  for (int i=0; i< strlen(s); i++) {
    if (!isdigit(*(s+i)) ) {
      return(false);
    }
  }
  return(true);
}

/* 
 *=======================================================================================================================
 * get_Bosch_ChipID ()  -  Return what Bosch chip is at specified address
 *   Chip ID BMP280 = 0x58 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)  
 *   Chip ID BME280 = 0x60 temp, preasure, humidity - I2C ADDRESS 0x77  (SD0 to GND = 0x76)  Register 0xE0 = Reset
 *   Chip ID BMP388 = 0x50 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *   Chip ID BMP390 = 0x60 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *=======================================================================================================================
 */
byte get_Bosch_ChipID (byte address) {
  byte chip_id = 0;
  byte error;

  Output ("get_Bosch_ChipID()");
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.

  // Important! Need to check the 0x00 register first. Doing a 0x0D (not chip id loaction) on a bmp388 
  // will return a value that could match one of the IDs 

  // Check Register 0x00
  sprintf (msgbuf, "  I2C:%02X Reg:%02X", address, 0x00);
  Output (msgbuf);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x00);  // BM3 CHIPID REGISTER
  error = Wire.endTransmission();
    //  0:success
    //  1:data too long to fit in transmit buffer
    //  2:received NACK on transmit of address
    //  3:received NACK on transmit of data
    //  4:other error 
  if (error) {
    sprintf (msgbuf, "  ERR_ET:%d", error);
    Output (msgbuf);
  }
  else if (Wire.requestFrom(address, 1)) {  // Returns the number of bytes returned from the slave device 
    chip_id = Wire.read();
    if (chip_id == BMP280_CHIP_ID) { // 0x58
      sprintf (msgbuf, "  CHIPID:%02X BMP280", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!
    }
    else if (chip_id == BMP388_CHIP_ID) {
      sprintf (msgbuf, "  CHIPID:%02X BMP388", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else if (chip_id == BME280_BMP390_CHIP_ID) {  // 0x60
      sprintf (msgbuf, "  CHIPID:%02X BME/390", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else {
      sprintf (msgbuf, "  CHIPID:%02X InValid", chip_id);
      Output (msgbuf);      
    }
  }
  else {
    sprintf (msgbuf, "  ERR_RF:0");
    Output (msgbuf);
  }

  // Check Register 0xD0
  chip_id = 0;
  sprintf (msgbuf, "  I2C:%02X Reg:%02X", address, 0xD0);
  Output (msgbuf);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0xD0);  // BM2 CHIPID REGISTER
  error = Wire.endTransmission();
    //  0:success
    //  1:data too long to fit in transmit buffer
    //  2:received NACK on transmit of address
    //  3:received NACK on transmit of data
    //  4:other error 
  if (error) {
    sprintf (msgbuf, "  ERR_ET:%d", error);
    Output (msgbuf);
  }
  else if (Wire.requestFrom(address, 1)) {  // Returns the number of bytes returned from the slave device 
    chip_id = Wire.read(); 
    if (chip_id == BMP280_CHIP_ID) {  // 0x58
      sprintf (msgbuf, "  CHIPID:%02X BMP280", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!
    }
    else if (chip_id == BMP388_CHIP_ID) {  // 0x50
      sprintf (msgbuf, "  CHIPID:%02X BMP388", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else if (chip_id == BME280_BMP390_CHIP_ID) {  // 0x60
      sprintf (msgbuf, "  CHIPID:%02X BME/390", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else {
      sprintf (msgbuf, "  CHIPID:%02X InValid", chip_id);
      Output (msgbuf);   
    }
  }
  else {
    sprintf (msgbuf, "  ERR_RF:0");
    Output (msgbuf);
  }
  return(0);
}

/*
 * ======================================================================================================================
 * Blink() - Count, delay between, delay at end
 * ======================================================================================================================
 */
void Blink(int count, int between)
{
  int c;

  for (c=0; c<count; c++) {
    digitalWrite(LED_PIN, HIGH);
    delay(between);
    digitalWrite(LED_PIN, LOW);
    delay(between);
  }
}

/* 
 *=======================================================================================================================
 * rtc_timestamp() - Read from RTC and set timestamp string
 *=======================================================================================================================
 */
void rtc_timestamp() {
  now = rtc.now(); //get the current date-time

  // ISO_8601 Time Format
  sprintf (timestamp, "%d-%02d-%02dT%02d:%02d:%02d", 
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());
}

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
 * ======================================================================================================================
 * OLED_sleepDisplay()
 * ======================================================================================================================
 */
void OLED_sleepDisplay() {
  if (DisplayEnabled) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

/*
 * ======================================================================================================================
 * OLED_wakeDisplay()
 * ======================================================================================================================
 */
void OLED_wakeDisplay() {
  if (DisplayEnabled) {
    display.ssd1306_command(SSD1306_DISPLAYON);
  }
}

/*
 * ======================================================================================================================
 * OLED_ClearDisplayBuffer() -- Clear display buffer with spaces
 * ======================================================================================================================
 */
void OLED_ClearDisplayBuffer() {
  int r,c;
  
  for (r=0; r<DISPLAY_LINES; r++) {
    for (c=0; c<22; c++) {
      oled_lines [r][c] = ' ';
    }
    oled_lines [r][c] = (char) NULL;
  }
}

/*
 * ======================================================================================================================
 * OLED_update() -- Output oled in memory map to display
 * ======================================================================================================================
 */
void OLED_update() {  
  if (DisplayEnabled) {
    display.clearDisplay();
    display.setCursor(0,0);             // Start at top-left corner
    display.print(oled_lines [0]);
    display.setCursor(0,8);
    display.print(oled_lines [1]);
    display.setCursor(0,16);
    display.print(oled_lines [2]);
    display.setCursor(0,24);  
    display.print(oled_lines [3]);
    
    if (DISPLAY_LINES == 8) {
      display.setCursor(0,32);  
      display.print(oled_lines [4]);
      display.setCursor(0,40);  
      display.print(oled_lines [5]);
      display.setCursor(0,48);  
      display.print(oled_lines [6]);
      display.setCursor(0,56);  
      display.print(oled_lines [7]);     
    }
    display.display();
  }
}

/*
 * ======================================================================================================================
 * OLED_write() 
 * ======================================================================================================================
 */
void OLED_write(const char *str) {
  int c, len, bottom_line = 3;
  
  if (DisplayEnabled) {
    // move lines up
    for (c=0; c<=21; c++) {
      oled_lines [0][c] = oled_lines [1][c];
      oled_lines [1][c] = oled_lines [2][c];
      oled_lines [2][c] = oled_lines [3][c];
      if (DISPLAY_LINES == 8) {
        oled_lines [3][c] = oled_lines [4][c];
        oled_lines [4][c] = oled_lines [5][c];
        oled_lines [5][c] = oled_lines [6][c];  
        oled_lines [6][c] = oled_lines [7][c];  
        bottom_line = 7;          
      }
    }

    // check length on new output line string
    len = strlen (str);
    if (len>21) {
      len = 21;
    }
    for (c=0; c<=len; c++) {
      oled_lines [bottom_line][c] = *(str+c);
    }

    // Adding Padding
    for (;c<=21; c++) {
      oled_lines [bottom_line][c] = ' ';
    }
    oled_lines [bottom_line][22] = (char) NULL;
    
    OLED_update();
  }
}

/*
 * ======================================================================================================================
 * OLED_write_noscroll() -- keep lines 1-3 and output on line 4
 * ======================================================================================================================
 */
void OLED_write_noscroll(const char *str) {
  int c, len, bottom_line = 3;

  if (DISPLAY_LINES == 8) {
    bottom_line = 7;
  }
  
  if (DisplayEnabled) {
    len = strlen (str);
    if (len>21) {
      len = 21;
    }
    
    for (c=0; c<=len; c++) {
      oled_lines [bottom_line][c] = *(str+c);
    }

    // Adding Padding
    for (;c<=21; c++) {
      oled_lines [bottom_line][c] = ' ';
    }
    oled_lines [bottom_line][22] = (char) NULL;
    
    OLED_update();
  }
}

/*
 * ======================================================================================================================
 * Serial_write() 
 * ======================================================================================================================
 */
void Serial_write(const char *str) {
  if (SerialConsoleEnabled) {
    Serial.println(str);
  }
}

/*
 * ======================================================================================================================
 * Output() - Count, delay between, delay at end
 * ======================================================================================================================
 */
void Output(const char *str) {
  OLED_write(str);
  Serial_write(str);
}

/*
 * ======================================================================================================================
 * OutputNS() - Output with no scroll on oled
 * ======================================================================================================================
 */
void OutputNS(const char *str) {
  OLED_write_noscroll(str);
  Serial_write(str);
}

/*
 *=======================================================================================================================
 * vbat_get() -- return battery voltage
 *=======================================================================================================================
 */
float vbat_get() {
  float v = analogRead(VBATPIN);
  v *= 2;    // we divided by 2, so multiply back
  v *= 3.3;  // Multiply by 3.3V, our reference voltage
  v /= 1024; // convert to voltage
  return (v);
}

/* 
 *=======================================================================================================================
 * rtc_initialize()
 *=======================================================================================================================
 */
void rtc_initialize() {

  if (!rtc.begin()) { // Always returns true
     Output("ERR:RTC NOT FOUND");
     SystemStatusBits |= SSB_RTC; // Turn on Bit
     return;
  }
  
  if (!I2C_Device_Exist(RTC_I2C_ADDRESS)) {
    Output("ERR:RTC-I2C NOTFOUND");
    SystemStatusBits |= SSB_RTC; // Turn on Bit
    delay (5000);
    return;
  }

  RTC_exists = true; // We have a clock hardware connected

  rtc_timestamp();
  sprintf (msgbuf, "%s*", timestamp);
  Output (msgbuf);

  // Do a validation check on the year. 
  // Asumption is: If RTC not set, it will not have the current year.

  if ((now.year() >= 2022) && (now.year() <= 2031)) {
    now = rtc.now();
    RTC_valid = true;
  }
  else {
    Output ("NEED TIME->RTC");
  }
}

/*
 * =======================================================================================================================
 * rtc_readserial() - // check for serial input, validate for rtc, set rtc, report result
 * =======================================================================================================================
 */
bool rtc_readserial()
{
  boolean ready = false;
  int cnt = 0;
  char buffer[32];
  char *p, *token;
  int year, month, day, hour, minute, second;
  
  while (Serial.available()) {
    char c = Serial.read();
    buffer[cnt++] = c;
    if ((c == '\n') || (cnt == 31) ){
      buffer[cnt] = '\0';  // Note: there will be a \r\n on end of string in buffer
      cnt = 0;
      Serial.flush(); // if anything left in the Serial buffer, get rid of it

      // Validate User input for a good date and time
      p = &buffer[0];
      token = strtok_r(p, ":", &p);
      if (isnumeric(token) && (year = atoi (token)) && (year >= 2022) && (year <= 2031) ) {   // FOO set back 2022
        token = strtok_r(p, ":", &p);
        if (isnumeric(token) && (month = atoi (token)) && (month >= 1) && (month <= 12) ) {
          token = strtok_r(p, ":", &p);        
          if (isnumeric(token) && (day = atoi (token)) && 
               (
                 ( (day>=1  && day<=31) && (month==1 || month==3 || month==5 || month==7 || month==8 || month==10 || month==12) ) ||
                 ( (day>=1  && day<=30) && (month==4 || month==6 || month==9 || month==11) ) ||
                 ( (day>=1  && day<=28) && (month==2) ) ||
                 ( (day==29)            && (month==2) && ( (year%400==0) || ( (year%4==0) && (year%100!=0) ) ) )
                ) 
             ) {
            token = strtok_r(p, ":", &p);
            hour = atoi (token);
            if ( (isnumeric(token) && (hour >= 0) && (hour <= 23)) ) {
              token = strtok_r(p, ":", &p);
              minute = atoi (token);
              if ( (isnumeric(token) && (minute >= 0) && (minute <= 59)) ) {
                token = strtok_r(p, "\r", &p);
                second = atoi (token);
                if ( (isnumeric(token) && (second >= 0) && (second <= 59)) ) { 
                  sprintf (msgbuf, ">%d.%d.%d.%d.%d.%d", 
                     year, month, day, hour, minute, second);
                  rtc.adjust(DateTime(year, month, day, hour, minute, second));
                  Output("RTC: Set");
                  RTC_valid = true;
                  rtc_timestamp();
                  sprintf (msgbuf, "%s=", timestamp);
                  Output (msgbuf);
                  return(true);
                }
                else {
                  sprintf (msgbuf, "Invalid Second: %s", token);
                  Output(msgbuf);
                  return(false);
                }
              }
              else {
                sprintf (msgbuf, "Invalid Minute: %s", token);
                Output(msgbuf);
                return(false);
              }
            }
            else {
              sprintf (msgbuf, "Invalid Hour: %s", token);
              Output(msgbuf);
              return(false);
            }
          }
          else {
            sprintf (msgbuf, "Invalid Day: %s", token);
            Output(msgbuf);
            return(false);
          }
        }
        else {
          sprintf (msgbuf, "Invalid Month: %s", token);
          Output(msgbuf);
          return(false);
        }                
      }
      else {
        sprintf (msgbuf, "Invalid Year: %s", token);
        Output(msgbuf);
        return(false);
      }
    } // if line
  } // while
  return(false);
}

/* 
 *=======================================================================================================================
 * SD_initialize()
 *=======================================================================================================================
 */
void SD_initialize() {
  if (!SD.begin(SD_ChipSelect)) {
    Output ("SD:NF");
    SystemStatusBits |= SSB_SD;
    delay (5000);
  }
  else {
    SD_exists = true;
    if (!SD.exists(SD_obsdir)) {
      if (SD.mkdir(SD_obsdir)) {
        Output ("SD:MKDIR OBS OK");
        Output ("SD:Online");
        SD_exists = true;
      }
      else {
        Output ("SD:MKDIR OBS ERR");
        Output ("SD:Offline");
        SystemStatusBits |= SSB_SD;  // Turn On Bit     
      } 
    }
    else {
      Output ("SD:Online");
      Output ("SD:OBS DIR Exists");
      SD_exists = true;
    }
  }
}

/* 
 *=======================================================================================================================
 * SD_LogObservation()
 *=======================================================================================================================
 */
void SD_LogObservation(char *observations) {
  char SD_logfile[24];
  File fp;

  if (!SD_exists) {
    return;
  }

  if (!RTC_valid) {
    return;
  }

  // Note: "now" is global and is set when ever timestamp() is called. Value last read from RTC.
  sprintf (SD_logfile, "%s/%4d%02d%02d.log", SD_obsdir, now.year(), now.month(), now.day());

  Output (SD_logfile);
  
  fp = SD.open(SD_logfile, FILE_WRITE); 
  if (fp) {
    fp.println(observations);
    fp.close();
    SystemStatusBits &= ~SSB_SD;  // Turn Off Bit
    Output ("OBS Logged to SD");
  }
  else {
    SystemStatusBits |= SSB_SD;  // Turn On Bit - Note this will be reported on next observation
    Output ("OBS Open Log Err");
    // At thins point we could set SD_exists to false and/or set a status bit to report it
    // SD_initialize();  // Reports SD NOT Found. Library bug with SD
  }
}

/*
 * =============================================================
 * getDSTempByAddr() - 
 * =============================================================
 */
bool getDSTempByAddr(int delayms) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  
  ds.reset();
  ds.select(ds_addr);

  // start conversion, with parasite power on at the end
  ds.write(0x44,0); // set to 1 for parasite otherwise 0
  
  delay(delayms);     // maybe 750ms is enough, maybe not
  
  present = ds.reset();
  ds.select(ds_addr);    
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
    
  if (OneWire::crc8(data, 8) != data[8]) {
    // CRC on the Data Read
    
    // Return false no temperture because of the CRC error
    ds_reading=0.0;
    ds_valid = false;
  }
  else {
    // convert the data to actual temperature
    unsigned int raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9bit res, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time

    float t = raw / 16.0;  // Max 85.0C, for fahrenheit = (raw / 16.0) * 1.8 + 32.0   or Max 185.00F
    ds_reading = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    
    if (ds_reading != QC_ERR_T) { 
      ds_valid = true;
    }
    else {
      ds_valid = false;
      ds_reading = QC_ERR_P;
    }
    
    // Have a temp,  but it might have value 85.00C / 185.00F which means it was just plugged in
  }
  return(ds_valid);
}

/*
 * =============================================================
 * getDSTemp()
 * =============================================================
 */
void getDSTemp() {
  int status = getDSTempByAddr(250);
  if (status == false) {
    // reread temp - it might of just been plugged in
    status = getDSTempByAddr(750);
  }

  // temperture returned in ds_reading
  // state returned in ds_valid
  // if false and temp was equal to 0.0 then a CRC happened
    
  /*
  if (status) { // Good Value Read
    sprintf (msgbuf, "%d TS %d.%02d OK", 
      probe, (int)ds_reading, (int)(ds_reading*100)%100);
    Output (msgbuf);
  }
  else { // We read a temp but it was a bad value
    sprintf (msgbuf, "%d TS %d.%02d BAD", 
      probe, (int)ds_reading, (int)(ds_reading*100)%100);
    Output (msgbuf);
  }
  */
}

/*
 * =============================================================
 * Scan1WireBus() - Get Sensor Address and Read Temperature
 * =============================================================
 */
bool Scan1WireBus() {
  byte i;
  byte present = 0;
  byte type_s;
  byte addr[8];
  bool found = false;
  
  // Reset and Start search
  ds.reset_search();
  delay(250);

  // Expecting one and only one probe on the pin
  
  if (!ds.search(ds_addr)) {
    // Sensor Not Found
    sprintf (msgbuf, "DS NF");
    Output (msgbuf);
  }
  else if (OneWire::crc8(ds_addr, 7) != ds_addr[7]) {
    // Bad CRC
    sprintf (msgbuf, "DS CRC"); // Bad CRC
    Output (msgbuf);
  }
  else if (ds_addr[0] != 0x28) { // DS18B20
    // Unknown Device Type
    sprintf (msgbuf, "DS UKN %d", ds_addr[0]); // Unknown Device type
    Output (msgbuf);
  } 
  else {
    ds_found = true;
    sprintf (msgbuf, "DS %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", 
       ds_addr[0], ds_addr[1], ds_addr[2], ds_addr[3],
       ds_addr[4], ds_addr[5], ds_addr[7], ds_addr[7]); 
    Output (msgbuf);
    found = true;
  }
  return (found);
}

/*
 *=======================================================================================================================
 * dallas_sensor_init - Dallas Sensor initialize
 *=======================================================================================================================
 */
void dallas_sensor_init() {
  ds_found = Scan1WireBus();  // Look for Dallas Sensor on pin get it's address   
  if (!ds_found) {
    // Retry
    delay (250);
    ds_found = Scan1WireBus();
    if (!ds_found) {
      SystemStatusBits |= SSB_DS_1;  // Turn On Bit
    }
  }  
  if (ds_found) {
    getDSTemp();
    if (ds_valid) { // Good Value Read
      sprintf (msgbuf, "DS %d.%02d OK", (int)ds_reading, (int)(ds_reading*100)%100);
    }
    else { // We read a temp but it was a bad value
      sprintf (msgbuf, "DS %d.%02d BAD", (int)ds_reading, (int)(ds_reading*100)%100);
    }
    Output (msgbuf);
  }
}

/* 
 *=======================================================================================================================
 * bmx_initialize() - Bosch sensor initialize
 *=======================================================================================================================
 */
void bmx_initialize() {
  Output("BMX:INIT");
  
  // 1st Bosch Sensor - Need to see which (BMP, BME, BM3) is plugged in
  BMX_1_chip_id = get_Bosch_ChipID(BMX_ADDRESS_1);
  switch (BMX_1_chip_id) {
    case BMP280_CHIP_ID :
      if (!bmp1.begin(BMX_ADDRESS_1)) { 
        msgp = (char *) "BMP1 ERR";
        BMX_1_exists = false;
        SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BMP280;
        msgp = (char *) "BMP1 OK";
      }
    break;

    case BME280_BMP390_CHIP_ID :
      if (!bme1.begin(BMX_ADDRESS_1)) { 
        if (!bm31.begin_I2C(BMX_ADDRESS_1)) {  // Perhaps it is a BMP390
          msgp = (char *) "BMX1 ERR";
          BMX_1_exists = false;
          SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
        }
        else {
          BMX_1_exists = true;
          BMX_1_type = BMX_TYPE_BMP390;
          msgp = (char *) "BMP390_1 OK";         
        }      
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BME280;
        msgp = (char *) "BME280_1 OK";
      }
    break;

    case BMP388_CHIP_ID :
      if (!bm31.begin_I2C(BMX_ADDRESS_1)) { 
        msgp = (char *) "BM31 ERR";
        BMX_1_exists = false;
        SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BMP388;
        msgp = (char *) "BM31 OK";
      }
    break;

    default:
      msgp = (char *) "BMX_1 NF";
    break;
  }
  Output (msgp);

  // 2nd Bosch Sensor - Need to see which (BMP, BME, BM3) is plugged in
  BMX_2_chip_id = get_Bosch_ChipID(BMX_ADDRESS_2);
  switch (BMX_2_chip_id) {
    case BMP280_CHIP_ID :
      if (!bmp1.begin(BMX_ADDRESS_2)) { 
        msgp = (char *) "BMP2 ERR";
        BMX_2_exists = false;
        SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BMP280;
        msgp = (char *) "BMP2 OK";
      }
    break;

    case BME280_BMP390_CHIP_ID :
      if (!bme2.begin(BMX_ADDRESS_2)) { 
        if (!bm31.begin_I2C(BMX_ADDRESS_2)) {  // Perhaps it is a BMP390
          msgp = (char *) "BMX2 ERR";
          BMX_2_exists = false;
          SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
        }
        else {
          BMX_2_exists = true;
          BMX_2_type = BMX_TYPE_BMP390;
          msgp = (char *) "BMP390_2 OK";          
        }
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BME280;
        msgp = (char *) "BME280_2 OK";
      }
    break;

    case BMP388_CHIP_ID :
      if (!bm31.begin_I2C(BMX_ADDRESS_2)) { 
        msgp = (char *) "BM31 ERR";
        BMX_2_exists = false;
        SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BMP388;
        msgp = (char *) "BM31 OK";
      }
    break;

    default:
      msgp = (char *) "BMX_2 NF";
    break;
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * mcp9808_initialize() - MCP9808 sensor initialize
 *=======================================================================================================================
 */
void mcp9808_initialize() {
  Output("MCP9808:INIT");
  
  // 1st MCP9808 Precision I2C Temperature Sensor (I2C ADDRESS = 0x18)
  mcp1 = Adafruit_MCP9808();
  if (!mcp1.begin(MCP_ADDRESS_1)) {
    msgp = (char *) "MCP1 NF";
    MCP_1_exists = false;
    SystemStatusBits |= SSB_MCP_1;  // Turn On Bit
  }
  else {
    MCP_1_exists = true;
    msgp = (char *) "MCP1 OK";
  }
  Output (msgp);
}


/* 
 *=======================================================================================================================
 * s_gauge_median()
 *=======================================================================================================================
 */
unsigned int s_gauge_median() {
  int i;

  for (i=0; i<SG_BUCKETS; i++) {
    // delay(500);
    delay(250);
    sg_buckets[i] = (int) analogRead(SGAUGE_PIN);
    // sprintf (Buffer32Bytes, "SG[%02d]:%d", i, sg_buckets[i]);
    // OutputNS (Buffer32Bytes);
  }
  
  mysort(sg_buckets, SG_BUCKETS);
  i = (SG_BUCKETS+1) / 2 - 1; // -1 as array indexing in C starts from 0
  
  return (sg_buckets[i]); // Pins are 10bit resolution (0-1023)
}

/*
 * ======================================================================================================================
 * OBS_Do() - Collect Observations, Build message, Send to logging site
 * ======================================================================================================================
 */
void OBS_Do (bool log_obs) {
  float bmx1_pressure = 0.0;
  float bmx1_temp = 0.0;
  float bmx1_humid = 0.0;
  float bmx2_pressure = 0.0;
  float bmx2_temp = 0.0;
  float bmx2_humid = 0.0;
  float mcp1_temp = 0.0;
  float batt = 0.0;
  int msgLength;
  unsigned short checksum;

  // Safty Check for Vaild Time
  if (!RTC_valid) {
    Output ("OBS_Do: Time NV");
    return;
  }

  Output ("OBS_Do()");
 
  // Take multiple readings and return the median, 15s spent reading guage
  int SG_Median = s_gauge_median();
  
  //
  // Add I2C Sensors
  //
  if (BMX_1_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_1_chip_id == BMP280_CHIP_ID) {
      p = bmp1.readPressure()/100.0F;       // bp1 hPa
      t = bmp1.readTemperature();           // bt1
    }
    else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_1_type == BMX_TYPE_BME280) {
        p = bme1.readPressure()/100.0F;     // bp1 hPa
        t = bme1.readTemperature();         // bt1
        h = bme1.readHumidity();            // bh1
      }
      if (BMX_1_type == BMX_TYPE_BMP390) {
        p = bm31.readPressure()/100.0F;     // bp1 hPa
        t = bm31.readTemperature();         // bt1       
      }
    }
    else { // BMP388
      p = bm31.readPressure()/100.0F;       // bp1 hPa
      t = bm31.readTemperature();           // bt1
    }
    bmx1_pressure = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
    bmx1_temp     = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    bmx1_humid    = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (BMX_2_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_2_chip_id == BMP280_CHIP_ID) {
      p = bmp2.readPressure()/100.0F;       // bp2 hPa
      t = bmp2.readTemperature();           // bt2
    }
    else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_2_type == BMX_TYPE_BME280) {
        p = bme2.readPressure()/100.0F;     // bp2 hPa
        t = bme2.readTemperature();         // bt2
        h = bme2.readHumidity();            // bh2 
      }
      if (BMX_2_type == BMX_TYPE_BMP390) {
        p = bm32.readPressure()/100.0F;       // bp2 hPa
        t = bm32.readTemperature();           // bt2
      }      
    }
    else { // BMP388
      p = bm32.readPressure()/100.0F;       // bp2 hPa
      t = bm32.readTemperature();           // bt2
    }
    bmx2_pressure = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
    bmx2_temp     = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    bmx2_humid    = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (MCP_1_exists) {
    mcp1_temp = mcp1.readTempC();
    mcp1_temp = (isnan(mcp1_temp) || (mcp1_temp < QC_MIN_T)  || (mcp1_temp > QC_MAX_T))  ? QC_ERR_T  : mcp1_temp;
  }
  
  batt = vbat_get();

  // Set the time for this observation
  rtc_timestamp();
  if (log_obs) {
    Output(timestamp);
  }
  
  // Build JSON log entry by hand  
  // {"at":"2021-03-05T11:43:59","sg":49,"bp1":3,"bt1":97.875,"bh1":40.20,"bv":3.5,"hth":9}

  sprintf (msgbuf, "{\"at\":\"%s\",\"sg\":%d,", timestamp, SG_Median);
  if (BMX_1_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"bp1\":%u.%04d,\"bt1\":%d.%02d,\"bh1\":%d.%02d,",
      (int)bmx1_pressure, (int)(bmx1_pressure*100)%100,
      (int)bmx1_temp, (int)(bmx1_temp*100)%100,
      (int)bmx1_humid, (int)(bmx1_humid*100)%100);
  }
  if (BMX_2_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"bp2\":%u.%04d,\"bt2\":%d.%02d,\"bh2\":%d.%02d,",
      (int)bmx2_pressure, (int)(bmx2_pressure*100)%100,
      (int)bmx2_temp, (int)(bmx2_temp*100)%100,
      (int)bmx2_humid, (int)(bmx2_humid*100)%100);
  }
  if (MCP_1_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"mt1\":%d.%04d,", (int)mcp1_temp, (int)(mcp1_temp*100)%100);   
  }
  if (ds_found) {
    getDSTemp();
    sprintf (msgbuf+strlen(msgbuf), "\"dt1\":%d.%04d,", (int)ds_reading, (int)(ds_reading*100)%100);   
  }
  sprintf (msgbuf+strlen(msgbuf), "\"bv\":%d.%02d,\"hth\":%d}", 
    (int)batt, (int)(batt*100)%100, SystemStatusBits);

  // Log Observation to SD Card
  if (log_obs) {
    SD_LogObservation(msgbuf);
  }
  Serial_write (msgbuf);
}

/* 
 *=======================================================================================================================
 * I2C_Device_Exist - does i2c device exist at address
 * 
 *  The i2c_scanner uses the return value of the Write.endTransmisstion to see 
 *  if a device did acknowledge to the address.
 *=======================================================================================================================
 */
bool I2C_Device_Exist(byte address) {
  byte error;

  Wire.begin();                     // Connect to I2C as Master (no addess is passed to signal being a slave)

  Wire.beginTransmission(address);  // Begin a transmission to the I2C slave device with the given address. 
                                    // Subsequently, queue bytes for transmission with the write() function 
                                    // and transmit them by calling endTransmission(). 

  error = Wire.endTransmission();   // Ends a transmission to a slave device that was begun by beginTransmission() 
                                    // and transmits the bytes that were queued by write()
                                    // By default, endTransmission() sends a stop message after transmission, 
                                    // releasing the I2C bus.

  // endTransmission() returns a byte, which indicates the status of the transmission
  //  0:success
  //  1:data too long to fit in transmit buffer
  //  2:received NACK on transmit of address
  //  3:received NACK on transmit of data
  //  4:other error 

  // Partice Library Return values
  // SEE https://docs.particle.io/cards/firmware/wire-i2c/endtransmission/
  // 0: success
  // 1: busy timeout upon entering endTransmission()
  // 2: START bit generation timeout
  // 3: end of address transmission timeout
  // 4: data byte transfer timeout
  // 5: data byte transfer succeeded, busy timeout immediately after
  // 6: timeout waiting for peripheral to clear stop bit

  if (error == 0) {
    return (true);
  }
  else {
    // sprintf (msgbuf, "I2CERR: %d", error);
    // Output (msgbuf);
    return (false);
  }
}

/*
 * ======================================================================================================================
 * I2C_Check_Sensors() - See if each I2C sensor responds on the bus and take action accordingly             
 * ======================================================================================================================
 */
void I2C_Check_Sensors() {

  // BMX_1 Barometric Pressure 
  if (I2C_Device_Exist (BMX_ADDRESS_1)) {
    // Sensor online but our state had it offline
    if (BMX_1_exists == false) {
      if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
        if (bmp1.begin(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BMP1 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        } 
      }
      else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
        if (bme1.begin(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BME1 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        }          
      }
      else {
        if (bm31.begin_I2C(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BM31 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        }                  
      }      
    }
  }
  else {
    // Sensor offline but our state has it online
    if (BMX_1_exists == true) {
      BMX_1_exists = false;
      Output ("BMX1 OFFLINE");
      SystemStatusBits |= SSB_BMX_1;  // Turn On Bit 
    }    
  }

  // BMX_2 Barometric Pressure 
  if (I2C_Device_Exist (BMX_ADDRESS_2)) {
    // Sensor online but our state had it offline
    if (BMX_2_exists == false) {
      if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
        if (bmp2.begin(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BMP2 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        } 
      }
      else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
        if (bme2.begin(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BME2 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        }          
      }
      else {
         if (bm32.begin_I2C(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BM32 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        }                         
      }     
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (BMX_2_exists == true) {
      BMX_2_exists = false;
      Output ("BMX2 OFFLINE");
      SystemStatusBits |= SSB_BMX_2;  // Turn On Bit 
    }    
  }
}

/*
 * ======================================================================================================================
 * JPO_ClearBits() - Clear System Status Bits related to initialization
 * ======================================================================================================================
 */
void JPO_ClearBits() {
  if (JustPoweredOn) {
    JustPoweredOn = false;
    SystemStatusBits &= ~SSB_PWRON; // Turn Off Power On Bit
    SystemStatusBits &= ~SSB_OLED;  // Turn Off OLED Missing Bit
    SystemStatusBits &= ~SSB_BMX_1; // Turn Off BMX280_1 Not Found Bit
    SystemStatusBits &= ~SSB_BMX_2; // Turn Off BMX280_1 Not Found Bit
    SystemStatusBits &= ~SSB_MCP_1; // Turn Off MCP_1 Not Found Bit
    SystemStatusBits &= ~SSB_DS_1;  // Turn Off Dallas Sensor Not Found Bit
  }
}

/*
 * ======================================================================================================================
 * StationMonitor() - On OLED display station information
 * ======================================================================================================================
 */
void StationMonitor() {
  int r, c, len;
  
  float bmx_pressure = 0.0;
  float bmx_temp = 0.0;
  float bmx_humid = 0.0;
  char Buffer16Bytes[16];

  float batt = vbat_get();

  OLED_ClearDisplayBuffer();

  // =================================================================
  // Line 0 of OLED
  // =================================================================
  rtc_timestamp();
  len = (strlen (timestamp) > 21) ? 21 : strlen (timestamp);
  for (c=0; c<=len; c++) oled_lines [0][c] = *(timestamp+c);
  Serial_write (timestamp);

  // =================================================================
  // Line 1 of OLED
  // =================================================================
  if (BMX_1_exists) {
    if (BMX_1_chip_id == BMP280_CHIP_ID) {
      bmx_pressure = bmp1.readPressure()/100.0F;           // bmxp1
      bmx_temp = bmp1.readTemperature();                   // bmxt1
      bmx_humid = 0.0;
    }
    else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
      bmx_pressure = bme1.readPressure()/100.0F;           // bmxp1
      bmx_temp = bme1.readTemperature();                   // bmxt1
      bmx_humid = bme1.readHumidity();                     // bmxh1 
    }
    else { // BMP388
      bmx_pressure = bm31.readPressure()/100.0F;           // bmxp1 hPa
      bmx_temp = bm31.readTemperature();                   // bmxt1
      bmx_humid = 0.0;
    }
    sprintf (Buffer32Bytes, "%d.%02d %d.%02d %d.%02d", 
      (int)bmx_pressure, (int)(bmx_pressure*100)%100,
      (int)bmx_temp, (int)(bmx_temp*100)%100,
      (int)bmx_humid, (int)(bmx_humid*100)%100);
  }
  else {
    sprintf (Buffer32Bytes, "BMX:NF");
  }
  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [1][c] = *(Buffer32Bytes+c);
  Serial_write (Buffer32Bytes);

  // =================================================================
  // Line 2 of OLED
  // =================================================================
  if (BMX_2_exists) {
    if (BMX_2_chip_id == BMP280_CHIP_ID) {
      bmx_pressure = bmp2.readPressure()/100.0F;           // bmxp1
      bmx_temp = bmp2.readTemperature();                   // bmxt1
      bmx_humid = 0.0;
    }
    else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
      bmx_pressure = bme2.readPressure()/100.0F;           // bmxp1
      bmx_temp = bme2.readTemperature();                   // bmxt1
      bmx_humid = bme2.readHumidity();                     // bmxh1 
    }
    else { // BMP388
      bmx_pressure = bm32.readPressure()/100.0F;           // bmpx1 hPa
      bmx_temp = bm32.readTemperature();                   // bmtx1
      bmx_humid = 0.0;
    }
    sprintf (Buffer32Bytes, "%d.%02d %d.%02d %d.%02d", 
      (int)bmx_pressure, (int)(bmx_pressure*100)%100,
      (int)bmx_temp, (int)(bmx_temp*100)%100,
      (int)bmx_humid, (int)(bmx_humid*100)%100);
  }
  else {
    sprintf (Buffer32Bytes, "BMX:NF");
  }
  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [2][c] = *(Buffer32Bytes+c);
  Serial_write (Buffer32Bytes);
  
  // =================================================================
  // Line 3 of OLED
  // =================================================================
  sprintf (Buffer32Bytes, "SG:%3d %d.%02d %04X", 
    (int) analogRead(SGAUGE_PIN),    // Pins are 12bit resolution (0-1023)
    (int)batt, (int)(batt*100)%100,
    SystemStatusBits); 

  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [3][c] = *(Buffer32Bytes+c);
  Serial_write (Buffer32Bytes);

  OLED_update();
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

  // serial console enable pin
  pinMode(SCE_PIN, INPUT_PULLUP);   // Internal pullup resistor biases the pin to supply voltage.
                                    // If jumper set to ground, we enable serial console (low = enable)
  if (digitalRead(SCE_PIN) == LOW) {
    SerialConsoleEnabled = true;
  }

  // Set up gauge pin for reading 
  pinMode(SGAUGE_PIN, INPUT);

  if (DisplayEnabled) {
    if (I2C_Device_Exist (OLED_I2C_ADDRESS)) {
      display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
      display.clearDisplay();
      display.setTextSize(1); // Draw 2X-scale text
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      for (int r=0; r<DISPLAY_LINES; r++) {
        oled_lines[r][0]=0;
      }
      OLED_write("OLED:OK");
    }
    else {
      DisplayEnabled = false;
      SystemStatusBits |= SSB_OLED; // Turn on Bit
    }
  }

  if (SerialConsoleEnabled) {
    Serial.begin(9600);

    // Wait for serial port to be available - Uncomment for testing
#if W4SC
    if (!Serial) {
      OLED_write("Waiting Serial Console");
    }
    while(!Serial) {
      Blink(1, 1000);
    }
#endif

    Output (VERSION_INFO);
    delay (5000);      // Pause so user can see version if not waiting for serial

    if (DisplayEnabled) {
      Serial_write ("OLED:Enabled");
    }
    else {
      Serial_write ("OLED:Disabled");
    }
    Output ("SC:Enabled");
  }

  Output (VERSION_INFO); // Doing it one more time for the OLED

  // Initialize SD card if we have one.
  SD_initialize();

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
