PRODUCT_VERSION(28);
#define COPYRIGHT "Copyright [2024] [University Corporation for Atmospheric Research]"
#define VERSION_INFO "FNAC-20240911"

/*
 *======================================================================================================================
 * ICDP_FN_AC FEWSNet Always Connected
 *   Board Type : Particle BoronLTE
 *   Description: FEWSNet Always Connected transmission times of 15m
 *   Author: Robert Bubon
 *   Date:  2023-03-07 RJB Initial Development Based on FN-ULP and Full Station
 *          2023-05-05 RJB Added NAN check on htu1
 *          2023-05-10 RJB Moved Heartbeat to pin A1
 *          2023-05-25 RJB Rain gauge inturrupts turn on LED, Background work shuts it off
 *          2023-06-27 RJB Added Min Max Quality Controls to sensor values
 *          2023-07-10 RJB Added WITH_ACK flags to Particle.Publish()
 *          2023-07-11 RJB Added Daily Reboot
 *          2023-07-27 RJB Added Daily Rian Totals for Today and Prior Day @ 600 UTC
 *                         N2S File Position now save to EEPROM 
 *                         Daily Reboot now calls DO_OBS() before report. This will save to EEPROM 
 *                         rain collected since last observation.
 *          2023-07-31 RJB Fixed bug to get RTC updated from network time
 *          2023-12-20 RJB Version 17 / 18
 *                         EEPROM_UpdateRainTotals() now updates NVRAM even if no change.  
 *                         Need the timestamp to be updated to not move today obs to yesterday after reboot.
 *          2024-01-05 RJB Version 19
 *                         Code fix for Daily Totals
 *          2024-01-14 RJB Version 20
 *                         Code Fix added eeprom_valid = true; in the Initialize
 *          2024-01-24 RJB Version 21/22
 *                         Added Getting International Mobile Subscriber Identity
 *          2024-06-12 RJB Version 23
 *                         Added output of the length of the particle payload when we send
 *                         Added support for wet bulb temperature (wbt)
 *                         Added support for heat index (hi)
 *                         Added support for WBGT
 *                         Station Monitor Updated
 *          2024-06-27 RJB Version 24
 *                         Split main code into include file
 *                         Added Copyright
 *                         Updated OLED deisplay code to support both size displays
 *                         Console Enable moved from A4 to D8 
 *          2024-07-23 RJB Bug Fix wbt_calculate(mcp1_temp, htu1_humid) Needs to use sht1_humid
 *          2024-08-02 RJB Version 26
 *                         Removed requirement for Serial Console Enable for external sim programming
 *          2024-08-07 RJB Version 27
 *                         Added code to support Particle DoAction (REBOOT, CRT). 
 *                         Support of Reset NOW removed
 *          2024-09-11 RJB When setting SIM to INTERNAL we now set changed = true to
 *                         report success and reboot message.
 *                         Changed Td to Ta in wbgt_calculate_opt2() function
 *          2024-10-07 RJB Version 28
 *                         Improved hi_calculate() function.
 *          2024-11-05 RJB Discovered BMP390 first pressure reading is bad. Added read pressure to bmx_initialize()
 *                         Bug fixes for 2nd BMP sensor in bmx_initialize() using first sensor data structure
 *                         Now will only send humidity if bmx sensor supports it.
 * NOTES:
 * When there is a successful transmission of an observation any need to send obersavations will be sent. 
 * On transmit a failure of these need to send observations, processing is stopped and the file is deleted.
 * 
 * Stream Gauge Calibration
 * Adding serial console jumper after boot will cause stream gauge to be read every 1 second and value printed.
 * Removing serial console jumper will resume normal operation
 * 
 *  Required Libraries:
 *  adafruit-HTU21DF        https://github.com/adafruit/Adafruit_HTU21DF_Library - 1.1.0 - I2C ADDRESS 0x40
 *  adafruit-BusIO          https://github.com/adafruit/Adafruit_BusIO - 1.8.2
 *  Adafruit_MCP9808        https://github.com/adafruit/Adafruit_MCP9808_Library - 2.0.0 - I2C ADDRESS 0x18
 *  Adafruit_BME280         https://github.com/adafruit/Adafruit_BME280_Library - 2.1.4 - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *  Adafruit_BMP280         https://github.com/adafruit/Adafruit_BMP280_Library - 2.3.0 -I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *  Adafruit_BMP3XX         https://github.com/adafruit/Adafruit_BMP3XX - 2.1.0 I2C ADDRESS 0x77 and (SD0 to GND = 0x76)
 *  Adafruit_GFX            https://github.com/adafruit/Adafruit-GFX-Library - 1.10.10
 *  Adafruit_Sensor         https://github.com/adafruit/Adafruit_Sensor - 1.1.4
 *  Adafruit_SHT31          https://github.com/adafruit/Adafruit_SHT31 - 2.2.0
 *  Adafruit_SSD1306        https://github.com/adafruit/Adafruit_SSD1306 - 2.4.6 - I2C ADDRESS 0x3C  
 *  RTCLibrary              https://github.com/adafruit/RTClib - 1.13.0
 *  SdFat                   https://github.com/greiman/SdFat.git - 1.0.16 by Bill Greiman 
 *  HIH8000                 No Library, Local functions hih8_initialize(), hih8_getTempHumid() - rjb
 *  EEPROM                  https://docs.particle.io/reference/device-os/api/eeprom/eeprom/
 *                          On Gen 3 devices (Argon, Boron, B Series SoM, Tracker SoM, and E404X) 
 *                          the EEPROM emulation is stored as a file on the flash file system. 
 *                          Since the data is spread across a large number of flash sectors, 
 *                          flash erase-write cycle limits should not be an issue in general. 
 * 
 * D8   = Serial Console (Ground Pin to Enable)
 * D7   = On Board LED - Reserved for LoRa IRQ when not using Grove Shield
 * D6   = 
 * D5   = SD Card Chip Select
 * D4   = SPI1 MSIO - Reserved for LoRa
 * D3   = SPI1 MOSI - Reserved for LoRa
 * D2   = SPI1 SCK  - Reserved for LoRa
 * D1   = I2C SCL
 * D0   = I2C SDA
 * 
 * A0   = WatchDog Monitor/Relay Reset Trigger
 * A1   = WatchDog Monitor Heartbeat
 * A2   = When using Grove Shield pin used for Rain2 Gauge IRQ
 * A3   = When using Grove Shield pin used for Rain1 Gauge IRQ
 * A4   = 
 * A5   = When using Grove Shield pin used for Lora IRQ
 * D13  = SPIO CLK    SD Card
 * D12  = SPI0 MOSI   SD Card
 * D11  = SPI0 MISO   SD Card
 * D10  = UART1 RX - Not used - Reserved for LoRa CS
 * D9   = UART1 TX - Not Used - Reserved for LoRa RESET
 * 
 * System.batteryState()
 *  0 = BATTERY_STATE_UNKNOWN
 *  1 = BATTERY_STATE_NOT_CHARGING
 *  2 = BATTERY_STATE_CHARGING
 *  3 = BATTERY_STATE_CHARGED
 *  4 = BATTERY_STATE_DISCHARGING
 *  5 = BATTERY_STATE_FAULT
 *  6 = BATTERY_STATE_DISCONNECTED
 * 
 * Publish to Particle
 *  Event Name: FN
 *  Event Variables:
 *   at     timestamp
 *   rg1    rain gauge 1
 *   rgt1   rain gauge 1 total today
 *   rgp1   rain gauge 1 total prior
 *   rg2    rain gauge 2
 *   rgt2   rain gauge 2 total today
 *   rgp2   rain gauge 2 total prior
 *   bp1    bmx_pressure
 *   bt1    bmx_temp
 *   bh1    bmx_humid
 *   bp2    bmx_pressure
 *   bt2    bmx_temp
 *   bh2    bmx_humid
 *   mt1    mcp_temp
 *   mt2    mcp_temp
 *   hh1    htu_humid
 *   ht1    htu_temp
 *   st1    sht_temp
 *   sh1    sht_humid
 *   st2    sht_temp
 *   sh2    sht_humid
 *   ht2    hih_temp
 *   hh2    hih_humid
 *   hi     heat index
 *   wbt    wet bulb temperature
 *   wbgt   wet bub globe temperature
 *   bcs    Battery Charger Status
 *   bpc    Battery Percent Charge
 *   cfr    Charger Fault Register
 *   css    Cell Signal Strength
 *   hth    Health 32bits - See System Status Bits in below define statements
 * 
 * AN002 Device Powerdown
 * https://support.particle.io/hc/en-us/articles/360044252554?input_string=how+to+handle+low+battery+and+recovery
 * 
 * NOTE: Compile Issues
 * If you have compile issues like multiple definations of functions then you need to clean the compile directory out
 *    ~/.particle/toolchains/deviceOS/2.0.1/build/target/user/...
 * 
 * Particle Cloud Debug
 * https://docs.particle.io/troubleshooting/guides/connectivity-troubleshooting/cellular-connectivity-troubleshooting-guide/
 * ======================================================================================================================
 *
 * Particle devices maximum size of a single event payload is 622 bytes.
 */

#define W4SC false   // Set true to Wait for Serial Console to be connected

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_SHT31.h>
#include <RTClib.h>
#include <SdFat.h>

/*
 * ======================================================================================================================
 *  Loop Timers
 * ======================================================================================================================
 */
#define DELAY_NO_RTC              1000*60    // Loop delay when we have no valided RTC
#define CLOUD_CONNECTION_TIMEOUT  90         // Wait for N seconds to connect to the Cell Network

/*
 * ======================================================================================================================
 *  Relay Power Control Pin
 * ======================================================================================================================
 */
#define REBOOT_PIN        A0  // Toggle power via relay board
#define HEARTBEAT_PIN     A1  // Connect to PICAXE-8M PIN-C3

/*
 * ======================================================================================================================
 * System Status Bits used for report health of systems - 0 = OK
 * 
 * OFF =   SSB &= ~SSB_PWRON
 * ON =    SSB |= SSB_PWROFF
 * 
 * ======================================================================================================================
 */
#define SSB_PWRON            0x1   // Set at power on, but cleared after first observation
#define SSB_SD               0x2   // Set if SD missing at boot or other SD related issues
#define SSB_RTC              0x4   // Set if RTC missing at boot
#define SSB_OLED             0x8   // Set if OLED missing at boot, but cleared after first observation
#define SSB_N2S             0x10   // Set if Need to Send observation file exists
#define SSB_FROM_N2S        0x20   // Set if observation is from the Need to Send file
#define SSB_AS5600          0x40   // Set if wind direction sensor AS5600 has issues - NOT USED with Stream Gauge                   
#define SSB_BMX_1           0x80   // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_BMX_2          0x100   // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_HTU21DF        0x200   // Set if Humidity & Temp Sensor missing
#define SSB_SI1145         0x400   // Set if UV index & IR & Visible Sensor missing
#define SSB_MCP_1          0x800   // Set if Precision I2C Temperature Sensor missing
#define SSB_MCP_2         0x1000   // Set if Precision I2C Temperature Sensor missing
#define SSB_SHT_1         0x2000   // Set if SHTX1 Sensor missing
#define SSB_SHT_2         0x4000   // Set if SHTX2 Sensor missing
#define SSB_HIH8          0x8000   // Set if HIH8000 Sensor missing
#define SSB_LUX          0x10000   // Set if VEML7700 Sensor missing

/*
  0  = All is well, no data needing to be sent, this observation is not from the N2S file
  16 = There is N2S data, This observation is not from the N2S file
  32 = This observation is from the N2S file. And when it was saved to the N2S file, the N2S file did not exist. So it is the first observation from the file.
  48 = This observation is from the N2S file. And when it was saved to the N2S file, the N2S file existed and this observation was appended.
*/

unsigned int SystemStatusBits = SSB_PWRON; // Set bit 0 for initial value power on. Bit 0 is cleared after first obs
bool JustPoweredOn = true;         // Used to clear SystemStatusBits set during power on device discovery

/*
 * ======================================================================================================================
 *  Globals
 * ======================================================================================================================
 */
#define MAX_MSGBUF_SIZE 1024
char msgbuf[MAX_MSGBUF_SIZE]; // Used to hold messages
char *msgp;                   // Pointer to message text
char Buffer32Bytes[32];       // General storage

int  LED_PIN = D7;            // Built in LED
bool TurnLedOff = false;      // Set true in rain gauge interrupt
bool PostedResults;           // How we did in posting Observation and Need to Send Observations

time32_t Time_of_last_obs = 0;
time32_t Time_of_next_obs = 0;

int countdown = 1800;         // Exit calibration mode when reaches 0 - protects against burnt out pin or forgotten jumper
uint64_t LastTimeUpdate = 0;
uint64_t StartedConnecting = 0;
uint64_t DailyRebootTime =  System.millis() + (1000 * 3600 * 23); 
bool ParticleConnecting = false;
bool TakeObservation = true;  // When set we take OBS and transmit it
bool PowerDown = false;

int  cf_reboot_countdown_timer = 79200; // There is overhead transmitting data so take off 2 hours from 86400s
                                        // Set to 0 to disable feature
int DailyRebootCountDownTimer;

/*
 * ======================================================================================================================
 * International Mobile Subscriber Identity
 * ======================================================================================================================
 */
char imsi[16] = "";
bool imsi_valid = false;
time32_t imsi_next_try = 0;

/*
 * ======================================================================================================================
 *  SD Card
 * ======================================================================================================================
 */
#define SD_ChipSelect D5                // GPIO 10 is Pin 10 on Feather and D5 on Particle Boron Board
SdFat SD;                               // File system object.
File SD_fp;
char SD_obsdir[] = "/OBS";              // Store our obs in this directory. At Power on, it is created if does not exist
bool SD_exists = false;                     // Set to true if SD card found at boot
char SD_n2s_file[] = "N2SOBS.TXT";          // Need To Send Observation file
uint32_t SD_n2s_max_filesz = 200 * 8 * 24;  // Keep a little over 2 days. When it fills, it is deleted and we start over.

char SD_sim_file[] = "SIM.TXT";         // File used to set Ineternal or External sim configuration
char SD_simold_file[] = "SIMOLD.TXT";   // SIM.TXT renamed to this after sim configuration set

/*
 * ======================================================================================================================
 *  Power Management IC (bq24195)
 * ======================================================================================================================
 */
PMIC pmic;

/*
 * ======================================================================================================================
 *  Local Code Includes - Do not change the order of the below 
 * ======================================================================================================================
 */
#include "QC.h"                   // Quality Control Min and Max Sensor Values on Surface of the Earth
#include "SF.h"                   // Support Functions
#include "OP.h"                   // OutPut support for OLED and Serial Console
#include "TM.h"                   // Time Management
#include "Sensors.h"              // I2C Based Sensors
#include "Rain.h"                 // Rain Sensors
#include "EP.h"                   // EEPROM
#include "SDC.h"                  // SD Card
#include "OBS.h"                  // Do Observation Processing
#include "SM.h"                   // Station Monitor
#include "PS.h"                   // Particle Support Functions

/*
 * ======================================================================================================================
 * Particle_Publish() - Publish to Particle what is in msgbuf
 * ======================================================================================================================
 */
bool Particle_Publish(char *EventName) {
  // Calling Particle.publish() when the cloud connection has been turned off will not publish an event. 
  // This is indicated by the return success code of false. If the cloud connection is turned on and 
  // trying to connect to the cloud unsuccessfully, Particle.publish() may block for up to 20 seconds 
  // (normal conditions) to 10 minutes (unusual conditions). Checking Particle.connected() 
  // before calling Particle.publish() can help prevent this.
  if (Cellular.ready() && Particle.connected()) {
    if (Particle.publish(EventName, msgbuf, WITH_ACK)) { // PRIVATE flag is always used even when not specified
      // Currently, a device can publish at rate of about 1 event/sec, with bursts of up to 4 allowed in 1 second. 
      delay (1000);
      return(true);
    }
  }
  else {
    Output ("Particle:NotReady");
  }
  return(false);
}


/*
 * ======================================================================================================================
 * HeartBeat() - 
 * ======================================================================================================================
 */
void HeartBeat() {
  digitalWrite(HEARTBEAT_PIN, HIGH);
  delay(250);
  digitalWrite(HEARTBEAT_PIN, LOW);
}

/*
 * ======================================================================================================================
 * BackGroundWork() - Take Sensor Reading, Check LoRa for Messages, Delay 1 Second for use as timming delay            
 * ======================================================================================================================
 */
void BackGroundWork() {
  // Anything that needs sampling every second add below. Example Wind Speed and Direction, StreamGauge
  HeartBeat();  // 250ms
  delay (750);
  if (TurnLedOff) {     // Turned on by rain gauge interrupt handlers
    digitalWrite(LED_PIN, LOW);
    TurnLedOff = false;
  }
}

// You must use SEMI_AUTOMATIC or MANUAL mode so the battery is properly reconnected on
// power-up. If you use AUTOMATIC, you may be unable to connect to the cloud, especially
// on a 2G/3G device without the battery.
SYSTEM_MODE(SEMI_AUTOMATIC);

// https://docs.particle.io/cards/firmware/system-thread/system-threading-behavior/
SYSTEM_THREAD(ENABLED);

/*
 * ======================================================================================================================
 * setup() - runs once, when the device is first turned on.
 * ======================================================================================================================
 */
void setup() {
  // The device has booted, reconnect the battery.
	pmic.enableBATFET();

  // Set Default Time Format
  Time.setFormat(TIME_FORMAT_ISO8601_FULL);

  pinMode (LED_PIN, OUTPUT);
  Output_Initialize();
  delay(2000); // Prevents usb driver crash on startup, Arduino needed this so keeping for Particle

  Serial_write(COPYRIGHT);
  Output (VERSION_INFO);
  delay(4000);

  // Set Daily Reboot Timer
  DailyRebootCountDownTimer = cf_reboot_countdown_timer;

  // WatchDog - By default all pins are LOW when board is first powered on. Setting OUTPUT keeps pin LOW.
  Output ("SETUP WATCHDOG PINs");  // 
  pinMode (REBOOT_PIN, OUTPUT);
  pinMode (HEARTBEAT_PIN, OUTPUT);

  // Initialize SD card if we have one.
  SD_initialize();

  // Report if we have Need to Send Observations
  if (SD_exists && SD.exists(SD_n2s_file)) {
    SystemStatusBits |= SSB_N2S; // Turn on Bit
    Output("N2S:Exists");
  }
  else {
    SystemStatusBits &= ~SSB_N2S; // Turn Off Bit
    Output("N2S:None");
  }

  // Display EEPROM Information 
  EEPROM_Dump();

  // Check if correct time has been maintained by RTC
  // Uninitialized clock would be 2000-01-00T00:00:00
  stc_timestamp();
  sprintf (msgbuf, "%s+", timestamp);
  Output(msgbuf);

  // Read RTC and set system clock if RTC clock valid
  rtc_initialize();

  if (Time.isValid()) {
    Output("STC: Valid");
  }
  else {
    Output("STC: Not Valid");
  }

  stc_timestamp();
  sprintf (msgbuf, "%s=", timestamp);
  Output(msgbuf);

  //==================================================
  // Check if we need to program for Sim change
  //==================================================
  SimChangeCheck();

  // Optipolar Hall Effect Sensor SS451A - Rain1 Gauge
  raingauge1_interrupt_count = 0;
  raingauge1_interrupt_stime = System.millis();
  raingauge1_interrupt_ltime = 0;  // used to debounce the tip
  attachInterrupt(RAINGAUGE1_IRQ_PIN, raingauge1_interrupt_handler, FALLING);

  // Optipolar Hall Effect Sensor SS451A - Rain2 Gauge
  raingauge2_interrupt_count = 0;
  raingauge2_interrupt_stime = System.millis();
  raingauge2_interrupt_ltime = 0;  // used to debounce the tip
  attachInterrupt(RAINGAUGE2_IRQ_PIN, raingauge2_interrupt_handler, FALLING);

  // Adafruit i2c Sensors
  bmx_initialize();
  htu21d_initialize();
  mcp9808_initialize();
  sht_initialize();
  hih8_initialize();

  // Derived Observations
  wbt_initialize();
  hi_initialize();
  wbgt_initialize();

  // Connect the device to the Cloud. 
  // This will automatically activate the cellular connection and attempt to connect 
  // to the Particle cloud if the device is not already connected to the cloud.
  // Upon connection to cloud, time is synced, aka Particle.syncTime()

  // Note if we call Particle.connect() and are not truely connected to the Cell network, Code blocks in particle call
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));
  Particle.connect();

  // Setup Remote Function to DoAction, Expects a parameter to be passed from Particle to control what action
  if (Particle.function("DoAction", Function_DoAction)) {
    Output ("DoAction:OK");
  }
  else {
    Output ("DoAction:ERR");
  }

  Time_of_next_obs = Time.now() + 60;  // Schedule a obs 60s from not to give network a change to connect

  Output ("LOOP START");
}

/*
 * ======================================================================================================================
 * loop() runs over and over again, as quickly as it can execute.
 * ======================================================================================================================
 */
void loop() {
  BackGroundWork(); // Delays 1 second

  if (countdown && (digitalRead(SCE_PIN) == LOW)) {
    StationMonitor();
    // StartedConnecting = System.millis();  // Keep set timer so when we pull jumper we finish connecting and not time out
    countdown--;
  }
  else {
    if (!imsi_valid && Particle.connected() && (Time.now() >= imsi_next_try)) {
      IMSI_Request(); // International Mobile Subscriber Identity
      if (!imsi_valid) {
        imsi_next_try = Time.now() + 3600;
      }
    }

    // This will be invalid if the RTC was bad at poweron and we have not connected to Cell network
    // Upon connection to cell network system Time is set and this becomes valid
    if (Time.isValid()) {  
 
      // Set RTC from Cell network time.
      RTC_UpdateCheck();

      if (!eeprom_valid) {
        // We now a a valid clock so we can initialize the EEPROM
        EEPROM_Initialize();
      }

      // Perform an Observation, Write to SD, and Transmit observation
      if (Time.now() >= Time_of_next_obs) {
        I2C_Check_Sensors(); // Make sure Sensors are online
        OBS_Do();

        // Time of last obs in ms MOD 900 = Seconds since last period
        // 900 - Seconds since last period = Seconds to next period
        // Seconds to next period + current time in MS = next OBS time
        Time_of_next_obs = (900 - (Time_of_last_obs % 900)) + Time.now();
        
        // stc_timestamp();
        // Output(timestamp);

        // next_obs_timestamp(); // Next Observation Time
        // Output(timestamp);

        // Shutoff System Status Bits related to initialization after we have logged first observation 
        JPO_ClearBits();

        // We are staying connected to the Cell network
        // request time synchronization from the Cell network - Every 4 Hours
        if ((System.millis() - LastTimeUpdate) > (4*3600*1000)) {
          // Note that this function sends a request message to the Cloud and then returns. 
          // The time on the device will not be synchronized until some milliseconds later when 
          // the Cloud responds with the current time between calls to your loop.

          // !!! What if we drop the Cell connection before we get a time update for the Cloud?
          Output ("NW TimeSync REQ");
          Particle.syncTime();
          LastTimeUpdate = System.millis();
        }
      }
    }
    else {
      stc_timestamp();
      Output(timestamp);
      Output("ERR: No Clock");
      delay (DELAY_NO_RTC);
    }

    // Reboot Boot Every 22+ hours - Not using time but a loop counter.
    if ((cf_reboot_countdown_timer>0) && (--DailyRebootCountDownTimer<=0)) {
      Output ("Daily Reboot");

      // Send an observation, this will then handle any unreported rain.
      OBS_Do();

      // Lets not rip the rug out from the modem. Do a graceful shutdown.
      Particle.disconnect();
      waitFor(Particle.disconnected, 1000);  // Returns true when disconnected from the Cloud.

      // Be kind to the cell modem and try to shut it down
      Cellular.disconnect();
      delay(1000);
      Cellular.off();

      Output("Rebooting");  
      delay(1000);
   
      DeviceReset();

      // we should never get here, but just incase 
      Output("I'm Alive! Why?");  

		  Cellular.on();
      delay(1000);

		  Particle.connect();

      DailyRebootCountDownTimer = cf_reboot_countdown_timer; // Reset count incase reboot fails
    }   

    // Check our power status
    // If we are not connected to a charging source and our battery is at a low level
    // then power down the display and board. Wait for power to return.
    // Do this at a high enough battery level to avoid the board from powering
    // itself down out of our control. Also when power returns to be able to charge
    // the battery and transmit with out current drops causing the board to reset or 
    // power down out of our control.
    if (!pmic.isPowerGood() && (System.batteryCharge() <= 10.0)) {

      Output("Low Power!");

      // While this function will disconnect from the Cloud, it will keep the connection to the network.
      Particle.disconnect();
      waitFor(Particle.disconnected, 1000);  // Returns true when disconnected from the Cloud.
      
      Cellular.disconnect();
      delay(1000);
      Cellular.off();

      Output("Powering Down");

      OLED_sleepDisplay();
      delay(5000);

      // Disabling the BATFET disconnects the battery from the PMIC. Since there
		  // is no longer external power, this will turn off the device.
		  pmic.disableBATFET();

		  // This line should not be reached. When power is applied again, the device
		  // will cold boot starting with setup().

		  // However, there is a potential for power to be re-applied while we were in
		  // the process of shutting down so if we're still running, enable the BATFET
		  // again and reconnect to the cloud. Wait a bit before doing this so the
		  // device has time to actually power off.
		  delay(2000);

      OLED_wakeDisplay();   // May need to toggle the Display reset pin.
		  delay(2000);
		  Output("Power Re-applied");

		  pmic.enableBATFET();

		  Cellular.on();

		  Particle.connect();
    }
  }
}