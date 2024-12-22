// modified to allow the use of the very miniature Ebyte E77 mcu & radio module all in one: STM32WLE5CCU6 
// and the miniature Ublox CAM-M8Q gps module for a very small (and light) form factor


/*
 Read NMEA sentences over I2C using Ublox module SAM-M8Q, NEO-M8P, ZED-F9P, etc
 By: Nathan Seidle
 SparkFun Electronics
 Date: August 22nd, 2018
 License: MIT. See license file for more information but you can
 basically do whatever you want with this code.


 This example reads the NMEA setences from the Ublox module over I2c and outputs
 them to the serial port
  Feel like supporting open source hardware?
 Buy a board from SparkFun!
 ZED-F9P RTK2: https://www.sparkfun.com/products/15136
 NEO-M8P RTK: https://www.sparkfun.com/products/15005
 SAM-M8Q: https://www.sparkfun.com/products/15106


 Hardware Connections:
 Plug a Qwiic cable into the GPS and a BlackBoard
 If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
 Open the serial monitor at 115200 baud to see the output
*/
#include <SPI.h>
#include <SubGhz.h>
#include <Wire.h> //Needed for I2C to GPS
#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
#include "STM32RTC.h"
#include "STM32LowPower.h"
#include <RadioLib.h>

uint32_t GEOFENCE_APRS_frequency = 433775000;   // temp frequency before geofencing
uint32_t GEOFENCE_APRS_spreading_factor = 12;   // temp spreading factor before geofencing
uint32_t GEOFENCE_APRS_coding_rate = 5;         // temp coding rate before geofencing
uint32_t GEOFENCE_no_tx = 1;               

 float loraFrequency = 433.775;
 float loraBandWith = 125.0f;
 uint8_t spreadingFactor = 12; 
 uint8_t codingRate = 5;

//#include "SparkFun_Ublox_Arduino_Library.h" // this is older library and is no longer supported
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS this is the newer library
SFE_UBLOX_GNSS myGPS; // changed from SFE_UBLOX_GPS (the old library)


#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
char* status = "LoRa TX STM32WLE5";
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ublox_high_alt_mode_enabled = false;


// no need to configure pins, signals are routed to the radio internally
STM32WLx radio = new STM32WLx_Module();


APRSClient aprs(&radio);


// set RF switch configuration for EBytes E77 dev board
// PB3 is an LED - activates while transmitting
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
static const uint32_t rfswitch_pins[] =
                        {PA6,  PA7,  PB3, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
 {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
 {STM32WLx::MODE_RX,    {LOW, HIGH, LOW}},
 {STM32WLx::MODE_TX_LP, {HIGH, LOW, HIGH}},
 {STM32WLx::MODE_TX_HP, {HIGH, LOW, HIGH}},
 END_OF_MODE_TABLE,
};


void setupUBloxDynamicModel() {
   // each time we power the GPS on, we will have to reset this parameter (I don't think the backup battery will hold this setting)
   // If we are going to change the dynamic platform model, let's do it here.
   // Possible values are:
   // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
   //DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters.
 if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
   {
     Serial.println(F("***!!! Warning: setDynamicModel failed !!!***"));
   }
     else
   {
     ublox_high_alt_mode_enabled = true;
     Serial.print(F("SetDynamicModel: "));
     Serial.println(myGPS.getDynamicModel());
     #if defined(DEVMODE)
       Serial.print(F("Ublox dynamic platform model (DYN_MODEL_AIRBORNE4g) changed successfully! : "));
       Serial.println(myGPS.getDynamicModel());
     #endif 
   }
 }

void setupLoRaRadio()  {
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
 // set RF switch control configuration
 // this has to be done prior to calling begin()
 //radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);


 // initialize STM32WL with default settings, except frequency
 Serial.print(F("[STM32WL] Initializing ... "));
 //int state = radio.begin(433.775);
 // frequency:                   433.775 MHz
 // bandwidth:                   125 kHz
 // spreading factor:            12
 // coding rate:                 4/5

 int state = radio.begin(loraFrequency, loraBandWith, spreadingFactor, codingRate);


 if (state == RADIOLIB_ERR_NONE) {
   Serial.println(F("success!"));
 } else {
   Serial.print(F("failed, code "));
   Serial.println(state);
   while (true) { delay(10); }
 }


 // initialize APRS client
 Serial.print(F("[APRS] Initializing ... "));
 // symbol:                      '>' (car)
 // callsign                     "NOCALL"  // your call sign
 // SSID                         1
 char source[] = "K6ATV"; // insert your amateur radio callsign here
 state = aprs.begin('>', source, 1);


 if(state == RADIOLIB_ERR_NONE) {
   Serial.println(F("success!"));
 } else {
   Serial.print(F("failed, code "));
   Serial.println(state);
   while (true) { delay(10); }
 }


 // set appropriate TCXO voltage for Nucleo WL55JC1, WL55JC2, or E77 boards
 state = radio.setTCXO(1.7);
 Serial.print(F("Set TCXO voltage ... "));
  if (state == RADIOLIB_ERR_NONE) {
   Serial.println(F("success!"));
 } else {
   Serial.print(F("failed, code "));
   Serial.println(state);
   while (true) { delay(10); }
 }


 // set output power to 22 dBm (accepted range is -17 - 22 dBm)
 if (radio.setOutputPower(22) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
   Serial.println(F("Selected output power is invalid for this module!"));
   while (true) { delay(10); }
 }


}

int32_t pointInPolygonF(int32_t polyCorners, float * polygon, float latitude, float longitude)
{
  int32_t i;
  int32_t j = polyCorners * 2 - 2;
  int32_t oddNodes = 0;

  for(i = 0; i < polyCorners * 2; i += 2)
  {
    if((polygon[i + 1] < latitude && polygon[j + 1] >= latitude
    || polygon[j + 1] < latitude && polygon[i + 1] >= latitude)
    && (polygon[i] <= longitude || polygon[j] <= longitude))
    {
      oddNodes ^= (polygon[i] + (latitude - polygon[i + 1])
      / (polygon[j + 1] - polygon[i + 1]) * (polygon[j] - polygon[i]) < longitude);
    }

    j = i;
  }

  return oddNodes;
}

void printGPSandSensorData() {
     float latitude_mdeg = nmea.getLatitude();
   Serial.print("Latitude (deg): ");
   Serial.print(latitude_mdeg / 1000000., 4);

   float longitude_mdeg = nmea.getLongitude();
   Serial.print(" , Longitude (deg): ");
   Serial.print(longitude_mdeg /1000000., 4);

   long speed = myGPS.getGroundSpeed();
   Serial.print(F(" Speed: "));
   Serial.print(speed / 514.4);
   Serial.print(F(" (knots)"));

   long heading = myGPS.getHeading();
   Serial.print(F(" Heading: "));
   Serial.print((heading / 1000000));
   Serial.print(F(" degrees"));

   long altitude = myGPS.getAltitude();
   Serial.print(F(" Alt: "));
   Serial.print(altitude/1000);
   Serial.print(F(" (m)"));

   long altitudeMSL = myGPS.getAltitudeMSL();
   Serial.print(F(" AltMSL: "));
   Serial.print(altitudeMSL /1000);
   Serial.print(F(" (m)"));

   byte SIV = myGPS.getSIV();
   Serial.print(F(" SIV: "));
   Serial.print(SIV);


   //Serial.println();
   Serial.print(" Date: ");
   Serial.print(myGPS.getYear());
   Serial.print("-");
   Serial.print(myGPS.getMonth());
   Serial.print("-");
   Serial.print(myGPS.getDay());
   Serial.print(" ");
   Serial.print(myGPS.getHour());
   Serial.print(":");
   Serial.print(myGPS.getMinute());
   Serial.print(":");
   Serial.println(myGPS.getSecond());
   delay(500);

}

void GEOFENCE_position(float latitude, float longitude)
{

      float  UKF[] = {
        -0.65920,   60.97310,
        -7.58060,   58.07790,
        -8.21780,   54.23960,
        -4.76810,   53.80070,
        -5.86670,   49.76710,
        1.30740,    50.85450,
        1.86770,    52.78950,
        -2.04350,   55.97380,
        -0.65920,   60.97310
      }; 
  
      float  PolandF[] = {
        54.439234, 14.103456,
        51.022075, 14.897034,
        49.537732, 18.860178,
        49.113780, 22.481393,
        52.742317, 23.889542,
        54.156275, 23.456098,
        54.439234, 14.103456
      };      
      static float LatviaF[] = {
        26.64180,   55.68380,
        28.17990,   56.20670,
        27.78440,   57.33250,
        25.00490,   58.00230,
        24.14790,   57.17200,
        21.78590,   57.68650,
        20.81910,   56.07200,
        22.19240,   56.44430,
        25.68600,   56.18230,
        26.64180,   55.68380
      };      
      static float YemenF[] = {
        52.20302, 19.48287,
        41.92009, 17.42198,
        43.59620, 12.27820,
        53.68201, 15.80024,
        52.20302, 19.48287
      };
      static float North_KoreaF[] = {
        130.14189, 43.21627,
        124.03574, 39.8949,
        125.18292, 37.39202,
        128.47240, 38.66888,
        130.69478, 42.3274,
        130.14189, 43.21627
      };             

      if(pointInPolygonF(9, UKF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1; //Airborne TX is not allowed in UK
          GEOFENCE_APRS_frequency = 439912500;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;
        } else if(pointInPolygonF(7, PolandF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 0;
          GEOFENCE_APRS_frequency = 434855000;
          GEOFENCE_APRS_spreading_factor = 9;
          GEOFENCE_APRS_coding_rate = 7;          
        } else if(pointInPolygonF(10, LatviaF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;                
        } else if(pointInPolygonF(5, YemenF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5; 
        } else if(pointInPolygonF(6, North_KoreaF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;              
        } else {
          GEOFENCE_no_tx = 0;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;      
        }
}

void configureFreqbyLocation() {

  float tempLat = myGPS.getLatitude() / 10000000.f;
  float tempLong = myGPS.getLongitude() / 10000000.f; 

  GEOFENCE_position(tempLat,tempLong);  
  loraFrequency = GEOFENCE_APRS_frequency / 1000000.f;
  if (radio.setFrequency(loraFrequency,true) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  if (radio.setSpreadingFactor(GEOFENCE_APRS_spreading_factor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  if (radio.setCodingRate(GEOFENCE_APRS_coding_rate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

}

void setup()
{
 Serial.begin(115200);
 Serial.println("SparkFun Ublox Example");


 Wire.begin();


 if (myGPS.begin() == false)
 {
   Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
   while (1);
 }
 setupUBloxDynamicModel();
 setupLoRaRadio();
 LowPower.begin();
}


void loop()
{
 int sleepTime = 180000;
 delay(400); // Wakeup!
 setup(); // go through setup again, becouse I was sleeping, and cant remember anything
 char destination[] = "APZATV"; // APZxxx means experimental. More info can be read on page 13-14 of http://www.aprs.org/doc/APRS101.PDF
 if(!ublox_high_alt_mode_enabled){
   setupUBloxDynamicModel();
 }
 myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

 //if(nmea.isValid() == true)
 // is GPS info fresh, valid fix with more than 3 satellites?
 if (myGPS.getPVT() && (myGPS.getFixType() !=0) && (myGPS.getSIV() > 3))
 {
   printGPSandSensorData();

   configureFreqbyLocation(); // check geofence 
   //int state = aprs.sendMicE(36.9976, -121.5877, 270, 0, RADIOLIB_APRS_MIC_E_TYPE_EN_ROUTE, NULL, 0, NULL, status, altitude);
   //int state = aprs.sendMicE(36.9976, -121.5877, 270, 0, RADIOLIB_APRS_MIC_E_TYPE_EN_ROUTE);  //minimal
   if (GEOFENCE_no_tx == 0) {   // if it is legal to transmit, transmit
   int state = aprs.sendMicE((nmea.getLatitude() / 1000000.), (nmea.getLongitude() / 1000000.), (myGPS.getHeading() / 1000000), (myGPS.getGroundSpeed() / 514.4), RADIOLIB_APRS_MIC_E_TYPE_EN_ROUTE, NULL, 0, NULL, status, (myGPS.getAltitudeMSL() / 1000));
      if(state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
   } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
   }
   (5000); // let sendMicE complete before going to sleep
   }

 }
 else
 {
   Serial.print("No Fix - ");
   Serial.print("Num. satellites: ");
   Serial.println(nmea.getNumSatellites());
 }
 delay(500); // let printing complete prior to sleeping
 LowPower.deepSleep(sleepTime);
 //delay(1000); //Don't pound too hard on the I2C bus
}


//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
 //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
 //for sentence cracking
 nmea.process(incoming);
}
