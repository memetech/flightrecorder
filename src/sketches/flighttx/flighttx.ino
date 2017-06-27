
/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_GPS.h>

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "HWUART"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

__FlashStringHelper *lastErr = 0L;

// A small helper
void error(const __FlashStringHelper*err) {
  if(Serial) Serial.println((const char * ) (err==0L?(lastErr==0L?"":lastErr):err));
  else lastErr=err;
}
void log(const __FlashStringHelper*msg) {
  if(Serial) Serial.print(msg);
}
void logln(const __FlashStringHelper*msg) {
  if(Serial) Serial.println(msg);
}

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
int gi = 0;
bool isConnected = false;
bool isSerialConnected = false;

  // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

void setupGPS() {
    
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  //GPS.sendCommand(PMTK_SET_BAUD_57600);
  //GPS.begin(57600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);   // every 5 seconds update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ); // slow fix too


  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  GPS.wakeup();
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  // I think this means wait for a serial monitor connection so i am turning it off
  // while (!Serial);  // required for Flora & Micro
  // delay(500);


  /* Initialise the module */
  log(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  logln( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    if(Serial) Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  // initialize gps
  setupGPS();
  

  isConnected = false;
  isSerialConnected = false;
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
    // Check for user input
    char n, inputs[BUFSIZE+1];
    char g, gpsBuf[200];
    g = GPS.read();


  if(!isSerialConnected && Serial) {
      delay(500);
      Serial.begin(115200);
      isSerialConnected = true;
      
      //Serial.println(F("Flight Controller: Bluefruit Command <-> Data Mode Example"));
      //Serial.println(F("------------------------------------------------"));

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  //Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  //Serial.println(F("Then Enter characters to send to Bluefruit"));
  //Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  //Serial.println(F("GPS Version"));
  // Ask for firmware version
  //Serial.println(PMTK_Q_RELEASE);
  
  Serial.println();

  }

  if(!isConnected && ble.isConnected()) {

    isConnected = true;

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));    
  } else if(isConnected && !ble.isConnected()) {
    Serial.println( F("Switching to COMMAND mode!") );
    ble.setMode(BLUEFRUIT_MODE_COMMAND);
    isConnected = false;
  }

  bool nmeaReceived = GPS.newNMEAreceived();

  if(!isConnected && isSerialConnected && g) Serial.print(g);
  if(!isConnected) return;

    if (nmeaReceived) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
    {
      /*ble.print("Fix: "); */ble.print((int)GPS.fix);
      ble.print(","); 
      /*ble.print("Satellites: "); */ble.print((int)GPS.satellites);
    if (GPS.fix) {
      ble.print(","); 
      /* ble.print("Location: ");
      ble.print(GPS.latitude, 4); //ble.print(GPS.lat);
      ble.print(", "); 
      ble.print(GPS.longitude, 4); //ble.print(GPS.lon);
      ble.print(", "); 
      */
     /* ble.print("Speed (knots): ");*/ ble.print(floor(GPS.speed));
      ble.print(","); 
      /*ble.print("Angle: "); */ble.print(floor((360.0/GPS.angle) * (16)));
      ble.print(","); 
      /*ble.print("Altitude: "); */ble.print(floor(GPS.altitude));
    }
    ble.println("");
 
    }
  }

 /*   gpsBuf[gi++] = g;
    if(g=='\n') {
      gpsBuf[gi] = 0;
      gi = 0;
      // Send input data to host via Bluefruit
      ble.print(gpsBuf);
    } else {
      if(gi == 199) gi = 0;
    }    
  }
*/
  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }

  // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");
  }
}
