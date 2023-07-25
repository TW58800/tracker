#include <RadioHead.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Adafruit_GPS.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Feather M0 w/Radio
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define GPSSerial Serial1 // what's the name of the hardware serial port?
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

float frequency = 868.0;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();


void setup() 
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");
  while (!Serial) ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
	driver.setFrequency(frequency);
  driver.setTxPower(20, false);
  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
  //  driver.setCADTimeout(10000);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint32_t counter = 0;
uint16_t gpsDataLen = 0;

void loop()
{
  // read data from the GPS in the 'main loop'
  if(char c = GPS.read()) {
    counter++;
  }
  // if you want to debug, this is a good time to do it!
  //if (GPSECHO)
  //  if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    gpsDataLen = counter;
    counter = 0;
    if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      Serial.println("\nFailed to parse GPS data\n");
      //return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 10000) {
    timer = millis(); // reset the timer
    /*
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    */
    Serial.print("Character Count: "); Serial.println(gpsDataLen);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }

    Serial.println("\nSending to rf95_reliable_datagram_server");

    // Send a message to manager_server
    snprintf((char *)buf, sizeof(buf), GPS.lastNMEA()); //"to server counter=%d", ++counter);

    if (manager.sendtoWait(buf, sizeof(buf), SERVER_ADDRESS))
    {
      // Now wait for a reply from the server
      Serial.println((char *)buf);
      uint8_t len = sizeof(buf);
      uint8_t from;   
      if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
      {
        buf[len] = 0;
        Serial.print("Got reply from 0x");
        Serial.println(from, HEX);
        //Serial.print("Data: ");
        Serial.println((char*)buf);
      }
      else
      {
        Serial.println("No reply, is rf95_reliable_datagram_server running?");
      }
    }
    else
      Serial.println("sendtoWait failed");
    //delay(3000);
  }
}

