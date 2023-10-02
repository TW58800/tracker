/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/timwh/Projects/tracker/server/src/Lora-p2p.ino"
// This #include statement was automatically added by the Particle IDE.
//#include <RF9X-RK.h>

// rf95_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W 

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Adafruit_GPS.h>

void setup();
void loop();
#line 16 "c:/Users/timwh/Projects/tracker/server/src/Lora-p2p.ino"
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

//SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);

// Connect to the GPS on the hardware port
//Adafruit_GPS GPS(&Serial1);
Adafruit_GPS GPS;  // as we are only parsing GPS data received from the client, there is no need to connect to the harware port

uint32_t timer = millis();

// Singleton instance of the radio driver
RH_RF95 driver(D6, D2);

// Frequency is typically 868.0 or 915.0 in the Americas, or 433.0 in the EU
float frequency = 868.0;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

TCPServer server = TCPServer(23);
TCPClient client;

int LED = D7;

void setup() 
{
	pinMode(LED, OUTPUT);   
	WiFi.connect();
	Serial.begin(9600);
	// Wait for a USB serial connection for up to 15 seconds
	waitFor(Serial.isConnected, 15000);
    Serial.println("connected");
  	Serial.printlnf("localIP=%s", WiFi.localIP().toString().c_str());
  	Serial.printlnf("subnetMask=%s", WiFi.subnetMask().toString().c_str());
  	Serial.printlnf("gatewayIP=%s", WiFi.gatewayIP().toString().c_str());

  	// start listening for clients
  	server.begin();

	if (!manager.init())
		Serial.println("init failed");

	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// Setup ISM frequency
	driver.setFrequency(frequency);

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	driver.setTxPower(23, false);

	// If you are using Modtronix inAir4 or inAir9,or any other module which uses the
	// transmitter RFO pins and not the PA_BOOST pins
	// then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
	// Failure to do that will result in extremely low transmit powers.
	//  driver.setTxPower(14, true);
	// You can optionally require this module to wait until Channel Activity
	// Detection shows no activity on the channel before transmitting by setting
	// the CAD timeout to non-zero:
	//  driver.setCADTimeout(10000);
}

// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
float* batt;

void loop() {
	if (manager.available()) {
		// Wait for a message addressed to us from the client
		uint8_t len = sizeof(buf);
		uint8_t from;
		if (manager.recvfromAck(buf, &len, &from)) {
			buf[len] = 0;
			Serial.printlnf("\nGot packet from 0x%02x rssi=%d %s", from, driver.lastRssi(), (char *)buf+4);
			//Serial.printlnf("Batt: %3.2f", (float *)buf);
    		batt = (float*)buf;
			Serial.printlnf("Batt: %3.2f", *batt);
			
    		if (!GPS.parse((char*)buf+4)) { // this also sets the newNMEAreceived() flag to false
				Serial.println("\nFailed to parse GPS data\n");
			}
			else {
  				digitalWrite(LED, HIGH); // sets the LED on
  				delay(200);              // waits for 200mS
  				digitalWrite(LED, LOW);  // sets the LED off
  				if (client.status()) { 
      				byte payload = client.read(); 
      				Serial.printlnf("TCP bytes received: %i\n", payload);
					byte* lat = reinterpret_cast<byte*>(&GPS.latitudeDegrees);
					byte* lng = reinterpret_cast<byte*>(&GPS.longitudeDegrees);
					int16_t Rssi = driver.lastRssi();
					byte* rssi = reinterpret_cast<byte*>(&Rssi);
					uint8_t data[14]; // = {0,0,0,0,0,0,0,0,0,0};
					data[0] = lat[0];
					data[1] = lat[1];
					data[2] = lat[2];
					data[3] = lat[3];
					data[4] = lng[0];
					data[5] = lng[1];
					data[6] = lng[2];
					data[7] = lng[3];
					data[8] = rssi[1];
					data[9] = rssi[0];
					data[10] = buf[0];
					data[11] = buf[1];
					data[12] = buf[2];
					data[13] = buf[3];
					server.write(data, 14, 5000); 
      				//Serial.printlnf("Bytes sent: %c", lat);
					
					//Particle.publish("gpsdata", (char *)buf);

					/*
					float latitude = 51.24855439074274f;
					float longditude = -0.5447383774659881f;
					std::memcpy(&data, &latitude, 4);
					std::memcpy(&data+4, &longditude, 4);
					Serial.printlnf("Latitude: %f, Longditude: %f", &data, &data+4);
					int bytes = server.write(reinterpret_cast<byte*>(&data), 8, 10000);
					int err = server.getWriteError();
					//if (err != 0) {
  					Serial.printlnf("TCPServer::write() failed (error = %d), number of bytes written: %d", err, bytes);
					*/
				}
				else {
					// if no client is yet connected, check for a new connection
					client = server.available();
				}
			}
			// Send a reply back to the originator client
			if (!manager.sendtoWait(buf, len, from))
				Serial.println("sendtoWait failed");
		}
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
		Serial.print("Fix: "); Serial.print((int)GPS.fix);
		Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
		if (GPS.fix) {
		  Serial.print("Location: ");
		  Serial.print(GPS.latitudeDegrees, 4); Serial.print(GPS.lat);
		  Serial.print(", ");
		  Serial.print(GPS.longitudeDegrees, 4); Serial.println(GPS.lon);
		  Serial.print("Speed (knots): "); Serial.println(GPS.speed);
		  Serial.print("Angle: "); Serial.println(GPS.angle);
		  Serial.print("Altitude: "); Serial.println(GPS.altitude);
		  Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
		  Serial.println("\n");
		  //Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);  // GPS class has no member 'antenna'
		}
	}
}		