/*
  Author: Nathan Seidle
  SparkFun Electronics
  Created: June 19 2019 /  adjusted for SAMD21 April 2023
  License: MIT. See SparkFun Arduino Project for more information

  Feel like supporting open source hardware? Buy a board from SparkFun!

  This example shows how to send characters at 9600bps. Any pin can be used for
  TX or RX.

  Note: In this SoftwareSerial library you cannot TX and RX at the same time.
  TX gets priority. So if SAMD21 is receiving a string of characters
  and you do a Serial.print() the print will begin immediately and any additional
  RX characters will be lost.

  Hardware Connections:
  Attach a USB to serial converter (https://www.sparkfun.com/products/15096)
  Connect
    GND on SerialBasic <-> GND on SAMD21
    RXO on SerialBasic <-> Pin 6 on SAMD21
  Load this code
  Open Arduino serial monitor at 115200
  Open Terminal window (TeraTerm) at 9600
*/

#define Serial SerialUSB

#include <SoftwareSerial.h>
SoftwareSerial mySerial(5, 6); //RX, TX - Any pins can be used

int counter = 0;

void setup() {
  //We set the serial monitor speed high so that we spend less time printing the output
  Serial.begin(115200);
  Serial.println("Software Serial Example1");

  if(! mySerial.begin(9600) ){
    Serial.println("Invalid pins. freeze!");
    while(1);
  }
}

void loop() {

  mySerial.print("Hello world: ");
  mySerial.println(counter++);

  delay(20); //Small delay because it takes time to send chars at 9600bps
}
