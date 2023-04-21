/*
  Author: Nathan Seidle
  SparkFun Electronics
  Created: June 19 2019 /  adjusted for SAMD21 April 2023
  License: MIT. See SparkFun Arduino Project for more information

  Feel like supporting open source hardware? Buy a board from SparkFun!

  This example shows how to send characters at 57600bps. It's counter intuitive
  but by printing faster and receiving faster we can do more with interrupt
  based software serial.
  Any pin can be used for TX or RX.

  Hardware Connections:
  Attach a USB to serial converter (https://www.sparkfun.com/products/15096)
  Connect
    GND on SerialBasic <-> GND on SAMD21
    RXO on SerialBasic <-> Pin 5 on SAMD21
    TXO on SerialBasic <-> Pin 6 on SAMD21
  Load this code
  Open Arduino serial monitor at 115200
  Open Terminal window (TeraTerm) at 57600
  Press a button in terminal window, you should see it in Arduino monitor
*/

#define Serial SerialUSB

#include <SoftwareSerial.h>
SoftwareSerial mySerial(5, 6); //RX, TX - Any pins can be used

int counter = 0;

void setup() {
  //We set the serial monitor speed high so that we spend less time printing the output
  //and more time checking mySerial.available()
  Serial.begin(115200);
  Serial.println("Software Serial Example3");

  if(! mySerial.begin(57600) ){
    Serial.println("Invalid pins. freeze!");
    while(1);
  }
}

void loop() {

  mySerial.print("Hi: ");
  mySerial.println(counter++);

  while (mySerial.available())
  {
    byte incoming = mySerial.read();
    Serial.write(incoming);
  }

  delay(25); //Small delay between prints so that we can detect incoming chars if any
}
