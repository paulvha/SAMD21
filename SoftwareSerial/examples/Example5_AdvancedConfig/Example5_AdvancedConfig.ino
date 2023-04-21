/*
  Author: Nathan Seidle
  SparkFun Electronics
  Created: June 19 2019 /  adjusted for SAMD21 April 2023
  License: MIT. See SparkFun Arduino Project for more information

  Feel like supporting open source hardware? Buy a board from SparkFun!

  This example shows how to use different data, parity, and stop bits.
  See Arduino reference for all settings: https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/

  Hardware Connections:
  Attach a USB to serial converter (https://www.sparkfun.com/products/15096)
  Connect
    GND on SerialBasic <-> GND on SAMD21
    RXO on SerialBasic <-> Pin 5 on SAMD21
    TXO on SerialBasic <-> Pin 6 on SAMD21
  Load this code
  Open Arduino serial monitor at 115200
  Open Terminal window (TeraTerm) at 19200 with special settings *8 bits, even parity, 2 stop!*
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
  Serial.println("Software Serial Example5");

  if(! mySerial.begin(9600, SERIAL_8E2) ){
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

  delay(100); //Small delay between prints so that we can detect incoming chars if any
}
