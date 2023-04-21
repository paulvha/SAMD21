# Software Serial on SAMD21

## ===========================================================
April 2023

## Background

Some people raised question: Is softwareSerial available for SAMD21?
Sounds like a nice challenge to try. I started off with the version that was written by Nathan Seidle for V1.2.x Apollo3 library.
This has been adjusted mainly for the Timer functions. The SAMD21 TC4 & TC5 timers are used in 32 bit mode and thus this will not work in combination with TONE and/OR PWM (like analogWrite())

## Result

After long testing and debugging Software Serial it is possible with sending 115200 / receiving 57600.
This has been tested on Sparkfun SparkFun Qwiic Micro - SAMD21 Development Board (https://www.sparkfun.com/products/15423) with Arduino IDE 1.8.16

## More detailed information

For usage see https://www.arduino.cc/en/Reference/SoftwareSerial

## Installation

1. Copy the complete SofwareSerial-directory in the directory :   Sparkfun/hardware/samd/1.8.9/libraries

## Versioning

### version 1.0 April 2023 / paulvha@hotmail.com
 * Initial version SoftwareSerial for Library 1.8.9 for samd21

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. It is delivered as-is without support.
