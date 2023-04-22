/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
  This is a library written for the SparkFun Artemis and adjusted for SAMD21

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support open source hardware. Buy a board!

  Written by Nathan Seidle @ SparkFun Electronics, August 12th, 2019
  Adjusted by paulvha for SAMD21, April 2023

  https://github.com/sparkfun/SparkFun_Apollo3

  SoftwareSerial support for the SAMD21
  Any pin can be used for software serial receive or transmit
  at 300 to 57600bps and anywhere inbetween.
  Limitations (similar to Arduino core SoftwareSerial):
    * RX on one pin at a time.
    * No TX and RX at the same time.
    * TX gets priority. So if SAMD21 is receiving a string of characters
    and you do a mySerial.print() the print will begin immediately and any additional
    RX characters will be lost.
    * Uses TC4 and TC5. This will remove PWM and Tone capabilities
    * Parity is supported during TX but not checked during RX.
    * Enabling multiple ports causes 57600 RX to fail (because there is additional instance switching overhead)

  Development environment specifics:
  Arduino IDE 1.8.x

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _SoftwareSerial_H
#define _SoftwareSerial_H
#include "Arduino.h"
#include <Stream.h>

#define SWS_BUFFER_SIZE 128      // RX and TX buffer size
#define TIMER_FREQ 6000000L      // 6Mhz (see configTimer())

class SoftwareSerial : public Stream
{
  public:
    SoftwareSerial(uint32_t rxPin, uint32_t txPin, bool invertLogic = false);

    bool begin(uint32_t baudRate);
    bool begin(uint32_t baudRate, uint16_t config);
    void end(void);

    int available(void);
    int read(void);
    int peek(void);
    void flush(void);
    bool overflow();
    void listen();
    bool isListening();

    virtual size_t write(uint8_t toSend);
    virtual size_t write(const uint8_t *buffer, size_t size);
    virtual size_t write(const char *str);

    void rxBit(void);
    void rxEndOfByte(void);
    void txHandler();
    volatile bool rxInUse = false;
    volatile bool txInUse = false;

  private:
    // pin and pad info
    uint32_t _rxPin;
    uint32_t _txPin;
    uint32_t _debugPin = 4;

    EPortType _txport;
    uint32_t _txpad;
    uint32_t _txpadMask;

    // byte information
    uint8_t _dataBits = 0; //5, 6, 7, or 8
    uint8_t _parity = 0;   //Type of parity (0, 1, 2)
    uint8_t _stopBits = 0;
    uint8_t _parityBits = 0; //Number of parity bits (0 or 1)
    bool _invertLogic;
    bool softwareserialSetConfig(uint16_t config);

    // timer
    void configTimer();
    void startTimer(uint16_t comp);
    void stopTimer();
    void restartTimer(uint16_t comp);

    //For RX
    void stopListening();
    volatile uint8_t rxBuffer[SWS_BUFFER_SIZE];
    volatile uint8_t rxBufferHead = 0;
    uint8_t rxBufferTail = 0;
    bool _rxBufferOverflow = false;

    uint16_t rxSysTicksPerBit = 0;
    uint32_t rxSysTicksPerByte = 0;
    uint16_t rxSysTicksPartialBit = 0;

    volatile uint8_t incomingByte = 0;
    volatile uint32_t lastBitTime = 0;
    volatile uint8_t bitCounter;
    volatile bool bitType = false;

    //For TX
    uint8_t txBuffer[SWS_BUFFER_SIZE];
    uint16_t txBufferHead = 0;
    volatile uint16_t txBufferTail = 0;
    bool _txBufferOverflow = false;

    volatile uint8_t outgoingByte = 0;
    uint8_t _parityForByte = 0; //Calculated per byte

    uint16_t txSysTicksPerBit = 0;
    uint16_t txSysTicksPerStopBit = 0;

    void beginTX();
    void calcParityBit();
};
#endif
