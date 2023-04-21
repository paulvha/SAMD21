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
    and you do a Serial.print() the print will begin immediately and any additional
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

#include "SoftwareSerial.h"
#include "variant.h"

volatile bool IsActive = false;
volatile bool firstTimeRunning = false;
SoftwareSerial *SWS_handle = 0;

#define SWS_TC         TC4        // as we use 32 bit counter TC5 is also used
#define SWS_TC_IRQn    TC4_IRQn

//#define DEBUG

void TC4_Handler (void) __attribute__ ((weak, alias("SwS_Handler")));

static inline void resetTC (Tc* TCx)
{
  // Disable TCx
  TCx->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while(TCx->COUNT32.STATUS.bit.SYNCBUSY);

  // Reset TCx
  TCx->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
  while(TCx->COUNT32.STATUS.bit.SYNCBUSY);
  while (TCx->COUNT32.CTRLA.bit.SWRST);
}

inline void _software_serial_isr(void)
{
  SWS_handle->rxBit();
}

SoftwareSerial::SoftwareSerial(uint32_t rxPin, uint32_t txPin, bool invertLogic)
{
  _rxPin = rxPin;
  _txPin = txPin;

  _invertLogic = invertLogic;
}

bool SoftwareSerial::begin(uint32_t baudRate)
{
  return(begin(baudRate, SERIAL_8N1));
}

bool SoftwareSerial::begin(uint32_t baudRate, uint16_t config)
{

  // in case baudrate change is requested
  if (IsActive) end();
  IsActive = true;

  if ( g_APinDescription[_rxPin].ulPinType == PIO_NOT_A_PIN )
  {
    return false ;
  }

  if ( g_APinDescription[_txPin].ulPinType == PIO_NOT_A_PIN )
  {
    return false ;
  }

  if (!firstTimeRunning)
  {
    firstTimeRunning = true;

    NVIC_SetPriority(SWS_TC_IRQn, 0);

    // Enable GCLK for TC4 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY);
  }

  stopTimer();

#ifdef DEBUG
  digitalWrite(_debugPin, LOW);
  pinMode(_debugPin, OUTPUT);
#endif

  // set _RXPin
  if (! _invertLogic)  pinMode(_rxPin, INPUT_PULLUP);
  else pinMode(_rxPin, INPUT_PULLDOWN);

  // set TXpin information
  digitalWrite(_txPin, _invertLogic ? LOW : HIGH);
  pinMode(_txPin, OUTPUT);

  _txport = g_APinDescription[_txPin].ulPort;
  _txpad = g_APinDescription[_txPin].ulPin;
  _txpadMask = (1ul << _txpad);

  // set config
  softwareserialSetConfig(config);

  // calculate timing
  rxSysTicksPerBit = (TIMER_FREQ / baudRate) * 0.93; //Shorten the number of sysTicks a small amount because we are doing a mod operation
  txSysTicksPerBit = (TIMER_FREQ / baudRate) - 8;   //Shorten the txSysTicksPerBit by the number of ticks needed to run the txHandler ISR

  // fine tine (as much as possible)
  if (baudRate == 19200) txSysTicksPerBit = txSysTicksPerBit - 4;
  if (baudRate == 38400) txSysTicksPerBit = txSysTicksPerBit - 4;
  if (baudRate == 57600) txSysTicksPerBit = txSysTicksPerBit - 5;
  if (baudRate == 115200) txSysTicksPerBit = txSysTicksPerBit - 5;

  rxSysTicksPerByte = (TIMER_FREQ / baudRate) * (_dataBits + _parityBits + _stopBits);

  //During RX, if leftover systicks is more than a fraction of a bit, we will call it a bit
  //This is needed during high speed when cmpr ISR extends into the start bit of the following byte
  rxSysTicksPartialBit = rxSysTicksPerBit / 4;

  txSysTicksPerStopBit = txSysTicksPerBit * _stopBits;

  // set handle as this instance
  SWS_handle = this;

  // Config the timer
  configTimer();

  // start waiting input
  listen();

  return true;
}

void SoftwareSerial::end(void)
{
  stopTimer();
  resetTC(SWS_TC);
  pinMode(_txPin, INPUT);
  pinMode(_rxPin, INPUT);
  IsActive = false;
}

int SoftwareSerial::available()
{
  return (rxBufferHead + SWS_BUFFER_SIZE - rxBufferTail) % SWS_BUFFER_SIZE;
}

//Returns true if overflow flag is set
//Clears flag when called
bool SoftwareSerial::overflow()
{
  if (_rxBufferOverflow || _txBufferOverflow)
  {
    _rxBufferOverflow = false;
    _txBufferOverflow = false;
    return (true);
  }
  return (false);
}

int SoftwareSerial::read()
{
  if (available() == 0)
    return (-1);

  rxBufferTail++;
  rxBufferTail %= SWS_BUFFER_SIZE;
  return (rxBuffer[rxBufferTail]);
}

int SoftwareSerial::peek()
{
  if (available() == 0)
    return (-1);

  uint8_t tempTail = rxBufferTail + 1;
  tempTail %= SWS_BUFFER_SIZE;
  return (rxBuffer[tempTail]);
}

void SoftwareSerial::flush()
{
  while (txInUse)
  {
  }
}

//ISR that is called each bit transition on RX pin
void SoftwareSerial::rxBit(void)
{
  uint32_t bitTime;
#ifdef DEBUG
  digitalWrite(_debugPin, HIGH);
#endif

  if (lastBitTime == 0)
  {
    startTimer(rxSysTicksPerByte);  // set BYTE time out
    bitCounter = 0;
    bitType = false;
    rxInUse = true;                 // Indicate we are now in process of receiving a byte
    lastBitTime = SWS_TC->COUNT32.COUNT.reg;
  }
  else
  {
    bitTime = SWS_TC->COUNT32.COUNT.reg; // Capture current running

    //Calculate the number of bits that have occured since last PCI
    uint8_t numberOfBits = (bitTime - lastBitTime) / rxSysTicksPerBit;

    if (bitCounter == 0)
    {
      //Catch any partial bits
      //For very high bauds the final interrupt spills over into the
      //start bit of the next byte. This catches the partial systicks and correctly
      //identifies the start bit as such.
      uint16_t partialBits = (bitTime - lastBitTime) % rxSysTicksPerBit;
      if (partialBits > rxSysTicksPartialBit)
      {
#ifdef DEBUG
//      SerialUSB.println("Partial!");
#endif
        numberOfBits++;
      }

      //Exclude start bit from byte shift
      bitCounter++;
      numberOfBits--;
    }

    if (_parity)
    {
      if (numberOfBits + bitCounter > _dataBits + _parityBits)
      {
        numberOfBits--; //Exclude parity bit from byte shift
      }
    }

    for (uint8_t y = 0; y < numberOfBits; y++) //Add bits of the current bitType (either 1 or 0) to our byte
    {
      incomingByte >>= 1;
      if (bitType == true)
        incomingByte |= 0x80;
    }
    bitCounter += numberOfBits;
    bitType = !bitType;    //Next bit will be inverse of this bit
    lastBitTime = bitTime; //Remember this bit time as the starting time for the next PCI
  }

#ifdef DEBUG
  digitalWrite(_debugPin, LOW);
#endif
}

//Starts the transmission of the next available byte from the buffer
void SoftwareSerial::beginTX()
{
  bitCounter = 0;

  //Initiate start bit
  if (_invertLogic == false)
  {
    PORT->Group[_txport].OUTCLR.reg = _txpadMask; //Normal logic, low is start bit
  }
  else
  {
    PORT->Group[_txport].OUTSET.reg = _txpadMask;
  }

  startTimer(txSysTicksPerBit);
}

//Assumes the global variables have been set: _parity, _dataBits, outgoingByte
//Sets global variable _parityBit
void SoftwareSerial::calcParityBit()
{
  if (_parity == 0)
    return; //No parity

  uint8_t ones = 0;
  for (uint8_t x = 0; x < _dataBits; x++)
  {
    if (outgoingByte & (0x01 << x))
    {
      ones++;
    }
  }

  if (_parity == 1) //Odd
  {
    _parityForByte = !(ones % 2);
  }
  else //Even
  {
    _parityForByte = (ones % 2);
  }
}

void SoftwareSerial::txHandler()
{
#ifdef DEBUG
  digitalWrite(_debugPin, HIGH);
#endif

  if (bitCounter < _dataBits) // Data bits 0 to 7
  {
    restartTimer(txSysTicksPerBit);                   //Direct reg write to decrease execution time
      if (outgoingByte & 0x01) PORT->Group[_txport].OUTSET.reg = _txpadMask;
      else PORT->Group[_txport].OUTCLR.reg = _txpadMask;

    /*if (_invertLogic == false){ // normal operation
      if (outgoingByte & 0x01) PORT->Group[_txport].OUTSET.reg = _txpadMask;
      else PORT->Group[_txport].OUTCLR.reg = _txpadMask;
    }
    else {
      if (outgoingByte & 0x01) PORT->Group[_txport].OUTCLR.reg = _txpadMask;
      else PORT->Group[_txport].OUTSET.reg = _txpadMask;
    }
*/
    outgoingByte >>= 1;
    bitCounter++;
  }
  else if (bitCounter == _dataBits) //Send parity bit or stop bit(s)
  {
    if (_parity)
    {
      //Send parity bit
      restartTimer(txSysTicksPerBit);                   //Direct reg write to decrease execution time
      if (_parityForByte)
      {
        if (_invertLogic == false) PORT->Group[_txport].OUTSET.reg = _txpadMask;
        else PORT->Group[_txport].OUTCLR.reg = _txpadMask;
      }
      else
      {
        if (_invertLogic == false) PORT->Group[_txport].OUTCLR.reg = _txpadMask;
        else PORT->Group[_txport].OUTSET.reg = _txpadMask;
      }
    }
    else
    {
      //Send stop bit
      restartTimer(txSysTicksPerStopBit);                   //Direct reg write to decrease execution time

      if (_invertLogic == false) PORT->Group[_txport].OUTSET.reg = _txpadMask;
      else PORT->Group[_txport].OUTCLR.reg = _txpadMask;

    }
    bitCounter++;
  }
  else if (bitCounter == (_dataBits + 1))                 //Send stop bit or begin next byte
  {
    if (_parity)
    {
      //Send stop bit
      restartTimer(txSysTicksPerStopBit);                 //Direct reg write to decrease execution time

      if (_invertLogic == false) PORT->Group[_txport].OUTSET.reg = _txpadMask;
      else PORT->Group[_txport].OUTCLR.reg = _txpadMask;

      bitCounter++;
    }
    else
    {
      //If done sending
      if (txBufferTail == txBufferHead)
      {
        // Disable the timer interrupt in the NVIC.
        stopTimer();

        //All done!
        txInUse = false;

        //Reattach PCI so we can hear rx bits coming in
        listen();
      }
      else
      {
        //Send next byte in buffer
        txBufferTail = (txBufferTail + 1) % SWS_BUFFER_SIZE;
        outgoingByte = txBuffer[txBufferTail];
        calcParityBit();
        beginTX();
      }
    }
  }
  else if (bitCounter == (_dataBits + 2)) //Begin next byte
  {
    //if done sending
    if (txBufferTail == txBufferHead)
    {
      // Disable the timer interrupt in the NVIC.
      stopTimer();

      //All done!
      txInUse = false;

      //Reattach PCI so we can hear rx bits coming in
      listen();
    }
    else
    {
      //Send next byte in buffer
      txBufferTail = (txBufferTail + 1) % SWS_BUFFER_SIZE;
      outgoingByte = txBuffer[txBufferTail];
      calcParityBit();
      beginTX();
    }
  }
#ifdef DEBUG
  digitalWrite(_debugPin, LOW);
#endif
}

void SoftwareSerial::rxEndOfByte()
{
  stopTimer();

  //Finish out bytes that are less than 8 bits
#ifdef DEBUG
   // SerialUSB.println("bitCounter: ");
   // SerialUSB.println(bitCounter);
   // SerialUSB.println("incoming: ");
   // SerialUSB.println(incomingByte, HEX);

#endif
  bitCounter--; //Remove start bit from count

  //Edge case where we need to do an additional byte shift because we had data bits followed by a parity bit of same value
  if (_parity)
  {
    bitCounter = bitCounter - _parityBits; //Remove parity bit from count
    if (bitType == true)
      bitCounter++;
  }

  while (bitCounter < 8)
  {
    incomingByte >>= 1;
    if (bitType == true)
      if (bitCounter < _dataBits)
      {
        incomingByte |= 0x80;
      }
    bitCounter++;
  }

  //TODO - Check parity bit if parity is enabled

  if (_invertLogic)
    incomingByte = ~incomingByte;

  //See if we are going to overflow buffer
  uint8_t nextSpot = (rxBufferHead + 1) % SWS_BUFFER_SIZE;
  if (nextSpot != rxBufferTail)
  {
    //Add this byte to the buffer
    rxBuffer[nextSpot] = incomingByte;
    rxBufferHead = nextSpot;
  }
  else
  {
    _rxBufferOverflow = true;
  }

  lastBitTime = 0; //Reset for next byte

  rxInUse = false; //Release so that we can TX if needed
}

size_t SoftwareSerial::write(uint8_t toSend)
{
  //As soon as user wants to send something, turn off RX interrupts
  if (txInUse == false)
  {
    detachInterrupt(_rxPin);

    rxInUse = false;
  }

  //See if we are going to overflow buffer
  uint8_t nextSpot = (txBufferHead + 1) % SWS_BUFFER_SIZE;
  if (nextSpot != txBufferTail)
  {
    //Add this byte into the circular buffer
    txBuffer[nextSpot] = toSend;
    txBufferHead = nextSpot;
  }
  else
  {
    _txBufferOverflow = true;
  }

  //See if hardware is available
  if (txInUse == false)
  {
    txInUse = true;

    //Start sending this byte immediately
    txBufferTail = (txBufferTail + 1) % SWS_BUFFER_SIZE;
    outgoingByte = txBuffer[txBufferTail];

    //Calc parity
    calcParityBit();

    beginTX();
  }

  return (1);
}

size_t SoftwareSerial::write(const uint8_t *buffer, size_t size)
{
  for (uint16_t x = 0; x < size; x++)
  {
    write(buffer[x]);
  }
  return (size);
}

size_t SoftwareSerial::write(const char *str)
{
  if (str == NULL)  return 0;
  return write((const uint8_t *)str, strlen(str));
}

bool SoftwareSerial::softwareserialSetConfig(uint16_t config)
{
  bool retval = true;
  switch (config)
  {
    case SERIAL_5N1:
      _dataBits = 5;
      _parity = 0;
      _stopBits = 1;
      break;
    case SERIAL_6N1:
      _dataBits = 6;
      _parity = 0;
      _stopBits = 1;
      break;
    case SERIAL_7N1:
      _dataBits = 7;
      _parity = 0;
      _stopBits = 1;
      break;
    case SERIAL_8N1:
      _dataBits = 8;
      _parity = 0;
      _stopBits = 1;
      break;
    case SERIAL_5N2:
      _dataBits = 5;
      _parity = 0;
      _stopBits = 2;
      break;
    case SERIAL_6N2:
      _dataBits = 6;
      _parity = 0;
      _stopBits = 2;
      break;
    case SERIAL_7N2:
      _dataBits = 7;
      _parity = 0;
      _stopBits = 2;
      break;
    case SERIAL_8N2:
      _dataBits = 8;
      _parity = 0;
      _stopBits = 2;
      break;
    case SERIAL_5E1:
      _dataBits = 5;
      _parity = 2;
      _stopBits = 1;
      break;
    case SERIAL_6E1:
      _dataBits = 6;
      _parity = 2;
      _stopBits = 1;
      break;
    case SERIAL_7E1:
      _dataBits = 7;
      _parity = 2;
      _stopBits = 1;
      break;
    case SERIAL_8E1:
      _dataBits = 8;
      _parity = 2;
      _stopBits = 1;
      break;
    case SERIAL_5E2:
      _dataBits = 5;
      _parity = 2;
      _stopBits = 2;
      break;
    case SERIAL_6E2:
      _dataBits = 6;
      _parity = 2;
      _stopBits = 2;
      break;
    case SERIAL_7E2:
      _dataBits = 7;
      _parity = 2;
      _stopBits = 2;
      break;
    case SERIAL_8E2:
      _dataBits = 8;
      _parity = 2;
      _stopBits = 2;
      break;
    case SERIAL_5O1:
      _dataBits = 5;
      _parity = 1;
      _stopBits = 1;
      break;
    case SERIAL_6O1:
      _dataBits = 6;
      _parity = 1;
      _stopBits = 1;
      break;
    case SERIAL_7O1:
      _dataBits = 7;
      _parity = 1;
      _stopBits = 1;
      break;
    case SERIAL_8O1:
      _dataBits = 8;
      _parity = 1;
      _stopBits = 1;
      break;
    case SERIAL_5O2:
      _dataBits = 5;
      _parity = 1;
      _stopBits = 2;
      break;
    case SERIAL_6O2:
      _dataBits = 6;
      _parity = 1;
      _stopBits = 2;
      break;
    case SERIAL_7O2:
      _dataBits = 7;
      _parity = 1;
      _stopBits = 2;
      break;
    case SERIAL_8O2:
      _dataBits = 8;
      _parity = 1;
      _stopBits = 2;
      break;
    default:
      retval = false;
      break;
  }

  _parityBits = 0;
  if (_parity)
    _parityBits = 1;

  return retval;
}

void SoftwareSerial::configTimer()
{
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Use the 48MHz system clock
                     GCLK_GENDIV_ID(5);            // Set division on Generic Clock Generator (GCLK) 5
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK 5
                      GCLK_GENCTRL_SRC_DFLL48M |   // Set the clock source to 48MHz
                      GCLK_GENCTRL_ID(5);          // Set clock source on GCLK 5
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK5 |     // ....on GCLK5
                      GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK5 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  TC4->COUNT32.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 |     // Set prescaler to 8, 48MHz/8 = 6Mhz
                            TC_CTRLA_PRESCSYNC_PRESC |    // Reload timer on next prescaler clock
                            TC_CTRLA_MODE_COUNT32;        // Set the TC4 timer to 32-bit mode in conjuction with timer TC5
}

void SoftwareSerial::startTimer(uint16_t comp)
{
  SWS_TC->COUNT32.COUNT.reg = (uint32_t)(0);

  SWS_TC->COUNT32.CC[1].reg = comp;                   // set the moment to trigger
  while ( SWS_TC->COUNT32.STATUS.bit.SYNCBUSY );      // wait for sync

  SWS_TC->COUNT32.INTENSET.bit.MC1 = 1;               // Enable the SWS_TC interrupt request

  SWS_TC->COUNT32.CTRLA.bit.ENABLE = 1;               // Enable SWS_TC
  while (SWS_TC->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  SWS_TC->COUNT32.READREQ.reg = TC_READREQ_RCONT |    // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);   // Offset of the 32-bit COUNT register
  while (SWS_TC->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for (read) synchronization

  NVIC_EnableIRQ(SWS_TC_IRQn);                        // Enable the timer interrupt in the NVIC.
}

void SoftwareSerial::stopTimer()
{
  SWS_TC->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while ( SWS_TC->COUNT32.STATUS.bit.SYNCBUSY );      // wait for sync

  NVIC_DisableIRQ(SWS_TC_IRQn);                       // disable time interrupt
  NVIC_ClearPendingIRQ(SWS_TC_IRQn);
}

void SoftwareSerial::restartTimer(uint16_t comp)
{
  SWS_TC->COUNT32.COUNT.reg = (uint32_t)(0);
  SWS_TC->COUNT32.CC[1].reg = comp;                   // set the moment to interrupt
  while ( SWS_TC->COUNT32.STATUS.bit.SYNCBUSY );      // wait for sync
}

void SoftwareSerial::listen()
{
  // see example4_multiport
  if (SWS_handle != this) {
    SWS_handle->stopListening();                     // Gracefully shut down previous instance
  }

  lastBitTime = 0;

  SWS_handle = this;

  // Attach callback to RX pin
  attachInterrupt(_rxPin, _software_serial_isr, CHANGE);
}

void SoftwareSerial::stopListening()
{
  // Disable the timer interrupt in the NVIC.
  stopTimer();

  detachInterrupt(_rxPin);

  SWS_handle == NULL;
}

bool SoftwareSerial::isListening()
{
  return (this == SWS_handle);
}

#ifdef __cplusplus
extern "C" {
#endif

void SwS_Handler(void)
{
  // Clear the interrupt
  SWS_TC->COUNT32.INTFLAG.bit.MC1 = 1;

  if (SWS_handle->rxInUse == true)
  {
    SWS_handle->rxEndOfByte();
  }
  else if (SWS_handle->txInUse == true)
  {
    SWS_handle->txHandler();
  }
}

#ifdef __cplusplus
}
#endif
