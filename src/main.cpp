#include <Arduino.h>
#include <Pins_Arduino.h>
#include <SoftwareSerial.h>
#include "fifo.c"
#include "nmea.c"

SoftwareSerial ss(D6, D7); // Here are the aliased pins

void ICACHE_RAM_ATTR rxINT();

static uint8_t rx_pkt[128];
static uint8_t pkt_size;

uint8_t rxstate;
uint16_t bitcount;

enum eRxState
{
  RX_INIT,
  RX_PREAMBLE,
  RX_START,
  RX_PRELOAD,
  RX_DATA
};

unsigned long vhfMillis = 0;
bool scan = false;
unsigned long buttonMillis = 0;

void setup()
{
  Serial.begin(115200);
  ss.begin(9600);

  ss.print("AT+DMOCONNECT");
  ss.print("\r\n");
  ss.readStringUntil('\n');
  ss.print("AT+SETFILTER=1,1,1");
  ss.print("\r\n");
  ss.readStringUntil('\n');
  ss.print("AT+DMOSETVOLUME=8");
  ss.print("\r\n");
  ss.readStringUntil('\n');
  ss.print("AT+DMOSETGROUP=1,161.9750,161.9750,0000,0,0000");
  ss.print("\r\n");
  ss.readStringUntil('\n');

  pinMode(D4, INPUT); // declare pushbutton as input
  pinMode(D2, INPUT);
  attachInterrupt(digitalPinToInterrupt(D2), rxINT, RISING);
  FIFO_Init();
}

void loop()
{
  if (Serial.available())
  {                          // If anything comes in Serial (USB),
    ss.write(Serial.read()); // read it and send it out Serial1 (pins 0 & 1)
  }

  if (ss.available())
  {                          // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(ss.read()); // read it and send it out Serial (USB)
  }

  //if (millis() > buttonMillis)
  { // check if the input is HIGH (button released)
    buttonMillis = millis() + 1UL;

    pkt_size = FIFO_Get_Packet(rx_pkt, 128); //SIZE OF BUFFER

    // A good packet must be bigger than the 2-byte CRC !
    if (pkt_size > 2)
    {
      // Send NMEA message - discarding the CRC
      NMEA_Send(rx_pkt, pkt_size - 2);
    }
  }
}

void rxINT()
{
  //static local variables retain their value when the function is exited
  uint8_t dat;
  static uint8_t lastdat; //removing storage class specifier: static should break functionality
  static uint8_t dbyte;
  static uint16_t shiftreg;
  static uint16_t newbit;
  static uint16_t crcreg;

  dat = digitalRead(D1);

  // NRZI Decode latest bit and shift into register
  shiftreg <<= 1;
  if (dat == lastdat)
    // Data the same = 1
    shiftreg |= 0x0001;
  else
    // Data changed = 0;
    shiftreg &= 0xfffe;

  // Save the last state of the data line
  lastdat = dat;

  // Execute state-machine
  switch (rxstate)
  {
  // Set RxDCaqc and PLLacq after an initial delay
  // Go to the RX_PREAMBLE state
  case RX_INIT:
    if (!(--bitcount))
    {
      rxstate = RX_PREAMBLE;
      shiftreg = 0l;
    }
    break;

  // Look for 16 bits of preamble
  case RX_PREAMBLE:
    if (shiftreg == 0x5555)
    {
      bitcount = 0x00;
      rxstate = RX_START;
    }
    break;

  // Look for start flag
  case RX_START:
    //Serial.println("RX_START");
    if ((shiftreg & 0x00ff) == 0x007e)
    {
      bitcount = 0;
      rxstate = RX_PRELOAD;
    }
    else if (bitcount >= 24)
    {
      // Start flag not found - go back and look for
      // preamble
      rxstate = RX_PREAMBLE;
    }
    break;

  // Load an initial 8 bits into the shift register.
  // This means that when we see the end flag we have just
  // passed the last CRC bit through the CRC calculation
  case RX_PRELOAD:
    //Serial.println("RX_PRELOAD");
    if (++bitcount == 8)
    {
      // Go to the receiving state
      bitcount = 0;
      crcreg = 0xffff;
      rxstate = RX_DATA;
      FIFO_Reset_Packet();
    }
    break;

  // Receive data
  case RX_DATA:

    //Serial.println("RX_DATA");
    // Remove stuffing bits
    // An extra zero is inserted after five 1's by the transmitter
    if ((shiftreg & 0x3f00) != 0x3e00)
    {
      // It's not a stuffing bit

      // Increment the bit count
      ++bitcount;
      // Extract the new data bit
      newbit = (shiftreg >> 8) & 0x0001;
      // Shift new bit into a byte
      dbyte = (dbyte >> 1) | ((shiftreg >> 1) & 0x0080);

      // If 8 bits received but into FIFO
      if (!(bitcount & 0x07))
        FIFO_Put(dbyte);

      // Pass new bit through CRC calculation
      if ((crcreg ^ newbit) & 0x0001)
        // Xor with the CRC polynomial (X^16 + X^12 + X^5 + 1)
        crcreg = (crcreg >> 1) ^ 0x8408;
      else
      {
        // There appears to be a bug in the Keil compiler - right shift
        // doesn't work unless something else is done on the same line.
        // Here I xor shifted result with 1 and then xor again to correct
        // this back
        crcreg = (crcreg >> 1); // ^ 0x0001;
        //crcreg ^= 0x0001;
      }
    }

    // Have we got an end flag ?
    if ((shiftreg & 0x00ff) == 0x007e)
    {
      //Serial.println("End");
      // Reset state machine to look for preamble
      rxstate = RX_PREAMBLE;
      // Check for good CRC
      // This should give a result of 0xF0B8
      if (crcreg == 0xf0b8)
      {
        // Good packet received
        // ... Release into FIFO
        FIFO_Write_Packet();
        Serial.printf("%04X\n", crcreg);
        Serial.printf("\n---%d\n", bitcount);
      }
    }
    break;

  default:
    break;
  }
  // Clear the GPIO interrupt.
  //GIFR |= (1 << INTF0); // cleared automatically, alternatively can clear by writing logical 1 to it.
}