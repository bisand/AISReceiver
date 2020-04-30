#include <Arduino.h>
#include <Pins_Arduino.h>
#include <SoftwareSerial.h>
#include "fifo.c"
#include "nmea.c"

#define PRINTBIN(Num)                                           \
  for (uint32_t t = (1UL << (sizeof(Num) * 8) - 1); t; t >>= 1) \
    Serial.write(Num &t ? '1' : '0'); // Prints a binary number with leading zeros (Automatic Handling)
#define PRINTBINL(Num)                        \
  for (int i = 0; i < (sizeof(Num) * 8); i++) \
    Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); // Prints a binary number with following Placeholder Zeros  (Automatic Handling)

SoftwareSerial ss(D6, D7); // Here are the aliased pins

void ICACHE_RAM_ATTR RX_CMX589();

#define ais_rx_buf_len 32 // Size of radio rx ring buffer
#define pinRXD_U D1

volatile byte rx_status = 0; // Status of transmission

volatile unsigned int time_out_timer;
volatile byte input_rx_buffer;                       // Temp input buffer from CMX589, 8 bit buffer is filled by RX_589.
volatile byte input_rx_buffer_bit_counter = 0;       // When 8 bits are received byte full wil be placed in ais_rx_buffer,
volatile int ais_rx_buffer_counter = ais_rx_buf_len; // pointer for end of buffer indication

// buffer: <8 bit ramp up><24 bit preamble><8 bit start flag><168 bit payload><16 bit CRC><8 bit stop flag><24 bit buffer>
volatile byte ais_rx_buffer[ais_rx_buf_len + 1];

volatile boolean rx_int_flag = false;
volatile boolean six_bit_ready = false;
volatile boolean eight_bit_ready = false;

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
  // ss.print("AT+DMOSETGROUP=1,162.0250,162.0250,0000,0,0000");
  ss.print("\r\n");
  ss.readStringUntil('\n');

  pinMode(D4, INPUT); // declare pushbutton as input
  pinMode(D2, INPUT);
  attachInterrupt(digitalPinToInterrupt(D2), RX_CMX589, RISING);

  Serial.println("");
  Serial.println("Searching...");
  for (size_t i = 0; i < sizeof(ais_rx_buf_len); i++)
  {
    ais_rx_buffer[i] = 0;
  }
}

//========================================= Wait for 24 bit Preamble ===================================================
void Wait_for_preamble()
{
  if ((ais_rx_buffer[ais_rx_buf_len - 2] == B01010101 && ais_rx_buffer[ais_rx_buf_len - 1] == B01010101 && ais_rx_buffer[ais_rx_buf_len] == B01010101) ||
      (ais_rx_buffer[ais_rx_buf_len - 2] == B10101010 && ais_rx_buffer[ais_rx_buf_len - 1] == B10101010 && ais_rx_buffer[ais_rx_buf_len] == B10101010))
  {
    rx_status = 1;
    Serial.println(millis());
    Serial.println(" - Preamble found!");
    input_rx_buffer_bit_counter = 0;
  }
}

//========================================= Wait for Start flag ===================================================
void Wait_for_start_flag()
{
  if (ais_rx_buffer[ais_rx_buf_len] == B01111110)
  {
    rx_status = 2;
    Serial.println("Start flag found!");
  }
}

//========================================= Wait for Stop flag ===================================================
void Wait_for_stop_flag()
{
  if (ais_rx_buffer[ais_rx_buf_len] == B01111110)
  {
    rx_status = 0;
    Serial.println("Stop flag found!!");
    for (int i = 0; i < sizeof(ais_rx_buffer); i++)
      PRINTBINL(ais_rx_buffer[i]);
    Serial.println();
  }
}

//============================ Rotate temp_shift_buffer ===========================================================
void RX_buffer_shift()
{
  size_t size = sizeof(ais_rx_buffer);
  for (size_t i = 0; i < size; i++)
  {
    // if (ais_rx_buffer[i] < 0x10)
    //   Serial.print('0');
    // Serial.print(ais_rx_buffer[i], HEX);
    // PRINTBINL(ais_rx_buffer[i]);
    ais_rx_buffer[i] = ais_rx_buffer[i] << 1;
    if (i < size - 1)
      bitWrite(ais_rx_buffer[i], 0, bitRead(ais_rx_buffer[i + 1], 7));
  }
}
//========================================= Interrupt routine RX clk from CMX589 =================================
void RX_CMX589()
{
  time_out_timer += 1;
  bitWrite(input_rx_buffer, 0, digitalRead(pinRXD_U)); // Read RX data pin normal
  input_rx_buffer_bit_counter += 1;                    // Increment bit counter
  ais_rx_buffer[ais_rx_buf_len] = input_rx_buffer;     //!     // Copy byte to ringbuffer
  input_rx_buffer = input_rx_buffer << 1;              // Rotate input buffer

  if (rx_status == 2 && input_rx_buffer_bit_counter == 6) // Start collecting 6 bit data stream.
  {
    // ais_rx_buffer[ais_rx_buf_len] = input_rx_buffer; //!     // Copy byte to ringbuffer
    // input_rx_buffer = 0;
    six_bit_ready = true;
    input_rx_buffer_bit_counter = 0; //
  }
  else if (input_rx_buffer_bit_counter == 8) // If byte is full copy byte to ringbuffer
  {
    // ais_rx_buffer[ais_rx_buf_len] = input_rx_buffer; //!     // Copy byte to ringbuffer
    // input_rx_buffer = 0;
    eight_bit_ready = true;
    input_rx_buffer_bit_counter = 0; //
  }
  else
  { // If byte is not full
    // input_rx_buffer = input_rx_buffer << 1; // Rotate input buffer
  }
  rx_int_flag = true; // Set rx_int_flag for general purposes.
}

void loop()
{
  // if (Serial.available())
  // {                          // If anything comes in Serial (USB),
  //   ss.write(Serial.read()); // read it and send it out Serial1 (pins 0 & 1)
  // }

  // if (ss.available())
  // {                          // If anything comes in Serial1 (pins 0 & 1)
  //   Serial.write(ss.read()); // read it and send it out Serial (USB)
  // }

  if (rx_int_flag == true)
  {
    RX_buffer_shift();
    // Wait for preamble
    if ((rx_status == 0) && (rx_int_flag == true))
    {
      Wait_for_preamble();
    }
    // Wait for frame sync
    else if ((rx_status == 1) && (rx_int_flag == true))
    {
      Wait_for_start_flag();
    }
    else if ((rx_status == 2) && (rx_int_flag == true))
    {
      Wait_for_stop_flag();
    }
    if (eight_bit_ready == true)
    {
      // size_t size = sizeof(ais_rx_buffer);
      // for (size_t i = 0; i < size; i++)
      // {
      //   eight_bit_ready = false;
      //   if (ais_rx_buffer[i] < 0x10)
      //     Serial.print('0');
      //   Serial.print(ais_rx_buffer[i], HEX);
      // }
      // Serial.println();
    }
  }

  rx_int_flag = false;
}
