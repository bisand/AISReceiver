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

#define radio_tx_buf_len 850 // Size of radio tx ring buffer
#define radio_rx_buf_len 300 // Size of radio rx ring buffer
#define pinRXD_U D1

volatile byte rx_status = 0;               // Status of transmission
volatile byte status_flags_hi = B00000000; // Stream parameters
volatile byte status_flags_lo = B00000000;

volatile unsigned int time_out_timer;
volatile byte input_rx_buffer;                           // Temp input buffer from CMX589, 8 bit buffer is filled by RX_589.
volatile byte input_rx_buffer_bit_counter = 0;           // When 8 bits are received byte full wil be placed in radio_rx_buffer,
volatile int radio_rx_buffer_counter = radio_rx_buf_len; // pointer for end of buffer indication
volatile byte radio_tx_buffer[radio_tx_buf_len + 1];     // ringbuffer for datastream from CMX589
volatile int radio_tx_buffer_in_counter = 0;             // pointer for end of buffer indication
volatile int radio_tx_buffer_out_counter = 0;
volatile int radio_tx_buffer_bit_counter = 0;
volatile int radio_tx_buffer_gen_counter = 0;
volatile boolean rx_int_flag = false;

volatile byte temp_shift_buffer[23];                            // Fifo buffer for data analisys. 21 X 8 bits can be shift right.
volatile int temp_shift_buffer_counter = radio_rx_buf_len - 50; // When 8 bits are shifted a new byte pointed by counter will be
volatile byte temp_shift_buffer_bit_counter = 0;                // placed in temp_shift_buffer[0].

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
}

//========================================= Wait for 24 bit Preamble ===================================================
void Wait_for_preamble()
{
  if (temp_shift_buffer[2] == B10101010 && temp_shift_buffer[1] == B10101010 && temp_shift_buffer[0] == B10101010)
  {
    rx_status = 1;
    Serial.println("Preamble found!");
  }
}

//========================================= Wait for Start flag ===================================================
void Wait_for_start_flag()
{
  if (temp_shift_buffer[0] == B01111110)
  {
    rx_status = 2;
    Serial.println("Start flag found!");
  }
}

//========================================= Wait for Stop flag ===================================================
void Wait_for_stop_flag()
{
  if (temp_shift_buffer[0] == B01111110)
  {
    rx_status = 0;
    Serial.println("Stop flag found!!");
  }
}

//============================ Rotate temp_shift_buffer ===========================================================
void RX_buffer_shift()
{
  temp_shift_buffer_bit_counter += 1; // Increment curent possition

  if (temp_shift_buffer_bit_counter > 7) // Check if byte is full
  {
    temp_shift_buffer_bit_counter = 0;                                 // If full clear bit counter
    temp_shift_buffer[0] = radio_tx_buffer[temp_shift_buffer_counter]; //!       // Read byte from ringbuffer
    temp_shift_buffer_counter -= 1;                                    // Relocate ringbuffer
    if (temp_shift_buffer_counter == 0)
      temp_shift_buffer_counter = radio_rx_buf_len; // Reloop ringbuffer
  }
  // RX shift buffer
  for (int RX_buffer_shift_counter = 22; RX_buffer_shift_counter >= 0; RX_buffer_shift_counter--)
  {
    temp_shift_buffer[RX_buffer_shift_counter] = temp_shift_buffer[RX_buffer_shift_counter] >> 1;
    bitWrite(temp_shift_buffer[RX_buffer_shift_counter], 7, bitRead(temp_shift_buffer[RX_buffer_shift_counter - 1], 0));
  }
}
//========================================= Interrupt routine RX clk from CMX589 =================================
void RX_CMX589()
{

  time_out_timer += 1;
  bitWrite(input_rx_buffer, 7, digitalRead(pinRXD_U)); // Read RX data pin normal
  input_rx_buffer_bit_counter += 1;                    // Increment bit counter

  if (input_rx_buffer_bit_counter == 8) // If byte is full copy byte to ringbuffer
  {
    // PRINTBIN(input_rx_buffer);
    // Serial.print(" - ");
    // Serial.println(input_rx_buffer, HEX);

    radio_tx_buffer[radio_rx_buffer_counter] = input_rx_buffer; //!     // Copy byte to ringbuffer
    radio_rx_buffer_counter -= 1;                               // Rotate ringbuffer
    if (radio_rx_buffer_counter == 0)
      radio_rx_buffer_counter = radio_rx_buf_len; // Loop buffer around if needed
    input_rx_buffer_bit_counter = 0;              //
  }
  else
  {                                         // If byte is not full
    input_rx_buffer = input_rx_buffer >> 1; // Rotate input buffer
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
    RX_buffer_shift(); // Rotate RX buffer

    // Wait for preamble
    if ((rx_status == 0) && (rx_int_flag == true))
      Wait_for_preamble();

    // Wait for frame sync
    if ((rx_status == 1) && (rx_int_flag == true))
      Wait_for_start_flag();

    if ((rx_status == 2) && (rx_int_flag == true))
      Wait_for_stop_flag();
  }

  rx_int_flag = false;
}
