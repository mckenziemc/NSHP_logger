#include <AStar32U4.h>
#include <SD.h>

// TODO: rewrite to still forward data to USB if no SD card

AStar32U4LCD lcd;

const int SD_CS = 4;

String filename;
File log_file;

bool created_file = false;
bool write_to_file = true;

unsigned long byte_count = 0;

#define UART1_bitrate 115200

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#define RX_BUFFER_SIZE 1024
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_buffer_head = 0;
volatile uint16_t rx_buffer_tail = 0;
volatile bool rx_buffer_full = false;

unsigned long next_lcd_update = 0;

unsigned long next_voltage_check = 0;

uint8_t power_status;
#define POWER_BAD 0
#define POWER_OK 1
#define POWER_GOOD 2

uint16_t good_voltage = 5700;
uint16_t bad_voltage = 4800;

void setup() {
  // put your setup code here, to run once:
  lcd.clear();

  Serial.begin(115200);
  //while (!Serial) ;

  if (!SD.begin(SD_CS)) {
    Serial.println("Couldn't initialize SD card!");
    String s = "no SD";
    lcd.write(s.c_str(), s.length());
    write_to_file = false;
  } else {
  
    bool valid_file = false;
  
    // could count down and delete empty files, but 
    // there's potential value in allowing empty files 
    // to start a new section of names, versus requiring 
    // the user find a way to put something in the files.
    
    for (unsigned i = 0; i < 100; i++) {
      filename = "serial";
      
      if (i < 10) {
        filename += "0";
      }
  
      filename += i;
      filename += ".log";
  
      //Serial.println(filename);
      
      if (!SD.exists(filename)) {
        Serial.print(filename);
        Serial.println("is available.");
  
        lcd.write(filename.c_str(), filename.length());
  
        //log_file = SD.open(filename, FILE_WRITE);
        valid_file = true;
        break;
      }
    }
  
    if (!valid_file) {
      Serial.println("Could not create a log file!");
      String s = "no file";
      lcd.write(s.c_str(), s.length());
      write_to_file = false;
    }
  }

  begin_UART1(UART1_bitrate);
}

void loop() {
  if (millis() > next_voltage_check) {
    next_voltage_check += 1000;

    if (usbPowerPresent()) {
      power_status = POWER_GOOD;
    } else {
      // NOTE: this code assumes A-Star LV
      uint16_t voltage = readBatteryMillivoltsLV();
  
      if (voltage >= good_voltage) {
        power_status = POWER_GOOD;
      } else if (voltage > bad_voltage) {
        power_status = POWER_OK;
      } else {
        power_status = POWER_BAD;
      }
    }

    switch (power_status) {
      case POWER_GOOD:
        ledYellow(false);
        ledGreen(true);
        break;

      case POWER_OK:
        ledGreen(false);
        ledYellow(true);
        break;

      default:
        ledYellow(false);
        ledGreen(false);
        // FIXME: better sleep action
        // also pull NS-HP reset lines low?
        // Also allow user to connect USB, change batteries?
        while (true) ;
    }
  }

  if (rx_buffer_full || rx_buffer_tail != rx_buffer_head) {
    // potential issue if opening the file takes so much 
    // time that the serial buffer fills
    if (!created_file && write_to_file) {
      created_file = true;
      log_file = SD.open(filename, FILE_WRITE);
    }

    tend_rx_buffer();
    
  } else if (millis() >= next_lcd_update) {
    next_lcd_update = millis() + 1000;
  
    String line(byte_count);
    //Serial.println(byte_count);
    lcd.clear();
    lcd.write(filename.c_str(), filename.length());
    lcd.gotoXY(0, 1);
    lcd.write(line.c_str(), line.length());
  } else {
    log_file.flush();
  }
}

void forward_data(uint8_t* buffer, uint16_t length) {
  if (write_to_file) {
    log_file.write(buffer, length);
  } else {
    Serial.write(buffer, length);
  }
}

void tend_rx_buffer() {
  // cache tail to allow it to update
  uint16_t tail = rx_buffer_tail;
  
  if (tail > rx_buffer_head) {
    uint16_t len = tail - rx_buffer_head;
    
    forward_data(rx_buffer + rx_buffer_head, len);
    
    byte_count += len;

    // make the reset atomic
    noInterrupts();
    rx_buffer_head = tail;
    rx_buffer_full = false;
    interrupts();
  } else {
    uint16_t len = RX_BUFFER_SIZE - rx_buffer_head;
    forward_data(rx_buffer + rx_buffer_head, len);

    byte_count += len;
    
    // sync head and tail to allow a little more room if needed
    noInterrupts();
    rx_buffer_head = 0;
    rx_buffer_full = false;
    interrupts();
    
    tail = rx_buffer_tail;

    // could potentially be full again (head == tail) at this point?
    if (rx_buffer_full) {
      // head and tail will both be 0
      forward_data(rx_buffer, RX_BUFFER_SIZE);
      byte_count += RX_BUFFER_SIZE;
      
      rx_buffer_full = false;
      
    } else {
      forward_data(rx_buffer, tail);
      byte_count += tail;
      
      noInterrupts();
      rx_buffer_head = tail;
      rx_buffer_full = false;
      interrupts();
    }
  }
}

ISR(USART1_RX_vect)
{
  uint8_t byte_in = UDR1;

  if (!rx_buffer_full) {
    rx_buffer[rx_buffer_tail++] = byte_in;

    if (rx_buffer_tail == RX_BUFFER_SIZE) {
      rx_buffer_tail = 0;
    }

    if (rx_buffer_tail == rx_buffer_head) {
      rx_buffer_full = true;
    }
  }
}


// modification of HardwareSerial.cpp provided with Arduino 1.8.3, licensed 
// as GNU LGPL 2.1 or later
void begin_UART1(unsigned long baud)
{
  byte config = SERIAL_8N1;

  // Try u2x mode first
  uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
  UCSR1A = 1 << U2X1;

  // hardcoded exception for 57600 for compatibility with the bootloader
  // shipped with the Duemilanove and previous boards and the firmware
  // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
  // be > 4095, so switch back to non-u2x mode if the baud rate is too
  // low.
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
  {
    UCSR1A = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  UBRR1H = baud_setting >> 8;
  UBRR1L = baud_setting;

  //_written = false;

  //set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
  config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
  UCSR1C = config;
  
  sbi(UCSR1B, RXEN1);
  sbi(UCSR1B, TXEN1);
  sbi(UCSR1B, RXCIE1);
  cbi(UCSR1B, UDRIE1);
}

