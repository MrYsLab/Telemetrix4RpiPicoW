/*
  Copyright (c) 2020-2022 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,f
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


// This file is rather large, so it has been rearranged in logical sections.
// Here is the list of sections to help make it easier to locate items of interest,
// and aid when adding new features.

// 1. Feature Enabling Defines
// 2. Arduino ID
// 3. Client Command Related Defines and Support
// 4. Server Report Related Defines
// 5. i2c Related Defines
// 6. Pin Related Defines And Data Structures
// 7. Feature Related Defines, Data Structures and Storage Allocation
// 8. Command Functions
// 9. Scanning Inputs, Generating Reports And Running Steppers
// 10. Setup and Loop



#include <Arduino.h>
#include "Telemetrix4RpiPicoW.h"
#include <Servo.h>
#include <NanoConnectHcSr04.h>
#include <Wire.h>
#include <dhtnew.h>
#include <SPI.h>
//#include <OneWire.h> // library is not available for the pico
#include <AccelStepper.h>
#include <NeoPixelConnect.h>



/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*         Client Command Related Defines and Support               */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Commands Sent By The Client


// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define SERVO_ATTACH 6
#define SERVO_WRITE 7
#define SERVO_DETACH 8
#define I2C_BEGIN 9
#define I2C_READ 10
#define I2C_WRITE 11
#define SONAR_NEW 12
#define DHT_NEW 13
#define STOP_ALL_REPORTS 14
#define SET_ANALOG_SCANNING_INTERVAL 15
#define ENABLE_ALL_REPORTS 16
#define RESET 17
#define SPI_INIT 18
#define SPI_WRITE_BLOCKING 19
#define SPI_READ_BLOCKING 20
#define SPI_SET_FORMAT 21
#define SPI_CS_CONTROL 22
#define ONE_WIRE_INIT 23
#define ONE_WIRE_RESET 24
#define ONE_WIRE_SELECT 25
#define ONE_WIRE_SKIP 26
#define ONE_WIRE_WRITE 27
#define ONE_WIRE_READ 28
#define ONE_WIRE_RESET_SEARCH 29
#define ONE_WIRE_SEARCH 30
#define ONE_WIRE_CRC8 31
#define SET_PIN_MODE_STEPPER 32
#define STEPPER_MOVE_TO 33
#define STEPPER_MOVE 34
#define STEPPER_RUN 35
#define STEPPER_RUN_SPEED 36
#define STEPPER_SET_MAX_SPEED 37
#define STEPPER_SET_ACCELERATION 38
#define STEPPER_SET_SPEED 39
#define STEPPER_SET_CURRENT_POSITION 40
#define STEPPER_RUN_SPEED_TO_POSITION 41
#define STEPPER_STOP 42
#define STEPPER_DISABLE_OUTPUTS 43
#define STEPPER_ENABLE_OUTPUTS 44
#define STEPPER_SET_MINIMUM_PULSE_WIDTH 45
#define STEPPER_SET_ENABLE_PIN 46
#define STEPPER_SET_3_PINS_INVERTED 47
#define STEPPER_SET_4_PINS_INVERTED 48
#define STEPPER_IS_RUNNING 49
#define STEPPER_GET_CURRENT_POSITION 50
#define STEPPER_GET_DISTANCE_TO_GO 51
#define STEPPER_GET_TARGET_POSITION 52
#define RESET_BOARD 53
#define INIT_NEOPIXELS 54
#define SHOW_NEOPIXELS 55
#define SET_NEOPIXEL 56
#define CLEAR_NEOPIXELS 57
#define FILL_NEOPIXELS 58
#define PWM_FREQ 59
#define PWM_RANGE 60
#define GET_CPU_TEMP 61

/* Command Forward References*/

// If you add a new command, you must add the command handler
// here as well.

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void analog_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void dht_new();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

extern void reset_data();

extern void init_pin_structures();

extern void init_spi();

extern void write_blocking_spi();

extern void read_blocking_spi();

extern void set_format_spi();

extern void spi_cs_control();

extern void onewire_init();

extern void onewire_reset();

extern void onewire_select();

extern void onewire_skip();

extern void onewire_write();

extern void onewire_read();

extern void onewire_reset_search();

extern void onewire_search();

extern void onewire_crc8();

extern void set_pin_mode_stepper();

extern void stepper_move_to();

extern void stepper_move();

extern void stepper_run();

extern void stepper_run_speed();

extern void stepper_set_max_speed();

extern void stepper_set_acceleration();

extern void stepper_set_speed();

extern void stepper_get_distance_to_go();

extern void stepper_get_target_position();

extern void stepper_get_current_position();

extern void stepper_set_current_position();

extern void stepper_run_speed_to_position();

extern void stepper_stop();

extern void stepper_disable_outputs();

extern void stepper_enable_outputs();

extern void stepper_set_minimum_pulse_width();

extern void stepper_set_3_pins_inverted();

extern void stepper_set_4_pins_inverted();

extern void stepper_set_enable_pin();

extern void stepper_is_running();

extern void reset_board();

extern void init_neo_pixels();

extern void show_neo_pixels();

extern void set_neo_pixel();

extern void clear_all_neo_pixels();

extern void fill_neo_pixels();

extern void reset_board();

extern void set_pwm_freq();

extern void set_pwm_range();

extern void get_cpu_temp();

// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.

// The command_func is a pointer the command's function.
struct command_descriptor
{
  // a pointer to the command processing function
  void (*command_func)(void);
};


// An array of pointers to the command functions.
// The list must be in the same order as the command defines.

command_descriptor command_table[] =
{
  {&serial_loopback},
  {&set_pin_mode},
  {&digital_write},
  {&analog_write},
  {&modify_reporting},
  {&get_firmware_version},
  {&servo_attach},
  {&servo_write},
  {&servo_detach},
  {&i2c_begin},
  {&i2c_read},
  {&i2c_write},
  {&sonar_new},
  {&dht_new},
  {&stop_all_reports},
  {&set_analog_scanning_interval},
  {&enable_all_reports},
  {&reset_data},
  {&init_spi},
  {&write_blocking_spi},
  {&read_blocking_spi},
  {&set_format_spi},
  {&spi_cs_control},
  {&onewire_init},
  {&onewire_reset},
  {&onewire_select},
  {&onewire_skip},
  {&onewire_write},
  {&onewire_read},
  {&onewire_reset_search},
  {&onewire_search},
  {&onewire_crc8},
  {&set_pin_mode_stepper},
  {&stepper_move_to},
  {&stepper_move},
  {&stepper_run},
  {&stepper_run_speed},
  {&stepper_set_max_speed},
  {&stepper_set_acceleration},
  {&stepper_set_speed},
  (&stepper_set_current_position),
  (&stepper_run_speed_to_position),
  (&stepper_stop),
  (&stepper_disable_outputs),
  (&stepper_enable_outputs),
  (&stepper_set_minimum_pulse_width),
  (&stepper_set_enable_pin),
  (&stepper_set_3_pins_inverted),
  (&stepper_set_4_pins_inverted),
  (&stepper_is_running),
  (&stepper_get_current_position),
  {&stepper_get_distance_to_go},
  (&stepper_get_target_position),
  (&reset_board),
  {init_neo_pixels},
  {show_neo_pixels},
  {set_neo_pixel},
  {clear_all_neo_pixels},
  {fill_neo_pixels},
  {set_pwm_freq},
  {set_pwm_range},
  {get_cpu_temp},
};

// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

// buffer to hold incoming command data
byte command_buffer[MAX_COMMAND_LENGTH];


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                 Reporting Defines and Support                    */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Reports sent to the client

#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define FIRMWARE_REPORT 5
#define I_AM_HERE 6
#define SERVO_UNAVAILABLE 7
#define I2C_TOO_FEW_BYTES_RCVD 8
#define I2C_TOO_MANY_BYTES_RCVD 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11
#define DHT_REPORT 12
#define SPI_REPORT 13
#define ONE_WIRE_REPORT 14
#define STEPPER_DISTANCE_TO_GO 15
#define STEPPER_TARGET_POSITION 16
#define STEPPER_CURRENT_POSITION 17
#define STEPPER_RUNNING_REPORT 18
#define STEPPER_RUN_COMPLETE_REPORT 19
#define CPU_TEMP_REPORT 20
#define DEBUG_PRINT 99

// A buffer to hold i2c report data
byte i2c_report_message[64];

// a pointer to i2c port in use
TwoWire *current_i2c_port = NULL;

// a pointer to the spi port in use
SPIClassRP2040 *current_spi_port = NULL;

// spi settings values
uint32_t clock0;
BitOrder bitOrder0;
SPIMode dataMode0;
uint8_t chipSelect0;

uint32_t clock1;
BitOrder bitOrder1;
SPIMode dataMode1;
uint8_t chipSelect1;


// A buffer to hold spi report data
byte spi_report_message[64];

bool stop_reports = false; // a flag to stop sending all report messages

bool rebooting = false;


// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

// init dht command offsets
#define DHT_DATA_PIN 1

// DHT Report sub-types
#define DHT_DATA 0
#define DHT_READ_ERROR 1

// firmware version - update this when bumping the version
#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 0
#define FIRMWARE_PATCH 0

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*           Pin Related Defines And Data Structures                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


// Pin mode definitions

// INPUT defined in Arduino.h = 0
// OUTPUT defined in Arduino.h = 1
// INPUT_PULLUP defined in Arduino.h = 2
// The following are defined for arduino_telemetrix (AT)
#define AT_ANALOG 5
#define AT_MODE_NOT_SET 255

// maximum number of pins supported
#define MAX_DIGITAL_PINS_SUPPORTED 100
#define MAX_ANALOG_PINS_SUPPORTED 15

// flag to indicate that an i2c command does not specify a register
#define I2C_NO_REGISTER 254


// To translate a pin number from an integer value to its analog pin number
// equivalent, this array is used to look up the value to use for the pin.

int analog_read_pins[4] = {A0, A1, A2, A3};

// a descriptor for digital pins
struct pin_descriptor
{
  byte pin_number;
  byte pin_mode;
  bool reporting_enabled; // If true, then send reports if an input pin
  int last_value;         // Last value read for input mode
};

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// a descriptor for digital pins
struct analog_pin_descriptor
{
  byte pin_number;
  byte pin_mode;
  bool reporting_enabled; // If true, then send reports if an input pin
  int last_value;         // Last value read for input mode
  int differential;       // difference between current and last value needed
  // to generate a report
};

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

unsigned long current_millis;  // for analog input loop
unsigned long previous_millis; // for analog input loop
uint8_t analog_sampling_interval = 19;

// cpu temp comparison threshold
float cpu_temp_threshold = 0.0;
float cpu_temp_last_value = 0.0;
bool monitor_cpu_temp = false;
unsigned long cpu_temp_current_millis;  // for cpu temp loop
unsigned long cpu_temp_previous_millis; // for cpu temp loop
uint16_t cpu_temp_sampling_interval = 1000;

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*  Feature Related Defines, Data Structures and Storage Allocation */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// servo management
Servo servos[MAX_SERVOS];

// this array allows us to retrieve the servo object
// associated with a specific pin number
byte pin_to_servo_index_map[MAX_SERVOS];

// HC-SR04 Sonar Management
#define MAX_SONARS 6

struct Sonar
{
  uint8_t trigger_pin;
  unsigned int last_value;
  NanoConnectHcSr04 *usonic;
};

// an array of sonar objects
Sonar sonars[MAX_SONARS];

byte sonars_index = 0; // index into sonars struct

// used for scanning the sonar devices.
byte last_sonar_visited = 0;

unsigned long sonar_current_millis;  // for analog input loop
unsigned long sonar_previous_millis; // for analog input loop

uint8_t sonar_scan_interval = 33;    // Milliseconds between sensor pings
// (29ms is about the min to avoid = 19;

// DHT Management
#define MAX_DHTS 2                // max number of devices
// #define READ_FAILED_IN_SCANNER 0  // read request failed when scanning
// #define READ_IN_FAILED_IN_SETUP 1 // read request failed when initially setting up

NeoPixelConnect *np;


struct DHT
{
  uint8_t pin;
  uint8_t dht_type;
  unsigned int last_value;
  DHTNEW *dht_sensor;
};

// an array of dht objects
DHT dhts[MAX_DHTS];

byte dht_index = 0; // index into dht struct

unsigned long dht_current_millis;      // for analog input loop
unsigned long dht_previous_millis;     // for analog input loop
unsigned int dht_scan_interval = 2200; // scan dht's every 2 seconds

// spi interface pins
#define SPI0_MISO 16
#define SPI1_MISO 12

#define SPI0_MOSI 19
#define SPI1_MOSI 19

#define SPI0_CLK 18
#define SPI1_CLK 14


/* OneWire Object*/

// a pointer to a OneWire object
// OneWire *ow = NULL;

#define MAX_NUMBER_OF_STEPPERS 4

// stepper motor data
AccelStepper *steppers[MAX_NUMBER_OF_STEPPERS];

// stepper run modes
uint8_t stepper_run_modes[MAX_NUMBER_OF_STEPPERS];


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                       Command Functions                          */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


// A method to send debug data across the serial link
void send_debug_info(byte id, int value)
{
  byte debug_buffer[5] = {(byte)4, (byte)DEBUG_PRINT, 0, 0, 0};
  debug_buffer[2] = id;
  debug_buffer[3] = highByte(value);
  debug_buffer[4] = lowByte(value);
  Serial.write(debug_buffer, 5);
}

// a function to loop back data over the serial port
void serial_loopback()
{
  byte loop_back_buffer[3] = {2, (byte)SERIAL_LOOP_BACK, command_buffer[0]};
  Serial.write(loop_back_buffer, 3);
}

void set_pin_mode()
/*
    Set a pin to digital input, digital input_pullup, digital output,
    and analog input. PWM is considered digital output, and i2c, spi, dht,
    sonar, and servo have their own init methods.
*/
{
  byte pin;
  byte mode;
  pin = command_buffer[0];
  mode = command_buffer[1];

  switch (mode)
  {
    case INPUT:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT);
      break;
    case INPUT_PULLUP:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT_PULLUP);
      break;
    case INPUT_PULLDOWN:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT_PULLDOWN);
      break;
    case OUTPUT:
      the_digital_pins[pin].pin_mode = mode;
      pinMode(pin, OUTPUT);
      break;
    case AT_ANALOG:
      the_analog_pins[pin].pin_mode = mode;
      the_analog_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
      the_analog_pins[pin].reporting_enabled = command_buffer[4];
      break;
    default:
      break;
  }
}

// set the analog scanning interval
void set_analog_scanning_interval()
{
  analog_sampling_interval = command_buffer[0];
}

// set the state of digital output pin
void digital_write()
{
  byte pin;
  byte value;
  pin = command_buffer[0];
  value = command_buffer[1];
  digitalWrite(pin, value);
}

// set the pwm value for a digital output pin
// The term analog is confusing here, but it is what
// Arduino uses.
void analog_write()
{
  // command_buffer[0] = PIN, command_buffer[1] = value_msb,
  // command_buffer[2] = value_lsb
  byte pin; // command_buffer[0]
  unsigned int value;

  pin = command_buffer[0];

  value = (command_buffer[1] << 8) + command_buffer[2];

  analogWrite(pin, value);
}

void set_pwm_freq() {
  // command_buffer[0] = freq_msb
  // command_buffer[1] = msb3
  // command_buffer[2] = msb2
  // command_buffer[3] = lsb

  uint32_t frequency = (command_buffer[0] << 24) + (command_buffer[1] << 16) +
                       (command_buffer[2] << 8) + command_buffer[3];

  analogWriteFreq(frequency);
}

void set_pwm_range() {
  // command_buffer[0] = range_msb
  // command_buffer[1] = msb3
  // command_buffer[2] = msb2
  // command_buffer[3] = lsb

  uint32_t pwm_range = (command_buffer[0] << 24) + (command_buffer[1] << 16) +
                       (command_buffer[2] << 8) + command_buffer[3];

  analogWriteRange(pwm_range);

}

void get_cpu_temp() {

  //byte b[4] = {0,0,0,0};

  memcpy(&cpu_temp_threshold, &command_buffer[0], sizeof(float));

  uint16_t cpu_temp_sampling_interval =  (command_buffer[4] << 8) + command_buffer[5];

  monitor_cpu_temp = true;
}

// This method allows you modify what reports are generated.
// You can disable all reports, including dhts, and sonar.
// You can disable only digital and analog reports on a
// pin basis, or enable those on a pin basis.
void modify_reporting()
{
  int pin = command_buffer[1];

  switch (command_buffer[0])
  {
    case REPORTING_DISABLE_ALL:
      for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
      {
        the_digital_pins[i].reporting_enabled = false;
      }
      for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
      {
        the_analog_pins[i].reporting_enabled = false;
      }
      break;
    case REPORTING_ANALOG_ENABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_analog_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_ANALOG_DISABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_analog_pins[pin].reporting_enabled = false;
      }
      break;
    case REPORTING_DIGITAL_ENABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_digital_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_DIGITAL_DISABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_digital_pins[pin].reporting_enabled = false;
      }
      break;
    default:
      break;
  }
}

void reset_board() {
  stop_all_reports();

  delay(100);
  rebooting = true;
  rp2040.reboot();

}

// Return the firmware version number
void get_firmware_version()
{
  byte report_message[5] = {4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR,
                            FIRMWARE_PATCH
                           };
  Serial.write(report_message, 5);
}

/***************************************************
   Servo Commands
 **************************************************/

// Find the first servo that is not attached to a pin
// This is a helper function not called directly via the API
int find_servo()
{
  int index = -1;

  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (servos[i].attached() == false)
    {
      index = i;
      break;
    }
  }

  return index;
}

// Associate a pin with a servo
void servo_attach()
{
  byte pin = command_buffer[0];
  int servo_found = -1;

  int minpulse = (command_buffer[1] << 8) + command_buffer[2];
  int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

  // find the first available open servo
  servo_found = find_servo();
  if (servo_found != -1)
  {
    pin_to_servo_index_map[servo_found] = pin;
    servos[servo_found].attach(pin, minpulse, maxpulse);
  }
  else
  {
    // no open servos available, send a report back to client
    byte report_message[2] = {SERVO_UNAVAILABLE, pin};
    Serial.write(report_message, 2);
  }
}

// set a servo to a given angle
void servo_write()
{
  byte pin = command_buffer[0];
  int angle = command_buffer[1];
  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (pin_to_servo_index_map[i] == pin)
    {

      servos[i].write(angle);
      return;
    }
  }
}

// detach a servo and make it available for future use
void servo_detach()
{
  byte pin = command_buffer[0];

  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (pin_to_servo_index_map[i] == pin)
    {

      pin_to_servo_index_map[i] = -1;
      servos[i].detach();
    }
  }
}

/***********************************
   i2c functions
 **********************************/

// initialize i2c data transfers
void i2c_begin()
{
  byte i2c_port = command_buffer[0];
  if (not i2c_port)
  {
    Wire.begin();
  }
  else {
    Wire1.begin();
  }
}


// read a number of bytes from a specific i2c register
void i2c_read()
{
  // command = [PrivateConstants.I2C_READ, i2c_port, address, register,
  //                   number_of_bytes, no_stop]

  // data in the incoming message:
  // i2c port [0]
  // address, [1]
  // register, [2] if I2C_NO_REGISTER, don't send the register value
  // number of bytes to read, [3]
  // stop transmitting flag [4]

  int message_size = 0;
  byte port = command_buffer[0];
  byte address = command_buffer[1];
  byte the_register = command_buffer[2];
  byte num_of_bytes = command_buffer[3];
  bool stop = bool(command_buffer[4]);


  // set the current i2c port if this is for the primary i2c
  if (port == 0)
  {
    current_i2c_port = &Wire;
  }
  else {
    current_i2c_port = &Wire1;
  }


  // write byte is true, then write the register
  if ( the_register != I2C_NO_REGISTER)
  {
    current_i2c_port->beginTransmission(address);
    current_i2c_port->write((byte)the_register);
    current_i2c_port->endTransmission(stop);      // default = true
  }

  // all bytes are returned in requestFrom
  current_i2c_port->requestFrom(address, num_of_bytes, stop);

  // check to be sure correct number of bytes were returned by slave
  if (num_of_bytes < current_i2c_port->available())
  {
    byte report_message[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }
  else if (num_of_bytes > current_i2c_port->available())
  {
    byte report_message[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }

  // packet length
  i2c_report_message[0] = num_of_bytes + 5;

  // report type
  i2c_report_message[1] = I2C_READ_REPORT;

  // i2c_port
  i2c_report_message[2] = port;

  // number of bytes read
  i2c_report_message[3] = num_of_bytes; // number of bytes

  // device address
  i2c_report_message[4] = address;

  // device register
  i2c_report_message[5] = the_register;

  // append the data that was read
  for (message_size = 0; message_size < num_of_bytes && current_i2c_port->available()
       ; message_size++)
  {
    i2c_report_message[6 + message_size] = current_i2c_port->read();
  }
  // send slave address, register and received bytes

  for (int i = 0; i < message_size + 6; i++)
  {
    Serial.write(i2c_report_message[i]);
  }
}

// write a specified number of bytes to an i2c device
void i2c_write()
{
  // command_buffer[0] = i2c port
  // command_buffer[1] is the device address
  // command_buffer[2] is the number of bytes to send
  // command_buffer[3...] are the bytes to write


  // set the current i2c port if this is for the primary i2c
  if (command_buffer[0] == 0)
  {
    current_i2c_port = &Wire;
  }
  else {
    current_i2c_port = &Wire1;
  }

  current_i2c_port->beginTransmission(command_buffer[1]);

  // write the data to the device
  for (int i = 0; i < command_buffer[2]; i++)
  {
    current_i2c_port->write(command_buffer[i + 3]);
  }
  current_i2c_port->endTransmission();
  delayMicroseconds(70);
}

/***********************************
   HC-SR04 adding a new device
 **********************************/

// associate 2 pins as trigger and echo pins for a sonar device
void sonar_new()
{

  // command_buffer[0] = trigger pin,  command_buffer[1] = echo pin
  sonars[sonars_index].usonic = new NanoConnectHcSr04((uint8_t) command_buffer[0], (uint8_t) command_buffer[1], pio0,
      1);
  sonars[sonars_index].trigger_pin = command_buffer[0];
  sonars_index++;
}

/***********************************
   DHT adding a new device
 **********************************/

// associate a pin with a dht device
void dht_new()
{

  if (dht_index < MAX_DHTS) {
    dhts[dht_index].dht_sensor = new DHTNEW(command_buffer[0]);

    dhts[dht_index].pin = command_buffer[0];
    dhts[dht_index].dht_type = command_buffer[1];
    dht_index++;
  }
}

// initialize the SPI interface
void init_spi() {

  // command buffer received:
  // spi_port, chip_select, freq_bytes[0], freq_bytes[1], freq_bytes[2], freq_bytes[3],
  // data_order, data_mode

  int spi_port = command_buffer[0];
  int chip_select = command_buffer[1];

  if (spi_port == 0) {
    clock0 = (uint32_t)(command_buffer[2]) << 24;
    clock0 += (uint32_t)(command_buffer[3]) << 16;
    clock0 += command_buffer[4] << 8;
    clock0 += command_buffer[5] ;

    bitOrder0 = (BitOrder)command_buffer[6];
    dataMode0 = (SPIMode)command_buffer[7];
    chipSelect0 = chip_select;

    current_spi_port = &SPI;
  }

  else {
    clock1 = (uint32_t)(command_buffer[2]) << 24;
    clock1 += (uint32_t)(command_buffer[3]) << 16;
    clock1 += command_buffer[4] << 8;
    clock1 += command_buffer[5] ;

    bitOrder1 = (BitOrder)command_buffer[6];
    dataMode1 = (SPIMode)command_buffer[7];

    chipSelect1 = chip_select;

    current_spi_port = &SPI1;


  }

  // Chip select is active-low, so we'll initialise it to a driven-high state
  pinMode(chip_select, OUTPUT);
  digitalWrite(chip_select, HIGH);

  current_spi_port->begin();
}

// write a number of bytes to the SPI device
void write_blocking_spi() {

  int chipSelect;
  // command[0] == spi port
  // command[1] == number of bytes to write
  // command[2 ...] bytes to write
  int num_bytes = command_buffer[1];

  if (command_buffer[0] == 0) {
    current_spi_port = &SPI;
    SPISettings mySetting(clock0, bitOrder0, dataMode0);
    chipSelect = chipSelect0;
    current_spi_port->beginTransaction(mySetting);

  }
  else {
    current_spi_port = &SPI1;
    SPISettings mySetting(clock1, bitOrder1, dataMode1);
    chipSelect = chipSelect1;
    current_spi_port->beginTransaction(mySetting);

  }
  digitalWrite(chipSelect, LOW);
  for (int i = 0; i < num_bytes; i++) {
    current_spi_port->transfer(command_buffer[2 + i] );
  }
  digitalWrite(chipSelect, LOW);
  current_spi_port->endTransaction();

}

// read a number of bytes from the SPI device
void read_blocking_spi() {
  int chipSelect;

  // command_buffer[0] == spi register
  // command_buffer[1] == number of bytes to read
  // command_buffer[2] == spi_port

  uint8_t spi_register, number_of_bytes, spi_port;

  spi_register = command_buffer[0];
  number_of_bytes = command_buffer[1];
  spi_port = command_buffer[2];

  if (spi_port == 0) {
    current_spi_port = &SPI;
    SPISettings mySetting(clock0, bitOrder0, dataMode0);
    chipSelect = chipSelect0;
    current_spi_port->beginTransaction(mySetting);

  }
  else {
    current_spi_port = &SPI1;
    SPISettings mySetting(clock1, bitOrder1, dataMode1);
    chipSelect = chipSelect1;
    current_spi_port->beginTransaction(mySetting);

  }

  // spi_report_message[0] = length of message including this element
  // spi_report_message[1] = SPI_REPORT
  // spi_report_message[2] = port number
  // spi_report_message[2] = register used for the read
  // spi_report_message[3] = number of bytes returned
  // spi_report_message[4..] = data read

  // configure the report message
  // calculate the packet length
  spi_report_message[0] = command_buffer[2] + 3; // packet length
  spi_report_message[1] = SPI_REPORT;
  spi_report_message[2] = command_buffer[0];
  spi_report_message[3] = command_buffer[1]; // register
  spi_report_message[43] = command_buffer[2]; // number of bytes read

  // write the register out. OR it with 0x80 to indicate a read
  current_spi_port->transfer(spi_register | 0x80);

  // now read the specified number of bytes and place
  // them in the report buffer

  digitalWrite(chipSelect, LOW);
  for (int i = 0; i < number_of_bytes ; i++) {
    spi_report_message[i + 4] = current_spi_port->transfer(0x00);

  }
  digitalWrite(chipSelect, LOW);
  current_spi_port->endTransaction();
  Serial.write(spi_report_message, command_buffer[2] + 4);
}

// modify the SPI format
void set_format_spi() {

  //#if defined(__AVR__)
  //  SPISettings(command_buffer[0], command_buffer[1], command_buffer[2]);
  //#else
  //  BitOrder b;

  //  if (command_buffer[1]) {
  //    b = MSBFIRST;
  // } else {
  //    b = LSBFIRST;
  //  }
  //  SPISettings(command_buffer[0], b, command_buffer[2]);
  //#endif // avr
}

// set the SPI chip select line
void spi_cs_control() {
  //int spi_cs_pin = command_buffer[0];
  //int cs_state = command_buffer[1];

  //digitalWrite(spi_cs_pin, cs_state);
}

/* onewire is not yet implemented for the pico. these are place holders */

// Initialize the OneWire interface
void onewire_init() {
  //ow = new OneWire(command_buffer[0]);
}

// send a OneWire reset
void onewire_reset() {

  //uint8_t reset_return = ow->reset();
  //uint8_t onewire_report_message[] = {3, ONE_WIRE_REPORT, ONE_WIRE_RESET, reset_return};

  //Serial.write(onewire_report_message, 4);
}

// send a OneWire select
void onewire_select() {

  //uint8_t dev_address[8];

  //for (int i = 0; i < 8; i++) {
  //  dev_address[i] = command_buffer[i];
  //}
  //ow->select(dev_address);
}

// send a OneWire skip
void onewire_skip() {
  //ow->skip();
}

// write 1 byte to the OneWire device
void onewire_write() {

  // write data and power values
  //ow->write(command_buffer[0], command_buffer[1]);
}

// read one byte from the OneWire device
void onewire_read() {

  // onewire_report_message[0] = length of message including this element
  // onewire_report_message[1] = ONEWIRE_REPORT
  // onewire_report_message[2] = message subtype = 29
  // onewire_report_message[3] = data read

  //uint8_t data = ow->read();

  //uint8_t onewire_report_message[] = {3, ONE_WIRE_REPORT, ONE_WIRE_READ, data};

  //Serial.write(onewire_report_message, 4);
}

// Send a OneWire reset search command
void onewire_reset_search() {

  //ow->reset_search();
}

// Send a OneWire search command
void onewire_search() {

  //uint8_t onewire_report_message[] = {10, ONE_WIRE_REPORT, ONE_WIRE_SEARCH,
  //                                    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  //                                    0xff
  //                                   };

  //ow->search(&onewire_report_message[3]);
  //Serial.write(onewire_report_message, 11);
}

// Calculate a OneWire CRC8 on a buffer containing a specified number of bytes
void onewire_crc8() {

  //uint8_t crc = ow->crc8(&command_buffer[1], command_buffer[0]);
  //uint8_t onewire_report_message[] = {3, ONE_WIRE_REPORT, ONE_WIRE_CRC8, crc};
  //Serial.write(onewire_report_message, 4);

}

// Stepper Motor supported
// Stepper Motor supported
void set_pin_mode_stepper() {


  // motor_id = command_buffer[0]
  // interface = command_buffer[1]
  // pin1 = command_buffer[2]
  // pin2 = command_buffer[3]
  // pin3 = command_buffer[4]
  // pin4 = command_buffer[5]
  // enable = command_buffer[6]

  // instantiate a stepper object and store it in the stepper array
  steppers[command_buffer[0]] = new AccelStepper(command_buffer[1], command_buffer[2],
      command_buffer[3], command_buffer[4],
      command_buffer[5], command_buffer[6]);
}

// Neo Pixel support
void init_neo_pixels() {
  byte pin_number, num_pixels;

  pin_number = command_buffer[0];
  num_pixels = command_buffer[1];

  np = new NeoPixelConnect(pin_number, num_pixels);
}

void show_neo_pixels() {
  np->neoPixelShow();
}

void set_neo_pixel() {
  np->neoPixelSetValue(command_buffer[0], command_buffer[1],
                       command_buffer[2], command_buffer[3],
                       command_buffer[4]);
}

void clear_all_neo_pixels() {
  np->neoPixelClear(command_buffer[0]);
}

void fill_neo_pixels() {
  np->neoPixelFill(command_buffer[0], command_buffer[1],
                   command_buffer[2], command_buffer[3]);
}

void stepper_move_to() {

  // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]
  // polarity = command_buffer[5]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4] ;
  if (command_buffer[5]) {
    position *= -1;
  }
  steppers[command_buffer[0]]->moveTo(position);
}

void stepper_move() {

  // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]
  // polarity = command_buffer[5]


  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4] ;
  if (command_buffer[5]) {
    position *= -1;
  }
  steppers[command_buffer[0]]->move(position);
}

void stepper_run() {
  stepper_run_modes[command_buffer[0]] = STEPPER_RUN;
}

void stepper_run_speed() {
  // motor_id = command_buffer[0]

  stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED;
}

void stepper_set_max_speed() {

  // motor_id = command_buffer[0]
  // speed_msb = command_buffer[1]
  // speed_lsb = command_buffer[2]

  float max_speed = (float) ((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setMaxSpeed(max_speed);
}

void stepper_set_acceleration() {

  // motor_id = command_buffer[0]
  // accel_msb = command_buffer[1]
  // accel = command_buffer[2]

  float acceleration = (float) ((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setAcceleration(acceleration);
}

void stepper_set_speed() {

  // motor_id = command_buffer[0]
  // speed_msb = command_buffer[1]
  // speed_lsb = command_buffer[2]

  float speed = (float) ((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setSpeed(speed);
}

void stepper_get_distance_to_go() {
  // motor_id = command_buffer[0]

  // report = STEPPER_DISTANCE_TO_GO, motor_id, distance(8 bytes)



  byte report_message[7] = {6, STEPPER_DISTANCE_TO_GO, command_buffer[0]};

  long dtg = steppers[command_buffer[0]]->distanceToGo();


  report_message[3] = (byte) ((dtg & 0xFF000000) >> 24);
  report_message[4] = (byte) ((dtg & 0x00FF0000) >> 16);
  report_message[5] = (byte) ((dtg & 0x0000FF00) >> 8);
  report_message[6] = (byte) ((dtg & 0x000000FF));

  // motor_id = command_buffer[0]
  Serial.write(report_message, 7);
}

void stepper_get_target_position() {
  // motor_id = command_buffer[0]

  // report = STEPPER_TARGET_POSITION, motor_id, distance(8 bytes)



  byte report_message[7] = {6, STEPPER_TARGET_POSITION, command_buffer[0]};

  long target = steppers[command_buffer[0]]->targetPosition();


  report_message[3] = (byte) ((target & 0xFF000000) >> 24);
  report_message[4] = (byte) ((target & 0x00FF0000) >> 16);
  report_message[5] = (byte) ((target & 0x0000FF00) >> 8);
  report_message[6] = (byte) ((target & 0x000000FF));

  // motor_id = command_buffer[0]
  Serial.write(report_message, 7);
}

void stepper_get_current_position() {
  // motor_id = command_buffer[0]

  // report = STEPPER_CURRENT_POSITION, motor_id, distance(8 bytes)



  byte report_message[7] = {6, STEPPER_CURRENT_POSITION, command_buffer[0]};

  long position = steppers[command_buffer[0]]->currentPosition();


  report_message[3] = (byte) ((position & 0xFF000000) >> 24);
  report_message[4] = (byte) ((position & 0x00FF0000) >> 16);
  report_message[5] = (byte) ((position & 0x0000FF00) >> 8);
  report_message[6] = (byte) ((position & 0x000000FF));

  // motor_id = command_buffer[0]
  Serial.write(report_message, 7);
}

void stepper_set_current_position() {
  // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[2]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4] ;

  steppers[command_buffer[0]]->setCurrentPosition(position);
}

void stepper_run_speed_to_position() {
  stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED_TO_POSITION;

}

void stepper_stop() {
  steppers[command_buffer[0]]->stop();
  steppers[command_buffer[0]]->disableOutputs();
  stepper_run_modes[command_buffer[0]] = STEPPER_STOP;


}

void stepper_disable_outputs() {
  steppers[command_buffer[0]]->disableOutputs();
}

void stepper_enable_outputs() {
  steppers[command_buffer[0]]->enableOutputs();
}

void stepper_set_minimum_pulse_width() {
  unsigned int pulse_width = (command_buffer[1] << 8) + command_buffer[2];
  steppers[command_buffer[0]]->setMinPulseWidth(pulse_width);
}

void stepper_set_enable_pin() {
  steppers[command_buffer[0]]->setEnablePin((uint8_t) command_buffer[1]);
}

void stepper_set_3_pins_inverted() {
  // command_buffer[1] = directionInvert
  // command_buffer[2] = stepInvert
  // command_buffer[3] = enableInvert
  steppers[command_buffer[0]]->setPinsInverted((bool) command_buffer[1],
      (bool) command_buffer[2],
      (bool) command_buffer[3]);
}

void stepper_set_4_pins_inverted() {
  // command_buffer[1] = pin1
  // command_buffer[2] = pin2
  // command_buffer[3] = pin3
  // command_buffer[4] = pin4
  // command_buffer[5] = enable
  steppers[command_buffer[0]]->setPinsInverted((bool) command_buffer[1],
      (bool) command_buffer[2],
      (bool) command_buffer[3],
      (bool) command_buffer[4],
      (bool) command_buffer[5]);
}

void stepper_is_running() {
  // motor_id = command_buffer[0]

  // report = STEPPER_IS_RUNNING, motor_id, distance(8 bytes)


  byte report_message[3] = {2, STEPPER_RUNNING_REPORT, command_buffer[0]};

  report_message[2]  = steppers[command_buffer[0]]->isRunning();

  Serial.write(report_message, 3);

}

// stop all reports from being generated

void stop_all_reports()
{
  stop_reports = true;
  delay(20);
  Serial.flush();
}

// enable all reports to be generated
void enable_all_reports()
{
  Serial.flush();
  stop_reports = false;
  delay(20);
}

// retrieve the next command from the serial link
void get_next_command()
{
  byte command;
  byte packet_length;
  command_descriptor command_entry;

  // clear the command buffer
  memset(command_buffer, 0, sizeof(command_buffer));

  // if there is no command waiting, then return
  if (not Serial.available())
  {
    return;
  }
  // get the packet length
  packet_length = (byte)Serial.read();

  while (not Serial.available())
  {
    delay(1);
  }

  // get the command byte
  command = (byte)Serial.read();

  // uncomment the next line to see the packet length and command
  //send_debug_info(packet_length, command);
  command_entry = command_table[command];

  if (packet_length > 1)
  {
    // get the data for that command
    for (int i = 0; i < packet_length - 1; i++)
    {
      // need this delay or data read is not correct
      while (not Serial.available())
      {
        delay(1);
      }
      command_buffer[i] = (byte)Serial.read();
      // uncomment out to see each of the bytes following the command
      // send_debug_info(i, command_buffer[i]);
    }
  }
  command_entry.command_func();
}

// reset the internal data structures to a known state
void reset_data() {
  // reset the data structures

  // fist stop all reporting
  stop_all_reports();

  current_millis = 0;  // for analog input loop
  previous_millis = 0; // for analog input loop
  analog_sampling_interval = 19;

  // detach any attached servos
  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (servos[i].attached() == true)
    {
      servos[i].detach();
    }
  }

  sonars_index = 0; // reset the index into the sonars array

  sonar_current_millis = 0;  // for analog input loop
  sonar_previous_millis = 0; // for analog input loop
  sonar_scan_interval = 33;  // Milliseconds between sensor pings
  memset(sonars, 0, sizeof(sonars));

  dht_index = 0; // index into dht array

  dht_current_millis = 0;      // for analog input loop
  dht_previous_millis = 0;     // for analog input loop
  dht_scan_interval = 2200;    // scan dht's every 2 seconds

  memset(dhts, 0, sizeof(dhts));
  enable_all_reports();
}

// initialize the pin data structures
void init_pin_structures() {
  for (byte i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
  {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = 0;
  }

  // establish the analog pin array
  for (byte i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
  {
    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = 0;
    the_analog_pins[i].differential = 0;
  }
}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*    Scanning Inputs, Generating Reports And Running Steppers      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


// scan the digital input pins for changes
void scan_digital_inputs()
{
  byte value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = value
  byte report_message[4] = {3, DIGITAL_REPORT, 0, 0};

  for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
  {
    if (the_digital_pins[i].pin_mode == INPUT ||
        the_digital_pins[i].pin_mode == INPUT_PULLUP)
    {
      if (the_digital_pins[i].reporting_enabled)
      {
        // if the value changed since last read
        value = (byte)digitalRead(the_digital_pins[i].pin_number);
        if (value != the_digital_pins[i].last_value)
        {
          the_digital_pins[i].last_value = value;
          report_message[2] = (byte)i;
          report_message[3] = value;
          Serial.write(report_message, 4);
        }
      }
    }
  }
}

// scan the analog input pins for changes
void scan_analog_inputs()
{
  int value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value

  byte report_message[5] = {4, ANALOG_REPORT, 0, 0, 0};

  uint8_t adjusted_pin_number;
  int differential;

  current_millis = millis();
  if (current_millis - previous_millis > analog_sampling_interval)
  {
    previous_millis = current_millis;

    for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
    {
      if (the_analog_pins[i].pin_mode == AT_ANALOG)
      {
        if (the_analog_pins[i].reporting_enabled)
        {
          // if the value changed since last read
          // adjust pin number for the actual read
          adjusted_pin_number = (uint8_t)(analog_read_pins[i]);
          value = analogRead(adjusted_pin_number);
          differential = abs(value - the_analog_pins[i].last_value);
          if (differential >= the_analog_pins[i].differential)
          {
            //trigger value achieved, send out the report
            the_analog_pins[i].last_value = value;
            // input_message[1] = the_analog_pins[i].pin_number;
            report_message[2] = (byte)i;
            report_message[3] = highByte(value); // get high order byte
            report_message[4] = lowByte(value);
            Serial.write(report_message, 5);
            delay(1);
          }
        }
      }
    }
  }
}

void scan_cpu_temp() {

  float cpu_temp, differential ;
  char output[sizeof(float)];

  byte report_message[6] = {5, CPU_TEMP_REPORT, 0, 0, 0, 0};

  if (monitor_cpu_temp) {
    cpu_temp_current_millis = millis();
    if (cpu_temp_current_millis - cpu_temp_previous_millis > cpu_temp_sampling_interval)
    {
      cpu_temp_previous_millis = cpu_temp_current_millis;
      cpu_temp = analogReadTemp();

      differential = abs(cpu_temp - cpu_temp_last_value);
      if (differential >= cpu_temp_threshold)
      {

        cpu_temp_last_value = cpu_temp;

        memcpy(output, &cpu_temp, sizeof(float));
        report_message[2] = output[0];
        report_message[3] = output[1];
        report_message[4] = output[2];
        report_message[5] = output[3];
        Serial.write(report_message, 6);
        delay(1);
      }
    }
  }
}

// scan the sonar devices for changes
void scan_sonars()
{
  float distance;
  float j, f;
  uint8_t integ, frac;

  if (sonars_index) {
    {
      sonar_current_millis = millis();
      if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval) {
        sonar_previous_millis = sonar_current_millis;
        distance = sonars[last_sonar_visited].usonic->readSonar();
        if (distance != sonars[last_sonar_visited].last_value) {
          sonars[last_sonar_visited].last_value = distance;

          // byte 0 = packet length
          // byte 1 = report type
          // byte 2 = trigger pin number
          // byte 3 = distance integer portion
          // byte 4 = distance fractional portion

          f = modff(distance, &j);

          integ = (uint8_t) j;
          frac = (uint8_t) f;
          byte report_message[5] = {4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
                                    integ, frac
                                   };
          //client.write(report_message, 5);
          Serial.write(report_message, 5);
        }
        last_sonar_visited++;
        if (last_sonar_visited == sonars_index) {
          last_sonar_visited = 0;
        }
      }
    }
  }
}

// scan dht devices for changes
void scan_dhts() {
  // prebuild report for valid data
  // reuse the report if a read command fails

  // data returned is in floating point form - 4 bytes
  // each for humidity and temperature

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = report sub type - DHT_DATA or DHT_ERROR
  // byte 3 = pin number
  // byte 4 = humidity positivity flag 0=positive, 1= negative
  // byte 5 = temperature positivity flag 0=positive, 1= negative
  // byte 6 = humidity integer portion
  // byte 7 = humidity fractional portion
  // byte 8 = temperature integer portion
  // byte 9= temperature fractional portion

  byte report_message[10] = {9, DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0};

  int rv;

  float humidity, temperature;

  // are there any dhts to read?
  if (dht_index) {
    // is it time to do the read? This should occur every 2 seconds
    dht_current_millis = millis();
    if (dht_current_millis - dht_previous_millis > dht_scan_interval) {
      // update for the next scan
      dht_previous_millis = dht_current_millis;

      // read and report all the dht sensors
      for (int i = 0; i < dht_index; i++) {
        //report_message[0] = 9; //message length
        report_message[1] = DHT_REPORT;
        // error type in report_message[2] will be set further down
        report_message[3] = dhts[i].pin;

        rv = dhts[i].dht_sensor->read();

        if (rv != DHTLIB_OK) {
          rv = 0xff;
        }
        report_message[2] = (uint8_t) rv;

        // if rv is not zero, this is an error report
        if (rv) {
          Serial.write(report_message, 10);
          return;
        } else {
          float j, f;
          float humidity = dhts[i].dht_sensor->getHumidity();
          if (humidity >= 0.0) {
            report_message[4] = 0;
          } else {
            report_message[4] = 1;
          }
          f = modff(humidity, &j);
          report_message[6] = (uint8_t) j;
          report_message[7] = (uint8_t)(f * 100);


          float temperature = dhts[i].dht_sensor->getTemperature();
          if (temperature >= 0.0) {
            report_message[5] = 0;
          } else {
            report_message[5] = 1;
          }

          f = modff(temperature, &j);

          report_message[8] = (uint8_t) j;
          report_message[9] = (uint8_t)(f * 100);
          Serial.write(report_message, 10);
        }
      }
    }
  }
}



void run_steppers() {
  boolean running;
  long target_position;


  for ( int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    if (stepper_run_modes[i] == STEPPER_STOP) {
      continue;
    }
    else {
      steppers[i]->enableOutputs();
      switch (stepper_run_modes[i]) {
        case STEPPER_RUN:
          steppers[i]->run();
          running = steppers[i]->isRunning();
          if (!running) {
            byte report_message[3] = {2, STEPPER_RUN_COMPLETE_REPORT, (byte)i};
            Serial.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }
          break;
        case STEPPER_RUN_SPEED:
          steppers[i]->runSpeed();
          break;
        case STEPPER_RUN_SPEED_TO_POSITION:
          running = steppers[i]->runSpeedToPosition();
          target_position = steppers[i]->targetPosition();
          if (target_position == steppers[i]->currentPosition()) {
            byte report_message[3] = {2, STEPPER_RUN_COMPLETE_REPORT, (byte)i};
            Serial.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;

          }
          break;
        default:
          break;
      }
    }
  }
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                    Setup And Loop                                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void setup()
{

  for ( int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    stepper_run_modes[i] = STEPPER_STOP ;
  }
  // set the range to be compatible with the non-wifi pico telemetrix library
  analogWriteRange(20000) ;
  init_pin_structures();

  Serial.begin(115200);
}

void loop()
{
  if (!rebooting) {
    // keep processing incoming commands
    get_next_command();

    if (!stop_reports)
    {
      scan_digital_inputs();
      scan_analog_inputs();
      scan_sonars();
      scan_dhts();
      scan_cpu_temp();
      run_steppers();
    }
  }
}