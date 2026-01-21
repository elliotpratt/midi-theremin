/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v4LED/v4LED

  This example shows methods for running the LIDAR-Lite v4 LED in various
  modes of operation. To exercise the examples open a serial terminal
  program (or the Serial Monitor in the Arduino IDE) and send ASCII
  characters to trigger the commands. See "loop" function for details.

  *** NOTE ***
  The LIDAR-Lite v4 LED is strictly a 3.3V system. The Arduino Due is a
  3.3V system and is recommended for use with the LIDAR-Lite v4 LED.
  Care MUST be taken if connecting to a 5V system such as the Arduino Uno.
  See comment block in the setup() function for details on I2C connections.
  It is recommended to use a voltage level-shifter if connecting the GPIO
  pins to any 5V system I/O.

  Connections:
  LIDAR-Lite 5 VDC   (pin 1) to Arduino 5V
  LIDAR-Lite Ground  (pin 2) to Arduino GND
  LIDAR-Lite I2C SDA (pin 3) to Arduino SDA
  LIDAR-Lite I2C SCL (pin 4) to Arduino SCL

  Optional connections to utilize GPIO triggering:
  LIDAR-Lite GPIOA   (pin 5) to Arduino Digital 2
  LIDAR-Lite GPIOB   (pin 6) to Arduino Digital 3

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5V
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information

------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLidarLite;


void setup() {
    // Initialize Arduino serial port (for display of ASCII output to PC)
    Serial.begin(115200);

    // Initialize Arduino I2C (for communication to LidarLite)
    Wire.begin();
    Wire.setClock(400000);

    // ----------------------------------------------------------------------
    // The LIDAR-Lite v4 LED is strictly a 3.3V system. The Arduino Due is a
    // 3.3V system and is recommended for use with the LIDAR-Lite v4 LED.
    // Care MUST be taken if connecting to a 5V system such as the Arduino Uno.
    //
    // I2C is a two wire communications bus that requires a pull-up resistor
    // on each signal. In the Arduino microcontrollers the Wire.begin()
    // function (called above) turns on pull-up resistors that pull the I2C
    // signals up to the system voltage rail. The Arduino Uno is a 5V system
    // and using the Uno's internal pull-ups risks damage to the LLv4.
    //
    // The two digitalWrite() functions (below) turn off the micro's internal
    // pull-up resistors. This protects the LLv4 from damage via overvoltage
    // but requires external pullups to 3.3V for the I2C signals.
    //
    // External pull-ups are NOT present on the Arduino Uno and must be added
    // manually to the I2C signals. 3.3V is available on pin 2 of the 6pin
    // "POWER" connector and can be used for this purpose. See the Uno
    // schematic for details:
    // https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf
    //
    // External 1.5k ohm pull-ups to 3.3V are already present on the
    // Arduino Due. If using the Due no further action is required
    // ----------------------------------------------------------------------
    digitalWrite(SCL, LOW);
    digitalWrite(SDA, LOW);

    // ----------------------------------------------------------------------
    // Optionally configure the LidarLite parameters to lend itself to
    // various modes of operation by altering 'configure' input integer.
    // See LIDARLite_v4LED.cpp for details.
    // ----------------------------------------------------------------------
    myLidarLite.configure(0);
}


void loop() {
  uint16_t distance;
  uint16_t oldDistance = 0;
  uint8_t  newDistance;

  // Continuous loop
  while (1) {
    newDistance = distanceContinuous(&distance);

    if (newDistance) {
        Serial.println(abs((int)oldDistance - (int)distance));
        oldDistance = distance;
    }
  }
}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (myLidarLite.getBusyFlag() == 0)
    {
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}