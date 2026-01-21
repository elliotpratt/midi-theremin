#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLidarLite;

const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
const int OCTAVE = 12;

uint16_t distance;
uint16_t oldDistance = 0;
uint8_t  newDistance;
int velocity = 100;
int note = 60;
bool midiOn = false;
int switchState;
int keyOffset = 0;


void setup() {
  // Initialize Arduino serial port at MIDI baud rate
  Serial.begin(31250);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
  // 400 kbits
  Wire.setClock(400000);

  // Disable internal pull-up resistors 
  // (external 1.5K pull-ups used instead)
  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  myLidarLite.configure(0);

  pinMode(8, INPUT);
}

void loop() {
  // Continuous loop
  while (1) {
    // value from 0 to 1023 (10 bits), converting to value from 0 to 127 (7 bits)
    velocity = analogRead(A1) / 8;
    // value from 0 to 1023 (10 bits), converting to value from 0 to 11
    keyOffset = analogRead(A0) / 85;
    if (keyOffset == 12) { keyOffset = 11; }
    // get state of switch
    switchState = digitalRead(8);
    // get new distance measurement
    newDistance = distanceContinuous(&distance);

    if (newDistance) {
      // if a new pitch
      if (distance/4 != oldDistance/4) {
        // if a note is already sounding
        if (midiOn) {
          send_midi(NOTE_OFF, note, velocity);
          midiOn = false;
        }

        // responds to measurements between 0 and 99 cm, 25 pitches
        // waits until distance stabilizes
        if (distance < 100 && abs((int)oldDistance - (int)distance) <= 10) {
          note = distToPitch(distance/4);
          send_midi(NOTE_ON, note, velocity);
          midiOn = true;
        }
      }
      
      oldDistance = distance;
    }
  }
}

int distToPitch(int dividedDist) {
  int pitch;

  if(switchState) {
    pitch = 84 - dividedDist;
    return pitch - keyOffset;
  }

  switch(dividedDist % 7) {
    case 0:
      pitch = 84;
      break;
    case 1:
      pitch = 83;
      break;
    case 2:
      pitch = 81;
      break;
    case 3:
      pitch = 79;
      break;
    case 4:
      pitch = 77;
      break;
    case 5:
      pitch = 76;
      break;
    case 6:
      pitch = 74;
      break;
  }

  pitch -= (dividedDist / 7) * OCTAVE;
  return pitch - keyOffset;
}

void send_midi(byte midiStatus, byte pitch, byte velocity) {
  Serial.write(midiStatus);
  Serial.write(pitch);
  Serial.write(velocity);
}

uint8_t distanceContinuous(uint16_t * distance) {
  uint8_t newDistance = 0;
  
  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (myLidarLite.getBusyFlag() == 0) {
    // Trigger the next range measurement
    myLidarLite.takeRange();

    // Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}