const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;

int velocity;
unsigned long prevTicks;
bool midiOn;
int note;

void setup() {
  Serial.begin(31250);
  velocity = 100;
  prevTicks = 0;
  midiOn = false;
  note = 60;

  // pinMode(7, INPUT);
  // pinMode(8, INPUT);
  // pinMode(9, INPUT);
  // pinMode(10, INPUT);
  // pinMode(11, INPUT);
  // pinMode(12, INPUT);
  // pinMode(13, INPUT);
}

void send_midi(byte midiStatus, byte pitch, byte velocity) {
  Serial.write(midiStatus);
  Serial.write(pitch);
  Serial.write(velocity);
}

void loop() {
  unsigned long currTicks = millis();
  // value from 0 to 1023 (10 bits), converting to value from 0 to 127 (7 bits)
  velocity = analogRead(A0) / 8;

  if (currTicks - prevTicks >= 1000) {
    prevTicks = currTicks;
  
    if (midiOn) {
      send_midi(NOTE_OFF, note, velocity);
      if (note == 72) {
        note = 60;
      } else {
        note++;
      }
    } else {
      send_midi(NOTE_ON, note, velocity);
    }
    
    midiOn = !midiOn;
  }
}