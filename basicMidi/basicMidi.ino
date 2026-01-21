// int velocity = 100;
const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
// bool state1 = false;
// bool state2 = false;
// bool state3 = false;
// bool state4 = false;
// bool state5 = false;
// bool state6 = false;
// bool state7 = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(31250);

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
  // put your main code here, to run repeatedly:
  for(int note = 60; note < 72; ++note) {
    send_midi(NOTE_ON, note, 100);
    delay(1000);
    send_midi(NOTE_OFF, note, 100);
    delay(1000);
  }
  // Serial.write("Hallo");
  // delay(1000);
}
