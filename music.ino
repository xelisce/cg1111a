#include <MeMCore.h>

MeBuzzer buzzer;  // Define the mBot buzzer

// Melody and note durations (same as before)
int melody[] = {
  NOTE_D5, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_G4, NOTE_A4,
  NOTE_D5, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_G4, NOTE_A4,
  NOTE_F5, NOTE_F5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_B5,
  NOTE_D6, NOTE_D6, NOTE_A5, NOTE_B5, NOTE_A5, NOTE_F5, NOTE_E5, NOTE_D5,
};

int noteDurations[] = {
  8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8,
  4, 8, 8, 4, 4, 8, 8, 8,
  4, 4, 8, 8, 8, 8, 8, 8
};

void setup() {
  // Loop through the notes in the melody
  for (int thisNote = 0; thisNote < sizeof(melody) / sizeof(int); thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    buzzer.tone(melody[thisNote], noteDuration);  // Play note on mBot buzzer

    // Pause between notes
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // Stop the tone to create a silence between notes
    buzzer.noTone();
  }
}

void loop() {
  // Empty loop as we only play the melody once
}
