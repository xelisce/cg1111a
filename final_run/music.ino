// define note frequencies (in Hz) for the melody
#define NOTE_A3 220
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_D5 587

int melody[] = {
    NOTE_D4,
    NOTE_F4,
    NOTE_D4,
    NOTE_D4,
    NOTE_G4,
    NOTE_D4,
    NOTE_C4,
    NOTE_D4,
    NOTE_A4,
    NOTE_D4,
    NOTE_D4,
    NOTE_AS4,
    NOTE_A4,
    NOTE_F4,
    NOTE_D4,
    NOTE_A4,
    NOTE_D5,
    NOTE_D4,
    NOTE_C4,
    NOTE_C4,
    NOTE_A3,
    NOTE_E4,
    NOTE_D4,
    0,
    NOTE_D4,
    NOTE_D4,
};

int noteDurations[] = {
    4,
    6,
    8,
    16,
    8,
    8,
    8,
    4,
    6,
    8,
    16,
    8,
    8,
    8,
    8,
    8,
    8,
    16,
    8,
    16,
    8,
    8,
    8,
    2,
    4,
    4,
};

// loop through the notes in the melody
void playMelody()
{
    for (int thisNote = 0; thisNote < sizeof(melody) / sizeof(int); thisNote++)
    {
        int noteDuration = 1000 / noteDurations[thisNote];
        buzzer.tone(melody[thisNote], noteDuration); // play note on mBot buzzer
        // pause between notes
        int pauseBetweenNotes = noteDuration * 1.30; // reduce the 1.30 for a faster tempo
        delay(pauseBetweenNotes);
        buzzer.noTone(); // stop the tone to create a silence between notes
    }
};