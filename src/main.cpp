#include <Arduino.h>
#include <Audio.h>
#include <SD.h>

// WAV с SD
AudioPlaySdWav  playWav1;

// I2S OUT (Teensy 4.1): BCLK=21, LRCLK=20, DATA=7
AudioOutputI2S  i2s1;

// Соединения аудио-блоков (левый/правый)
AudioConnection patchCord1(playWav1, 0, i2s1, 0);
AudioConnection patchCord2(playWav1, 1, i2s1, 1);

const int BTN = 1; // кнопка на GND

static void playFile() {
  // stop() — остановить текущее воспроизведение (если было)
  if (playWav1.isPlaying()) {
    playWav1.stop();
    delay(5);
  }

  // play("...") — старт проигрывания WAV по имени
  bool ok = playWav1.play("SINE.WAV");
  if (ok) Serial.println("PLAY SINE.WAV");
  else    Serial.println("PLAY FAIL (name/format/SD)");
}

void setup() {
  pinMode(13, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);     // INPUT_PULLUP — внутр. подтяжка, кнопка на GND

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println("READY");

  AudioMemory(16);                // AudioMemory — буферы аудиодвижка

  if (!SD.begin(BUILTIN_SDCARD)) { // встроенный microSD слот Teensy 4.1
    Serial.println("SD FAIL");
    while (1) { digitalWrite(13, !digitalRead(13)); delay(200); }
  }
  Serial.println("SD OK");
}

void loop() {
  static bool last = HIGH;
  bool now = digitalRead(BTN);

  // фронт нажатия (было HIGH, стало LOW)
  if (last == HIGH && now == LOW) {
    digitalWrite(13, HIGH);
    playFile();
    delay(40);
    digitalWrite(13, LOW);
  }

  last = now;
}
