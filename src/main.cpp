#include <Arduino.h>
#include <Audio.h>   // Teensy Audio Library: I2S, Queue, Connections, AudioMemory
#include <SD.h>      // SD.begin, SD.open, SD.exists
#include <SPI.h>     // нужно SD библиотеке

// ------------------- НАСТРОЙКИ -------------------
const int BTN_REC = 2;                     // кнопка на GND
const unsigned long DEBOUNCE_MS = 20;

const uint32_t WAV_SR = 44100;             // частота WAV (Teensy Audio = 44.1 кГц)
const uint16_t WAV_BITS = 16;              // 16-bit PCM
const uint16_t WAV_CH = 2;                 // stereo

// ------------------- AUDIO GRAPH -------------------
// AudioInputI2S — вход по I2S (стерео). DATA IN = pin 8 на Teensy 4.1
AudioInputI2S i2sIn;

// AudioOutputI2S — выход по I2S (стерео). НУЖЕН, чтобы Teensy ТОЧНО был мастером и генерировал BCLK/LRCLK
// BCLK = pin 21, LRCLK = pin 20, DATA OUT = pin 7
AudioOutputI2S i2sOut;

// AudioRecordQueue — очередь блоков (каждый блок = 128 сэмплов int16)
AudioRecordQueue qL;
AudioRecordQueue qR;

// AudioConnection — "провода" между блоками аудиографа
AudioConnection c1(i2sIn, 0, qL, 0);   // левый канал -> очередь
AudioConnection c2(i2sIn, 1, qR, 0);   // правый канал -> очередь

// Чтобы выход был “тишиной”, ничего на i2sOut не подключаем.
// НО сам факт наличия AudioOutputI2S обычно включает генерацию I2S clocks.

// ------------------- SD/WAV -------------------
File wavFile;
bool isRecording = false;
uint32_t dataBytesWritten = 0;

// ------------------- ЛОГИ -------------------
static uint32_t lastLogMs = 0;
static uint32_t maxQL = 0, maxQR = 0;

// ------------------- WAV HEADER -------------------
// пишем "заглушку" заголовка WAV (44 байта). Размеры допишем при stop.
static void writeWavHeaderPlaceholder(File &f) {
  uint8_t hdr[44] = {0};

  // "RIFF"
  hdr[0]='R'; hdr[1]='I'; hdr[2]='F'; hdr[3]='F';
  // bytes 4..7 = chunkSize (36 + dataSize) — запишем позже
  // "WAVE"
  hdr[8]='W'; hdr[9]='A'; hdr[10]='V'; hdr[11]='E';

  // "fmt "
  hdr[12]='f'; hdr[13]='m'; hdr[14]='t'; hdr[15]=' ';
  // subchunk1Size = 16 (PCM)
  hdr[16]=16; hdr[17]=0; hdr[18]=0; hdr[19]=0;
  // audioFormat = 1 (PCM)
  hdr[20]=1; hdr[21]=0;
  // numChannels
  hdr[22]=WAV_CH; hdr[23]=0;

  // sampleRate (little-endian)
  uint32_t sr = WAV_SR;
  hdr[24]=sr & 0xFF; hdr[25]=(sr>>8)&0xFF; hdr[26]=(sr>>16)&0xFF; hdr[27]=(sr>>24)&0xFF;

  // byteRate = sampleRate * channels * bits/8
  uint32_t byteRate = (uint32_t)WAV_SR * (uint32_t)WAV_CH * (uint32_t)(WAV_BITS/8);
  hdr[28]=byteRate & 0xFF; hdr[29]=(byteRate>>8)&0xFF; hdr[30]=(byteRate>>16)&0xFF; hdr[31]=(byteRate>>24)&0xFF;

  // blockAlign = channels * bits/8
  uint16_t blockAlign = WAV_CH * (WAV_BITS/8);
  hdr[32]=blockAlign & 0xFF; hdr[33]=(blockAlign>>8)&0xFF;

  // bitsPerSample
  hdr[34]=WAV_BITS & 0xFF; hdr[35]=(WAV_BITS>>8)&0xFF;

  // "data"
  hdr[36]='d'; hdr[37]='a'; hdr[38]='t'; hdr[39]='a';
  // bytes 40..43 = dataSize — запишем позже

  f.write(hdr, sizeof(hdr)); // File.write — запись байтов в файл
}

// дописываем реальные размеры в WAV header
static void finalizeWavHeader(File &f, uint32_t dataSize) {
  uint32_t chunkSize = 36 + dataSize;

  f.seek(4);                   // File.seek — перейти к позиции в файле
  f.write((uint8_t*)&chunkSize, 4);

  f.seek(40);
  f.write((uint8_t*)&dataSize, 4);
}

// автогенерация имени REC0001.WAV, REC0002.WAV...
static String makeFilename() {
  for (int i = 1; i < 10000; i++) {
    char name[13];
    snprintf(name, sizeof(name), "REC%04d.WAV", i);
    if (!SD.exists(name)) {    // SD.exists — проверка, есть ли файл
      return String(name);
    }
  }
  return String("REC9999.WAV");
}

static void startRecording() {
  if (isRecording) return;

  String fn = makeFilename();
  wavFile = SD.open(fn.c_str(), FILE_WRITE);   // SD.open — открыть файл на запись
  if (!wavFile) {
    Serial.println("[REC] ERROR: SD.open failed");
    return;
  }

  writeWavHeaderPlaceholder(wavFile);
  dataBytesWritten = 0;

  qL.clear();   // clear — очистить очередь, если там что-то было
  qR.clear();

  qL.begin();   // begin — начать складывать новые блоки в очередь
  qR.begin();

  isRecording = true;
  maxQL = maxQR = 0;
  Serial.print("[REC] START file=");
  Serial.println(fn);

  digitalWrite(13, HIGH); // LED — индикатор записи
}

static void stopRecording() {
  if (!isRecording) return;

  qL.end();   // end — прекратить наполнение очереди
  qR.end();

  // допишем всё, что осталось в очередях
  while (qL.available() > 0 && qR.available() > 0) { // available — сколько блоков готово
    int16_t *l = (int16_t*)qL.readBuffer();          // readBuffer — указатель на данные блока
    int16_t *r = (int16_t*)qR.readBuffer();

    int16_t stereo[256];                             // 128 сэмплов * 2 канала
    for (int i = 0; i < 128; i++) {
      stereo[2*i]   = l[i];
      stereo[2*i+1] = r[i];
    }

    wavFile.write((uint8_t*)stereo, sizeof(stereo));
    dataBytesWritten += sizeof(stereo);

    qL.freeBuffer(); // freeBuffer — освободить блок (важно!)
    qR.freeBuffer();
  }

  finalizeWavHeader(wavFile, dataBytesWritten);
  wavFile.close(); // close — закрыть файл (важно, иначе WAV битый)

  isRecording = false;

  Serial.print("[REC] STOP bytes=");
  Serial.print(dataBytesWritten);
  Serial.print(" maxQ(L/R)=");
  Serial.print(maxQL);
  Serial.print("/");
  Serial.println(maxQR);

  digitalWrite(13, LOW);
}

static void serviceRecording() {
  if (!isRecording) return;

  // мониторим, не накапливается ли очередь (это признак, что SD не успевает)
  uint32_t aL = qL.available();
  uint32_t aR = qR.available();
  if (aL > maxQL) maxQL = aL;
  if (aR > maxQR) maxQR = aR;

  // Пишем блоки, пока есть и L и R
  while (qL.available() > 0 && qR.available() > 0) {
    int16_t *l = (int16_t*)qL.readBuffer();
    int16_t *r = (int16_t*)qR.readBuffer();

    int16_t stereo[256];
    for (int i = 0; i < 128; i++) {
      stereo[2*i]   = l[i];
      stereo[2*i+1] = r[i];
    }

    wavFile.write((uint8_t*)stereo, sizeof(stereo));
    dataBytesWritten += sizeof(stereo);

    qL.freeBuffer();
    qR.freeBuffer();
  }

  // раз в секунду — лог прогресса
  if (millis() - lastLogMs >= 1000) {
    lastLogMs = millis();
    Serial.print("[REC] bytes=");
    Serial.print(dataBytesWritten);
    Serial.print(" q(L/R)=");
    Serial.print(qL.available());
    Serial.print("/");
    Serial.println(qR.available());
  }
}

void setup() {
  pinMode(13, OUTPUT);
  pinMode(BTN_REC, INPUT_PULLUP); // INPUT_PULLUP — внутренняя подтяжка к 3.3В, кнопка на GND

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {} // ждём, пока монитор подключится
  Serial.println("[BOOT] I2S recorder (hold BTN2)");

  AudioMemory(48); // AudioMemory — буферы аудиодвижка. Больше = меньше шанс дропов.

  Serial.println("[SD] init...");
  if (!SD.begin(BUILTIN_SDCARD)) { // BUILTIN_SDCARD — встроенный microSD слот Teensy 4.1
    Serial.println("[SD] FAIL");
    while (!SD.begin(BUILTIN_SDCARD)) { digitalWrite(13, !digitalRead(13)); delay(150); } // мигаем, если SD не поднялась
  }
  Serial.println("[SD] OK");

  Serial.println("[I2S] NOTE: Teensy generates BCLK/LRCLK because AudioOutputI2S is present.");
}

void loop() {
  // антидребезг на millis()
  static bool lastRaw = HIGH;
  static bool stable = HIGH;
  static uint32_t lastChangeMs = 0;

  bool raw = digitalRead(BTN_REC);
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = millis();
  }

  if ((millis() - lastChangeMs) > DEBOUNCE_MS && stable != raw) {
    stable = raw;

    if (stable == LOW) {
      Serial.println("[BTN] pressed -> START");
      startRecording();
    } else {
      Serial.println("[BTN] released -> STOP");
      stopRecording();
    }
  }

  serviceRecording();
}
