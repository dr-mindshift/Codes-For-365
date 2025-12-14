#include <driver/i2s.h>
#include <SD.h>
#include <SPI.h>

// I2S Pins (shared between mic and speaker)
#define I2S_WS 25      // Shared: MIC WS & SPEAKER LRC
#define I2S_SCK 33     // Shared: MIC SCK & SPEAKER BCLK
#define I2S_SD_IN 32   // MIC SD (data in)
#define I2S_SD_OUT 26  // SPEAKER DIN (data out)

// SD Card Pins
#define SD_CS 5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18

// Button Pins
#define RECORD_BUTTON 15  // Record/Stop button
#define PLAYBACK_BUTTON 2 // Playback button

// Recording Settings
#define SAMPLE_RATE 16000
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define I2S_PORT I2S_NUM_0
#define BUFFER_LEN 512

bool isRecording = false;
int fileCounter = 1;
String lastRecordedFile = "";

// Global buffers to avoid stack allocation
int32_t* i2s_buffer = NULL;
int16_t* wav_buffer = NULL;

// Button debounce variables
unsigned long lastRecordPress = 0;
unsigned long lastPlaybackPress = 0;
const unsigned long debounceDelay = 50;

// Global file handle for continuous recording
File recordFile;
uint32_t samplesWritten = 0;
unsigned long recordStartTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Audio Recorder & Player Starting...");
  
  // Allocate buffers on heap instead of stack
  i2s_buffer = (int32_t*)malloc(BUFFER_LEN * sizeof(int32_t));
  wav_buffer = (int16_t*)malloc(BUFFER_LEN * sizeof(int16_t));
  
  if (!i2s_buffer || !wav_buffer) {
    Serial.println("Failed to allocate memory for buffers!");
    return;
  }
  
  // Setup buttons
  pinMode(RECORD_BUTTON, INPUT_PULLUP);
  pinMode(PLAYBACK_BUTTON, INPUT_PULLUP);
  
  // Initialize SD Card
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
  Serial.println("SD Card initialized successfully");
  
  // Find next available file number
  while (SD.exists("/rec_" + String(fileCounter) + ".wav")) {
    fileCounter++;
  }
  
  // Set last recorded file to the previous one if it exists
  if (fileCounter > 1) {
    lastRecordedFile = "/rec_" + String(fileCounter - 1) + ".wav";
  }
  
  Serial.println("Ready!");
  Serial.println("GPIO 15: Press to start/stop recording");
  Serial.println("GPIO 2: Press to play last recording");
}

void loop() {
  unsigned long currentTime = millis();
  
  // If recording, continuously capture audio
  if (isRecording && recordFile) {
    size_t bytesRead;
    
    // Read from I2S
    i2s_read(I2S_PORT, i2s_buffer, BUFFER_LEN * sizeof(int32_t), &bytesRead, 100);
    
    if (bytesRead > 0) {
      int samplesRead = bytesRead / sizeof(int32_t);
      
      // Convert 32-bit I2S data to 16-bit WAV data
      for (int i = 0; i < samplesRead; i++) {
        wav_buffer[i] = (i2s_buffer[i] >> 14);
      }
      
      // Write to file
      recordFile.write((byte*)wav_buffer, samplesRead * sizeof(int16_t));
      samplesWritten += samplesRead;
      
      // Print progress dot every second
      if (samplesWritten % SAMPLE_RATE < BUFFER_LEN) {
        Serial.print(".");
      }
    }
  }
  
  // Check RECORD button (GPIO 15)
  if (digitalRead(RECORD_BUTTON) == LOW && (currentTime - lastRecordPress > debounceDelay)) {
    lastRecordPress = currentTime;
    delay(50);
    
    if (digitalRead(RECORD_BUTTON) == LOW) {
      if (!isRecording) {
        // Start recording
        isRecording = true;
        lastRecordedFile = "/rec_" + String(fileCounter) + ".wav";
        startRecording(lastRecordedFile);
      } else {
        // Stop recording
        stopRecording();
        isRecording = false;
        fileCounter++;
        Serial.println("\nReady! Press GPIO 15 to record, GPIO 2 to play last recording.");
      }
      
      // Wait for button release
      while (digitalRead(RECORD_BUTTON) == LOW) {
        delay(10);
      }
    }
  }
  
  // Check PLAYBACK button (GPIO 2) - only if not recording
  if (!isRecording && digitalRead(PLAYBACK_BUTTON) == LOW && (currentTime - lastPlaybackPress > debounceDelay)) {
    lastPlaybackPress = currentTime;
    delay(50);
    
    if (digitalRead(PLAYBACK_BUTTON) == LOW) {
      if (lastRecordedFile != "" && SD.exists(lastRecordedFile)) {
        Serial.println("Playing last recording: " + lastRecordedFile);
        playAudio(lastRecordedFile);
        Serial.println("Ready! Press GPIO 15 to record, GPIO 2 to play last recording.");
      } else {
        Serial.println("No recordings found!");
      }
      
      // Wait for button release
      while (digitalRead(PLAYBACK_BUTTON) == LOW) {
        delay(10);
      }
    }
  }
  
  delay(10);
}

void configureI2SForRecording() {
  // Only uninstall if already installed
  i2s_driver_uninstall(I2S_PORT);  // This is safe - will just return error if not installed
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_IN
  };
  
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void configureI2SForPlayback() {
  // Only uninstall if already installed
  i2s_driver_uninstall(I2S_PORT);  // This is safe - will just return error if not installed
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_SD_OUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void startRecording(String filename) {
  Serial.println("Starting recording to: " + filename);
  
  // Configure I2S for recording
  configureI2SForRecording();
  
  recordFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!recordFile) {
    Serial.println("Failed to open file for writing");
    isRecording = false;
    return;
  }
  
  // Write placeholder WAV header (will update when recording stops)
  byte header[WAV_HEADER_SIZE];
  writeWavHeader(header, 0, SAMPLE_RATE, SAMPLE_BITS);
  recordFile.write(header, WAV_HEADER_SIZE);
  
  samplesWritten = 0;
  recordStartTime = millis();
  
  Serial.println("Recording... Press GPIO 15 to stop.");
}

void stopRecording() {
  if (!recordFile) {
    return;
  }
  
  unsigned long recordTime = millis() - recordStartTime;
  
  // Calculate actual data size
  uint32_t dataSize = samplesWritten * (SAMPLE_BITS / 8);
  
  // Update WAV header with correct size
  recordFile.seek(0);
  byte header[WAV_HEADER_SIZE];
  writeWavHeader(header, dataSize, SAMPLE_RATE, SAMPLE_BITS);
  recordFile.write(header, WAV_HEADER_SIZE);
  
  recordFile.close();
  
  Serial.println("\nRecording stopped!");
  Serial.println("Duration: " + String(recordTime / 1000.0) + " seconds");
  Serial.println("Samples: " + String(samplesWritten));
  Serial.println("File: " + lastRecordedFile);
}

void playAudio(String filename) {
  // Configure I2S for playback
  configureI2SForPlayback();
  
  File file = SD.open(filename.c_str());
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  
  // Skip WAV header
  file.seek(WAV_HEADER_SIZE);
  
  size_t bytesWritten;
  int bytesRead;
  
  Serial.println("Playback started...");
  
  // Read and play audio data
  while (file.available()) {
    bytesRead = file.read((byte*)wav_buffer, BUFFER_LEN * sizeof(int16_t));
    
    if (bytesRead > 0) {
      // Write to I2S (playback)
      i2s_write(I2S_PORT, wav_buffer, bytesRead, &bytesWritten, portMAX_DELAY);
    }
  }
  
  // Wait for I2S to finish transmitting
  delay(100);
  
  file.close();
  Serial.println("Playback finished!");
}

void writeWavHeader(byte* header, uint32_t dataSize, uint32_t sampleRate, uint16_t bitsPerSample) {
  uint32_t fileSize = dataSize + WAV_HEADER_SIZE - 8;
  uint16_t numChannels = 1;
  uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
  uint16_t blockAlign = numChannels * (bitsPerSample / 8);
  
  // RIFF chunk descriptor
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  
  // fmt sub-chunk
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 0x10; header[17] = 0x00; header[18] = 0x00; header[19] = 0x00;
  header[20] = 0x01; header[21] = 0x00; // PCM
  header[22] = (byte)(numChannels & 0xFF); header[23] = (byte)((numChannels >> 8) & 0xFF);
  header[24] = (byte)(sampleRate & 0xFF);
  header[25] = (byte)((sampleRate >> 8) & 0xFF);
  header[26] = (byte)((sampleRate >> 16) & 0xFF);
  header[27] = (byte)((sampleRate >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = (byte)(blockAlign & 0xFF);
  header[33] = (byte)((blockAlign >> 8) & 0xFF);
  header[34] = (byte)(bitsPerSample & 0xFF);
  header[35] = (byte)((bitsPerSample >> 8) & 0xFF);
  
  // data sub-chunk
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF);
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF);
}