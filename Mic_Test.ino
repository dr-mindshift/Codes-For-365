#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_WS      25   // LRCLK / WS
#define I2S_SD      32   // DOUT from mic
#define I2S_SCK     33   // BCLK
#define SAMPLE_RATE 16000

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting INMP441 Serial Plotter RMS Output...");

  // Configure I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);
}

void loop() {
  const int samples = 512;
  int32_t buffer[samples];
  size_t bytes_read;

  // Read samples
  i2s_read(I2S_NUM_0, (void*)buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);

  // Compute RMS
  double sum = 0;
  for (int i = 0; i < samples_read; i++) {
    float sample = (float)buffer[i] / 2147483648.0f; // normalize [-1, 1]
    sum += sample * sample;
  }
  double rms = sqrt(sum / samples_read);

  // Scale up RMS for visibility in plot
  float displayValue = rms * 1000.0;  // scale (adjust as needed)

  // Output just one numeric value per line
  Serial.println(displayValue);

  delay(50); // update 20x per second (reduce for smoother plot)
}
