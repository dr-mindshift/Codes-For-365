#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Pin definitions for ESP32-WROOM-32E
#define TFT_CS    5   // Chip Select
#define TFT_RST   22  // Reset
#define TFT_DC    21  // Data/Command

// Hardware SPI pins (don't need to define, but for reference):
// MOSI = GPIO23
// MISO = GPIO19
// SCK  = GPIO18

// Create display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ILI9341 Test Starting...");

  // Initialize the display
  Serial.println("Initializing display...");
  tft.begin();
  
  Serial.println("Display initialized!");
  
  // Set rotation (0-3, try different values if display is rotated wrong)
  tft.setRotation(1); // 1 = landscape, 0 = portrait
  
  // Clear screen with red (easier to see than black)
  Serial.println("Filling screen RED...");
  tft.fillScreen(ILI9341_RED);
  delay(2000);
  
  Serial.println("Filling screen GREEN...");
  tft.fillScreen(ILI9341_GREEN);
  delay(2000);
  
  Serial.println("Filling screen BLUE...");
  tft.fillScreen(ILI9341_BLUE);
  delay(2000);
  
  Serial.println("Drawing shapes...");
  testDisplay();
  
  Serial.println("Test complete!");
}

void loop() {
  // Do nothing - test runs once in setup
  delay(1000);
}

void testDisplay() {
  // Clear to black background
  tft.fillScreen(ILI9341_BLACK);
  delay(500);
  
  // Draw text at top
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.println("ESP32");
  
  tft.setCursor(10, 45);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.println("ILI9341 Display");
  
  tft.setCursor(10, 70);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("Shape Test:");
  
  delay(1000);
  
  // Draw rectangles
  tft.drawRect(10, 90, 80, 60, ILI9341_GREEN);
  tft.fillRect(100, 90, 80, 60, ILI9341_RED);
  tft.drawRect(190, 90, 80, 60, ILI9341_BLUE);
  
  // Draw circles
  tft.drawCircle(50, 200, 30, ILI9341_MAGENTA);
  tft.fillCircle(140, 200, 30, ILI9341_ORANGE);
  tft.drawCircle(230, 200, 30, ILI9341_CYAN);
  
  // Draw lines
  tft.drawLine(10, 260, 310, 260, ILI9341_WHITE);
  tft.drawLine(10, 270, 310, 270, ILI9341_YELLOW);
  
  // Draw triangles
  tft.drawTriangle(30, 280, 10, 310, 50, 310, ILI9341_GREEN);
  tft.fillTriangle(120, 280, 100, 310, 140, 310, ILI9341_RED);
  
  // Draw rounded rectangles
  tft.drawRoundRect(170, 280, 60, 30, 8, ILI9341_BLUE);
  tft.fillRoundRect(240, 280, 60, 30, 8, ILI9341_MAGENTA);
}