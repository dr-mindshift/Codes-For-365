// KY-040 Rotary Encoder Counter for ESP32
// Wiring: VCC->3.3V, GND->GND, SW->GPIO13, DT->GPIO27, CLK->GPIO14

#define CLK_PIN 14
#define DT_PIN 27
#define SW_PIN 13

volatile int counter = 0;
volatile bool clkState;
volatile bool lastClkState;
volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 5; // 5ms debounce

void IRAM_ATTR handleRotary() {
  unsigned long currentTime = millis();
  
  // Debounce check
  if (currentTime - lastInterruptTime < debounceDelay) {
    return;
  }
  
  clkState = digitalRead(CLK_PIN);
  
  // If CLK state changed and is now LOW (falling edge)
  if (clkState != lastClkState && clkState == LOW) {
    // Check DT pin to determine direction
    if (digitalRead(DT_PIN) == HIGH) {
      counter++; // Clockwise
    } else {
      counter--; // Counter-clockwise
    }
    Serial.print("Counter: ");
    Serial.println(counter);
  }
  
  lastClkState = clkState;
  lastInterruptTime = currentTime;
}

void IRAM_ATTR handleButton() {
  static unsigned long lastButtonPress = 0;
  unsigned long currentTime = millis();
  
  // Button debounce (50ms)
  if (currentTime - lastButtonPress > 50) {
    counter = 0; // Reset counter on button press
    Serial.println("Counter RESET: 0");
    lastButtonPress = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set up pins
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  
  // Read initial state
  lastClkState = digitalRead(CLK_PIN);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handleRotary, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_PIN), handleButton, FALLING);
  
  Serial.println("KY-040 Rotary Encoder Ready");
  Serial.println("Rotate to count, press button to reset");
  Serial.print("Counter: ");
  Serial.println(counter);
}

void loop() {
  // Main loop can do other tasks
  // Counter updates happen in interrupt handlers
  delay(10);
}