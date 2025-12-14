#include <WiFi.h>
#include <HTTPClient.h>
#include <SD.h>
#include <SPI.h>
#include <WiFiClientSecure.h>

const char* ssid = "WIFI NAME";
const char* password = "WIFI PASSWORD";

#define SD_CS 4
#define BUTTON_PIN 21

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  if(!SD.begin(SD_CS)) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  Serial.println("SD Card initialized!");
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());
}

void loop() {
  if(digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Button pressed! Uploading...");
    uploadFileChunked("/rec_9.wav");
    delay(2000);  // Prevent double-press
  }
}
void uploadFileChunked(String filename) {
  File file = SD.open(filename);
  if(!file) {
    Serial.println("Failed to open file: " + filename);
    return;
  }
  
  size_t fileSize = file.size();
  Serial.print("File size: ");
  Serial.print(fileSize);
  Serial.println(" bytes");
  
  String host = "storage.googleapis.com";
  String path = "THIS IS WHERE YOU HAVE TO PUT YOUR WORKING AUTHORIZED LINK";
  int port = 443;
  
  WiFiClientSecure client;
  client.setInsecure();
  
  Serial.print("Connecting to ");
  Serial.println(host);
  
  if(!client.connect(host.c_str(), port)) {
    Serial.println("Connection failed!");
    file.close();
    return;
  }
  
  Serial.println("Connected! Sending headers...");
  
  // Send HTTP PUT headers - ONLY host header (as specified in x-goog-signedheaders)
  client.print("PUT ");
  client.print(path);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(host);
  client.println("Content-Type: audio/wav");
  client.print("Content-Length: ");
  client.println(fileSize);
  client.println("Connection: close");
  client.println();
  
  // ... rest of the code stays the same
  // Send file in chunks
  const size_t bufferSize = 1024;  // 1KB chunks
  uint8_t buffer[bufferSize];
  size_t totalSent = 0;
  
  Serial.println("Sending file data...");
  
  while(file.available()) {
    size_t bytesRead = file.read(buffer, bufferSize);
    size_t bytesWritten = client.write(buffer, bytesRead);
    totalSent += bytesWritten;
    
    if(totalSent % 10240 == 0 || !file.available()) {  // Print every 10KB
      Serial.print("Sent: ");
      Serial.print(totalSent);
      Serial.print(" / ");
      Serial.print(fileSize);
      Serial.print(" bytes (");
      Serial.print((totalSent * 100) / fileSize);
      Serial.println("%)");
    }
    
    yield();  // Let ESP32 handle WiFi stack
  }
  
  Serial.println("\nWaiting for response...");
  
  // Wait for and print response
  unsigned long timeout = millis();
  while(client.connected() && millis() - timeout < 10000) {
    if(client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
      
      if(line.startsWith("HTTP/")) {
        if(line.indexOf("200") > 0) {
          Serial.println("✓ Upload successful to Google Cloud Storage!");
        } else {
          Serial.println("✗ Upload failed - check response code");
        }
      }
    }
  }
  
  file.close();
  client.stop();
  Serial.println("Done!");
}