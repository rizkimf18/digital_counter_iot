#include <Arduino.h>
#include <Wifi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6U5RXJ5k_"
#define BLYNK_TEMPLATE_NAME "Summary Counting"
#define BLYNK_AUTH_TOKEN "f8dbB5E9_w1LGEh9imLd59ro7ExnGRmK"
#include <BlynkSimpleEsp32.h>

#define sensorPin       4  // Proximity Pin
#define pushButton      17
#define powerDetecTPin  14

float analogThreshold = 4000; //analog value 

char ssid[] = "KOK";
char pass[] = "123123123";

volatile unsigned long counter = 0;  
unsigned long lastCounter = 0;
unsigned long lastInterruptTime = 0;
const unsigned long debounceSensor = 50;  // in milliseconds (ms) debounce delay
bool wifiStatus = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);  

void IRAM_ATTR detectSensor();
void wifiAndSendDataTask(void *parameter);
void lcdDisplayTask(void *paramter);
void resetCounterTask(void *parameter);
void checkPowerDownTask(void *parameter);
unsigned long readCounterFromEEPROM();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(pushButton, INPUT_PULLUP);

  EEPROM.begin(64);
  counter = readCounterFromEEPROM();

  attachInterrupt(digitalPinToInterrupt(sensorPin), detectSensor, FALLING);

  // Task pada Core 0
  xTaskCreatePinnedToCore(wifiAndSendDataTask, "Wifi and Send Data Task", 5120, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(checkPowerDownTask, "Check Power Down Task", 2048, NULL, 1, NULL, 1);  

  // Task pada Core 1
  xTaskCreatePinnedToCore(lcdDisplayTask, "LCD Display Task", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(resetCounterTask, "Reset Counter Task", 4096, NULL, 1, NULL, 1);
}

void loop() { 
  // Serial.println(counter);
}

void IRAM_ATTR detectSensor() {
  unsigned long interruptTime = millis();
  bool readSensor = digitalRead(sensorPin);

  if(interruptTime - lastInterruptTime > debounceSensor) {
    if (readSensor == LOW) {
      counter++;
    }
    lastInterruptTime = interruptTime;
  }
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi ..");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) { // Timeout setelah 30 detik
    Serial.print('.');
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Memberi kesempatan task lain berjalan
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    wifiStatus = 1;
  } else {
    Serial.println("Failed to connect to WiFi");
    wifiStatus = 0;
  }

  // Serial.print("Task SetupWifi: ");
  // Serial.println(uxTaskGetStackHighWaterMark(NULL)); // NULL berarti task ini
}


void wifiAndSendDataTask(void *parameter) {
  setupWifi();
  while(1) {
    // Logic pengiriman data ke server
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();
    Blynk.virtualWrite(V0, counter);
    Serial.print("Send data to Blynk: ");
    Serial.println(counter);
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    Serial.print("Task wifiAndSendata: ");
     Serial.println(uxTaskGetStackHighWaterMark(NULL)); // NULL berarti task ini
  }
}

void lcdDisplayTask(void *parameter) {
  while(1) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("COUNTER     :");
    lcd.setCursor(14,0);
    lcd.print(counter);
    
    lcd.setCursor(0,1);
    lcd.print("LAST COUNTER:");
    lcd.setCursor(14,1);
    lcd.print(lastCounter);
    
    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.print("Last Counter: ");
    Serial.println(lastCounter);

    if(wifiStatus == 1) {
      lcd.setCursor(5,3);
      lcd.print("Connected");
    } else {
      lcd.setCursor(3,3);
      lcd.print("Disconnected");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  //   Serial.print("Task lcdDisplay: ");
  // Serial.println(uxTaskGetStackHighWaterMark(NULL)); // NULL berarti task ini
  }
}

void resetCounterTask(void *parameter) {
  unsigned long lastSDebounceTime = 0;
  int debounceButton = 100;
  bool lastButtonState = HIGH;
  bool buttonState;
  bool buttonPressed = false;

  while(1) {
    buttonState = digitalRead(pushButton);

    if (buttonState != lastButtonState) {
      lastSDebounceTime = millis();
      buttonPressed = true;
    }    

    if (millis() - lastSDebounceTime > debounceButton) {
      if (buttonState == LOW && buttonPressed == true) {
        Serial.println("Reset Counter");
        lastCounter = counter;
        Blynk.virtualWrite(V1, lastCounter);
        counter = 0;
        buttonPressed = false;
      } 
    }

    lastButtonState = buttonState;
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay kecil untuk mengurangi beban CPU
  //   Serial.print("Task ResetCounter: ");
  // Serial.println(uxTaskGetStackHighWaterMark(NULL)); // NULL berarti task ini
  }
}

void saveCounterToEEPROM(unsigned long value) {
  EEPROM.write(0, (value >> 24) & 0xFF);  // Byte pertama
  EEPROM.write(1, (value >> 16) & 0xFF);  // Byte kedua
  EEPROM.write(2, (value >> 8) & 0xFF);   // Byte ketiga
  EEPROM.write(3, value & 0xFF);          // Byte keempat
  EEPROM.commit();  // Pastikan data tersimpan ke EEPROM
}

unsigned long readCounterFromEEPROM() {
  unsigned long value = 0;
  value |= ((unsigned long)EEPROM.read(0)) << 24;
  value |= ((unsigned long)EEPROM.read(1)) << 16;
  value |= ((unsigned long)EEPROM.read(2)) << 8;
  value |= EEPROM.read(3);
  return value;
}

void checkPowerDownTask(void *parameter) {
  while(1) {
    int analogValue = analogRead(powerDetecTPin);
    
    if (analogValue < analogThreshold) {
      saveCounterToEEPROM(counter);
      vTaskDelay(500 / portTICK_PERIOD_MS);  // Beri waktu untuk penyimpanan
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Cek tegangan setiap 100ms
  }
}