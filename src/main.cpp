#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define LATCH_PIN 19
#define CLOCK_PIN 18
#define DATA_PIN 21

#include "config.h"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "0.ntp.bksp.in");

void updateTime() {
    struct timeval new_time;
    unsigned long nowMillis;
    new_time.tv_sec = timeClient.getEpochTime(&nowMillis);
    new_time.tv_usec = nowMillis * 1000;
    Serial.print("New time: ");
    Serial.println(new_time.tv_sec);
    settimeofday(&new_time, NULL);
}

IPAddress localIp(10, 0, 2, 105);
IPAddress gateway(10, 0, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDns(10, 0, 2, 1);

// https://github.com/skeeto/hash-prospector with [16 21f0aaad 15 735a2d97 15] = 0.10704308166917044
uint32_t lowbias32(uint32_t x) {
  x ^= x >> 16;
  x *= UINT32_C(0x21f0aaad);
  x ^= x >> 15;
  x *= UINT32_C(0x735a2d97);
  x ^= x >> 15;
  return x;
}

uint32_t hashSeed;

static void randomSeed(void) {
  unsigned long r = random(LONG_MAX); // in case if it's already seeded
  for (int i = 0; i < sizeof(r) * 8; i++)
    r ^= ((unsigned long)analogRead(0)) << i; // analogRead() is just 10..12 bits
  randomSeed(r);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Booting");
    randomSeed();
    hashSeed = random(UINT32_MAX);
    WiFi.mode(WIFI_MODE_STA);
    if (!WiFi.config(localIp, gateway, subnet, primaryDns)) {
        Serial.println("STA Failed to configure");
        delay(5000);
        ESP.restart();
        return;
    }
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
        return;
    }
    ArduinoOTA
        .onStart([]() {
            if (ArduinoOTA.getCommand() == U_FLASH) {
                Serial.println("Start updating sketch"); 
            } else {
                Serial.println("Start updating fs"); 
            }
        })
        .onEnd([]() { 
            Serial.println("\nEnd"); 
        })
        .onProgress([] (unsigned int progress, unsigned int total) { 
            Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            switch (error) {
                case OTA_AUTH_ERROR:
                    Serial.println("Auth failed");
                    return;
                case OTA_BEGIN_ERROR:
                    Serial.println("Begin failed");
                    return;
                case OTA_CONNECT_ERROR:
                    Serial.println("Connect failed");
                    return;
                case OTA_RECEIVE_ERROR:
                    Serial.println("Receive failed");
                    return;
                case OTA_END_ERROR:
                    Serial.println("End failed");
                    return;
            }
        });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    pinMode(LATCH_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);

    timeClient.begin();
    timeClient.setTimeOffset(3 * 3600);

    Serial.println("Starting time");
    
    if (!timeClient.update()) {
        Serial.println("Failed to update time");
    } else {
        Serial.println("Updating time");
        updateTime();
    }
}

// first:
// abcdefgD
// gcbafedD
// 76543210

byte digits[10] = {
    0b11111100,
    0b01100000,
    0b11011010,
    0b11110010,
    0b01100110,
    0b10110110,
    0b10111110,
    0b11100000,
    0b11111110,
    0b11110110,
};

const byte kDots = 0b00000001;

byte current[] = {
    0b10011110,
    0b10001100,
    0b11111100,
    0b11001110,
    0,
    0
};

byte transformSegments(byte b) {
    b = 
        (((b >> 7) & 0x01) << 4) |
        (((b >> 6) & 0x01) << 5) |
        (((b >> 5) & 0x01) << 6) |
        (((b >> 4) & 0x01) << 1) |
        (((b >> 3) & 0x01) << 2) |
        (((b >> 2) & 0x01) << 3) |
        (((b >> 1) & 0x01) << 7) |
        (((b >> 0) & 0x01));
    return b;
}

void writeAll(byte* data, byte dot_a, byte dot_b) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, transformSegments(data[2]) | (kDots & dot_a));
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, transformSegments(data[1]));
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, transformSegments(data[0]));
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data[5]);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data[4]);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data[3] | (kDots & dot_b));
    digitalWrite(LATCH_PIN, HIGH);
}

uint32_t lastTimeHashInput = 0;
uint8_t lastDots = 0;

void loop() {
    ArduinoOTA.handle();
    
    if (timeClient.update()) {
        Serial.println("Updating time");
        updateTime();
    }

    unsigned long nowMillis;
    unsigned long epochTime = timeClient.getEpochTime(&nowMillis);
    unsigned long timeOfDay = epochTime % 86400;
    unsigned hour = timeOfDay / 3600;
    unsigned min = (timeOfDay % 3600) / 60;
    unsigned sec = (timeOfDay % 3600) % 60;

    current[0] = digits[hour / 10];
    current[1] = digits[hour % 10];
    current[2] = digits[min / 10];
    current[3] = digits[min % 10];
    current[4] = digits[sec / 10];
    current[5] = digits[sec % 10];

    const uint32_t hashInput = (timeOfDay << 1) | (nowMillis >= 500 ? 1 : 0);
    if (hashInput != lastTimeHashInput) {
      for (uint32_t i = 0; i < 4096; i++) {
        uint8_t maybe = 3 & lowbias32(hashSeed ^ hashInput ^ (i << 20));
        if (maybe != lastDots) {
          lastTimeHashInput = hashInput;
          lastDots = maybe;
          break;
        }
      }
    }
    writeAll(current, lastDots & 1, lastDots >> 1);
}
