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

void setup() {
    Serial.begin(115200);
    Serial.println("Booting");
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

byte a = 1;

// first:
// abcdefgD
// gcbafedD
// 76543210

byte alphabet[10] = {
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

unsigned long long value = 0;

void loop() {
    ArduinoOTA.handle();
    
    if (timeClient.update()) {
        Serial.println("Updating time");
        updateTime();
    }

    time_t current_time;
    time(&current_time);
    struct tm timeinfo;
    localtime_r(&current_time, &timeinfo);
    current[0] = alphabet[timeinfo.tm_hour / 10];
    current[1] = alphabet[timeinfo.tm_hour % 10];
    current[2] = alphabet[timeinfo.tm_min / 10];
    current[3] = alphabet[timeinfo.tm_min % 10];
    current[4] = alphabet[timeinfo.tm_sec / 10];
    current[5] = alphabet[timeinfo.tm_sec % 10];

    struct timeval current_time_v;
    gettimeofday(&current_time_v, NULL);
    bool dots = true;
    if (current_time_v.tv_usec / 1000 % 1000 > 500) {
        dots = false;
    }
    writeAll(current, dots, dots);
}
