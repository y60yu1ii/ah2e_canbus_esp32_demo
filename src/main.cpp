#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>

#define LED_PIN 13
char ssid[32] = "ah2e_wifi";
char mdns[32] = "ah2e";
char password[64] = "123456789";

WiFiManager wm;
void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Basic Demo - ESP32-WiFiManager");
    bool res;
    wm.resetSettings();
    res = wm.autoConnect(ssid, password);  // password protected ap
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);

    if (!res) {
        Serial.println("Failed to connect");
        digitalWrite(LED_PIN, false);
    } else {
        // if you get here you have connected to the WiFi
        Serial.println("Connected... yeeeeeey");
        digitalWrite(LED_PIN, false);
    }

    if (!MDNS.begin(mdns)) {
        Serial.println("Error starting mDNS");
        return;
    } else {
        Serial.print("mDNS OK, search for ");
        Serial.print(mdns);
        Serial.println(".local");
    }
}

void loop() {
    // put your main code here, to run repeatedly:
}