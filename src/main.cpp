#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>
// Modbus server include
#include "ModbusServerWiFi.h"
// #include "ModbusClientTCP.h"

#define LED_PIN 13
char ssid[32] = "ah2e_wifi";
char mdns[32] = "ah2e";
char password[64] = "123456789";
WiFiManager wm;

uint16_t port = 502;
ModbusServerWiFi MBserver;
int hearbeat = 0;
// Server function to handle FC 0x03 and 0x04
ModbusMessage FC0304(ModbusMessage request) {
    ModbusMessage response;  // The Modbus message we are going to give back
    uint16_t addr = 0;       // Start address
    uint16_t words = 0;      // # of words requested
    request.get(2, addr);    // read address from request
    request.get(4, words);   // read # of words from request

    // Address overflow?
    if ((addr + words) > 20) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    } else {
        // Set up response
        response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
        // Request for FC 0x03?
        if (request.getFunctionCode() == READ_HOLD_REGISTER) {
            // Yes. Complete response
            for (uint8_t i = 0; i < words; ++i) {
                if (i == 0) {
                    response.add((uint16_t)(hearbeat));
                } else {
                    response.add((uint16_t)(addr + i));
                }
            }
        }
        if (request.getFunctionCode() == READ_INPUT_REGISTER) {
            // No, this is for FC 0x04. Response is random
            for (uint8_t i = 0; i < words; ++i) {
                // send increasing data values
                // response.add((uint16_t)random(1, 65535));
                if (i == 0) {
                    response.add((uint16_t)(hearbeat + 1));
                } else {
                    response.add((uint16_t)random(1, 10));
                }
            }
        }
    }
    // Send response back
    return response;
}

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Basic Demo - ESP32-WiFiManager");
    bool res;
    // wm.resetSettings();
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

    // Set up TCP server to react on FCs 0x03 and 0x04
    MBserver.registerWorker(1, READ_HOLD_REGISTER, &FC0304);
    MBserver.registerWorker(1, READ_INPUT_REGISTER, &FC0304);
    // Start server
    MBserver.start(port, 2, 2000);
}

void loop() {
    hearbeat++;
    if (hearbeat > 15) {
        hearbeat = 0;
    }
    delay(100);
}