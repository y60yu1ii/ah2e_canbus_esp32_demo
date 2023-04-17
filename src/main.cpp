#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <stdlib.h>

#include "ModbusServerWiFi.h"

#define LED_PIN 2
char mdns[32] = "ah2e";
char password[64] = "123456789";
WiFiManager wm;

uint16_t port = 502;
ModbusServerWiFi MBserver;

CAN_device_t CAN_cfg;          // CAN Config
const int interval = 500;      // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;  // Receive Queue size

uint8_t table[128];
int idTable[] = {0x1, 0x2, 0x3, 0x4};

CAN_frame_t rx_frame;

int heartbeat = 0;
void taskOne(void *parameter);
void taskTwo(void *parameter);
void taskThree(void *parameter);

int cmp(const void *lhs, const void *rhs) {
    if (*(const int *)lhs < *(const int *)rhs)
        return -1;
    else if (*(const int *)rhs < *(const int *)lhs)
        return 1;
    else
        return 0;
}

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
                    response.add((uint16_t)(heartbeat));
                } else {
                    response.add((uint16_t)(addr + i));
                }
                // response.add((uint16_t)(table[i]));
            }
        }
        if (request.getFunctionCode() == READ_INPUT_REGISTER) {
            // No, this is for FC 0x04. Response is random
            for (uint8_t i = 0; i < words; ++i) {
                // send increasing data values
                // response.add((uint16_t)random(1, 65535));
                if (i == 0) {
                    response.add((uint16_t)(heartbeat + 1));
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

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);
    bool res;
    // wm.resetSettings();
    res = wm.autoConnect("AH2E_WIFI", password);  // password protected ap

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

    xTaskCreate(
        taskOne, "TaskOne", 10000, NULL, 1, NULL);
    xTaskCreate(
        taskTwo, "TaskTwo", 10000, NULL, 1, NULL);
    xTaskCreate(
        taskThree, "TaskThree", 10000, NULL, 1, NULL);
}

void loop() {
}

void taskOne(void *parameter) {
    // CAN_cfg.speed = CAN_SPEED_125KBPS;
    // my arduino & ESP32 have bug so all bitrate should be doubled
    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_4;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();

    while (1) {
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            if (rx_frame.FIR.B.FF == CAN_frame_std) {
                printf("New standard frame");
            } else {
                printf("New extended frame");
            }

            if (rx_frame.FIR.B.RTR == CAN_RTR) {
                printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
            } else {
                printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
                for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
                    printf("0x%02X ", rx_frame.data.u8[i]);
                }
                printf("\n");
            }
            // // TODO
            // if (rx_frame.FIR.B.RTR == CAN_no_RTR) {
            //     int *p = (int *)bsearch(&rx_frame.MsgID, idTable, 3, sizeof(int), cmp);
            //     printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
            //     printf("\n");
            // }
        }
    }
}
void taskTwo(void *parameter) {
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        vTaskDelay(500);
    }
}
void taskThree(void *parameter) {
    while (1) {
        heartbeat++;
        if (heartbeat > 15) {
            heartbeat = 0;
        }
        // CAN_frame_t tx_frame;
        // tx_frame.FIR.B.FF = CAN_frame_ext;
        // tx_frame.MsgID = 0x2;
        // tx_frame.FIR.B.DLC = 8;
        // tx_frame.data.u8[0] = heartbeat;
        // tx_frame.data.u8[1] = 0x01;
        // tx_frame.data.u8[2] = 0x02;
        // tx_frame.data.u8[3] = 0x03;
        // tx_frame.data.u8[4] = 0x04;
        // tx_frame.data.u8[5] = 0x05;
        // tx_frame.data.u8[6] = 0x06;
        // tx_frame.data.u8[7] = 0x07;
        // ESP32Can.CANWriteFrame(&tx_frame);

        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        vTaskDelay(500);
    }
}
