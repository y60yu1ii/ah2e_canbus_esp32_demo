#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <stdlib.h>

#include "ModbusServerWiFi.h"
#include "freertos/ringbuf.h"
#include "freertos/timers.h"

#define LED_PIN 2
#define OBJ_LEN 15
#define BUF_COUNT 200
#define OBS_COUNT 5

char mdns[32] = "ah2e";
char password[64] = "123456789";
WiFiManager wm;

uint16_t port = 502;
ModbusServerWiFi MBserver;

CAN_device_t CAN_cfg;          // CAN Config
const int interval = 500;      // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;  // Receive Queue size

CAN_frame_t rx_frame;

int heartbeat = 0;
void taskOne(void *parameter);
void taskTwo(void *parameter);
void taskThree(void *parameter);

RingbufHandle_t bufHandle;
uint8_t canbusData[OBJ_LEN * BUF_COUNT];

u_int16_t getHead = 0;
u_int16_t head = 0;  // head for canbus data buffer
// Server function to handle FC 0x03 and 0x04
ModbusMessage
FC0304(ModbusMessage request) {
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
            for (uint8_t i = 0; i < words; ++i) {
                response.add((uint16_t)canbusData[getHead * 15 + i]);
            }
            getHead++;
            if (getHead >= head) {
                getHead = 0;
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
            uint16_t time = (esp_timer_get_time() / 1000) & 0xFFFF;
            uint8_t txItem[] = {
                (uint8_t)(rx_frame.FIR.B.DLC | (rx_frame.FIR.B.FF ? 0x20 : 0x00)),  // DLC & 0x20 for extended
                (uint8_t)((rx_frame.MsgID >> 24) & 0xFF),                           // ADDR 0
                (uint8_t)((rx_frame.MsgID >> 16) & 0xFF),                           // ADDR 1
                (uint8_t)((rx_frame.MsgID >> 8) & 0xFF),                            // ADDR 2
                (uint8_t)(rx_frame.MsgID & 0xFF),                                   // ADDR 3
                rx_frame.data.u8[0],
                rx_frame.data.u8[1],
                rx_frame.data.u8[2],
                rx_frame.data.u8[3],
                rx_frame.data.u8[4],
                rx_frame.data.u8[5],
                rx_frame.data.u8[6],
                rx_frame.data.u8[7],
                (uint8_t)((time >> 8) & 0xFF),
                (uint8_t)(time & 0xFF),
            };

            UBaseType_t res = xRingbufferSend(bufHandle, txItem, sizeof(txItem), pdMS_TO_TICKS(1000));
            if (res != pdTRUE) {
                printf("Send Item Failed\r\n");
            }
        }
    }
}

void taskTwo(void *parameter) {
    bufHandle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
    if (bufHandle == NULL) {
        printf("Create RingBuffer Failed");
    }
    // printf("\r\n===0x%p\r\n", bufHandle);
    while (1) {
        size_t itemSize;
        // printf("\r\n=%p\r\n", bufHandle);
        char *item = (char *)xRingbufferReceive(bufHandle, &itemSize, pdMS_TO_TICKS(1000));
        int check = 0;
        int pos = -1;
        if (item != NULL) {
            for (int i = 0; i < head + 1; i++) {
                pos = i;
                for (int j = 0; j < 5; j++) {  // 0, 1, 2, 3, 4,
                    // printf("%d %d\n", canbusData[itemSize * i + j], item[j]);
                    if (canbusData[itemSize * i + j] != item[j]) {
                        break;
                    }
                    check = j;
                }
                if (check == 4 && pos != -1) {  // address matches
                    // printf(" check is %d pos is %d\n", check, pos);
                    for (int x = 0; x < itemSize; x++) {
                        canbusData[pos * itemSize + x] = item[x];
                    }
                    break;
                }
            }
            if (check != 4) {
                // printf(" pos not found %d %d %d\n", pos, head, check);
                for (int i = 0; i < itemSize; i++) {
                    canbusData[head * itemSize + i] = item[i];
                }
                if (head < BUF_COUNT) {
                    head++;
                }
            }
            // for (int i = 0; i < itemSize; i++) {
            //     printf("%02X ", item[i]);
            // }
            // printf("\n");
            // for (int i = 0; i < itemSize * OBS_COUNT; i++) {
            //     if ((i % 15) == 0 && i > 1) {
            //         printf("\n");
            //     }
            //     printf("%02X ", canbusData[i]);
            // }
            // printf("\n---- \n");
            vRingbufferReturnItem(bufHandle, (void *)item);
        } else {
            printf("Receive none\n");
        }
    }
    vTaskDelete(NULL);
}
void taskThree(void *parameter) {
    while (1) {
        heartbeat++;
        if (heartbeat > 15) {
            heartbeat = 0;
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        vTaskDelay(500);
    }
}
