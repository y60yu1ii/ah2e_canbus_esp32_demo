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

#define MBTCP_ID 1
#define LED_PIN 2
#define OBJ_LEN 17
#define BUF_COUNT 200
#define OBS_COUNT 5
#define MBPV_MAX 128

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
RingbufHandle_t writeBufHandle;
uint8_t canbusData[OBJ_LEN * BUF_COUNT];

u_int16_t getHead = 0;
u_int16_t head = 0;  // head for canbus data buffer
uint16_t mbPV[MBPV_MAX];
// Server function to handle FC 0x03 and 0x04
// https :  // github.com/eModbus/eModbus/discussions/147
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
            response.add((uint16_t)canbusData[getHead * OBJ_LEN]);
            // printf("[%d] %04X \n", getHead * OBJ_LEN, canbusData[getHead * OBJ_LEN]);
            for (uint8_t i = 0; i < words - 1; ++i) {
                response.add((uint16_t)((canbusData[getHead * OBJ_LEN + 2 * i + 1] << 8) | canbusData[getHead * OBJ_LEN + 2 * (i + 1)]));
                // printf("[%d] %04X \n", getHead * OBJ_LEN + 2 * i + 1, (canbusData[getHead * OBJ_LEN + 2 * i + 1] << 8) | canbusData[getHead * OBJ_LEN + 2 * (i + 1)]);
            }
            getHead++;
            if (getHead >= head) {
                getHead = 0;
            }
            // printf("h %d\n", getHead);
            // printf("=== \n");
        }
    }
    // Send response back
    return response;
}

// Server function to handle FC 0x10 (FC16)
ModbusMessage FC16(ModbusMessage request) {
    // Serial.println(request);
    ModbusMessage response;  // The Modbus message we are going to give back
    uint16_t addr = 0;       // Start address
    uint16_t words = 0;      // total words to write
    uint8_t bytes = 0;       // # of data bytes in request
    uint16_t val = 0;        // value to be written
    request.get(2, addr);    // read address from request
    request.get(4, words);   // read # of words from request
    request.get(6, bytes);   // read # of data bytes from request (seems redundant with # of words)
    char debugString[1000];

    // printf("FC16 received: write %d words @ %d\r\n", words, addr);

    // # of registers proper?
    if ((bytes != (words * 2))  // byte count in request must match # of words in request
        || (words > 123))       // can't support more than this in request packet
    {                           // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        printf("ERROR - ILLEGAL DATA VALUE\r\n");
        return response;
    }
    // Address overflow?
    if ((addr + words) > MBPV_MAX) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
        printf("ERROR - ILLEGAL DATA ADDRESS\r\n");
        return response;
    }

    // Do the writes
    // sprintf(debugString, "Write : ");
    for (uint8_t i = 0; i < words; ++i) {
        request.get(7 + (i * 2), val);  // data starts at byte 6 in request packet
        mbPV[addr + i] = val;
        // sprintf(debugString + strlen(debugString), "%i ", mbPV[addr + i]);
    }
    // printf("%s\r\n", debugString);

    uint16_t config = mbPV[addr + 0];

    uint8_t txItem[] = {
        (uint8_t)(config & 0x20 == 0x20),         // extended
        (uint8_t)((mbPV[addr + 1] >> 8) & 0xFF),  // ADDR 0
        (uint8_t)(mbPV[addr + 1] & 0xFF),         // ADDR 0
        (uint8_t)((mbPV[addr + 2] >> 8) & 0xFF),  // ADDR 1
        (uint8_t)(mbPV[addr + 2] & 0xFF),         // ADDR 1
        (uint8_t)(config & 0x0F),                 // DLC
        (uint8_t)(mbPV[addr + 3] >> 8),
        (uint8_t)(mbPV[addr + 3] & 0xFF),
        (uint8_t)(mbPV[addr + 4] >> 8),
        (uint8_t)(mbPV[addr + 4] & 0xFF),
        (uint8_t)(mbPV[addr + 5] >> 8),
        (uint8_t)(mbPV[addr + 5] & 0xFF),
        (uint8_t)(mbPV[addr + 6] >> 8),
        (uint8_t)(mbPV[addr + 6] & 0xFF),
    };

    UBaseType_t res = xRingbufferSend(writeBufHandle, txItem, sizeof(txItem), pdMS_TO_TICKS(1000));
    if (res != pdTRUE) {
        printf("Send Write Item Failed\r\n");
    }

    // Set up response
    response.add(request.getServerID(), request.getFunctionCode(), addr, words);
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
    MBserver.registerWorker(MBTCP_ID, READ_HOLD_REGISTER, &FC0304);
    MBserver.registerWorker(MBTCP_ID, READ_INPUT_REGISTER, &FC0304);
    MBserver.registerWorker(MBTCP_ID, WRITE_MULT_REGISTERS, &FC16);
    // Start server
    MBserver.start(port, 4, 2000);

    xTaskCreate(
        taskOne, "TaskOne", 10000, NULL, 1, NULL);
    xTaskCreate(
        taskTwo, "TaskTwo", 10000, NULL, 1, NULL);
    xTaskCreate(
        taskThree, "TaskThree", 10000, NULL, 1, NULL);
}

void loop() {
    heartbeat++;
    if (heartbeat > 15) {
        heartbeat = 0;
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(500);
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
            uint32_t time = esp_timer_get_time();
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
                (uint8_t)((time >> 24) & 0xFF),
                (uint8_t)((time >> 16) & 0xFF),
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
    bufHandle = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);
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
            // } else {
            //     printf("Receive none\n");
        }
    }
    vTaskDelete(NULL);
}

void taskThree(void *parameter) {
    writeBufHandle = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);
    if (writeBufHandle == NULL) {
        printf("Create Write RingBuffer Failed");
    }
    while (1) {
        size_t itemSize;
        // printf("\r\n=%p\r\n", writeBufHandle);
        char *item = (char *)xRingbufferReceive(writeBufHandle, &itemSize, pdMS_TO_TICKS(1000));
        if (item != NULL) {
            // for (int i = 0; i < itemSize; i++) {
            //     printf("%02X ", item[i]);
            // }
            // printf("\n");
            CAN_frame_t tx_frame;
            tx_frame.FIR.B.FF = (item[0] == 0) ? CAN_frame_ext : CAN_frame_std;
            tx_frame.MsgID = item[1] << 24 | item[2] << 16 | item[3] << 8 | item[4];
            tx_frame.FIR.B.DLC = item[5];
            for (int i = 0; i < 8; i++) {
                tx_frame.data.u8[i] = item[6 + i];
            }
            // printf("%04X \n", tx_frame.MsgID);
            ESP32Can.CANWriteFrame(&tx_frame);
            vRingbufferReturnItem(writeBufHandle, (void *)item);
            // } else {
            // printf("Receive Write Buffer None\n");
        }
    }
    vTaskDelete(NULL);
}