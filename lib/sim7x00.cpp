/**
*  @filename   :   sim7x00.cpp
*  @brief      :   Implements for sim7x00 library
*  @author     :   Kaloha from Waveshare
*
*  Copyright (C) Waveshare     April 27 2018
*  http://www.waveshare.com  http://www.waveshare.net
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documnetation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to  whom the Software is
* furished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include "sim7x00.h"
#include "arduPi.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <cstdlib>

Sim7x00::Sim7x00() {}
Sim7x00::~Sim7x00() {}

/**************************Power on Sim7x00**************************/
void Sim7x00::PowerOn(int PowerKey /*= powerkey*/) {
    uint8_t answer = 0;

    Serial.begin(115200);

    // checks if the module is started
    answer = sendATcommand("AT", "OK", 2000);
    if (answer == 0) {
        printf("Starting up...\n");

        pinMode(PowerKey, OUTPUT);
        // power on pulse
        digitalWrite(PowerKey, HIGH);
        delay(600);
        digitalWrite(PowerKey, LOW);

        // waits for an answer from the module
        while (answer == 0) { // Send AT every two seconds and wait for the answer
            answer = sendATcommand("AT", "OK", 2000);
        }
    }

    delay(5000);

    while ((sendATcommand("AT+CREG?", "+CREG: 0,1", 500) ||
            sendATcommand("AT+CREG?", "+CREG: 0,5", 500)) == 0) {
        delay(500);
    }
}

/**************************GPS positioning**************************/
bool Sim7x00::GPSPositioning() {
    uint8_t answer = 0;
    bool RecNull = true;

    char RecMessage[200];
    memset(RecMessage, 0, sizeof(RecMessage));
    int i = 0;

    char LatDD[3], LatMM[10], LogDD[4], LogMM[10], DdMmYy[7], UTCTime[7];

    float Lat = 0.0f, Log = 0.0f;

    printf("Start GPS session...\n");
    // Start GPS session, standalone mode
    sendATcommand("AT+CGPS=1,1", "OK", 1000);

    delay(2000);

    while (RecNull) {
        answer = sendATcommand("AT+CGPSINFO", "+CGPSINFO: ", 1000);

        if (answer == 1) {
            answer = 0;

            // reset buffer for this read attempt
            memset(RecMessage, 0, sizeof(RecMessage));
            i = 0;

            // Wait for any bytes to arrive (you may want a timeout here)
            while (Serial.available() == 0) {}

            do {
                if (Serial.available() > 0) {
                    if (i < (int)sizeof(RecMessage) - 1) {
                        RecMessage[i++] = Serial.read();
                        RecMessage[i] = '\0'; // keep it a valid C-string
                    } else {
                        // buffer full; stop reading further to avoid overflow
                        break;
                    }

                    // check if the desired answer (OK) is in the response of the module
                    if (strstr(RecMessage, "OK") != NULL) {
                        answer = 1;
                    }
                }
            } while (answer == 0);

            // At this point RecMessage is already null-terminated.

            if (strstr(RecMessage, ",,,,,,,,") != NULL) {
                // No fix yet; retry
                RecNull = true;
                answer = 0;
                delay(1000);
            } else {
                RecNull = false;
            }
        } else {
            printf("error %u\n", answer);
            sendATcommand("AT+CGPS=0", "OK", 1000);
            return false;
        }

        delay(1500);
    }

    strncpy(LatDD, RecMessage, 2);     LatDD[2] = '\0';
    strncpy(LatMM, RecMessage + 2, 9); LatMM[9] = '\0';
    Lat = atoi(LatDD) + (atof(LatMM) / 60.0f);

    if (RecMessage[12] == 'N')
        printf("Latitude is %f N\n", Lat);
    else if (RecMessage[12] == 'S')
        printf("Latitude is %f S\n", Lat);
    else
        return false;

    strncpy(LogDD, RecMessage + 14, 3); LogDD[3] = '\0';
    strncpy(LogMM, RecMessage + 17, 9); LogMM[9] = '\0';
    Log = atoi(LogDD) + (atof(LogMM) / 60.0f);

    if (RecMessage[27] == 'E')
        printf("Longitude is %f E\n", Log);
    else if (RecMessage[27] == 'W')
        printf("Longitude is %f W\n", Log);
    else
        return false;

    strncpy(DdMmYy, RecMessage + 29, 6); DdMmYy[6] = '\0';
    printf("Day Month Year is %s\n", DdMmYy);

    strncpy(UTCTime, RecMessage + 36, 6); UTCTime[6] = '\0';
    printf("UTC time is %s\n", UTCTime);

    sendATcommand("AT+CGPS=0", "OK", 1000);
    return true;
}

/**************************Other functions**************************/
char Sim7x00::sendATcommand(const char* ATcommand, unsigned int timeout) {
    uint8_t x = 0;
    char response[100];
    unsigned long previous;

    memset(response, 0, sizeof(response));

    delay(100);
    while (Serial.available() > 0) Serial.read(); // Clean the input buffer

    Serial.println(ATcommand); // Send the AT command
    previous = millis();

    // Wait for "OK" or "ERROR" or timeout
    while ((millis() - previous) < timeout) {
        while (Serial.available() != 0) {
            if (x < sizeof(response) - 1) {
                response[x++] = Serial.read();
                response[x] = '\0';
            } else {
                // buffer full; stop accumulating
                break;
            }
        }

        if (strstr(response, "OK") != NULL)    return 1;
        if (strstr(response, "ERROR") != NULL) return 0;
    }

    return 0;
}

char Sim7x00::sendATcommand(const char* ATcommand, const char* expected_answer, unsigned int timeout) {
    uint8_t x = 0;
    char response[100];
    unsigned long previous;

    memset(response, 0, sizeof(response));

    delay(100);
    while (Serial.available() > 0) Serial.read(); // Clean the input buffer

    Serial.println(ATcommand); // Send the AT command
    previous = millis();

    while ((millis() - previous) < timeout) {
        while (Serial.available() != 0) {
            if (x < sizeof(response) - 1) {
                response[x++] = Serial.read();
                response[x] = '\0';
            } else {
                break;
            }

            if (strstr(response, expected_answer) != NULL) {
                return 1;
            }
        }
    }

    return 0;
}

char Sim7x00::sendATcommand(const char* ATcommand, const char* expected_answer, char* response, unsigned int timeout) {
    uint8_t x = 0;
    unsigned long previous;

    memset(response, 0, 100); // original API assumes 100 bytes

    delay(100);
    while (Serial.available() > 0) Serial.read(); // Clean the input buffer

    Serial.println(ATcommand); // Send the AT command
    previous = millis();

    while ((millis() - previous) < timeout) {
        while (Serial.available() != 0) {
            if (x < 99) {
                response[x++] = Serial.read();
                response[x] = '\0';
            } else {
                break;
            }

            if (strstr(response, expected_answer) != NULL) {
                return 1;
            }
        }
    }

    return 0;
}

char Sim7x00::sendATcommand2(const char* ATcommand,
                            const char* expected_answer1,
                            const char* expected_answer2,
                            unsigned int timeout) {
    uint8_t x = 0;
    char response[100];
    unsigned long previous;

    memset(response, 0, sizeof(response));

    delay(100);
    while (Serial.available() > 0) Serial.read(); // Clean the input buffer

    Serial.println(ATcommand); // Send the AT command
    previous = millis();

    while ((millis() - previous) < timeout) {
        while (Serial.available() != 0) {
            if (x < sizeof(response) - 1) {
                response[x++] = Serial.read();
                response[x] = '\0';
            } else {
                break;
            }

            if (strstr(response, expected_answer1) != NULL) return 1;
            if (strstr(response, expected_answer2) != NULL) return 2;
        }
    }

    return 0;
}

Sim7x00 sim7600 = Sim7x00();
