#ifndef ARDUPI_H
#define ARDUPI_H

#define INPUT  0
#define OUTPUT 1

#define LOW  0
#define HIGH 1

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int  digitalRead(int pin);
void delay(unsigned long ms);
unsigned long millis();

/* Minimal Serial replacement */
class SerialPi {
public:
    void begin(unsigned baud);
    int  available();
    int  read();
    void print(const char* s);
    void println(const char* s);
};

extern SerialPi Serial;

#endif

