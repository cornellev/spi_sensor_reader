#include "arduPi.h"
#include <pigpiod_if2.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstddef>
#include <cstdio>


static int pigpio_handle = -1;
static std::chrono::steady_clock::time_point start_time;

// Ensure pigpio is initialized once
static void ensure_pigpio()
{
    if (pigpio_handle < 0) {
        pigpio_handle = pigpio_start(nullptr, nullptr);
        if (pigpio_handle < 0) {
            std::perror("pigpio_start failed");
        }
        start_time = std::chrono::steady_clock::now();
    }
}

/* Arduino-style API */

void pinMode(int pin, int mode)
{
    ensure_pigpio();
    if (mode == OUTPUT)
        set_mode(pigpio_handle, pin, PI_OUTPUT);
    else
        set_mode(pigpio_handle, pin, PI_INPUT);
}

void digitalWrite(int pin, int value)
{
    ensure_pigpio();
    gpio_write(pigpio_handle, pin, value ? 1 : 0);
}

int digitalRead(int pin)
{
    ensure_pigpio();
    return gpio_read(pigpio_handle, pin);
}

void delay(unsigned long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

unsigned long millis()
{
    ensure_pigpio();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

static int uart_handle = -1;

void SerialPi::begin(unsigned baud)
{
    ensure_pigpio();
    uart_handle = serial_open(pigpio_handle, const_cast<char*>("/dev/serial0"), baud, 0);
    if (uart_handle < 0) {
        std::perror("serial_open failed");
    }
}

int SerialPi::available()
{
    if (uart_handle < 0) return 0;
    return serial_data_available(pigpio_handle, uart_handle);
}

int SerialPi::read()
{
    if (uart_handle < 0) return -1;
    return serial_read_byte(pigpio_handle, uart_handle);
}

void SerialPi::print(const char* s)
{
    if (uart_handle < 0) return;
    serial_write(pigpio_handle, uart_handle, const_cast<char*>(s), strlen(s));
}

void SerialPi::println(const char* s)
{
    if (uart_handle < 0) return;
    serial_write(pigpio_handle, uart_handle, const_cast<char*>(s), strlen(s));
    serial_write(pigpio_handle, uart_handle, const_cast<char*>("\r\n"), 2);
}

/* Global instance */
SerialPi Serial;
