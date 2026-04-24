#ifndef RAWHID_H
#define RAWHID_H

#include <Arduino.h>
#include <string.h>
#include <inttypes.h>

// Raw HID definitions
#define RAWHID_BUFFER_SIZE 64
#define RAWHID_TX_ENDPOINT 3
#define RAWHID_RX_ENDPOINT 4

// LED pin definition
#define LED_PIN 6

/**
 * Initialize Raw HID interface
 */
void rawhid_init(void);

/**
 * Send data via Raw HID
 * @param buffer Pointer to data to send
 * @param length Number of bytes to send
 * @param timeout Timeout in milliseconds (0 = no timeout)
 * @return Number of bytes sent, or negative on error
 */
int rawhid_send(const void *buffer, uint32_t length, uint32_t timeout);

/**
 * Receive data via Raw HID
 * @param timeout Timeout in milliseconds (0 = no timeout)
 * @return Number of bytes received, 0 if no data, or negative on error
 */
int rawhid_recv(uint32_t timeout);

/**
 * Process incoming Raw HID data
 */
void process_rawhid_data(void);

#endif // RAWHID_H
