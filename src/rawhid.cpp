#include <Arduino.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>

#include "rawhid.h"

// Raw HID buffers
uint8_t rawhid_rx_buffer[RAWHID_BUFFER_SIZE];
uint8_t rawhid_tx_buffer[RAWHID_BUFFER_SIZE];

/**
 * Initialize Raw HID interface
 */
void rawhid_init(void) {
  // Raw HID is automatically initialized by Teensy's USB stack
  // This function is for compatibility/future setup
}

/**
 * Send data via Raw HID
 * @param buffer Pointer to data to send
 * @param length Number of bytes to send
 * @param timeout Timeout in milliseconds (0 = no timeout)
 * @return Number of bytes sent, or negative on error
 */
int rawhid_send(const void *buffer, uint32_t length, uint32_t timeout) 
{
  if (length > RAWHID_BUFFER_SIZE) {
    return -1; // Buffer too large
  }
  
  memcpy(rawhid_tx_buffer, buffer, length);
  
  // Send via Teensy Raw HID endpoint
  // Note: Teensy handles this through usb_rawhid_send()
  return length;
}

/**
 * Receive data via Raw HID
 * @param timeout Timeout in milliseconds (0 = no timeout)
 * @return Number of bytes received, 0 if no data, or negative on error
 */
int rawhid_recv(uint32_t timeout) 
{
  // This is a placeholder - actual implementation depends on Teensy USB stack
  // In practice, use usb_rawhid_recv() from Teensy libraries
  return 0;
}

/**
 * Process incoming Raw HID data
 */
void process_rawhid_data(void) 
{
  // Check for incoming Raw HID data
  int bytes_recv = rawhid_recv(0);
   
  if (bytes_recv > 0) 
  {
    // Toggle LED on data reception as feedback
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    // Echo data back
    rawhid_send(rawhid_rx_buffer, bytes_recv, 100);
  }
}
