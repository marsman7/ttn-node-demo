/****************************************************************/ /*
/ Rename this file to private-keys.h and change the keys
/* *****************************************************************/

#include <Arduino.h>
#include <lmic.h>

#ifndef PRIVATE_KEYS_H
#define PRIVATE_KEYS_H


// Keys for TTN-ABP-Node
// LoRaWAN NwkSKey, network and application session key; This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const PROGMEM u1_t APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr); The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x00000000 ; // <-- Change this address for every node!

#endif