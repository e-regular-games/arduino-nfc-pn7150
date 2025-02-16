/**
 * Library for using NFC Card Emulation PN7150 devices with
 * Uno/Nano and other ATMEGA328P/PB microcontrollers.
 * The library operates with as small a memory foot print as possible.
 * 
 * This library does not include NDEF parsing.
 * Heavily adapted from https://github.com/ElectronicCats/ElectronicCats-PN7150
 * 
 * Author: S. Ryan Edgar
 * Date: February 15th, 2025
 */

#ifndef NFC_EMULATE_CARD_H
#define NFC_EMULATE_CARD_H

#include <Arduino.h>

// Status Codes 
#define SUCCESS 0x00
#define NCI_CORE_INIT_ERR 0x30
#define NCI_CORE_RESET_ERR_1 0x10
#define NCI_CORE_RESET_ERR_2 0x20
#define NCI_WRITE_ERR_RANGE 0x08
#define NCI_EMU_DISCOVERY_ERR 0x11
#define NCI_EMU_ROUTING_ERR 0x12
#define NCI_EMU_NFCA_SEL_ERR 0x13
#define NCI_EMU_START_DISCOVERY_ERR 0X14
#define NCI_NO_DATA 0xF0;
#define NCI_UNEXPECTED_DATA 0xF1
#define NCI_FORCE_CLOSE_COMMS 0xF2

namespace nfc {

// the offset of the data, the data, and the length of the data.
// returns anything.
typedef word TagWrite(const word, const byte* const, const word);

// the callback is given the offset and length of data to be populated.
// the callback will populate the data array
// may be called with offset=0, length=0, and should return the total length of ndef file.
// returns the total length of the ndef file.
typedef word TagRead(const word, const word, byte*);

// Initialize the PN7150 using NCI RESET and INIT
// must be called before starting emulation.
// The provided IRQ pin must support interrupts.
// @returns a status code from above (0 on success)
byte initialize(const byte ven_pin, const byte irq_pin);

// Begin discovery and tag emulation.
// The read callback will be called when a device attempts to read the tag.
// The write callback will be called when a device attempts to write the tag.
// @returns a status code from above (0 on success)
byte emulate_start(TagRead* read, TagWrite* write);

// @returns a status code from above (0 on success)
byte emulate_stop();

// Shutdown the NFC device.
void shutdown();

} // namespace nfc

#endif
