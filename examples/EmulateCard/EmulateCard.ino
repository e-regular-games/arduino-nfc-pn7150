
#include "Arduino.h"
#include "nfc-emulate-card.h"

#define NDEF_HEADER_LEN 10

// includes the short NDEF header and 1 record.
const byte test_tag[] = {
  0xC1, 1, 0, 0, 0, 15, 'T', 2, 'e', 'n',
  'H', 'e', 'l', 'l', 'o', ' ',
  'W', 'o', 'r', 'l', 'd', '!'
};

word tag_read(const word offset, const word length, byte* data) {
  Serial.print(F("tag_read: "));
  Serial.print(offset);
  Serial.print(" ");
  Serial.print(length);
  Serial.println();

  if (length > 0) {
    memcpy(data, test_tag + offset, length);
  }
  return sizeof(test_tag);
}

void tag_write(const word offset, const byte* const data, const word len) {
  Serial.print(F("tag_write: "));
  Serial.print(offset);
  Serial.print(" ");
  Serial.print(len);
  Serial.println();

  if (offset == 0 && len > 7) {
    if (data[0] & 0x10 && data[3] == 'T') { // short msg
      for (int i = 0; i < 7; ++i) {
        Serial.print(data[i], 16);
        Serial.print(" ");
      }
      Serial.println();
      Serial.write(data+7, len - 7);
      Serial.println("");
    } else if (data[6] == 'T') {
      for (int i = 0; i < NDEF_HEADER_LEN; ++i) {
        Serial.print(data[i], 16);
        Serial.print(" ");
      }
      Serial.println();
      Serial.write(data + NDEF_HEADER_LEN, len - NDEF_HEADER_LEN);
      Serial.println("");
    }
  }
}

void setup() {
  delay(1000);

  Serial.begin(115200);

  Serial.println(F("---------------------"));
  Serial.println(F("nfc::EmulateCard demo"));
  Serial.println(F("  by S. Ryan Edgar"));
  Serial.println(F("  Feb 15th, 2025"));
  Serial.println(F("---------------------"));

  byte status = nfc::initialize(16, 2);
  if (status) {
    Serial.print(F("nfc failed to initialize: "));
    Serial.print(status, 16);
    Serial.println();
    while (true);
  } else {
    Serial.println(F("nfc initialized"));
  }

  status = nfc::emulate_start(&tag_read, &tag_write);
  if (status) {
    Serial.print(F("nfc failed to emulate: "));
    Serial.print(status, 16);
    Serial.println();
    while (true);
  } else {
    Serial.println(F("nfc emulation started"));
  }
}

byte timeout = 60; // wait for 10 minutes, then shutdown.
void loop() {
  delay(10000);
  timeout -= 1;
  if (timeout == 0) {
    nfc::emulate_stop();
    nfc::shutdown();
    Serial.println(F("nfc shutdown"));
    while (true);
  }
}
