
#include "nfc-emulate-card.h"

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>

// Enable detailed output of communication
// between the MCU and the PN7150
#define NFC_DEBUG

#define NFC_I2C_ADDR (0x28)
#define NFC_HEADER_LEN 3

namespace nfc {

struct __attribute__ ((packed)) NciCmdRsp {
  byte len;
  byte data[15];
};

const byte NCI_CORE_INIT[] PROGMEM = {3, 0x20, 0x01, 0x00};
const byte NCI_CORE_RESET[] PROGMEM = {4, 0x20, 0x00, 0x01, 0x01};
const byte NCI_DISCOVERY_MAP[] PROGMEM = {7, 0x21, 0x00, 0x04, 0x01, 0x04, 0x02, 0x02};
const byte NCI_ROUTING[] PROGMEM = {10, 0x21, 0x01, 0x07, 0x00, 0x01, 0x01, 0x03, 0x00, 0x01, 0x04};
const byte NCI_NFCA_SEL[] PROGMEM = {7, 0x20, 0x02, 0x04, 0x01, 0x32, 0x01, 0x20};
const byte NCI_START_DISCOVERY[] PROGMEM = {6, 0x21, 0x03, 0x03, 0x01, 0x80, 0x01}; // set mode to poll and listen.
const byte NCI_STOP_DISCOVERY[] PROGMEM = {4, 0x21, 0x06, 0x01, 0x00};

const byte NCI_SEND_DATA_RSP[] PROGMEM = {6, 0x60, 0x06, 0x03, 0x01, 0x00, 0x01};
const byte NFC_READER_RSP[] PROGMEM = {7, 0x61, 0x05, 0x0C, 0x01, 0x02, 0x04, 0x80};


enum NfcState {
  NfcDisconnected,
  NfcConnected,
  NfcDiscovery,
  NfcRemoteConnected,
};

const byte T4T_NDEF_EMU_APP_SELECT[] PROGMEM = {13, 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00};
const byte T4T_NDEF_EMU_CC[] PROGMEM = {15, 0x00, 0x0F, 0x20, 0x00, 0xFF, 0x00, 0xFF, 0x04, 0x06, 0xE1, 0x04, 0x40, 0x00, 0x00, 0x00};
const byte T4T_NDEF_EMU_CC_SELECT[] PROGMEM = {7, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
const byte T4T_NDEF_EMU_NDEF_SELECT[] PROGMEM = {7, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
const byte T4T_NDEF_EMU_READ[] PROGMEM = {2, 0x00, 0xB0};
const byte T4T_NDEF_EMU_WRITE[] PROGMEM = {2, 0x00, 0xD6};
const byte T4T_NDEF_EMU_OK[] = {0x90, 0x00};
const byte T4T_NDEF_EMU_NOK[] = {0x6A, 0x82};

enum NdefState {
  NdefReady,
  NdefApplicationSelected,
  NdefCCSelected,
  NdefSelected,
  NdefDESFireProd
};

NfcState nfc_state = NfcDisconnected;
NdefState ndef_state = NdefReady;
TagRead* tag_read = nullptr;
TagWrite* tag_write = nullptr;
byte nfc_ven_pin = 0;
byte nfc_irq_pin = 0;

enum NfcBufferMode {
  NfcBufferAvailable,
  NfcBufferCommandSend,
  NfcBufferMessageRecv,
};

byte buffer[266];
NfcBufferMode buffer_mode = NfcBufferAvailable;
word buffer_len = sizeof(buffer);
byte ctrl_generation = 0;
byte ctrl_fw_version[3] = {0, 0, 0};

void clear_buffer();
bool get_message(const byte timeout = 5);
byte write_data(const byte data[], const word len);
byte write_command_progmem(const void* cmd);
byte discovery_start();
byte discovery_stop();
bool check_message_progmem(const void * rsp);
void handle_t4t(const byte* const msg, const word msg_len, byte* cmd, word& cmd_len);
byte wakeup();

byte write_command_progmem(const void* cmd) {
  NciCmdRsp c;
  memcpy_P(&c, cmd, sizeof(NciCmdRsp));
  return write_data(c.data, c.len);
}

bool check_message_progmem(const byte* msg, const void* expected) {
  NciCmdRsp c;
  memcpy_P(&c, expected, sizeof(NciCmdRsp));
  return buffer_len >= c.len && memcmp(msg, c.data, c.len) == 0;
}

void handle_t4t(const byte* const msg, const word msg_len, byte* cmd, word& cmd_len) {
  bool status = false;
  volatile word dummy = 0;
  if (check_message_progmem(msg, T4T_NDEF_EMU_WRITE)) {
    if (NdefSelected == ndef_state && tag_write) {
      unsigned short offset = (msg[2] << 8) + msg[3];
      unsigned char length = msg[4];
      // the first 2 bytes are the length of the ndef file
      if (offset >= 2) {
        dummy = (*tag_write)(offset - 2, msg + 5, length);
      } else if (offset == 0 && length > 2) {
        dummy = (*tag_write)(offset, msg + 7, length-2);
      }
      cmd_len = 0;
      status = true;
    }
  } else if (check_message_progmem(msg, T4T_NDEF_EMU_APP_SELECT)) {
    cmd_len = 0;
    status = true;
    ndef_state = NdefApplicationSelected;
  } else if (check_message_progmem(msg, T4T_NDEF_EMU_CC_SELECT)) {
    if (NdefApplicationSelected == ndef_state) {
      cmd_len = 0;
      status = true;
      ndef_state = NdefCCSelected;
    }
  } else if (check_message_progmem(msg, T4T_NDEF_EMU_NDEF_SELECT)) {
    cmd_len = 0;
    status = true;
    ndef_state = NdefSelected;
  } else if (check_message_progmem(msg, T4T_NDEF_EMU_READ)) {
    if (NdefCCSelected == ndef_state) {
      unsigned short offset = (msg[2] << 8) + msg[3];
      unsigned char length = msg[4];
      clear_buffer(); // erase msg and prepare to load data into command
      buffer_mode = NfcBufferCommandSend;

      NciCmdRsp cc;
      memcpy_P(&cc, T4T_NDEF_EMU_CC, sizeof(NciCmdRsp));
      if (length <= (cc.len + offset + 2)) {
        memcpy(cmd, cc.data + offset, length);
        cmd_len = length;
        status = true;
      }
    } else if (NdefSelected == ndef_state && tag_read) {
      unsigned short offset = (msg[2] << 8) + msg[3];
      unsigned char length = msg[4];
      clear_buffer(); // erase msg and prepare to load data into command
      buffer_mode = NfcBufferCommandSend;

      if (offset == 0 && length >= 2) {
        volatile word total_len = tag_read(offset, length - 2, cmd+2);
        cmd[0] = (total_len & 0xFF00) >> 8;
        cmd[1] = (total_len & 0x00FF);
        cmd_len = length;
      } else {
        dummy = tag_read(offset-2, length, cmd);
        cmd_len = length;
      }
      status = true;
    }
  }

  if (status == true) {
    memcpy(cmd + cmd_len, T4T_NDEF_EMU_OK, sizeof(T4T_NDEF_EMU_OK));
    cmd_len += sizeof(T4T_NDEF_EMU_OK);
  } else {
    memcpy(cmd, T4T_NDEF_EMU_NOK, sizeof(T4T_NDEF_EMU_NOK));
    cmd_len = sizeof(T4T_NDEF_EMU_NOK);
    ndef_state = NdefReady;
  }
}

void message_handler() {
  if (nfc_state == NfcDiscovery && check_message_progmem(buffer, NFC_READER_RSP)) {
    nfc_state = NfcRemoteConnected;

  } else if (nfc_state == NfcRemoteConnected && (buffer[0] == 0x00 || buffer[0] == 0x10) && (buffer[1] == 0x00)) {
    word next_command_len = 0;
    handle_t4t(&buffer[3], buffer[2], &buffer[3], next_command_len);
    
    if (next_command_len == 0) {
      clear_buffer();
      return;
    }

    word to_send = next_command_len;
    word sent = 0;
    byte packet_size = 0;

    // experimentally determined that the PN7150 likes data packets at most 32 bytes.
    // this code chunks the message into 32 byte segements, including the 3 bytes of NCI header
    while (to_send > 0) {
      packet_size = (to_send + 3) > 0x20 ? 0x1D : to_send;
      buffer[sent] = (next_command_len == sent + packet_size) ? 0x00 : 0x10;
      buffer[sent+1] = 0x00;
      buffer[sent+2] = packet_size;
      write_data(buffer + sent, packet_size + 3);
      to_send -= packet_size;
      sent += packet_size;
      
      // recv the okay response, assumes the response will be less
      // than the initial packet size, if sending a large amount of data.
      buffer_len = packet_size;
      get_message(); 
      if (!check_message_progmem(buffer, NCI_SEND_DATA_RSP)) {
        break;
      }
    }

    buffer_len = next_command_len;
    clear_buffer(); // erase buffer after sending the command
  }
}

void message_int() {
  detachInterrupt(digitalPinToInterrupt(nfc_irq_pin));
  interrupts();

  // this was triggered because there is a message.
  // there may be multiple messages recved while the interrupt is disconnected.
  while (digitalRead(nfc_irq_pin)) {
    get_message();
    message_handler();
  }

  attachInterrupt(digitalPinToInterrupt(nfc_irq_pin), &message_int, RISING);
}

byte initialize(const byte ven_pin, const byte irq_pin) {
  clear_buffer();
  nfc_state = NfcDisconnected;

  nfc_ven_pin = ven_pin;
  nfc_irq_pin = irq_pin;

  Wire.begin();

  pinMode(nfc_ven_pin, OUTPUT);
  pinMode(nfc_irq_pin, INPUT);

  digitalWrite(nfc_ven_pin, HIGH);
  delay(1);
  digitalWrite(nfc_ven_pin, LOW);
  delay(1);
  digitalWrite(nfc_ven_pin, HIGH);
  delay(3);
  
  byte attempts = 3;
  byte status = 0;
  for (; (status = wakeup()) && attempts > 0; attempts--) {
    delay(500);
  }
  if (attempts == 0) {
    return status;
  }
  
  write_command_progmem(NCI_CORE_INIT);
  get_message();
  if (buffer_len < 2 || buffer[0] != 0x40 || buffer[1] != 0x01) {
    return NCI_CORE_INIT_ERR;
  }

  nfc_state = NfcConnected;

  // Retrieve NXP-NCI NFC Controller generation
  if (buffer[17 + buffer[8]] == 0x08) {
    ctrl_generation = 1;
  } else if (buffer[17 + buffer[8]] == 0x10) {
    ctrl_generation = 2;
  }

  // Retrieve NXP-NCI NFC Controller FW version
  ctrl_fw_version[0] = buffer[17 + buffer[8]];  // 0xROM_CODE_V
  ctrl_fw_version[1] = buffer[18 + buffer[8]];  // 0xFW_MAJOR_NO
  ctrl_fw_version[2] = buffer[19 + buffer[8]];  // 0xFW_MINOR_NO

  Serial.print(F("ROM_CODE_V: 0x"));
  Serial.println(ctrl_fw_version[0], 16);
  Serial.print(F("FW_MAJOR_NO: 0x"));
  Serial.println(ctrl_fw_version[1], 16);
  Serial.print(F("FW_MINOR_NO: 0x"));
  Serial.println(ctrl_fw_version[2], 16);
  Serial.print(F("GEN: 0x"));
  Serial.println(ctrl_generation, 16);

  return 0;
}

void shutdown() {
  digitalWrite(nfc_ven_pin, LOW);
}

byte emulate_start(TagRead* read, TagWrite* write) {
  tag_read = read;
  tag_write = write;

  discovery_stop();

  // set the discovery map.
  write_command_progmem(NCI_DISCOVERY_MAP);
  get_message(10);
  if (buffer_len < 2 || buffer[0] != 0x41 || buffer[1] != 0x00) {
    return NCI_EMU_DISCOVERY_ERR;
  }

  // set the routing
  write_command_progmem(NCI_ROUTING);
  get_message(10);
  if (buffer_len < 2 || buffer[0] != 0x41 || buffer[1] != 0x01) {
    return NCI_EMU_ROUTING_ERR;
  }


  write_command_progmem(NCI_NFCA_SEL);
  get_message(10);
  if (buffer_len < 2 || buffer[0] != 0x40 || buffer[1] != 0x02) {
    return NCI_EMU_NFCA_SEL_ERR;
  }

  const byte s = discovery_start();
  if (s) { // non-zero indicates a failure.
    return s;
  }

  nfc_state = NfcDiscovery;
  attachInterrupt(digitalPinToInterrupt(nfc_irq_pin), &message_int, RISING);

  return SUCCESS;
}

byte emulate_stop() {
  detachInterrupt(digitalPinToInterrupt(nfc_irq_pin));
  discovery_stop();
  nfc_state = NfcConnected;
  return SUCCESS;
}

byte discovery_stop() {
  write_command_progmem(NCI_STOP_DISCOVERY);
  get_message(10);
  return SUCCESS;
}

byte discovery_start() {
  write_command_progmem(NCI_START_DISCOVERY);
  get_message(10);
  if (buffer_len < 2 || buffer[0] != 0x41 || buffer[1] != 0x03) {
    return NCI_EMU_START_DISCOVERY_ERR;
  }
  return SUCCESS;
}

void clear_buffer() {
  for (int i = 0; i < buffer_len; ++i) {
    buffer[i] = 0;
  }
  buffer_len = 0;
  buffer_mode = NfcBufferAvailable;
}

byte write_data(const byte data[], const word len) {
  Wire.beginTransmission(NFC_I2C_ADDR);

#ifdef NFC_DEBUG
  Serial.print(F("nfc::write_data: "));
  Serial.println(len);
  for (word i = 0; i < len; ++i) {
    Serial.print(data[i], 16);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.println("-----");
#endif

  if (Wire.write(data, len) == len) {
    const byte s = Wire.endTransmission();
    return s != 0 ? (NCI_WRITE_ERR_RANGE | s) : SUCCESS;
  } else {
    Wire.endTransmission();
    return NCI_WRITE_ERR_RANGE | 0x07;
  }
}

byte wakeup() {  // the device has to wake up using a core reset
  write_command_progmem(NCI_CORE_RESET);

  get_message(15);
  if (buffer_len < 2 || buffer[0] != 0x40 || buffer[1] != 0x00) {
    return NCI_CORE_RESET_ERR_1;
  }

  get_message(10);
  if (buffer_len > 0) {
    return NCI_CORE_RESET_ERR_2;
  }

  return SUCCESS;
}

bool get_message(const byte timeout) {
  clear_buffer(); // zero buffer bytes and zero buffer_len

  // safety measure in case something goes wrong with the I2C bus, this will avoid locking the program
  Wire.setTimeout(5000);

  const unsigned long t0 = millis();
  unsigned long t1 = t0;
  while (t1 - t0 < timeout && t1 >= t0) {
    t1 = millis();
    if (!digitalRead(nfc_irq_pin) || Wire.requestFrom((uint8_t)NFC_I2C_ADDR, (uint8_t)NFC_HEADER_LEN) != NFC_HEADER_LEN) {
      continue;
    }

    Wire.readBytes(buffer, NFC_HEADER_LEN);
    byte payload_len = buffer[2];
    byte recv_total = 0;
    byte recv = 0;
    while (payload_len > recv_total) {
      recv = Wire.requestFrom((uint8_t)NFC_I2C_ADDR, (uint8_t)(payload_len - recv_total));
      Wire.readBytes(buffer + NFC_HEADER_LEN + recv_total, recv);
      recv_total += recv;
    }
    buffer_len = NFC_HEADER_LEN + payload_len;

    // multi-part message from nfc
    // this assumes the total message size will still be under 280 bytes.
    if (buffer[0] & 0x10) {
      while (!digitalRead(nfc_irq_pin) || Wire.requestFrom((uint8_t)NFC_I2C_ADDR, (uint8_t)NFC_HEADER_LEN) != NFC_HEADER_LEN);
      Wire.readBytes(buffer + buffer_len, NFC_HEADER_LEN);
      payload_len = buffer[buffer_len + 2];
      recv_total = 0;
      recv = 0;
      while (payload_len > recv_total) {
        recv = Wire.requestFrom((uint8_t)NFC_I2C_ADDR, (uint8_t)(payload_len - recv_total));
        Wire.readBytes(buffer + recv_total + buffer_len, recv);
        recv_total += recv;
      }
      buffer_len += payload_len;
      buffer[0] &= 0xEF;
    }
    break;
  }
  if (buffer_len > 0) {
    buffer_mode = NfcBufferMessageRecv;
  }

#ifdef NFC_DEBUG
  Serial.print(F("nfc::get_message: "));
  Serial.println(buffer_len);
  for (int i = 0; i < buffer_len; ++i) {
    Serial.print(buffer[i], 16);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.println(F("-----"));
#endif

  return buffer_len > 0;
}

} // namespace nfc

