

#ifndef SERVO_CHAIN_H
#define SERVO_CHAIN_H

#include <HardwareSerial.h>
#include <Stream.h>
#include <inttypes.h>

#include "control_table_xl320.h"

class XL320Chain {
 public:
  XL320Chain(uint8_t dirPin, HardwareSerial *stream);
  class Packet {
    bool freeData;

   public:
    unsigned char *data;
    size_t data_size;

    // wrap a received data stream in an Packet object for analysis
    Packet(unsigned char *data, size_t size);
    // build a packet into the pre-allocated data array
    // if data is null it will be malloc'ed and free'd on destruction.

    Packet(unsigned char *data, size_t data_size, unsigned char id,
           unsigned char instruction, int parameter_data_size,
           uint8_t *parameter_data);
    ~Packet();
    unsigned char getId();
    int getLength();
    int getSize();
    int getParameterCount();
    unsigned char getInstruction();
    unsigned char getParameter(int n);
    bool isValid();

    void toStream(Stream &stream);
  };
  int sendPacket(uint8_t id, int instruction, uint8_t *params, int length);
  int readPacket(unsigned char *buffer, size_t size);
  int verifyIDs(uint8_t *id_list, int length);
  int verifyIDs(uint8_t *id_list, int length, Stream *debugStream);
  int getServoData(uint8_t *id_list, unsigned short *data_buffer, int length);

  int getServoData(uint8_t *id_list, unsigned short *data_buffer, int length,
                   Stream *debugStream);
  void setServoPositions(uint8_t *id_list, unsigned short *data_buffer,
                         int length);

  void torqueON(uint8_t *id_list, int length);
  void torqueOFF(uint8_t *id_list, int length);

  void setPIDGains(uint8_t id, uint8_t d_gain, uint8_t i_gain, uint8_t p_gain);

  void syncWrite(uint8_t *id_list, uint8_t *data_buffer, int length,
                 int numParams, uint8_t addr);

  void servoWrite(uint8_t id, uint8_t *data_buffer, int numParams,
                  uint8_t addr);

 private:
  volatile uint8_t dirPin;
  Stream *stream;
};

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
                          unsigned short data_blk_size);

#endif  // SERVO_CHAIN_H