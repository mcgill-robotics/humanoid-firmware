#include "ServoChain.h"

// from http://stackoverflow.com/a/133363/195061

#define LOBYTE(w) ((unsigned char)(((unsigned long)(w)) & 0xff))
#define HIBYTE(w) ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

XL320Chain::XL320Chain(uint8_t dirPin, HardwareSerial *stream)
{
  this->dirPin = dirPin;
  this->stream = (Stream *)stream;
}

void XL320Chain::begin()
{
  ((HardwareSerial *)(this->stream))->begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  ((HardwareSerial *)(this->stream))->transmitterEnable(this->dirPin);
  ((HardwareSerial *)(this->stream))->setTimeout(100);
}

void XL320Chain::beginFullDuplex()
{
  ((HardwareSerial *)(this->stream))->begin(1000000);
  ((HardwareSerial *)(this->stream))->transmitterEnable(this->dirPin);
  ((HardwareSerial *)(this->stream))->setTimeout(2);
}

#define FSM
#define STATE(x)                                                    \
  s_##x : if (!stream->readBytes(&BUFFER[I++], 1)) goto sx_timeout; \
  if (I >= SIZE)                                                    \
    goto sx_overflow;                                               \
  sn_##x:
#define THISBYTE (BUFFER[I - 1])
#define NEXTSTATE(x) goto s_##x
#define NEXTSTATE_NR(x) goto sn_##x
#define OVERFLOW \
  sx_overflow:
#define TIMEOUT \
  sx_timeout:

int XL320Chain::readPacket(unsigned char *BUFFER, size_t SIZE)
{
  int C;
  int I = 0;

  int length = 0;

  // state names normally name the last parsed symbol

  FSM
  {
    STATE(start)
    {
      // SerialUSB.println("Start");
      if (THISBYTE == 0xFF)
        NEXTSTATE(header_ff_1);
      I = 0;
      NEXTSTATE(start);
    }
    STATE(header_ff_1)
    {
      // SerialUSB.println("header_ff_1");
      if (THISBYTE == 0xFF)
        NEXTSTATE(header_ff_2);
      I = 0;
      NEXTSTATE(start);
    }
    STATE(header_ff_2)
    {
      // SerialUSB.println("header_ff_2");
      if (THISBYTE == 0xFD)
        NEXTSTATE(header_fd);
      // yet more 0xFF's? stay in this state
      if (THISBYTE == 0xFF)
        NEXTSTATE(header_ff_2);
      // anything else? restart
      I = 0;
      NEXTSTATE(start);
    }
    STATE(header_fd)
    {
      // SerialUSB.println("header_fd");
      // reading reserved, could be anything in theory, normally 0
    }
    STATE(header_reserved)
    {
      // SerialUSB.println("header_reserved");
      // id = THISBYTE
    }
    STATE(id)
    {
      // SerialUSB.println("id");
      length = THISBYTE;
    }
    STATE(length_1)
    {
      // SerialUSB.println("length_1");
      length += THISBYTE << 8; // eg: length=4
    }
    STATE(length_2)
    {
      // SerialUSB.println("length_2");
    }
    STATE(instr)
    {
      // instr = THISBYTE
      // check length because
      // action and reboot commands have no parameters
      // SerialUSB.println("instr");
      if (I - length >= 5)
        NEXTSTATE(checksum_1);
    }
    STATE(params)
    {
      // check length and maybe skip to checksum
      // SerialUSB.println("params");
      if (I - length >= 5)
        NEXTSTATE(checksum_1);
      // or keep reading params
      NEXTSTATE(params);
    }
    STATE(checksum_1)
    {
      // SerialUSB.println("checksum1");
    }
    STATE(checksum_2)
    {
      // done
      // SerialUSB.println("checksum2");
      return I;
    }
    OVERFLOW { return -1; }
    TIMEOUT { return -2; }
  }
}

int XL320Chain::broadcastPing(Stream *debugStream, byte *IDbuf)
{

  int bufsize = 16;

  byte txbuffer[bufsize];
  uint8_t rxbuffer[255];
  int numValidIDs = 0;

  Packet p(txbuffer, bufsize, 0xFE, 0x01, 0, nullptr);

  int size = p.getSize();
  stream->write(txbuffer, size);
  int return_val = this->readPacket(rxbuffer, 255);
  while (return_val > 0)
  {
    Packet p(rxbuffer, 255);
    if (debugStream)
      p.toStream(*debugStream);
    if (p.isValid())
    {
      IDbuf[numValidIDs++] = p.getId();
    }
    return_val = this->readPacket(rxbuffer, 255);
  }
  return numValidIDs;
}

bool XL320Chain::validateIDs(uint8_t *id_list, int length, Stream *debugStream)
{

  byte id_buf[255];
  int num_ids_found = broadcastPing(nullptr, id_buf);
  if (num_ids_found)
  {
    if (length != num_ids_found)
      return false; // lengths don't match
    for (int i = 0; i < length; i++)
    {
      uint8_t cur_id = id_list[i];
      bool found = false;
      for (int j = 0; j < num_ids_found; j++)
      {
        if (id_buf[j] == cur_id)
          found = true;
      }
      if (!found)
        return false; // did not find this ID
    }
    return true;
  }
  else
  {
    return false; // found no ids, or overflow
  }
}

int XL320Chain::sendPacket(uint8_t id, int instruction, uint8_t *params,
                           int length)
{
  uint8_t txbuffer[10 + length];

  if (length)
  {
    Packet p(txbuffer, 10 + length, id, instruction, length, params);
    this->stream->write(txbuffer, p.getSize());
    return p.getSize();
  }
  else
  {
    Packet p(txbuffer, 10, id, instruction, 0, nullptr);
    this->stream->write(txbuffer, p.getSize());
    return p.getSize();
  }
}

void XL320Chain::setJointSpeed(int id, int value)
{
  uint8_t param_buffer[2];
  param_buffer[0] = LOBYTE(value);
  param_buffer[1] = HIBYTE(value);
  servoWrite(id, param_buffer, 2, GOAL_POSITION_ADR);
  this->stream->flush();
}

// int XL320Chain::verifyIDs(uint8_t *id_list, int length) {
//   return verifyIDs(id_list, length, nullptr);
// }

// int XL320Chain::verifyIDs(uint8_t *id_list, int length, Stream *debugStream) {
//   unsigned char buffer[255];
//   for (int i = 0; i < length; i++) {
//     sendPacket(id_list[i], 0x01, nullptr, 0);
//     this->stream->flush();
//     if (readPacket(buffer, 255) > 0) {
//       Packet p(buffer, 255);
//       if (debugStream) p.toStream(*debugStream);
//       if (p.isValid() == false || p.getParameterCount() != 3) {
//         return -1;  // invalid response
//       }
//     } else {
//       return -2;  // error reading packet
//     }
//   }
//   return 0;  // all IDs valid
// }

void XL320Chain::setPIDGains(uint8_t id, uint8_t d_gain, uint8_t i_gain,
                             uint8_t p_gain)
{
  uint8_t param_buffer[3];
  param_buffer[0] = d_gain;
  param_buffer[1] = i_gain;
  param_buffer[2] = p_gain;
  servoWrite(id, param_buffer, 3, D_GAIN_ADR);
}

void XL320Chain::torqueOFF(uint8_t *id_list, int length)
{
  uint8_t param_buffer[length];
  for (int i = 0; i < length; i++)
  {
    param_buffer[i] = 0;
  }
  syncWrite(id_list, param_buffer, length, 1, TORQUE_ENABLE_ADR);
}

void XL320Chain::torqueON(uint8_t *id_list, int length)
{
  uint8_t param_buffer[length];
  for (int i = 0; i < length; i++)
  {
    param_buffer[i] = 1;
  }
  syncWrite(id_list, param_buffer, length, 1, TORQUE_ENABLE_ADR);
}

void XL320Chain::setServoPositions(uint8_t *id_list,
                                   unsigned short *data_buffer, int length)
{
  uint8_t short_buffer[length * 2];
  for (int i = 0; i < length; i++)
  {
    short_buffer[i * 2] = LOBYTE(data_buffer[i]);
    short_buffer[i * 2 + 1] = HIBYTE(data_buffer[i]);
  }
  syncWrite(id_list, short_buffer, length, 2, GOAL_POSITION_ADR);
}

void XL320Chain::syncWrite(uint8_t *id_list, uint8_t *data_buffer, int length,
                           int numParams, uint8_t addr)
{
  uint8_t param_buffer[length * (numParams + 1) + 4];
  param_buffer[0] = LOBYTE(addr);
  param_buffer[1] = HIBYTE(addr);
  param_buffer[2] = LOBYTE(numParams);
  param_buffer[3] = HIBYTE(numParams);
  for (int i = 0; i < length; i++)
  {
    param_buffer[4 + i * (numParams + 1) + 0] = id_list[i];
    param_buffer[4 + i * (numParams + 1) + 1] = data_buffer[i * numParams];
    if (numParams > 1)
    {
      for (int j = 1; j < numParams + 1; j++)
      {
        param_buffer[4 + i * (numParams + 1) + j + 1] =
            data_buffer[i * numParams + j];
      }
    }
  }
  sendPacket(0xFE, 0x83, param_buffer, length * (numParams + 1) + 4);
  this->stream->flush();
}

void XL320Chain::servoWrite(uint8_t id, uint8_t *data_buffer, int numParams,
                            uint8_t addr)
{
  uint8_t param_buffer[2 + numParams];
  param_buffer[0] = LOBYTE(addr);
  param_buffer[1] = HIBYTE(addr);
  memcpy(param_buffer + 2, data_buffer, numParams);
  sendPacket(id, 0x03, param_buffer, numParams + 2);
}

int XL320Chain::getServoData(uint8_t *id_list, unsigned short *data_buffer,
                             int length)
{
  return getServoData(id_list, data_buffer, length, nullptr);
}

int XL320Chain::getServoData(uint8_t *id_list, unsigned short *data_buffer,
                             int length, Stream *debugStream)
{
  while (this->stream->available())
  {
    this->stream->read();
  }
  unsigned char buffer[255];
  uint8_t send_params[4 + length];
  send_params[0] = LOBYTE(PRESENT_POSITION_ADR);
  send_params[1] = HIBYTE(PRESENT_POSITION_ADR);
  send_params[2] =
      LOBYTE(PRESENT_POSITION_SIZE + PRESENT_SPEED_SIZE + PRESENT_LOAD_SIZE);
  send_params[3] =
      HIBYTE(PRESENT_POSITION_SIZE + PRESENT_SPEED_SIZE + PRESENT_LOAD_SIZE);
  for (int i = 0; i < length; i++)
  {
    send_params[i + 4] = id_list[i];
  }
  sendPacket(0xFE, 0x82, send_params, length + 4); // sync read
  this->stream->flush();

  for (int i = 0; i < length; i++)
  {
    if (readPacket(buffer, 255) > 0)
    {
      Packet p(buffer, 255);
      if (debugStream)
        p.toStream(*debugStream);
      if (p.isValid() == false)
      {
        return -1; // invalid response
      }
      data_buffer[i * 3 + 0] = ((unsigned short)p.getParameter(2) << 8) | (unsigned char)p.getParameter(1);
      data_buffer[i * 3 + 1] = ((unsigned short)p.getParameter(4) << 8) | (unsigned char)p.getParameter(3);
      data_buffer[i * 3 + 2] = ((unsigned short)p.getParameter(6) << 8) | (unsigned char)p.getParameter(5);
    }
    else
    {
      return -2; // error reading packet
    }
  }
  return 0; // all Servos Read
}

XL320Chain::Packet::Packet(unsigned char *data, size_t data_size,
                           unsigned char id, unsigned char instruction,
                           int parameter_data_size, uint8_t *parameter_data)
{
  // [ff][ff][fd][00][id][len1][len2] {
  // [instr][params(parameter_data_size)][crc1][crc2] }
  unsigned int length = 3 + parameter_data_size;
  if (!data)
  {
    // [ff][ff][fd][00][id][len1][len2] { [data(length)] }
    this->data_size = 7 + length;
    this->data = (unsigned char *)malloc(data_size);
    this->freeData = true;
  }
  else
  {
    this->data = data;
    this->data_size = data_size;
    this->freeData = false;
  }
  this->data[0] = 0xFF;
  this->data[1] = 0xFF;
  this->data[2] = 0xFD;
  this->data[3] = 0x00;
  this->data[4] = id;
  this->data[5] = length & 0xff;
  this->data[6] = (length >> 8) & 0xff;
  this->data[7] = instruction;
  for (int i = 0; i < parameter_data_size; i++)
  {
    this->data[8 + i] = parameter_data[i];
  }
  unsigned short crc = update_crc(0, this->data, this->getSize() - 2);
  this->data[8 + parameter_data_size] = crc & 0xff;
  this->data[9 + parameter_data_size] = (crc >> 8) & 0xff;
}

XL320Chain::Packet::Packet(unsigned char *data, size_t size)
{
  this->data = data;
  this->data_size = size;
  this->freeData = false;
}

XL320Chain::Packet::~Packet()
{
  if (this->freeData == true)
  {
    free(this->data);
  }
}

void XL320Chain::Packet::toStream(Stream &stream)
{
  stream.print("id: ");
  stream.println(this->getId(), DEC);
  stream.print("length: ");
  stream.println(this->getLength(), DEC);
  stream.print("instruction: ");
  stream.println(this->getInstruction(), HEX);
  stream.print("parameter count: ");
  stream.println(this->getParameterCount(), DEC);
  for (int i = 0; i < this->getParameterCount(); i++)
  {
    stream.print(this->getParameter(i), HEX);
    if (i < this->getParameterCount() - 1)
    {
      stream.print(",");
    }
  }
  stream.println();
  stream.print("valid: ");
  stream.println(this->isValid() ? "yes" : "no");
}

unsigned char XL320Chain::Packet::getId() { return data[4]; }

int XL320Chain::Packet::getLength()
{
  return data[5] + ((data[6] & 0xff) << 8);
}

int XL320Chain::Packet::getSize() { return getLength() + 7; }

int XL320Chain::Packet::getParameterCount() { return getLength() - 3; }

unsigned char XL320Chain::Packet::getInstruction() { return data[7]; }

unsigned char XL320Chain::Packet::getParameter(int n) { return data[8 + n]; }

bool XL320Chain::Packet::isValid()
{
  int length = getLength();
  unsigned short storedChecksum = data[length + 5] + (data[length + 6] << 8);
  return storedChecksum == update_crc(0, data, length + 5);
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
                          unsigned short data_blk_size)
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
      0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
      0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
      0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
      0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
      0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
      0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
      0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
      0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
      0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
      0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
      0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
      0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
      0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
      0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
      0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
      0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
      0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
      0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
      0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
      0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
      0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
      0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
      0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
      0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
      0x0208, 0x820D, 0x8207, 0x0202};

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}