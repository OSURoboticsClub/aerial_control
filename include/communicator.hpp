#ifndef COMMUNICATOR_HPP_
#define COMMUNICATOR_HPP_

const uint16_t MAX_PACKET_SIZE = 255;
const uint16_t MAGIC = 0xF0F0;

const uint16_t HEARTBEAT_MESSAGE_ID = 1;
const uint16_t LOG_MESSAGE_ID = 2;

struct message_header_t {
  uint16_t id;
};

struct heartbeat_message_t {
  uint64_t tick;
};

struct log_message_t {
  char data[64];
};

uint16_t message_length(uint16_t id) {
  switch(id) {
    case HEARTBEAT_MESSAGE_ID:
      return sizeof(struct heartbeat_message_t);
    case LOG_MESSAGE_ID:
      return sizeof(struct log_message_t);
  }

  return 0;
}

class Communicator {
  uint16_t read(uint8_t *buffer, struct message_header_t *header, void *message) {
    // Scan for the first magic in the buffer
    for(int16_t i = 0; i < MAX_PACKET_SIZE; i++) {
    }

    return 0;
  }
};

#endif
