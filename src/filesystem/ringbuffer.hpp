#ifndef RINGBUFFER_HPP_
#define RINGBUFFER_HPP_

#include <stdlib.h>
#include <stdint.h>
#include <array>

typedef struct {
	uint32_t size;   /* I.e., capacity */
	uint32_t count;   /* Number of slots filled */
	uint32_t head;   /* Head */
	uint8_t *elems;   /* Elements */
} rb_t;

/*
 * Initializer
 *
 * @param buf Ringbuffer
 * @param size Size
 * @param elems Buffer to use to store elements
 */
void rb_init(rb_t *buf, std::size_t size, uint8_t *elems);

/*
 * Adder
 *
 * If there is not enough room in buffer to accomodate all data, no data will
 * be written.
 *
 * @param buf Ringbuffer
 * @param num_bytes Number of bytes to add
 * @param input Data buffer to add
 *
 * @return Number of bytes actually added
 */
std::size_t rb_add(rb_t *buf, std::size_t num_bytes, uint8_t *input);

/*
 * Remover
 *
 * @param buf Ringbuffer
 * @param num_bytes Number of bytes to remove
 *
 * @return Number of bytes actually removed
 */
std::size_t rb_remove(rb_t *buf, std::size_t num_bytes, uint8_t *output);

/*
 * Peeker
 *
 * @param buf Ringbuffer
 * @param num_bytes Number of bytes to fetch
 *
 * @return Number of bytes actually fetched
 */
std::size_t rb_peek(rb_t *buf, std::size_t num_bytes, uint8_t *output);

#endif
