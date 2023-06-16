/*
 * circular_buffer.h
 *
 * Created: 6/14/2023 4:10:47 PM
 *  Author: KASO
 */ 


#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
	// Pointer to raw buffer
	uint8_t* p_buffer;
	// Buffer size in bytes
	size_t buffer_size;
	// Iterator that points to first element of buffer
	uint8_t* it_begin;
	// Iterator that points to one after the last element of a buffer
	uint8_t* it_end;
	// Number of bytes currently written in buffer
	size_t bytes_in_buffer;
} circular_buffer_t;

typedef enum {
	CBUF_ERROR_NO_ERROR = 0,
	CBUF_ERROR_INVALID_HANDLE = 20,
	CBUF_ERROR_INVALID_BUFFER_POINTER = 21,
	CBUF_ERROR_BUFFER_EMPTY = 23,
	CBUF_ERROR_BUFFER_FULL = 24,
	CBUF_ERROR_DESTINATION_BUFFER_INVALID = 25
} circular_buffer_error_e;

// Initializes circular array to buffer `p_buffer` with maximal size of `buffer_size` bytes.
// Returns circular_buffer_error_e
uint8_t CBuf_Init(circular_buffer_t* h_circ_buffer, uint8_t* p_buffer, size_t buffer_size);

// Write `value` byte to the end of the buffer.
// Returns circular_buffer_error_e
// If data is written it returns CBUF_ERROR_NO_ERROR else CBUF_ERROR_BUFFER_FULL or another error.
uint8_t CBuf_Write(circular_buffer_t* h_circ_buffer, uint8_t value);

// Read `value` byte from the front of the buffer.
// Returns circular_buffer_error_e
// If data is read it returns CBUF_ERROR_NO_ERROR else CBUF_ERROR_BUFFER_EMPTY or another error.
uint8_t CBuf_Read(circular_buffer_t* h_circ_buffer, uint8_t* p_dest);

// Returns CBUF_ERROR_NO_ERROR if byte can be read from the buffer,
// else returns CBUF_ERROR_BUFFER_EMPTY
uint8_t CBuf_AvailableForRead(circular_buffer_t circ_buffer);

// Returns CBUF_ERROR_NO_ERROR if byte can be written to the buffer,
// else returns CBUF_ERROR_BUFFER_FULL
uint8_t CBuf_AvailableForWrite(circular_buffer_t circ_buffer);

#endif /* CIRCULAR_BUFFER_H_ */