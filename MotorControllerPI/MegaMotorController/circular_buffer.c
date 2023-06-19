/*
 * circular_buffer.c
 *
 * Created: 6/14/2023 4:22:40 PM
 *  Author: KASO
 */ 

#include "circular_buffer.h"

#ifndef NULL
#define NULL (void*)0x00
#endif

// Initializes circular array to buffer `p_buffer` with maximal size of `buffer_size` bytes.
// Returns circular_buffer_error_e
uint8_t CBuf_Init(circular_buffer_t* h_circ_buffer, uint8_t* p_buffer, size_t buffer_size)
{
	if (h_circ_buffer == NULL)
	{
		return CBUF_ERROR_INVALID_HANDLE;
	}
	
	if (p_buffer == NULL)
	{
		return CBUF_ERROR_INVALID_BUFFER_POINTER;
	}
	
	h_circ_buffer->p_buffer = p_buffer;
	h_circ_buffer->buffer_size = buffer_size;
	h_circ_buffer->it_begin = p_buffer;
	h_circ_buffer->it_end = p_buffer;
	h_circ_buffer->bytes_in_buffer = 0;
	
	return CBUF_ERROR_NO_ERROR;
}

uint8_t CBuf_HasAnyIterOverflown_(circular_buffer_t circ_buffer)
{
	const uint8_t* it_buffer_end = &circ_buffer.p_buffer[circ_buffer.buffer_size];
	return (circ_buffer.it_begin >= it_buffer_end) || (circ_buffer.it_end >= it_buffer_end);
}

// Write `value` byte to the end of the buffer.
uint8_t CBuf_Write(circular_buffer_t* h_circ_buffer, uint8_t value)
{
	if (h_circ_buffer == NULL)
	{
		return CBUF_ERROR_INVALID_HANDLE;
	}
	
	if (!CBuf_AvailableForWrite(*h_circ_buffer))
	{
		return CBUF_ERROR_BUFFER_FULL;
	}
	
	*(h_circ_buffer->it_end++) = value;
	h_circ_buffer->bytes_in_buffer++;
	
	if (CBuf_HasAnyIterOverflown_(*h_circ_buffer))
	{
		h_circ_buffer->it_end = h_circ_buffer->p_buffer;
	}
	
	return CBUF_ERROR_NO_ERROR;
}

// Return `value` byte from the front of the buffer.
uint8_t CBuf_Read(circular_buffer_t* h_circ_buffer, uint8_t* p_dest)
{
	if (h_circ_buffer == NULL)
	{
		return CBUF_ERROR_INVALID_HANDLE;
	}
	
	if (!CBuf_AvailableForRead(*h_circ_buffer))
	{
		return CBUF_ERROR_BUFFER_EMPTY;
	}
	
	if (p_dest == NULL)
	{
		return CBUF_ERROR_DESTINATION_BUFFER_INVALID;
	}
	
	h_circ_buffer->bytes_in_buffer--;
	*p_dest = *(h_circ_buffer->it_begin++);
	
	if (CBuf_HasAnyIterOverflown_(*h_circ_buffer))
	{
		h_circ_buffer->it_begin = h_circ_buffer->p_buffer;
	}
	
	return CBUF_ERROR_NO_ERROR;
}

// Returns CBUF_ERROR_NO_ERROR if byte can be read from the buffer,
// else returns CBUF_ERROR_BUFFER_EMPTY
uint8_t CBuf_AvailableForRead(circular_buffer_t circ_buffer)
{
	return circ_buffer.bytes_in_buffer > 0;
}

// Returns CBUF_ERROR_NO_ERROR if byte can be written to the buffer,
// else returns CBUF_ERROR_BUFFER_FULL
uint8_t CBuf_AvailableForWrite(circular_buffer_t circ_buffer)
{
	return circ_buffer.bytes_in_buffer < circ_buffer.buffer_size;
}