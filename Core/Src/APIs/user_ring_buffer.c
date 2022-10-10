/*
 * user_ring_buffer.c
 *
 *  Created on: Sep 28, 2022
 *      Author: User31
 */


#include "user_ring_buffer.h"

/**
 * @brief  ring buffer data struct
 * @note   The definition of our ring buffer structure is hidden from the user
 * @struct ring_buffer_t
 *
 */
struct ring_buffer_t
{
    uint16_t *buffer;
    size_t head;
    size_t tail;
    size_t length;
    uint8_t full;
};

/**
 * @defgroup ring_Buffer_Private_Functions
 * @{
 */

/**
 * @brief Advance head pointer by 1 position
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 */
static void head_ptr_advance(ring_buffer_handle_t rb)
{
    assert(rb);

    if (rb->full)
    {
        rb->tail = (rb->tail + 1) % rb->length;
    }

    // We mark full because we will advance tail on the next time around
    rb->head = (rb->head + 1) % rb->length;
    rb->full = (rb->head == rb->tail);
}

/**
 * @brief Retreat tail pointer by 1 position
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 */
static void tail_ptr_retreat(ring_buffer_handle_t rb)
{
    assert(rb);

    rb->full = 0;
    rb->tail = (rb->tail + 1) % rb->length;
}

/**@} */

/**
 * @defgroup ring_Buffer_Public_Functions
 * @{
 */

/**
 * @brief Check if ring buffer is empty or not
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @return uint8_t return 1 if ring buffer is empty, return 0 otherwise.
 */
uint8_t is_ring_buffer_empty(ring_buffer_handle_t rb)
{
    assert(rb);

    return (!rb->full && (rb->tail == rb->head));
}

/**
 * @brief Check if ring buffer is full or not
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @return uint8_t returns 1 if ring buffer is full, return 0 otherwise.
 */
uint8_t is_ring_buffer_full(ring_buffer_handle_t rb)
{
    assert(rb);

    return rb->full;
}

/**
 * @brief Initialize ring buffer and get the handler associated.
 *
 * @param buffer  pointer to a buffer reserved in memory by the user that is going to be register in ring buffer
 * @param size    size of the buffer to be register.
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the initialized ring buffer.
 */
ring_buffer_handle_t ring_buffer_init(uint16_t *buffer, size_t size)
{
    assert(buffer && size);

    ring_buffer_handle_t rb = malloc(sizeof(ring_buffer_t));
    assert(rb);

    rb->buffer = buffer;
    rb->length = size;
    ring_buffer_reset(rb);

    assert(is_ring_buffer_empty(rb));

    return rb;
}

/**
 * @brief Deallocate an specified ring buffer handler
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 */
void ring_buffer_free(ring_buffer_handle_t rb)
{
    assert(rb);
    free(rb);
}

/**
 * @brief Reset ring buffer to default configuration
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 */
void ring_buffer_reset(ring_buffer_handle_t rb)
{
    assert(rb);
    rb->head = 0;
    rb->tail = 0;
    rb->full = 0;
}

/**
 * @brief Return the data available in ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @return size_t return number of bytes in buffer.
 */
size_t ring_buffer_get_data_len(ring_buffer_handle_t rb)
{
    assert(rb);

    size_t size = rb->length;

    if (!rb->full)
    {
        if (rb->head >= rb->tail)
        {
            size = (rb->head - rb->tail);
        }
        else
        {
            size = (rb->length + rb->head - rb->tail);
        }
    }

    return size;
}

/**
 * @brief Return the capacity of the ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @return size_t return length of the buffer registered in ring buffer
 */
size_t ring_buffer_get_capacity(ring_buffer_handle_t rb)
{
    assert(rb);
    return rb->length;
}

/**
 * @brief Return the free space available in ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @return size_t return the number of bytes available in ring buffer
 */
size_t ring_buffer_get_free_space(ring_buffer_handle_t rb)
{
    assert(rb);
    return (rb->length - ring_buffer_get_data_len(rb));
}

/**
 * @brief Put byte in ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @param data byte to be written in buffer.
 */
void ring_buffer_put(ring_buffer_handle_t rb, uint16_t data)
{
    assert(rb && rb->buffer);

    rb->buffer[rb->head] = data;

    head_ptr_advance(rb);
}

/**
 * @brief Get byte from ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @param data   pointer to a variable to be fill whit the data in buffer.
 * @return uint8_t  return 0 if there is not data available to be read, return 1 otherwise.
 */
uint8_t ring_buffer_get(ring_buffer_handle_t rb, uint16_t *data)
{
    assert(rb && data && rb->buffer);

    int r = 0;

    if (!is_ring_buffer_empty(rb))
    {
        *data = rb->buffer[rb->tail];
        tail_ptr_retreat(rb);

        r = 1;
    }

    return r;
}

/**
 * @brief Write data in ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @param data   pointer to a buffer that contains the data to be written in buffer
 * @param data_len number of bytes of data to be written in buffer
 * @return ring_buffer_st_t  return status of buffer.
 */
ring_buffer_st_t ring_buffer_write(ring_buffer_handle_t rb, uint16_t *data, uint8_t data_len)
{
    assert(rb && rb->buffer);

    if (rb->full)
    {
        return RING_BUFF_FULL;
    }

    if (ring_buffer_get_free_space(rb) < data_len)
    {
        return RING_BUFF_NOT_ENOUGH_SPACE;
    }
    else
    {
        size_t data_counter = 0;

        while (data_counter < data_len)
        {
            ring_buffer_put(rb, data[data_counter++]);
        }

        return RING_BUFF_OK;
    }
}

/**
 * @brief Read data from ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @param data pointer to a buffer to be filled.
 * @param data_len  number of bytes to be read in ring buffer.
 * @return uint8_t  return 1 if number of bytes requested to be read is correct, return 0 otherwise.
 */
uint8_t ring_buffer_read(ring_buffer_handle_t rb, uint16_t *data, size_t data_len)
{
    assert(rb && rb->buffer && data);

    size_t data_counter = 0;

    while (data_counter < data_len)
    {
        if (!ring_buffer_get(rb, &data[data_counter++]))
        {
            return RING_BUFF_ERROR;
        }
    }

    return RING_BUFF_OK;
}

/**
 * @brief Fetch data in ring buffer
 *
 * @param rb variable of type ring_buffer_t* which contains the struct associated to the ring buffer
 * @param data   buffer to be filled with the fetch data in ring buffer.
 * @param data_len number of bytes to be fetch.
 * @return uint8_t  return 1 if number of bytes requested to be fetch is correct, return 0 otherwise.
 */
uint8_t ring_buffer_fetch(ring_buffer_handle_t rb, uint16_t *data, size_t data_len)
{
    assert(rb && rb->buffer && data);

    size_t data_counter = 0;
    size_t tail_idx = rb->tail;

    assert(rb && rb->buffer);

    if (data_len > ring_buffer_get_data_len(rb))
    {
        return RING_BUFF_ERROR;
    }
    else
    {
        while (data_counter < data_len)
        {
            data[data_counter++] = rb->buffer[tail_idx++];
            tail_idx = (tail_idx) % rb->length;
        }
    }

    return RING_BUFF_OK;
}
