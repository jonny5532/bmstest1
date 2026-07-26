#include "logging.h"

#include "tusb.h"
#include "hardware/sync.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// Define the number of readers
#define RB_READERS 2

#define RB_STDOUT_READER 0
#define RB_HMI_READER 1

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    volatile uint32_t head;        // Current write index (0 to size-1)
    volatile uint32_t write_count; // Monotonic counter of total bytes written
    
    // Reader state stored in the struct for simplicity, 
    // though often kept external in other designs.
    struct {
        uint32_t tail;             // Current read index (0 to size-1)
        uint32_t read_count;       // Total bytes read by this reader
    } readers[RB_READERS];
    
} ringbuffer_t;

ringbuffer_t log_ringbuffer = {0};
uint8_t log_buffer[16384]; // 16KB buffer for logs

/**
 * @brief Initialize the ring buffer
 */
static void rb_init(ringbuffer_t *rb, uint8_t *buffer, uint32_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->write_count = 0;
    
    for (int i = 0; i < RB_READERS; i++) {
        rb->readers[i].tail = 0;
        rb->readers[i].read_count = 0;
    }
}

/**
 * @brief Write data to the buffer, overwriting old data if full.
 * Safe to call from ISR.
 */
static void rb_write(ringbuffer_t *rb, const uint8_t *data, uint32_t len) {
    if (len == 0) return;

    // Writers exist in both main-loop and ISR context (e.g. error_printf from
    // the DUART DMA IRQ). The head/write_count update below is a non-atomic
    // read-modify-write, so exclude concurrent writers for the duration.
    uint32_t irq_state = save_and_disable_interrupts();

    // 1. Copy data handling the wrap-around
    uint32_t head = rb->head;
    uint32_t size = rb->size;
    
    // Calculate space until the end of the linear buffer
    uint32_t chunk1 = size - head;
    
    if (len <= chunk1) {
        // Fits in one go
        memcpy(&rb->buffer[head], data, len);
        rb->head = (head + len) == size ? 0 : (head + len);
    } else {
        // Needs split
        memcpy(&rb->buffer[head], data, chunk1);
        uint32_t chunk2 = len - chunk1;
        // Handle case where write is larger than entire buffer (unlikely but possible)
        // We only copy the *end* of the data to the start of buffer if len > size
        uint32_t offset_in_data = chunk1;
        
        // If the write is huge, we wrap multiple times. 
        // We only care about the final resting place and the last 'size' bytes.
        // For simplicity here, we assume len is generally reasonable. 
        // But strictly:
        uint32_t remaining = chunk2;
        uint32_t write_idx = 0;
        
        while (remaining > 0) {
            uint32_t write_len = (remaining > size) ? size : remaining;
            memcpy(&rb->buffer[write_idx], &data[offset_in_data], write_len);
            write_idx = (write_idx + write_len) % size;
            remaining -= write_len;
            offset_in_data += write_len;
        }
        rb->head = write_idx;
    }

    // 2. Memory Barrier
    // Prevent the compiler from updating write_count before memcpy completes.
    // This ensures if a reader interrupts us, they don't see new count + old data.
    __asm volatile ("" ::: "memory");

    // 3. Update monotonic counter
    rb->write_count += len;

    restore_interrupts(irq_state);
}

/**
 * @brief Read data for a specific reader index (0 or 1).
 * Safe to call from Main Loop while ISR writes.
 * * @return Number of bytes actually read.
 */
static size_t rb_read(ringbuffer_t *rb, int reader_id, uint8_t *dest, uint32_t len) {
    if (reader_id >= RB_READERS) return 0;
    if (len == 0) return 0;

    // Snapshot volatile variables to ensure consistency during calculation
    uint32_t current_write_count = rb->write_count;
    uint32_t current_head = rb->head;
    
    // Check for overflow/overwrite
    // We calculate available data using the monotonic counters
    uint32_t unread_bytes = current_write_count - rb->readers[reader_id].read_count;
    
    // If unread_bytes > size, the writer has lapped us.
    // We must jump ahead to the oldest valid data.
    if (unread_bytes > rb->size) {
        // In an overwrite buffer, the oldest valid data starts exactly at 'head'.
        rb->readers[reader_id].tail = current_head;
        rb->readers[reader_id].read_count = current_write_count - rb->size;
        unread_bytes = rb->size;
    }

    if (unread_bytes == 0) {
        return 0;
    }

    // Clamp read length to available data
    uint32_t actual_read = (len > unread_bytes) ? unread_bytes : len;

    // Perform Copy
    uint32_t tail = rb->readers[reader_id].tail;
    uint32_t size = rb->size;
    uint32_t chunk1 = size - tail;

    if (actual_read <= chunk1) {
        memcpy(dest, &rb->buffer[tail], actual_read);
        rb->readers[reader_id].tail = (tail + actual_read) == size ? 0 : (tail + actual_read);
    } else {
        memcpy(dest, &rb->buffer[tail], chunk1);
        memcpy(dest + chunk1, &rb->buffer[0], actual_read - chunk1);
        rb->readers[reader_id].tail = actual_read - chunk1;
    }

    rb->readers[reader_id].read_count += actual_read;

    return actual_read;
}

size_t logging_read(uint8_t *dest, uint32_t len) {
    if (log_ringbuffer.size == 0) return 0;
    return rb_read(&log_ringbuffer, RB_HMI_READER, dest, len);
}

void logging_printf(log_level_t level, const char *format, ...) {
    if (level < LOG_LEVEL_DEBUG || level > LOG_LEVEL_ERROR) {
        return;
    }

    if(log_ringbuffer.size == 0) {
        // Lazy init on first log call
        rb_init(&log_ringbuffer, log_buffer, sizeof(log_buffer));
    }

    char buffer[256];

    // Format and print the actual message
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if(ret < 0) {
        return;
    }

    rb_write(&log_ringbuffer, (const uint8_t *)buffer, ret);
}

void logging_flush_to_stdout() {
    if(log_ringbuffer.size == 0) {
        // ringbuffer not initialized yet
        return;
    }
    if(!stdio_usb_connected()) {
        // USB not connected, can't flush
        return;
    }

    // Process any pending USB events (non-blocking)
    tud_task_ext(0, true);

    // See whether there's any room in the output buffer
    uint32_t available_bytes = tud_cdc_write_available();
    bool wrote = false;

    // Read from ring buffer and write to USB CDC
    uint8_t temp_buffer[1024];
    while (available_bytes > 0) {
        size_t to_read = (available_bytes > sizeof(temp_buffer)) ? sizeof(temp_buffer) : available_bytes;
        size_t read = rb_read(&log_ringbuffer, RB_STDOUT_READER, temp_buffer, to_read);
        if (read > 0) {
            tud_cdc_write(temp_buffer, read);
            wrote = true;
            available_bytes -= read;
        } else {
            break; // No more data to read
        }
    }

    if(wrote) {
        tud_cdc_write_flush();
    }
}
