#include <stdint.h>

typedef struct {
    // uint32_t uart0_bytes_received;
    // uint32_t uart0_bytes_sent;
    uint32_t uart0_packets_received;
    uint32_t uart0_packets_sent;
    uint32_t uart0_crc_errors;

    // uint32_t uart1_bytes_received;
    // uint32_t uart1_bytes_sent;
    uint32_t uart1_packets_received;
    uint32_t uart1_packets_sent;
    uint32_t uart1_crc_errors;

    uint32_t can_frames_sent;
    uint32_t can_frames_received;
} debug_counters_t;

extern debug_counters_t debug_counters;
