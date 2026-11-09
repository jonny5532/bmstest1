#include "supervisor_protocol.h"

#include <string.h>

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

static void put_u32(uint8_t *p, uint32_t v) {
    put_u16(p, v & 0xFFFF);
    put_u16(p + 2, v >> 16);
}

static void put_u64(uint8_t *p, uint64_t v) {
    put_u32(p, (uint32_t)v);
    put_u32(p + 4, (uint32_t)(v >> 32));
}

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p) {
    return get_u16(p) | ((uint32_t)get_u16(p + 2) << 16);
}

static uint64_t get_u64(const uint8_t *p) {
    return get_u32(p) | ((uint64_t)get_u32(p + 4) << 32);
}

size_t sup_pack_bms_status(const sup_bms_status_t *status, uint8_t *buf, size_t buf_size) {
    if(buf_size < SUP_BMS_STATUS_LEN) {
        return 0;
    }
    buf[0] = SUP_MSG_BMS_STATUS;
    buf[1] = SUP_PROTOCOL_VERSION;
    put_u64(&buf[2], status->uptime_ms);
    buf[10] = status->flags;
    put_u16(&buf[11], (uint16_t)status->cell_min_mV);
    put_u16(&buf[13], (uint16_t)status->cell_max_mV);
    buf[15] = status->map.chemistry;
    buf[16] = status->map.num_cells;
    buf[17] = status->map.num_modules;
    // Bits past the last channel aren't sent: cell_map_check() rejects them
    // on the BMS side, and they arrive as zero here
    for(int i=0; i<CELL_MAP_MASK_BYTES; i++) {
        buf[18 + i] = (uint8_t)(status->map.presence[i / 4] >> ((i % 4) * 8));
    }
    buf[18 + CELL_MAP_MASK_BYTES] = status->hmi_address;
    return SUP_BMS_STATUS_LEN;
}

bool sup_unpack_bms_status(const uint8_t *payload, size_t len, sup_bms_status_t *status) {
    // A different protocol version may have a different length, so only the
    // type and version are needed to report a mismatch
    if(len < 2 || payload[0] != SUP_MSG_BMS_STATUS) {
        return false;
    }
    memset(status, 0, sizeof(*status));
    status->version = payload[1];
    if(status->version != SUP_PROTOCOL_VERSION) {
        return true;
    }
    if(len != SUP_BMS_STATUS_LEN) {
        return false;
    }
    status->uptime_ms = get_u64(&payload[2]);
    status->flags = payload[10];
    status->cell_min_mV = (int16_t)get_u16(&payload[11]);
    status->cell_max_mV = (int16_t)get_u16(&payload[13]);
    status->map.chemistry = payload[15];
    status->map.num_cells = payload[16];
    status->map.num_modules = payload[17];
    for(int i=0; i<CELL_MAP_MASK_BYTES; i++) {
        status->map.presence[i / 4] |= (uint32_t)payload[18 + i] << ((i % 4) * 8);
    }
    status->hmi_address = payload[18 + CELL_MAP_MASK_BYTES];
    return true;
}

size_t sup_pack_reply(const sup_reply_t *reply, uint8_t *buf, size_t buf_size) {
    if(buf_size < SUP_REPLY_LEN) {
        return 0;
    }
    buf[0] = SUP_MSG_SUPERVISOR_REPLY;
    buf[1] = SUP_PROTOCOL_VERSION;
    buf[2] = reply->state;
    put_u16(&buf[3], reply->reasons);
    put_u16(&buf[5], reply->latch_reasons);
    put_u32(&buf[7], reply->map_hash);
    put_u16(&buf[11], (uint16_t)reply->cell_min_mV);
    put_u16(&buf[13], (uint16_t)reply->cell_max_mV);
    buf[15] = reply->cells_ok;
    put_u32(&buf[16], reply->scans);
    buf[20] = reply->chemistry;
    return SUP_REPLY_LEN;
}

bool sup_unpack_reply(const uint8_t *payload, size_t len, sup_reply_t *reply) {
    if(len < 2 || payload[0] != SUP_MSG_SUPERVISOR_REPLY) {
        return false;
    }
    memset(reply, 0, sizeof(*reply));
    reply->version = payload[1];
    if(reply->version != SUP_PROTOCOL_VERSION) {
        return true;
    }
    if(len != SUP_REPLY_LEN) {
        return false;
    }
    reply->state = payload[2];
    reply->reasons = get_u16(&payload[3]);
    reply->latch_reasons = get_u16(&payload[5]);
    reply->map_hash = get_u32(&payload[7]);
    reply->cell_min_mV = (int16_t)get_u16(&payload[11]);
    reply->cell_max_mV = (int16_t)get_u16(&payload[13]);
    reply->cells_ok = payload[15];
    reply->scans = get_u32(&payload[16]);
    reply->chemistry = payload[20];
    return true;
}

void sup_frame_parser_init(sup_frame_parser_t *parser, size_t max_payload_len) {
    memset(parser, 0, sizeof(*parser));
    parser->max_payload_len = max_payload_len < DUART_FRAME_MAX_PAYLOAD_LEN ? max_payload_len : DUART_FRAME_MAX_PAYLOAD_LEN;
}

static void drop(sup_frame_parser_t *parser, size_t n) {
    memmove(parser->buf, &parser->buf[n], parser->len - n);
    parser->len -= n;
}

void sup_frame_feed(sup_frame_parser_t *parser, uint8_t byte) {
    if(parser->len == sizeof(parser->buf)) {
        // Only if the caller never takes the frames (sup_frame_next() keeps
        // it under one frame otherwise)
        drop(parser, 1);
    }
    parser->buf[parser->len++] = byte;
}

size_t sup_frame_next(sup_frame_parser_t *parser, const uint8_t **payload) {
    duart_frame_parse_t r = duart_frame_parse(parser->buf, sizeof(parser->buf), 0, parser->len,
        parser->payload, parser->max_payload_len);
    drop(parser, r.consumed);
    parser->crc_errors += r.crc_errors;
    *payload = parser->payload;
    return r.payload_len;
}
