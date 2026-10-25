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
    put_u32(&buf[10], status->map_hash);
    buf[14] = status->flags;
    put_u16(&buf[15], (uint16_t)status->cell_min_mV);
    put_u16(&buf[17], (uint16_t)status->cell_max_mV);
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
    status->map_hash = get_u32(&payload[10]);
    status->flags = payload[14];
    status->cell_min_mV = (int16_t)get_u16(&payload[15]);
    status->cell_max_mV = (int16_t)get_u16(&payload[17]);
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
    return true;
}

uint16_t sup_crc16(const uint8_t *data, size_t len, uint16_t crc) {
    for(size_t i=0; i<len; i++) {
        crc ^= data[i];
        for(int j=0; j<8; j++) {
            if(crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

size_t sup_frame_encode(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_size) {
    if(payload_len == 0 || payload_len > 256 || out_size < payload_len + 4) {
        return 0;
    }
    out[0] = 0xFF;
    out[1] = (uint8_t)(payload_len - 1);
    memcpy(&out[2], payload, payload_len);
    uint16_t crc = sup_crc16(out, payload_len + 2, 0xFFFF);
    out[2 + payload_len] = crc & 0xFF;
    out[3 + payload_len] = crc >> 8;
    return payload_len + 4;
}

void sup_frame_parser_init(sup_frame_parser_t *parser, size_t max_payload_len) {
    memset(parser, 0, sizeof(*parser));
    parser->max_payload_len = max_payload_len;
}

size_t sup_frame_feed(sup_frame_parser_t *parser, uint8_t byte, const uint8_t **payload) {
    if(parser->pos == 0 && byte != 0xFF) {
        // Hunting for the sync byte
        return 0;
    }

    parser->buf[parser->pos++] = byte;
    if(parser->pos < 2) {
        return 0;
    }

    size_t payload_len = (size_t)parser->buf[1] + 1;
    if(parser->pos == 2 && payload_len > parser->max_payload_len) {
        // Longer than any message on this link, so that wasn't a sync byte.
        // Start again, from this byte if it could be one.
        parser->pos = 0;
        if(byte == 0xFF) {
            parser->buf[parser->pos++] = byte;
        }
        return 0;
    }
    if(parser->pos < payload_len + 4) {
        return 0;
    }

    // Full frame received
    parser->pos = 0;
    uint16_t crc = sup_crc16(parser->buf, payload_len + 2, 0xFFFF);
    uint16_t msg_crc = (uint16_t)(parser->buf[2 + payload_len] | (parser->buf[3 + payload_len] << 8));
    if(crc != msg_crc) {
        parser->crc_errors++;
        return 0;
    }
    *payload = &parser->buf[2];
    return payload_len;
}
