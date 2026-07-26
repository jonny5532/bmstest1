#include "i2c.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/gpio.h"
#include <stdatomic.h>

typedef enum {
    STATE_IDLE,
    STATE_READING,        // Reading data into buffer
    STATE_WRITING,        // Writing data from buffer
    STATE_ERROR
} i2c_state_e;

typedef struct {
    uint8_t *buffer;
    size_t len;
    volatile i2c_state_e state;
    i2c_async_callback_t callback;
    void *user_data;
} i2c_context_t;

static i2c_context_t contexts[2];

static void __not_in_flash_func(i2c_finish)(i2c_inst_t *i2c, i2c_state_e state) {
    int idx = (i2c == i2c0) ? 0 : 1;
    i2c_context_t *ctx = &contexts[idx];
    ctx->state = state;
    i2c_get_hw(i2c)->intr_mask = 0;
    if (ctx->callback) {
        ctx->callback(i2c, state == STATE_IDLE, ctx->user_data);
    }
}

static void i2c_irq_handler_internal(i2c_inst_t *i2c);

static void __not_in_flash_func(i2c0_irq_handler)(void) { i2c_irq_handler_internal(i2c0); }
static void __not_in_flash_func(i2c1_irq_handler)(void) { i2c_irq_handler_internal(i2c1); }

void i2c_async_init(i2c_inst_t *i2c, uint baudrate, uint sda_pin, uint scl_pin) {
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    int idx = (i2c == i2c0) ? 0 : 1;
    contexts[idx].state = STATE_IDLE;

    i2c_hw_t *hw = i2c_get_hw(i2c);
    hw->intr_mask = 0; // Disable all interrupts initially

    if (i2c == i2c0) {
        irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
        irq_set_enabled(I2C0_IRQ, true);
    } else {
        irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
        irq_set_enabled(I2C1_IRQ, true);
    }
}

static void __not_in_flash_func(i2c_irq_handler_internal)(i2c_inst_t *i2c) {
    int idx = (i2c == i2c0) ? 0 : 1;
    i2c_context_t *ctx = &contexts[idx];
    i2c_hw_t *hw = i2c_get_hw(i2c);
    uint32_t stat = hw->intr_stat;

    // Check for errors
    if (stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        hw->clr_tx_abrt;
        i2c_finish(i2c, STATE_ERROR);
        return;
    }

    if (ctx->state == STATE_READING && (stat & I2C_IC_INTR_STAT_R_RX_FULL_BITS)) {
        // Store the result after the FIFO has been filled to the anticipated level.
        // This is safe because we set rx_tl = ctx->len - 1 in i2c_async_read_reg.
        for (size_t i = 0; i < ctx->len; i++) {
            ctx->buffer[i] = (uint8_t)hw->data_cmd;
        }
        i2c_finish(i2c, STATE_IDLE);
    } else if (ctx->state == STATE_WRITING && (stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS)) {
        // All data pushed. Wait for bus to be idle before finishing.
        if (!(hw->status & I2C_IC_STATUS_MST_ACTIVITY_BITS)) {
            i2c_finish(i2c, STATE_IDLE);
        }
    }
}

bool i2c_async_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buffer, size_t len, i2c_async_callback_t callback, void *user_data) {
    int idx = (i2c == i2c0) ? 0 : 1;
    i2c_context_t *ctx = &contexts[idx];
    i2c_hw_t *hw = i2c_get_hw(i2c);

    // Only start if idle or previous error
    if (ctx->state != STATE_IDLE && ctx->state != STATE_ERROR) return false;

    // Check if there is room for the command byte (reg address) + read requests
    if (hw->txflr + 1 + len > 16) return false;

    ctx->buffer = buffer;
    ctx->len = len;
    ctx->state = STATE_READING;
    ctx->callback = callback;
    ctx->user_data = user_data;

    // Set target address
    i2c_set_slave_mode(i2c, false, 0);
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;

    // Push the command (register address)
    hw->data_cmd = reg;
    
    // Queue read requests. Since FIFO is 16 deep and we checked for room,
    // we can push them all here.
    for (size_t i = 0; i < len; i++) {
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS; // 1 = Read
        if (i == 0) cmd |= I2C_IC_DATA_CMD_RESTART_BITS;
        if (i == len - 1) cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        hw->data_cmd = cmd;
    }

    // Set threshold so it fires when all data is ready
    hw->rx_tl = len - 1;
    // Enable RX_FULL to fire once all data is read, and TX_ABRT for errors
    hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

    return true;
}

bool i2c_async_read_regs_dual(i2c_inst_t *i2c, uint8_t addr, uint8_t reg1, size_t len1, uint8_t reg2, size_t len2, uint8_t *buffer, i2c_async_callback_t callback, void *user_data) {
    int idx = (i2c == i2c0) ? 0 : 1;
    i2c_context_t *ctx = &contexts[idx];
    i2c_hw_t *hw = i2c_get_hw(i2c);

    // Only start if idle or previous error
    if (ctx->state != STATE_IDLE && ctx->state != STATE_ERROR) return false;

    // Check if there is room for the commands:
    // reg1_addr (1) + len1 reads + reg2_addr (1) + len2 reads
    if (hw->txflr + 2 + len1 + len2 > 16) return false;

    ctx->buffer = buffer;
    ctx->len = len1 + len2;
    ctx->state = STATE_READING;
    ctx->callback = callback;
    ctx->user_data = user_data;

    // Set target address
    i2c_set_slave_mode(i2c, false, 0);
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;

    // Push first register read sequence
    hw->data_cmd = reg1;
    for (size_t i = 0; i < len1; i++) {
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS;
        if (i == 0) cmd |= I2C_IC_DATA_CMD_RESTART_BITS;
        // No stop bit here, we're continuing to reg2
        hw->data_cmd = cmd;
    }

    // Push second register read sequence
    hw->data_cmd = reg2 | I2C_IC_DATA_CMD_RESTART_BITS;
    for (size_t i = 0; i < len2; i++) {
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS;
        if (i == 0) cmd |= I2C_IC_DATA_CMD_RESTART_BITS;
        if (i == len2 - 1) cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        hw->data_cmd = cmd;
    }

    // Set threshold so it fires when all data from both reads is ready
    hw->rx_tl = ctx->len - 1;
    hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

    return true;
}

bool i2c_async_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len, i2c_async_callback_t callback, void *user_data) {
    int idx = (i2c == i2c0) ? 0 : 1;
    i2c_context_t *ctx = &contexts[idx];
    i2c_hw_t *hw = i2c_get_hw(i2c);

    if (ctx->state != STATE_IDLE && ctx->state != STATE_ERROR) return false;

    // Check if there is enough room in the TX FIFO for the register address + data
    if (hw->txflr + 1 + len > 16) return false;

    ctx->buffer = NULL; // No longer used for writing
    ctx->len = len;
    ctx->state = STATE_WRITING;
    ctx->callback = callback;
    ctx->user_data = user_data;

    // Set target address
    i2c_set_slave_mode(i2c, false, 0);
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;

    // Push the command (register address)
    uint32_t cmd = reg;
    if (len == 0) cmd |= I2C_IC_DATA_CMD_STOP_BITS;
    hw->data_cmd = cmd;

    // Push the data bytes
    for (size_t i = 0; i < len; i++) {
        cmd = data[i];
        if (i == len - 1) cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        hw->data_cmd = cmd;
    }

    // Enable TX_EMPTY to fire once the FIFO has drained, and TX_ABRT for errors
    hw->intr_mask = I2C_IC_INTR_MASK_M_TX_EMPTY_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

    return true;
}

bool i2c_blocking_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buffer, size_t len, uint32_t timeout_us) {
    if (!i2c_async_read_reg(i2c, addr, reg, buffer, len, NULL, NULL)) return false;
    
    absolute_time_t timeout_time = make_timeout_time_us(timeout_us);
    while (i2c_async_is_busy(i2c)) {
        if (time_reached(timeout_time)) {
            i2c_finish(i2c, STATE_ERROR);
            return false;
        }
        __compiler_memory_barrier();
    }
    return i2c_async_last_success(i2c);
}

bool i2c_blocking_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len, uint32_t timeout_us) {
    if (!i2c_async_write_reg(i2c, addr, reg, data, len, NULL, NULL)) return false;
    
    absolute_time_t timeout_time = make_timeout_time_us(timeout_us);
    while (i2c_async_is_busy(i2c)) {
        if (time_reached(timeout_time)) {
            i2c_finish(i2c, STATE_ERROR);
            return false;
        }
        __compiler_memory_barrier();
    }
    return i2c_async_last_success(i2c);
}

bool i2c_async_is_busy(i2c_inst_t *i2c) {
    int idx = (i2c == i2c0) ? 0 : 1;
    return contexts[idx].state != STATE_IDLE && contexts[idx].state != STATE_ERROR;
}

bool i2c_async_last_success(i2c_inst_t *i2c) {
    int idx = (i2c == i2c0) ? 0 : 1;
    return contexts[idx].state == STATE_IDLE;
}
