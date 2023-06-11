#include "py/runtime.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

#include <string.h>

#define BUFF_SIZE 12482

STATIC bool is_init = false;
STATIC mp_int_t spi_id;
STATIC mp_int_t sck_pin;
STATIC mp_int_t tx_pin;
STATIC mp_int_t cs_pin;
STATIC int dma_channel;
STATIC uint8_t buffers[3][BUFF_SIZE];
STATIC critical_section_t crit_sec;
STATIC int consumer_using = 0;
STATIC int consumer_should_use = 0;
STATIC int producer_using = 1;
STATIC int no_one_using = 2;

STATIC int64_t transmit_alarm_callback(alarm_id_t id, void *user_data) {
    // atomically switch buffers
    critical_section_enter_blocking(&crit_sec);
    consumer_using = consumer_should_use;
    no_one_using = 3 - (producer_using + consumer_using);
    critical_section_exit(&crit_sec);

    // toggle the VCOM bit
    STATIC bool vcom = false;
    buffers[consumer_using][0] = vcom ? 0b10000000 : 0b11000000;
    vcom = !vcom;
    gpio_put(cs_pin, 1); // CS active high
    dma_channel_set_read_addr(dma_channel, buffers[consumer_using], true); // true starts it
    return 0; // return zero to not schedule the alarm again
}

STATIC void dma_done_handler() {
    // transmission done
    gpio_put(cs_pin, 0);
    dma_hw->ints0 = 1u << dma_channel; // clear the interrupt
    // keep the CS pin low for long enough before
    // starting the next transmission
    // 100 ms works, 75 ms doesn't, so use safety factor 2x, 200 ms
    add_alarm_in_us(200, transmit_alarm_callback, NULL, true);
}

STATIC uint8_t reverse_uint8_bits(uint8_t x) {
    uint8_t y = 0;
    int i = 7;
    while (x) {
        y |= (x & 1) << i;
        i--;
        x >>= 1;
    }
    return y;
}

// spi id, sck pin, tx pin, cs pin
STATIC mp_obj_t dmadisplay_init(size_t n_args, const mp_obj_t *args) {
    mp_int_t spi_id_in = mp_obj_get_int(args[0]);
    mp_int_t sck_pin_in = mp_obj_get_int(args[1]);
    mp_int_t tx_pin_in = mp_obj_get_int(args[2]);
    mp_int_t cs_pin_in = mp_obj_get_int(args[3]);

    if (is_init) {
        if (
            spi_id != spi_id_in ||
            sck_pin != sck_pin_in ||
            tx_pin != tx_pin_in ||
            cs_pin != cs_pin_in
        ) {
            mp_raise_NotImplementedError(MP_ERROR_TEXT("reinitialization not supported"));
        }
        return mp_const_none;
    }

    if (spi_id_in != 0 && spi_id_in != 1) {
        mp_raise_ValueError(MP_ERROR_TEXT("expected SPI id of 0 or 1"));
    }

    dma_channel = dma_claim_unused_channel(false);
    if (dma_channel == -1) {
        mp_raise_ValueError(MP_ERROR_TEXT("could not claim DMA channel"));
    }

    spi_id = spi_id_in;
    sck_pin = sck_pin_in;
    tx_pin = tx_pin_in;
    cs_pin = cs_pin_in;
    is_init = true;

    memset(buffers[consumer_using], 0xff, BUFF_SIZE);
    int i = 1;
    for (int row=1; row<241; row++) {
        buffers[consumer_using][i] = reverse_uint8_bits(row);
        buffers[consumer_using][i + 51] = 0;
        i += 52;
    }
    buffers[consumer_using][BUFF_SIZE - 1] = 0;
    memcpy(buffers[producer_using], buffers[consumer_using], BUFF_SIZE);

    critical_section_init(&crit_sec);

    spi_inst_t *spi_inst = (spi_id == 0) ? spi0 : spi1;

    spi_init(spi_inst, 2000 * 1000);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(tx_pin, GPIO_FUNC_SPI);
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);

    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(spi_inst, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(dma_channel, &c,
                          &spi_get_hw(spi_inst)->dr, // write address
                          buffers[consumer_using], // read address
                          BUFF_SIZE, // element count (each element is of size transfer_data_size)
                          false); // don't start yet

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_channel, true);

    // Configure the processor to run dma_done_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_done_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Manually call the transmit callback once, to trigger the first transfer
    transmit_alarm_callback(0, NULL);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(dmadisplay_init_obj, 4, 4, dmadisplay_init);


// buffer, width, height, pos x, pos y, tile width, tile height
STATIC mp_obj_t dmadisplay_tile(size_t n_args, const mp_obj_t *args) {
    if (!is_init) {
        mp_raise_ValueError(MP_ERROR_TEXT("was not initialized"));
    }
    mp_buffer_info_t buff_info;
    mp_get_buffer_raise(args[0], &buff_info, MP_BUFFER_READ);
    const int width = mp_obj_get_int(args[1]);
    const int height = mp_obj_get_int(args[2]);
    const int pos_x = mp_obj_get_int(args[3]);
    const int pos_y = mp_obj_get_int(args[4]);
    const int tile_width = mp_obj_get_int(args[5]);
    const int tile_height = mp_obj_get_int(args[6]);

    if (
        width < 1 ||
        height < 1 ||
        pos_x < 0 ||
        pos_y < 0 ||
        tile_width < 1 ||
        tile_height < 1 ||
        (pos_x + tile_width - 1) > 49 ||
        (pos_y + tile_height - 1) > 29
    ) {
        mp_raise_ValueError(MP_ERROR_TEXT("args out of bounds"));
    }

    if (buff_info.typecode != 'B') {
        mp_raise_ValueError(MP_ERROR_TEXT("needs buffer of unsigned bytes"));
    }
    if (buff_info.len != width * height * 8) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer length invalid"));
    }
    const uint8_t *buff = buff_info.buf;

    uint8_t *dest = buffers[producer_using];

    const int x_end = pos_x + tile_width;
    const int y_end = pos_y + tile_height;
    int src_y = 0;
    for (int y=pos_y; y<y_end; y++) {
        const int row_major = y * (52 * 8) + 2;
        for (int y_line=0; y_line<8; y_line++) {
            const int row_exact = row_major + (52 * y_line);
            int src_x = 0;
            for (int x=pos_x; x<x_end; x++) {
                const int byte_exact = row_exact + x;
                dest[byte_exact] = buff[(src_y * 8 + y_line) * width + src_x];
                if ((++src_x) >= width) src_x = 0;
            }
        }
        if ((++src_y) >= height) src_y = 0;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(dmadisplay_tile_obj, 7, 7, dmadisplay_tile);


STATIC mp_obj_t dmadisplay_flush() {
    if (!is_init) {
        mp_raise_ValueError(MP_ERROR_TEXT("was not initialized"));
    }
    // atomically switch buffers
    critical_section_enter_blocking(&crit_sec);
    consumer_should_use = producer_using;
    producer_using = no_one_using;
    no_one_using = 3 - (producer_using + consumer_using);
    critical_section_exit(&crit_sec);
    memcpy(buffers[producer_using], buffers[consumer_should_use], BUFF_SIZE);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(dmadisplay_flush_obj, dmadisplay_flush);


STATIC const mp_rom_map_elem_t dmadisplay_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_dmadisplay) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&dmadisplay_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_tile), MP_ROM_PTR(&dmadisplay_tile_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&dmadisplay_flush_obj) }
};
STATIC MP_DEFINE_CONST_DICT(dmadisplay_module_globals, dmadisplay_module_globals_table);

const mp_obj_module_t dmadisplay_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&dmadisplay_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_dmadisplay, dmadisplay_module);
