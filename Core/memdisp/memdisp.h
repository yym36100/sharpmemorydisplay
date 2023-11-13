#pragma once

#include <stdint.h>


// display resolution
#define mlcd_width	(400)
#define mlcd_height	(240)


// disp is write only
// disp buffer to hold one complete disp content together with the mode select and line addresses

#define mlcd_disp_buffer_stride (mlcd_width/8 + 2)
#define mlcd_disp_buffer_size (mlcd_disp_buffer_stride * mlcd_height+2)
extern volatile uint8_t mlcd_disp_buffer[mlcd_disp_buffer_size]; //12482 bytes


// public functions
void mlcd_init(void);
void mlcd_update(void);
void mlcd_clear(void);
void mlcd_setpixel(uint16_t x, uint16_t y, uint8_t pix);
void mlcd_setbyte(uint16_t x, uint16_t y, uint8_t b);

void mlcd_fill_with_rand(void);

// helper functions
void mlcd_dma_completed(void);




