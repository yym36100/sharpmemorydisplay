#include <memory.h>

#include "memdisp.h"

// disp buffer
volatile uint8_t mlcd_disp_buffer[mlcd_disp_buffer_size];

volatile static uint8_t mlcd_updatepending = 0;

static uint8_t reverse_byte(uint8_t b);

// main api
void mlcd_init(void) {
	memset(mlcd_disp_buffer, 0xff, mlcd_disp_buffer_size);
	for (int i = 0; i < mlcd_height; i++) {
		mlcd_disp_buffer[i * mlcd_disp_buffer_stride + 0] = 0x80; // data update no vcom or clear
		mlcd_disp_buffer[i * mlcd_disp_buffer_stride + 1] = reverse_byte(i+1);
	}
}

void mlcd_update(void) {
}

void mlcd_clear(void) {
	mlcd_init();
}

void mlcd_setpixel(uint16_t x, uint16_t y, uint8_t pix) {
	if (x >= mlcd_width) return;
	if (y >= mlcd_height) return;

	uint8_t bytepos = x / 8;
	uint8_t bitpos = x % 8;

	uint16_t offset = mlcd_disp_buffer_stride * y + bytepos + 2;
	uint8_t data = mlcd_disp_buffer[offset];

	if (pix)
		data |= 0x80u >> bitpos;
	else
		data &= ~(0x80u >> bitpos);
	mlcd_disp_buffer[offset] = data;
}

void mlcd_setbyte(uint16_t x, uint16_t y, uint8_t b){
	if (x >= mlcd_width/8) return;
	if (y >= mlcd_height) return;

	uint16_t offset = mlcd_disp_buffer_stride * y + x + 2;
	mlcd_disp_buffer[offset] = b;
}

extern unsigned int g_seed;
inline int fast_rand(void) {
    g_seed = (214013*g_seed+2531011);
    return (g_seed>>16)&0x7FFF;
}

void mlcd_fill_with_rand(void) {
	//for (int i = 0; i < 40000/8; i++)
	//mlcd_setbyte(fast_rand() % 50, fast_rand() % 240, fast_rand() & 0xff);
	for (int y = 0; y < 240; y++)
		for (int x = 0; x < 50; x++)
		{
			mlcd_disp_buffer[mlcd_disp_buffer_stride * y + x +2] = fast_rand();
		}
}

void mlcd_CopyFlashData(const char *p) {
	for (int y = 0; y < 240; y++)
		for (int x = 0; x < 50; x++)
		{
			mlcd_disp_buffer[mlcd_disp_buffer_stride * y + x +2] = reverse_byte(*p++);
		}
}

// helper
void mlcd_dma_completed(void) {
}

static uint8_t reverse_byte(uint8_t b) {
	uint32_t res;
	__asm__("rbit %[res], %[b]" : [res] "=r" (res) : [b] "r" (b));
	return res>>24;
}
