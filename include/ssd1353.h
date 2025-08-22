#ifndef _SSD1353_H
#define _SSD1353_H
#include "driver/spi_master.h"
#include "hagl/backend.h"
#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1353_SET_COLUMN_ADDRESS                  (0x15)
#define SSD1353_SET_ROW_ADDRESS                     (0x75)
#define SSD1353_WRITE_RAM_COMMAND                   (0x5C)
#define SSD1353_READ_RAM_COMMAND                    (0x5D)
#define SSD1353_SET_DISPLAY_OFF		                (0xAE)
#define SSD1353_SET_DISPLAY_ON		                (0xAF)
#define SSD1353_SET_MULTIPLEX_RATIO                 (0xA8)
#define SSD1353_SET_DISPLAY_START_LINE              (0xA1)
#define SSD1353_SET_DISPLAY_OFFSET                  (0xA2)
#define SSD1353_SET_DISPLAY_NORMAL                  (0xA4)
#define SSD1353_SET_REMAP_DUAL_COM_LINE             (0xA0)
#define SSD1353_SET_CONTRAST_COLOUR_A               (0x81)
#define SSD1353_SET_CONTRAST_COLOUR_B               (0x82)
#define SSD1353_SET_CONTRAST_COLOUR_C               (0x83)
#define SSD1353_SET_CONTRAST			            (0x87)
#define SSD1353_ENABLE_LINEAR_GRAYSCALE             (0xB9)
#define SSD1353_SET_PHASE_ADJUSTMENT                (0xB1)
#define SSD1353_SET_CLOCK_DIVIDER				    (0xB3)
#define SSD1353_SET_PRE_CHARGE	                    (0xBB)
#define SSD1353_SET_VCOMH		                    (0xBE)
#define SSD1353_SET_DISPLAY_ALL_OFF                 (0xA6)



/* Entering FDh 12h (A[2]=0b) can unlock the OLED driver IC. That means the     */
/* driver IC resume from the “Lock” state. And the driver IC will then respond  */
/* to the command and memory access. */
//#define SSD1353_SET_COMMAND_LOCK_UNLOCK             (0x12) /* 0b00010010 */
#define SSD1353_SET_COMMAND_LOCK_UNLOCK             (0xb1)

/* For SSD1353_SET_REMAP_DUAL_COM_LINE. */
#define SD1351_HORIZONTAL_ADDRESS_INCREMENT         (0b00000000) /* 0x00 */
#define SD1351_VERTICAL_ADDRESS_INCREMENT           (0b00000001) /* Swap XY 0x01 */
#define SD1351_COLUMN_ADDRESS_REMAP                 (0b00000010) /* Mirror X 0x02 */
#define SD1351_COLOR_REMAP                          (0b00000100) /* RGB to BGR 0x04 */
#define SD1351_COM_SCAN_DIRECTION_REMAP             (0b00001000) /* 0x08 */
#define SD1351_COM_ODD_EVEN_SPLIT                   (0b00010000) /* Mirror Y 0x10 */
#define SD1351_COLOR_MODE_65K                       (0b00100000) /* RGB565 0x20 */
#define SD1351_COLOR_MODE_262K                      (0b01000000) /* RGB666 0x40 */
#define SD1351_COLOR_MODE_262K_FORMAT2              (0b01100000) /* RGB666 format 2 */



esp_err_t esp_lcd_new_panel_ssd1353
(
	const esp_lcd_panel_io_handle_t io,
	const esp_lcd_panel_dev_config_t* panel_dev_config,
	esp_lcd_panel_handle_t* ret_panel
);


#define SSD1353_PANEL_BUS_SPI_CONFIG(sclk, mosi, max_trans_sz)  \
{																\
		.sclk_io_num = sclk,                                    \
		.mosi_io_num = mosi,                                    \
		.miso_io_num = -1,                                      \
		.quadwp_io_num = -1,                                    \
		.quadhd_io_num = -1,                                    \
		.flags = SPICOMMON_BUSFLAG_MASTER,                      \
		.max_transfer_sz = max_trans_sz,                        \
}


#define SSD1353_PANEL_IO_SPI_CONFIG(cs, dc, callback, callback_ctx) \
{																	\
		.cs_gpio_num = cs,                                          \
		.dc_gpio_num = dc,                                          \
		.spi_mode = 0,                                              \
		.pclk_hz = 20 * 1000 * 1000,                                \
		.trans_queue_depth = 7,										\
		.on_color_trans_done = callback,                            \
		.user_ctx = callback_ctx,                                   \
		.lcd_cmd_bits = 8,                                          \
		.lcd_param_bits = 8,                                        \
}


#ifdef __cplusplus
}
#endif
#endif /* _SSD1353_H */
