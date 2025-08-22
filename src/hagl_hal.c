#include <stdio.h>
#include <stdint.h>
#include <hagl/backend.h>
#include "hagl_hal.h"
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_ops.h"
#include "ssd1353.h"


static SemaphoreHandle_t mutex;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

static const char* TAG = "HAGL_HAL";

uint8_t* back_buffer;



static void put_pixel( void* self, int16_t x, int16_t y, hagl_color_t color )
{
	xSemaphoreTake( mutex, portMAX_DELAY );

	const int i = ( x * 2 ) + ( y * CONFIG_SOLOMON_DISPLAY_WIDTH * 2 );

	back_buffer[ i ] = color & 0xFF;
	back_buffer[ i + 1 ] = ( color >> 8 ) & 0xFF;

	xSemaphoreGive( mutex );

	// panel_handle->draw_bitmap( panel_handle, x, y, x + 1, y + 1, (uint8_t[ ]){ color & 0xFF, ( color >> 8 ) & 0xFF } );
}



static void ssd1353_blit( void* self, int16_t x0, int16_t y0, hagl_bitmap_t* bitmap )
{
	xSemaphoreTake( mutex, portMAX_DELAY );

	hagl_color_t color;
	hagl_color_t *ptr = (hagl_color_t *) bitmap->buffer;

	for (uint16_t y = 0; y < bitmap->height; y++) {
		for (uint16_t x = 0; x < bitmap->width; x++) {
			color = *(ptr++);

			const int i = ( (x0 + x) * 2 ) + ( (y0 + y) * CONFIG_SOLOMON_DISPLAY_WIDTH * 2 );
			back_buffer[ i ] = color & 0xFF;
			back_buffer[ i + 1 ] = ( color >> 8 ) & 0xFF;
		}
	}

	xSemaphoreGive( mutex );
}


static void ssd1353_hline( void* self, int16_t x0, int16_t y0, uint16_t width, hagl_color_t color )
{
	xSemaphoreTake( mutex, portMAX_DELAY );
	for (uint16_t x = 0; x < width; x++) {

		const int i = ( (x0 + x) * 2 ) + ( (y0) * CONFIG_SOLOMON_DISPLAY_WIDTH * 2 );
		back_buffer[ i ] = color & 0xFF;
		back_buffer[ i + 1 ] = ( color >> 8 ) & 0xFF;
	}
	xSemaphoreGive( mutex );
}


static void ssd1353_vline( void* self, int16_t x0, int16_t y0, uint16_t height, hagl_color_t color )
{
	xSemaphoreTake( mutex, portMAX_DELAY );
	for (uint16_t y = 0; y < height; y++) {

		const int i = ( x0 * 2 ) + ( (y0 + y) * CONFIG_SOLOMON_DISPLAY_WIDTH * 2 );
		back_buffer[ i ] = color & 0xFF;
		back_buffer[ i + 1 ] = ( color >> 8 ) & 0xFF;
	}
	xSemaphoreGive( mutex );
}


static unsigned int ssd1353_flush( void* self )
{
	xSemaphoreTake( mutex, portMAX_DELAY );
	panel_handle->draw_bitmap( panel_handle, 0, 0, CONFIG_SOLOMON_DISPLAY_WIDTH, CONFIG_SOLOMON_DISPLAY_HEIGHT, back_buffer );
	xSemaphoreGive( mutex );
	return 0;
}


void hagl_hal_init( hagl_backend_t* backend )
{
	mutex = xSemaphoreCreateMutex();
	esp_err_t ret = ESP_OK;

	ESP_LOGI( TAG, "Initialize SPI bus" );
	const spi_bus_config_t bus_config =
	{
		.sclk_io_num = CONFIG_SOLOMON_DISPLAY_PIN_CLK,
		.mosi_io_num = CONFIG_SOLOMON_DISPLAY_PIN_MOSI,
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.flags = SPICOMMON_BUSFLAG_MASTER,
		.max_transfer_sz = CONFIG_SOLOMON_DISPLAY_WIDTH * 80 * sizeof( uint16_t ),
	};

	ESP_GOTO_ON_ERROR( spi_bus_initialize( SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO ), err, TAG, "Failed to initialise spi bus" );

	ESP_LOGI( TAG, "Install panel IO" );
	const esp_lcd_panel_io_spi_config_t io_config =
	{
		.cs_gpio_num = CONFIG_SOLOMON_DISPLAY_PIN_CS,
		.dc_gpio_num = CONFIG_SOLOMON_DISPLAY_PIN_DC,
		.spi_mode = 0,
		.pclk_hz = 20 * 1000 * 1000,
		.trans_queue_depth = 7,
		.on_color_trans_done = NULL,
		.user_ctx = NULL,
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8,
	};

	ESP_GOTO_ON_ERROR( esp_lcd_new_panel_io_spi( SPI2_HOST, &io_config, &io_handle ), err, TAG, "Failed to install panel io" );

	ESP_LOGI( TAG, "Install SSD1353 panel driver" );

	const esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = CONFIG_SOLOMON_DISPLAY_PIN_DC,
		.bits_per_pixel = 16,
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB
	};

	ESP_GOTO_ON_ERROR( esp_lcd_new_panel_ssd1353( io_handle, &panel_config, &panel_handle ), err, TAG, "Failed install ssd1353" );
	ESP_GOTO_ON_ERROR( esp_lcd_panel_reset( panel_handle ), err, TAG, "Failed install ssd1353" );
	ESP_GOTO_ON_ERROR( esp_lcd_panel_init( panel_handle ), err, TAG, "Failed install ssd1353" );


	ESP_LOGI( TAG, "SSD1353 driver installed" );

	vTaskDelay( 10 / portTICK_PERIOD_MS );

	backend->width = CONFIG_SOLOMON_DISPLAY_WIDTH;
	backend->height = CONFIG_SOLOMON_DISPLAY_HEIGHT;
	backend->depth = CONFIG_SOLOMON_DISPLAY_DEPTH;
	// backend->buffer =
	back_buffer = calloc( CONFIG_SOLOMON_DISPLAY_WIDTH * CONFIG_SOLOMON_DISPLAY_HEIGHT * 2, sizeof( uint8_t ) );
	backend->put_pixel = put_pixel;
	backend->flush = ssd1353_flush;
	backend->hline = ssd1353_hline;
	backend->vline = ssd1353_vline;
	backend->blit = ssd1353_blit;
	backend->clip.x0 = 0;
	backend->clip.y0 = 0;

	backend->clip.x1 = CONFIG_SOLOMON_DISPLAY_WIDTH;
	backend->clip.y1 = CONFIG_SOLOMON_DISPLAY_HEIGHT;

	backend->flush( backend );
	ESP_GOTO_ON_ERROR( esp_lcd_panel_disp_on_off( panel_handle, true ), err, TAG, "Failed install ssd1353" );
	return;

err:
	io_handle = NULL;
	panel_handle = NULL;
	backend = NULL;
}
