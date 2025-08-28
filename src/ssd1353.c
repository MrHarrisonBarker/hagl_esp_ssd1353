#include "ssd1353.h"

#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>

#include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"

static const char* TAG = "SSD1353";


typedef struct
{
	uint8_t cmd;
	uint8_t data[ 16 ];
	uint8_t databytes;
} ssd1353_init_cmd_t;


static esp_err_t panel_ssd1353_del( esp_lcd_panel_t* panel );
static esp_err_t panel_ssd1353_reset( esp_lcd_panel_t* panel );
static esp_err_t panel_ssd1353_init( esp_lcd_panel_t* panel );
static esp_err_t panel_ssd1353_draw_bitmap( esp_lcd_panel_t* panel, int x_start, int y_start, int x_end, int y_end, const void* color_data );
static esp_err_t panel_ssd1353_invert_color( esp_lcd_panel_t* panel, bool invert_color_data );
static esp_err_t panel_ssd1353_mirror( esp_lcd_panel_t* panel, bool mirror_x, bool mirror_y );
static esp_err_t panel_ssd1353_swap_xy( esp_lcd_panel_t* panel, bool swap_axes );
static esp_err_t panel_ssd1353_set_gap( esp_lcd_panel_t* panel, int x_gap, int y_gap );
static esp_err_t panel_ssd1353_disp_on_off( esp_lcd_panel_t* panel, bool on_off );


typedef struct
{
	esp_lcd_panel_t base;
	esp_lcd_panel_io_handle_t io;
	int reset_gpio_num;
	bool reset_level;
} ssd1353_panel_t;


static const ssd1353_init_cmd_t init_cmds[ ] = {
	{ SSD1353_SET_DISPLAY_OFF, { 0 }, 0x80 },
	{ SSD1353_SET_MULTIPLEX_RATIO, { 0x7F }, 1 },
	{ SSD1353_SET_DISPLAY_OFFSET, { 0x00 }, 1 },
	{ SSD1353_SET_DISPLAY_START_LINE, { 0x00 }, 1 },
	{ SSD1353_SET_DISPLAY_NORMAL, { 0 }, 0x80 },
	{ SSD1353_SET_REMAP_DUAL_COM_LINE, { 0B01100110 }, 1 },
	{ SSD1353_SET_CONTRAST_COLOUR_A, { 0x75 }, 1 },
	{ SSD1353_SET_CONTRAST_COLOUR_B, { 0x60 }, 1 },
	{ SSD1353_SET_CONTRAST_COLOUR_C, { 0x6A }, 1 },
	{ SSD1353_SET_CONTRAST, { 0x0F }, 1 },
	{ SSD1353_ENABLE_LINEAR_GRAYSCALE, { 0 }, 0x80 },
	{ SSD1353_SET_PHASE_ADJUSTMENT, { 0x22 }, 1 },
	{ SSD1353_SET_CLOCK_DIVIDER, { 0x40 }, 1 },
	{ SSD1353_SET_PRE_CHARGE, { 0x08 }, 1 },
	{ SSD1353_SET_VCOMH, { 0x2F }, 1 },
	{ SSD1353_SET_DISPLAY_ALL_OFF, { 0 }, 0x80 },
	{ SSD1353_SET_DISPLAY_ON, { 0 }, 0x80 },
	{ 0x00, { 0 }, 0xFF }
};


esp_err_t esp_lcd_new_panel_ssd1353( const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t* panel_dev_config, esp_lcd_panel_handle_t* ret_panel )
{
	esp_err_t ret = ESP_OK;
	ssd1353_panel_t* panel = NULL;
	gpio_config_t io_conf = { 0 };

	ESP_GOTO_ON_FALSE( io && panel_dev_config, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument" );

	panel = ( ssd1353_panel_t* )calloc( 1, sizeof( ssd1353_panel_t ) );
	ESP_GOTO_ON_FALSE( panel, ESP_ERR_NO_MEM, err, TAG, "no mem for OLED panel" );

	if ( panel_dev_config->reset_gpio_num >= 0 )
	{
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
		ESP_GOTO_ON_ERROR( gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed" );

		ESP_LOGI( TAG, "Configured RST line" );
	}

	panel->io = io;
	panel->reset_gpio_num = panel_dev_config->reset_gpio_num;
	panel->reset_level = panel_dev_config->flags.reset_active_high;

	panel->base.del = panel_ssd1353_del;
	panel->base.reset = panel_ssd1353_reset;
	panel->base.init = panel_ssd1353_init;
	panel->base.draw_bitmap = panel_ssd1353_draw_bitmap;
	panel->base.invert_color = panel_ssd1353_invert_color;
	panel->base.set_gap = panel_ssd1353_set_gap;
	panel->base.mirror = panel_ssd1353_mirror;
	panel->base.swap_xy = panel_ssd1353_swap_xy;
	panel->base.disp_on_off = panel_ssd1353_disp_on_off;

	*ret_panel = &( panel->base );
	ESP_LOGI( TAG, "OLED panel create success" );

	return ESP_OK;

err:
	if ( panel )
	{
		if ( panel_dev_config->reset_gpio_num >= 0 )
		{
			gpio_reset_pin( panel_dev_config->reset_gpio_num );
		}
		free( panel );
	}
	return ret;
}


esp_err_t panel_ssd1353_del( esp_lcd_panel_t* panel )
{
	return ESP_OK;
}


esp_err_t panel_ssd1353_reset( esp_lcd_panel_t* panel )
{
	ssd1353_panel_t* ssd1353 = __containerof( panel, ssd1353_panel_t, base );

	gpio_set_level( ssd1353->reset_gpio_num, ssd1353->reset_level );
	vTaskDelay( pdMS_TO_TICKS( 10 ) );
	gpio_set_level( ssd1353->reset_gpio_num, !ssd1353->reset_level );
	vTaskDelay( pdMS_TO_TICKS( 10 ) );

	return ESP_OK;
}


esp_err_t panel_ssd1353_init( esp_lcd_panel_t* panel )
{
	ssd1353_panel_t* ssd1353 = __containerof( panel, ssd1353_panel_t, base );
	esp_lcd_panel_io_handle_t io = ssd1353->io;

	uint8_t cmd = 0;

	while ( init_cmds[ cmd ].databytes != 0xff )
	{
		ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_param(io, init_cmds[cmd].cmd, init_cmds[cmd].data, init_cmds[cmd].databytes), TAG, "send command failed" );

		if ( init_cmds[ cmd ].databytes & 0x80 )
		{
			vTaskDelay( 100 / portTICK_PERIOD_MS );
		}

		cmd++;
	}

	ESP_LOGI( TAG, "send init commands success" );
	return ESP_OK;
}


esp_err_t panel_ssd1353_draw_bitmap( esp_lcd_panel_t* panel, int x_start, int y_start, int x_end, int y_end, const void* color_data )
{
	// ESP_LOGI( TAG, "------ Drawing bitmap ------" );
	ssd1353_panel_t* ssd1353 = __containerof( panel, ssd1353_panel_t, base );
	esp_lcd_panel_io_handle_t io = ssd1353->io;


	ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_param(io, SSD1353_SET_COLUMN_ADDRESS, (uint8_t[]){x_start, x_end-1}, 2), TAG, "send command failed" );
	ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_param(io, SSD1353_SET_ROW_ADDRESS, (uint8_t[]){y_start, y_end-1}, 2), TAG, "send command failed" );

	size_t len = ( x_end - x_start ) * ( y_end - y_start ) * 2;

	ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_color(io, SSD1353_WRITE_RAM_COMMAND, color_data, len), TAG, "send colour failed" );

	return ESP_OK;
}


esp_err_t panel_ssd1353_invert_color( esp_lcd_panel_t* panel, bool invert_color_data )
{
	return ESP_OK;
}


esp_err_t panel_ssd1353_mirror( esp_lcd_panel_t* panel, bool mirror_x, bool mirror_y )
{
	return ESP_OK;
}


esp_err_t panel_ssd1353_swap_xy( esp_lcd_panel_t* panel, bool swap_axes )
{
	return ESP_OK;
}


esp_err_t panel_ssd1353_set_gap( esp_lcd_panel_t* panel, int x_gap, int y_gap )
{
	return ESP_OK;
}


esp_err_t panel_ssd1353_disp_on_off( esp_lcd_panel_t* panel, bool on_off )
{
	// ESP_LOGI(TAG, "DISPLAY ON OFF");
	ssd1353_panel_t* ssd1353 = __containerof( panel, ssd1353_panel_t, base );
	esp_lcd_panel_io_handle_t io = ssd1353->io;
	uint8_t command = 0;

	if ( on_off )
	{
		command = SSD1353_SET_DISPLAY_NORMAL;
	}
	else
	{
		command = SSD1353_SET_DISPLAY_ALL_OFF;
	}

	ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed" );

	return ESP_OK;
}



// static esp_err_t ssd1353_wait_for_queue( ssd1353_t* ssd1353 )
// {
// 	spi_transaction_t* transaction = NULL;
// 	const size_t num_trans_inflight = ssd1353->transactions_queued;
//
// 	for ( size_t i = 0; i < num_trans_inflight; i++ )
// 	{
// 		ESP_RETURN_ON_ERROR( spi_device_get_trans_result( ssd1353->spi_device, &transaction, portMAX_DELAY ), TAG, "recycle spi transactions failed" );
// 		ssd1353->transactions_queued--;
// 	}
//
// 	return ESP_OK;
// }
//
//
// static esp_err_t ssd1353_display_write_command( ssd1353_t* ssd1353, const uint8_t command, const uint8_t* param, size_t param_size )
// {
// 	esp_err_t ret = ESP_OK;
// 	ssd1353_transaction_t* transaction = NULL;
//
// 	ESP_GOTO_ON_ERROR( spi_device_acquire_bus( ssd1353->spi_device, portMAX_DELAY ), err, TAG, "spi_device_acquire_bus" );
//
// 	// before issue a polling transaction, need to wait queued transactions finished
// 	ssd1353_wait_for_queue( ssd1353 );
//
// 	transaction = &ssd1353->transaction_queue[ 0 ];
// 	memset( transaction, 0, sizeof( ssd1353_transaction_t ) );
//
// 	// if ( param && param_size )
// 	// {
// transaction->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
// 	// }
//
// 	transaction->dc_level = 0;
// 	transaction->base.length = 8;
// 	transaction->base.tx_buffer = &command;
//
// 	ESP_LOGI( TAG, "Sending command 0x%02x", command );
// 	ret = spi_device_polling_transmit( ssd1353->spi_device, &transaction->base );
// 	ESP_GOTO_ON_ERROR( ret, err, TAG, "spi_device_polling_transmit command" );
//
// 	if ( param && param_size )
// 	{
// 		transaction->dc_level = 1;
// 		transaction->base.length = param_size * 8;
// 		transaction->base.tx_buffer = param;
// 		// transaction->base.flags &= ~SPI_TRANS_CS_KEEP_ACTIVE;
//
// 		ret = spi_device_polling_transmit( ssd1353->spi_device, &transaction->base );
// 		ESP_GOTO_ON_ERROR( ret, err, TAG, "spi_device_polling_transmit param" );
// 		ESP_LOG_BUFFER_HEX( TAG, param, param_size );
// 	}
//
//
// err:
// 	spi_device_release_bus( ssd1353->spi_device );
// 	return ret;
//
// }
//
//
// // static void ssd1353_display_write_data( spi_device_handle_t spi, const uint8_t* data, const size_t length )
// // {
// // 	if ( 0 == length )
// // 	{
// // 		return;
// // 	};
// //
// // 	spi_transaction_t transaction;
// // 	memset( &transaction, 0, sizeof( transaction ) );
// //
// // 	/* Length in bits. */
// // 	transaction.length = length * 8;
// // 	transaction.user = ( void* )1; // DATA
// // 	transaction.tx_buffer = data;
// //
// // 	ESP_ERROR_CHECK( spi_device_polling_transmit(spi, &transaction) );
// // 	ESP_LOG_BUFFER_HEX_LEVEL( TAG, data, length, ESP_LOG_DEBUG );
// // 	ESP_LOGI( TAG, "Write data %d", length );
// // }
//
//
// static void ssd1353_display_set_address( spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 )
// {
// 	// uint8_t data[ 2 ];
// 	// static uint16_t prev_x1, prev_x2, prev_y1, prev_y2;
// 	//
// 	// x1 = x1 + CONFIG_SOLOMON_DISPLAY_OFFSET_X;
// 	// y1 = y1 + CONFIG_SOLOMON_DISPLAY_OFFSET_Y;
// 	// x2 = x2 + CONFIG_SOLOMON_DISPLAY_OFFSET_X;
// 	// y2 = y2 + CONFIG_SOLOMON_DISPLAY_OFFSET_Y;
// 	//
// 	// /* Change column address only if it has changed. */
// 	// if ( ( prev_x1 != x1 || prev_x2 != x2 ) )
// 	// {
// 	// 	ssd1353_display_write_command( spi, SSD1353_SET_COLUMN_ADDRESS );
// 	//
// 	// 	data[ 0 ] = x1;
// 	// 	data[ 1 ] = x2;
// 	// 	ssd1353_display_write_data( spi, data, 2 );
// 	//
// 	//
// 	// 	prev_x1 = x1;
// 	// 	prev_x2 = x2;
// 	// }
// 	//
// 	// /* Change row address only if it has changed. */
// 	// if ( ( prev_y1 != y1 || prev_y2 != y2 ) )
// 	// {
// 	// 	ssd1353_display_write_command( spi, SSD1353_SET_ROW_ADDRESS );
// 	//
// 	// 	data[ 0 ] = y1;
// 	// 	data[ 1 ] = y2;
// 	// 	ssd1353_display_write_data( spi, data, 2 );
// 	//
// 	// 	prev_y1 = y1;
// 	// 	prev_y2 = y2;
// 	// }
// 	//
// 	// ssd1353_display_write_command( spi, SSD1353_WRITE_RAM_COMMAND );
// }
//
//
// esp_err_t ssd1353_display_on_off( ssd1353_t* ssd1353, const bool on )
// {
// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_command(ssd1353, on ? 0xA4 : 0xA6, NULL, 0), TAG, "failed to send on/off command" );
// 	return ESP_OK;
// }
//
//
// static void spi_pre_transfer_callback( spi_transaction_t* transaction )
// {
// 	const ssd1353_transaction_t* trans = __containerof( transaction, ssd1353_transaction_t, base );
// 	gpio_set_level( CONFIG_SOLOMON_DISPLAY_PIN_DC, trans->dc_level );
// }
//
//
// static void spi_post_transfer_callback( spi_transaction_t* transaction )
// {
// 	// gpio_set_level( CONFIG_SOLOMON_DISPLAY_PIN_DC, ( int )transaction->user );
// }
//
//
// static esp_err_t ssd1353_display_spi_master_init( spi_device_handle_t* spi )
// {
// 	spi_bus_config_t spi_bus_config = {
// 		.miso_io_num = CONFIG_SOLOMON_DISPLAY_PIN_MISO,
// 		.mosi_io_num = CONFIG_SOLOMON_DISPLAY_PIN_MOSI,
// 		.sclk_io_num = CONFIG_SOLOMON_DISPLAY_PIN_CLK,
// 		.quadwp_io_num = -1,
// 		.quadhd_io_num = -1,
// 		.flags = SPICOMMON_BUSFLAG_MASTER
// 		/* Max transfer size in bytes. */
// 		// .max_transfer_sz = SPI_MAX_TRANSFER_SIZE
// 	};
//
// 	spi_device_interface_config_t spi_device_interface_config = {
// 		.clock_speed_hz = CONFIG_SOLOMON_DISPLAY_SPI_CLOCK_SPEED_HZ,
// 		.mode = CONFIG_SOLOMON_DISPLAY_SPI_MODE,
// 		.spics_io_num = CONFIG_SOLOMON_DISPLAY_PIN_CS,
// 		.queue_size = CONFIG_SOLOMON_DISPLAY_TRANSACTION_QUEUE_SIZE,
// 		.flags = SPI_DEVICE_HALFDUPLEX,
// 		.pre_cb = spi_pre_transfer_callback,
// 		.post_cb = spi_post_transfer_callback
// 	};
//
//
// 	ESP_RETURN_ON_ERROR( spi_bus_initialize(CONFIG_SOLOMON_DISPLAY_SPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize" );
// 	ESP_RETURN_ON_ERROR( spi_bus_add_device(CONFIG_SOLOMON_DISPLAY_SPI_HOST, &spi_device_interface_config, spi), TAG, "spi_bus_add_device" );
//
// 	return ESP_OK;
// }
//
//
// esp_err_t ssd1353_display_init( ssd1353_t* ssd1353 )
// {
// 	esp_err_t ret = ESP_OK;
// 	ssd1353 = calloc( 1, sizeof( ssd1353_t ) + sizeof( ssd1353_transaction_t ) * CONFIG_SOLOMON_DISPLAY_TRANSACTION_QUEUE_SIZE );
// 	ESP_GOTO_ON_FALSE( ssd1353, ESP_ERR_NO_MEM, err, TAG, "no mem for spi panel io" );
//
//
// 	if ( CONFIG_SOLOMON_DISPLAY_PIN_CS > 0 )
// 	{
// 		gpio_set_direction( CONFIG_SOLOMON_DISPLAY_PIN_CS, GPIO_MODE_OUTPUT );
// 		gpio_set_level( CONFIG_SOLOMON_DISPLAY_PIN_CS, 0 );
// 	}
//
// 	gpio_set_direction( CONFIG_SOLOMON_DISPLAY_PIN_DC, GPIO_MODE_OUTPUT );
//
//
// 	ESP_GOTO_ON_ERROR( ssd1353_display_spi_master_init( &ssd1353->spi_device ), err, TAG, "ssd1353_display_spi_master_init" );
// 	vTaskDelay( 100 / portTICK_PERIOD_MS );
//
//
// 	ssd1353->config.dc_command_level = 0;
// 	ssd1353->transaction_queue_size = CONFIG_SOLOMON_DISPLAY_TRANSACTION_QUEUE_SIZE;
// 	ssd1353->transactions_queued = 0;
//
// 	size_t max_trans_bytes = 0;
// 	ESP_GOTO_ON_ERROR( spi_bus_get_max_transaction_len(CONFIG_SOLOMON_DISPLAY_SPI_HOST, &max_trans_bytes), err, TAG, "get spi max transaction len failed" );
// 	ssd1353->transaction_max_bytes = max_trans_bytes;
//
//
// 	mutex = xSemaphoreCreateMutex();
//
//
// 	/* Reset the display. */
// 	if ( CONFIG_SOLOMON_DISPLAY_PIN_RST > 0 )
// 	{
// 		gpio_set_direction( CONFIG_SOLOMON_DISPLAY_PIN_RST, GPIO_MODE_OUTPUT );
// 		/* Low will reset ie. initialize the display with defaults. */
// 		gpio_set_level( CONFIG_SOLOMON_DISPLAY_PIN_RST, 0 );
// 		vTaskDelay( 100 / portTICK_PERIOD_MS );
// 		/* High resumes normal operation. */
// 		gpio_set_level( CONFIG_SOLOMON_DISPLAY_PIN_RST, 1 );
// 		vTaskDelay( 100 / portTICK_PERIOD_MS );
// 	}
//
// 	uint8_t cmd = 0;
//
// 	while ( init_cmds[ cmd ].databytes != 0xff )
// 	{
// 		ESP_GOTO_ON_ERROR(
// 			ssd1353_display_write_command( ssd1353, init_cmds[ cmd ].cmd, init_cmds[ cmd ].data, init_cmds[ cmd ].databytes ),
// 			err,
// 			TAG,
// 			"Failed to send command %x",
// 			init_cmds[ cmd ].cmd
// 		);
//
// 		if ( init_cmds[ cmd ].databytes & 0x80 )
// 		{
// 			vTaskDelay( 100 / portTICK_PERIOD_MS );
// 		}
// 		else
// 		{
// 			vTaskDelay( 10 / portTICK_PERIOD_MS );
// 		}
//
// 		cmd++;
// 	}
//
// 	vTaskDelay( 100 / portTICK_PERIOD_MS );
//
// 	ESP_GOTO_ON_ERROR( ssd1353_display_on_off( ssd1353, true ), err, TAG, "ssd1353_display_on_off" );
// 	vTaskDelay( 1000 / portTICK_PERIOD_MS );
// 	// ESP_GOTO_ON_ERROR( ssd1353_display_on_off( ssd1353, false ), err, TAG, "ssd1353_display_on_off" );
//
// 	uint8_t blank[ 160 * 2 ];
// 	for ( size_t j = 0; j < 160 * 2; j += 2 )
// 	{
// 		blank[ j ] = 0x00;
// 		blank[ j + 1 ] = 0x00;
// 	}
//
// 	for ( uint8_t i = 0; i < 128; i++ )
// 	{
// 		ESP_ERROR_CHECK( ssd1353_display_write(ssd1353, 0, i, 160, i + 1, blank) );
// 		vTaskDelay( 100 / portTICK_PERIOD_MS );
// 		// ESP_GOTO_ON_ERROR( ssd1353_display_on_off( ssd1353, false ), err, TAG, "ssd1353_display_on_off" );
// 		// vTaskDelay( 1000 / portTICK_PERIOD_MS );
// 		// ESP_GOTO_ON_ERROR( ssd1353_display_on_off( ssd1353, true ), err, TAG, "ssd1353_display_on_off" );
// 	}
//
// 	vTaskDelay( 100 / portTICK_PERIOD_MS );
//
// 	ESP_LOGI( TAG, "initialised" );
//
// 	return ESP_OK;
//
// err:
// 	if ( ssd1353 )
// 	{
// 		gpio_reset_pin( CONFIG_SOLOMON_DISPLAY_PIN_DC );
// 		free( ssd1353 );
// 	}
//
// 	return ret;
//
// 	// ssd1353_display_write_command( *spi, 0xA4 );
//
// 	// vTaskDelay( 100 / portTICK_PERIOD_MS );
//
// 	// uint8_t blank[ 160 * 2 ];
// 	// for ( size_t j = 0; j < 160 * 2; j += 2 )
// 	// {
// 	// 	blank[ j ] = 0x00;
// 	// 	blank[ j + 1 ] = 0x00;
// 	// }
// 	//
// 	// ssd1353_display_set_address( *spi, 0, 0, 160, 128 );
// 	//
// 	// for ( uint8_t i = 0; i < SSD1353_HEIGHT; i++ )
// 	// {
// 	// 	// ESP_ERROR_CHECK(panel_ssd1353_draw_bitmap(panel, 0, i, 160, i + 1, blank));
// 	//
// 	// 	// size_t len = (160 - 0) * (i+1 - i) * 2;
// 	// 	for ( uint8_t j = 0; j < CONFIG_SOLOMON_DISPLAY_WIDTH; j++ )
// 	// 	{
// 	// 		ssd1353_display_write_data( *spi, blank, 2 );
// 	// 		vTaskDelay( 50 / portTICK_PERIOD_MS );
// 	// 	}
// 	// }
//
//
// 	// for ( uint8_t i = 0; i < 128; i++ )
// 	// {
// 	// 	// oled_data( spi, rowData, 320 );
// 	// 	for ( uint8_t j = 0; j < 160; j++ )
// 	// 	{
// 	// 		ssd1353_display_write_data( *spi, pixelData, 3 );
// 	// 		// oled_data( spi, pixelData, 3 );
// 	//
// 	// 		vTaskDelay( 5 / portTICK_PERIOD_MS );
// 	// 	}
// 	// }
//
// }
//
//
// esp_err_t ssd1353_display_write_colour( ssd1353_t* ssd1353, const void* colour, size_t colour_size )
// {
// 	esp_err_t ret = ESP_OK;
// 	ssd1353_transaction_t* transaction = NULL;
//
//
// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_command( ssd1353, 0x5C, NULL, 0 ), TAG, "failed to send write command" );
// 	// ESP_GOTO_ON_ERROR( spi_device_acquire_bus( ssd1353->spi_device, portMAX_DELAY ), err, TAG, "spi_device_acquire_bus" );
//
//
// 	// before issue a polling transaction, need to wait queued transactions finished
// 	// ssd1353_wait_for_queue( ssd1353 );
// 	// transaction = &ssd1353->transaction_queue[ 0 ];
// 	// memset( transaction, 0, sizeof( ssd1353_transaction_t ) );
// 	//
// 	// transaction->dc_level = 0;
// 	// transaction->base.length = 8;
// 	// uint8_t cmd = SSD1353_WRITE_RAM_COMMAND;
// 	// transaction->base.tx_buffer = &cmd;
// 	//
// 	// // if ( colour && colour_size )
// 	// // {
// 	// // 	transaction->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
// 	// // }
// 	//
// 	// ret = spi_device_polling_transmit( ssd1353->spi_device, &transaction->base );
// 	// ESP_GOTO_ON_ERROR( ret, err, TAG, "spi transmit (polling) command failed" );
// 	// ESP_LOGI( TAG, "Sending write command" );
//
// 	// chunk colour buffer and send
// 	do
// 	{
// 		size_t chunk_size = colour_size;
//
// 		if ( ssd1353->transactions_queued < ssd1353->transaction_queue_size )
// 		{
// 			ESP_LOGI( TAG, "getting next available transaction from queue, %d", ssd1353->transactions_queued );
// 			// get the next available transaction
// 			transaction = &ssd1353->transaction_queue[ ssd1353->transactions_queued ];
// 		}
// 		else
// 		{
// 			ESP_LOGI( TAG, "recycling queued transactions" );
// 			spi_transaction_t* spi_transaction = NULL;
// 			// transaction pool has used up, recycle one transaction
// 			ret = spi_device_get_trans_result( ssd1353->spi_device, &spi_transaction, portMAX_DELAY );
// 			ESP_GOTO_ON_ERROR( ret, err, TAG, "recycle spi transactions failed" );
// 			transaction = __containerof( spi_transaction, ssd1353_transaction_t, base );
// 			ssd1353->transactions_queued--;
// 		}
// 		memset( transaction, 0, sizeof( ssd1353_transaction_t ) );
//
//
// 		// SPI per-transfer size has its limitation, if the color buffer is too big, we need to split it into multiple chunks
// 		if ( chunk_size > ssd1353->transaction_max_bytes )
// 		{
// 			// cap the transfer size to the maximum supported by the bus
// 			chunk_size = ssd1353->transaction_max_bytes;
// 			ESP_LOGI( TAG, "chunk size larger than mx transaction size setting to, %d", chunk_size );
// 			// transaction->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
// 		}
// 		else
// 		{
// 			// mark en_trans_done_cb only at the last round to avoid premature completion callback
// 			// lcd_trans->flags.en_trans_done_cb = 1;
// 			ESP_LOGI( TAG, "chunk size within max transaction size" );
// 			// transaction->base.flags &= ~SPI_TRANS_CS_KEEP_ACTIVE;
// 		}
//
// 		transaction->dc_level = 1; // set D/C level in data phase
// 		transaction->base.length = chunk_size * 8; // transaction length is in bits
// 		transaction->base.tx_buffer = colour;
//
// 		ESP_LOGI( TAG, "queuing colour chunk, %d", chunk_size );
//
// 		// color data is usually large, using queue+blocking mode
// 		// ret = spi_device_queue_trans( ssd1353->spi_device, &transaction->base, portMAX_DELAY );
// 		ret = spi_device_polling_transmit( ssd1353->spi_device, &transaction->base );
// 		ESP_GOTO_ON_ERROR( ret, err, TAG, "spi transmit (queue) color failed" );
// 		// ssd1353->transactions_queued++;
//
// 		ESP_LOGI( TAG, "Sending colour chunk, %d", chunk_size );
// 		ESP_LOG_BUFFER_HEX( TAG, transaction->base.tx_buffer, chunk_size );
//
// 		// move on to the next chunk
// 		colour = ( const uint8_t* )colour + chunk_size;
// 		colour_size -= chunk_size;
// 	}
// 	while ( colour_size > 0 );
//
// 	// xSemaphoreGive( mutex );
//
// 	// return ret;
//
// err:
// 	// spi_device_release_bus( ssd1353->spi_device );
// 	// xSemaphoreGive( mutex );
// 	return ret;
// }
//
//
// esp_err_t ssd1353_display_write( ssd1353_t* ssd1353, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint8_t* buffer )
// {
// 	ESP_LOGI( TAG, "setting row to %d->%d and column to %d->%d", y_start, y_end-1, x_start, x_end-1 );
//
// 	uint8_t d[ 2 ] = { x_start, x_end };
// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_command( ssd1353, 0x15, d, 2 ), TAG, "failed to set column address" );
// 	d[0] = y_start;
// 	d[1] = y_end;
// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_command( ssd1353, 0x75, d, 2 ), TAG, "failed to set row address" );
//
// 	const size_t colour_size = ( x_end - x_start ) * ( y_end - y_start ) * 2;
// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_colour( ssd1353, buffer, colour_size ), TAG, "failed to write colour" );
//
// 	return ESP_OK;
// 	// 	if ( 0 == w || 0 == h )
// 	// 	{
// 	// 		return 0;
// 	// 	}
// 	//
// 	// 	const int32_t x2 = x1 + w - 1;
// 	// 	const int32_t y2 = y1 + h - 1;
// 	// 	const size_t size = w * h * CONFIG_SOLOMON_DISPLAY_DEPTH / 8;
// 	//
// 	// 	// xSemaphoreTake( mutex, portMAX_DELAY );
// 	//
// 	// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_command( ssd1353, SSD1353_SET_COLUMN_ADDRESS, (uint8_t[ ]){ x1, x1 + w - 1 }, 2 ), TAG, "failed to set column address" );
// 	// 	ESP_RETURN_ON_ERROR( ssd1353_display_write_command( ssd1353, SSD1353_SET_ROW_ADDRESS, (uint8_t[ ]){ y1, y1 + w - 1 }, 2 ), TAG, "failed to set row address" );
// 	// 	// ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_param(io, 0x15, (uint8_t[]){x_start, x_end-1}, 2), TAG, "send command failed" );
// 	// 	// ESP_RETURN_ON_ERROR( esp_lcd_panel_io_tx_param(io, 0x75, (uint8_t[]){y_start, y_end-1}, 2), TAG, "send command failed" );
// 	//
// 	//
// 	// 	// ssd1353_display_set_address( spi, x1, y1, x2, y2 );
// 	// 	// ssd1353_display_write_data( spi, buffer, size );
// 	//
// 	// 	esp_err_t ret = ESP_OK;
// 	// 	ssd1353_transaction_t* transaction = NULL;
// 	//
// 	// 	ESP_GOTO_ON_ERROR( spi_device_acquire_bus( ssd1353->spi_device, portMAX_DELAY ), err, TAG, "spi_device_acquire_bus" );
// 	//
// 	// 	// before issue a polling transaction, need to wait queued transactions finished
// 	// 	ssd1353_wait_for_queue( ssd1353 );
// 	// 	transaction = &ssd1353->transaction_queue[ 0 ];
// 	// 	memset( transaction, 0, sizeof( ssd1353_transaction_t ) );
// 	//
// 	// 	transaction->dc_level = 0;
// 	// 	transaction->base.length = 8;
// 	// 	transaction->base.tx_buffer = (uint8_t[ ]){ SSD1353_WRITE_RAM_COMMAND };
// 	//
// 	// 	if ( buffer && color_size )
// 	// 	{
// 	// 		transaction->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
// 	// 	}
// 	//
// 	// 	// xSemaphoreGive( mutex );
// 	//
// 	// 	return ret;
// 	//
// 	// err:
// 	// 	spi_device_release_bus( ssd1353->spi_device );
// 	// 	// xSemaphoreGive( mutex );
// 	// 	return ret;
// }
