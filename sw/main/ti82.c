/*
    TI82_Matlab
    Edited 11 January 2022 by Georgij
*/

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_attr.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_types.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sys/param.h"

#include "mcp23s17.h"

// Expander SPI
#define EXPANDER_CS     26 // CONFIG_EXPANDER_CS
#define EXPANDER_SCK    27 // CONFIG_EXPANDER_SCK
#define EXPANDER_DATA   14 // CONFIG_EXPANDER_DATA
// Keyboard
#define KEYBOARD_COL0   36 // CONFIG_KEYBOARD_COL0
#define KEYBOARD_COL1   13 // CONFIG_KEYBOARD_COL1
#define KEYBOARD_COL2   34 // CONFIG_KEYBOARD_COL2 NOTE: PATCHED to GPIO34
#define KEYBOARD_COL3   22 // CONFIG_KEYBOARD_COL3
#define KEYBOARD_COL4   23 // CONFIG_KEYBOARD_COL4
#define KEYBOARD_ROW0   33 // CONFIG_KEYBOARD_ROW0
#define KEYBOARD_ROW1   25 // CONFIG_KEYBOARD_ROW1
#define KEYBIARD_ON     32
// Keyboard Column pin mask
#define KEYBOARD_COLS  ((1ULL<<KEYBOARD_COL0) | (1ULL<<KEYBOARD_COL1) | (1ULL<<KEYBOARD_COL2) | (1ULL<<KEYBOARD_COL3) | (1ULL<<KEYBOARD_COL4))
// Display IO
#define DISPLAY_RST     21 // Reset
#define DISPLAY_CE      19 // Chip Enable
#define DISPLAY_WR      18 // Write
#define DISPLAY_DI      17 // Data/Instruction
#define DISPLAY_STB     16 // Standby
// Display communication pin mask
#define DISPLAY_PINS  ((1ULL<<DISPLAY_RST) | (1ULL<<DISPLAY_CE) | (1ULL<<DISPLAY_WR) | (1ULL<<DISPLAY_DI) | (1ULL<<DISPLAY_STB))

#define BAT_ADC         35      

// static const char *TAG0 = "CORE0";
static const char *TAG1 = "CORE1";

static xQueueHandle col_isr_queue = NULL;
spi_device_handle_t spi_handle;
spi_transaction_t trans_desc;
char data[3];

void expander_spi_init(void);
void mcp23s17_init(void);
void mcp23s17_write_to_reg(char reg_addr, char val);
void t6k04_init(void);
void t6k04_onoff(char onoff);
void t6k04_write(char disp_data);
void t6k04_set_row(char row);

// Pin interrupt (Keyboard)
static void gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(col_isr_queue, &gpio_num, NULL);
    // Fixing bug for multiple presses by disabling interrupt temporary
    gpio_intr_disable(KEYBOARD_COL0);
    gpio_intr_disable(KEYBOARD_COL1);
    gpio_intr_disable(KEYBOARD_COL2);
    gpio_intr_disable(KEYBOARD_COL3);
    gpio_intr_disable(KEYBOARD_COL4);
}

void keyboard_isr_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = KEYBOARD_COLS,
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(KEYBOARD_COL0, gpio_isr_handler, (void*) KEYBOARD_COL0);
    gpio_isr_handler_add(KEYBOARD_COL1, gpio_isr_handler, (void*) KEYBOARD_COL1);
    gpio_isr_handler_add(KEYBOARD_COL2, gpio_isr_handler, (void*) KEYBOARD_COL2);
    gpio_isr_handler_add(KEYBOARD_COL3, gpio_isr_handler, (void*) KEYBOARD_COL3);
    gpio_isr_handler_add(KEYBOARD_COL4, gpio_isr_handler, (void*) KEYBOARD_COL4);
}

void expander_spi_init(void)
{
    ESP_LOGI(TAG1, "Setup Expander SPI.");

    // SPI bus configuration
    spi_bus_config_t spi_config = {
        .sclk_io_num = EXPANDER_SCK,  // CLK
        .mosi_io_num = EXPANDER_DATA, // MOSI
        .miso_io_num = -1,        // Not used
        .quadwp_io_num = -1,      // Not used
        .quadhd_io_num = -1,      // Not used
        .max_transfer_sz = 4094,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    // Device configuration
    spi_device_interface_config_t dev_config = {
        .address_bits = 0,
        .command_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_posttrans = 0,
        .cs_ena_pretrans = 0,
        .clock_speed_hz = CONFIG_DEVICE_SPI_CLOCK,
        .spics_io_num = EXPANDER_CS,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_config, 1));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &spi_handle));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle, portMAX_DELAY));

    trans_desc.addr = 0;
    trans_desc.cmd = 0;
    trans_desc.flags = 0;
    trans_desc.length = 24;
    trans_desc.rxlength = 0;
    trans_desc.tx_buffer = data;
    trans_desc.rx_buffer = 0;

    ESP_LOGI(TAG1, "Done confgiguring Expander SPI.");
}

void keyboard_driver( void * pvParameters )
{
    // Configure row output pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << KEYBOARD_ROW0) | (1ULL << KEYBOARD_ROW1),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    char row = 0;
    ESP_LOGI(TAG1, "Start keyboard_driver");
    while (true) {
        if (row >= 2) {
            gpio_set_level(KEYBOARD_ROW1, 0);
            mcp23s17_write_to_reg(GPIOA, (char) (1 << (row-2)));
        }
        else if (row == 1) {
            gpio_set_level(KEYBOARD_ROW0, 0);
            gpio_set_level(KEYBOARD_ROW1, 1);
        }
        else if (row == 0) {
            mcp23s17_write_to_reg(GPIOA, 0x00);
            gpio_set_level(KEYBOARD_ROW0, 1);
        }
        
        if (row == 9)
            row = 0;
        else 
            row++;
        vTaskDelay(10 / portTICK_RATE_MS); // TODO: adjust
    }
}

void keyboard_press(void * pvParameters){
    ESP_LOGI(TAG1, "Start keyboard_press");
    uint32_t io_num;
    while (true) {
        if(xQueueReceive(col_isr_queue, &io_num, portMAX_DELAY)) {

            switch((int) io_num)
            {
                case KEYBOARD_COL0:
                    ESP_LOGI(TAG1, "COL0");
                    break;

                case KEYBOARD_COL1:
                    ESP_LOGI(TAG1, "COL1");
                    break;

                case KEYBOARD_COL2:
                    ESP_LOGI(TAG1, "COL2");
                    break;

                case KEYBOARD_COL3:
                    ESP_LOGI(TAG1, "COL3");
                    break;

                case KEYBOARD_COL4:
                    ESP_LOGI(TAG1, "COL4");
                    break;

                default:
                    ESP_LOGI(TAG1, "Error: Column undefined.");
            }
            // Fixing bug for multiple presses by disabling interrupt temporary
            vTaskDelay(10 / portTICK_RATE_MS);
            gpio_intr_enable(KEYBOARD_COL0);
            gpio_intr_enable(KEYBOARD_COL1);
            gpio_intr_enable(KEYBOARD_COL2);
            gpio_intr_enable(KEYBOARD_COL3);
            gpio_intr_enable(KEYBOARD_COL4);
        }
    }
}

void display_driver(void * pvParameters)
{
    t6k04_init();
    char row = 32;
    t6k04_set_row(row);
    while(true)
    {
        ESP_LOGI(TAG1, "LOL");
        t6k04_write(0xaa);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


/** @brief	Initialise the display comminication.
 */
void t6k04_init(void)
{
    // Configure display communication pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = DISPLAY_PINS,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    gpio_set_level(DISPLAY_RST, 1);
    gpio_set_level(DISPLAY_STB, 1);
    gpio_set_level(DISPLAY_CE, 0);

    t6k04_onoff(1);
    // SCE:
    // Contrast (0 to 63)
    mcp23s17_write_to_reg(GPIOB, (1 << 7) | (1 << 6) | 32);
    gpio_set_level(DISPLAY_CE, 0);
    gpio_set_level(DISPLAY_DI, 0);
    gpio_set_level(DISPLAY_WR, 0);
    vTaskDelay(10 / portTICK_RATE_MS);  // TODO: Change to reasonable time
    gpio_set_level(DISPLAY_CE, 1);

    // UDE:
    // DB2 = 1
    // Counter Select (DB1): X = 0
    // Mode Select (DB0): Down = 0
    mcp23s17_write_to_reg(GPIOB, (1 << 2) | (0 << 1) | (0 << 0));
    gpio_set_level(DISPLAY_CE, 0);
    gpio_set_level(DISPLAY_DI, 0);
    gpio_set_level(DISPLAY_WR, 0);
    vTaskDelay(10 / portTICK_RATE_MS);  // TODO: Change to reasonable time
    gpio_set_level(DISPLAY_CE, 1);
}

/** @brief	Turn on/off T6K04 display. (DPE)
 *	@param	onoff ON = 1 , OFF = 0.
 */
void t6k04_onoff(char onoff)
{
    mcp23s17_write_to_reg(GPIOB, (1 << 1) | (onoff << 0));
    gpio_set_level(DISPLAY_CE, 0);
    gpio_set_level(DISPLAY_DI, 0);
    gpio_set_level(DISPLAY_WR, 0);
    vTaskDelay(10 / portTICK_RATE_MS);  // TODO: Change to reasonable time
    gpio_set_level(DISPLAY_CE, 1);
}

/** @brief	Write to T6K04 display. (DAWR)
 *	@param	disp_data Data.
 */
void t6k04_write(char disp_data)
{
    mcp23s17_write_to_reg(GPIOB, disp_data);
    gpio_set_level(DISPLAY_CE, 0);
    gpio_set_level(DISPLAY_DI, 1);
    gpio_set_level(DISPLAY_WR, 0);
    vTaskDelay(10 / portTICK_RATE_MS);  // TODO: Change to reasonable time
    gpio_set_level(DISPLAY_CE, 1);
}

/** @brief	Set T6K04 display's row. (SXE)
 *	@param	row Row (Range: 0 to 63).
 */
void t6k04_set_row(char row)
{
    mcp23s17_write_to_reg(GPIOB, (1 << 7) | row);
    gpio_set_level(DISPLAY_CE, 0);
    gpio_set_level(DISPLAY_DI, 0);
    gpio_set_level(DISPLAY_WR, 0);
    vTaskDelay(10 / portTICK_RATE_MS);  // TODO: Change to reasonable time
    gpio_set_level(DISPLAY_CE, 1);
}

/** @brief	Initializes MCP23S17 by reseting and
 *			setting 16 IO pins as output.
 */
void mcp23s17_init(void)
{	
    /* MCP23S17's OP code and write */
    data[0] = (char) (MCP_OPCODE | MCP_WRITE);
    /* Set all of the MCP23S17's PORTA (GPA) pins as output */
    mcp23s17_write_to_reg(IODIRA, 0x00);

	/* Set all of the MCP23S17's PORTB (GPB) pins as output */
    mcp23s17_write_to_reg(IODIRB, 0x00);
	
	/* Reset output on port A */
    mcp23s17_write_to_reg(GPIOA, 0x00);
	
	/* Reset output on port B */
    mcp23s17_write_to_reg(GPIOB, 0x00);
}

/** @brief	Writes in to MCP23S17's register.
 *	@param	reg_addr Writes to selected register.
 *	@param	val	The byte written in to register.
 */
void mcp23s17_write_to_reg(char reg_addr, char val)
{
    data[1] = reg_addr;
    data[2] = val;
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &trans_desc, portMAX_DELAY));
}

void app_main(void)
{
    expander_spi_init();
    mcp23s17_init();
    keyboard_isr_init();
    col_isr_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreatePinnedToCore(keyboard_driver, "keyboard_driver", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(keyboard_press, "keyboard_press", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(display_driver, "display_driver", 4096, NULL, 10, NULL, 1);
    // TODO: WiFi task;
}
