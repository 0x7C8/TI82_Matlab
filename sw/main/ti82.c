/*
    TI82_Matlab
    Edited 17 January 2022 by Georgij
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
#include "pcb.h"
#include "ti82_fonts.h"
#include "keymap.h"

#define EXPANDER_WRITE_DELAY_US 100 // TODO: Optimize this

// static const char *TAG0 = "CORE0";
static const char *TAG1 = "CORE1";

static xQueueHandle col_isr_queue = NULL;
spi_device_handle_t spi_handle;
spi_transaction_t trans_desc;
char data[3];   // Data that gets sent to Expander via PSI
int keyboard_row, keyboard_col = 0;
int display_res[2] = {96, 64};
char* msg;
size_t msg_len;
int key_state = 0;  // TODO: implement key state switch in keyboard_press

void expander_spi_init(void);
void mcp23s17_init(void);
void mcp23s17_write_to_reg(char reg_addr, char val);
void display_init(void);
void display_word(int letter_index, int letter_row);
// Display instructions
void display_word_length(char word_length);
void display_onoff(char onoff);
void display_counter(char xy, char updown);
void display_opamp_level(char level);
void display_opamp_strength(char duty);
void display_set_y(char y);
void display_set_z(char z);
void display_set_x(char x);
void display_contrast(char contrast);

void display_data(char disp_data);
void display_instruction(char disp_instr);
void display_clear(void);


/** @brief	External interrupt handler.
 */
static void gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(col_isr_queue, &gpio_num, NULL);
    // Fixing bug for multiple presses by temporary disabling interrupt
    gpio_intr_disable(KEYBOARD_COL0);
    gpio_intr_disable(KEYBOARD_COL1);
    gpio_intr_disable(KEYBOARD_COL2);
    gpio_intr_disable(KEYBOARD_COL3);
    gpio_intr_disable(KEYBOARD_COL4);
}

/** @brief	Initialize external interrupt GPIOs.
 */
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

/** @brief	Initialize communication with expander through SPI.
 */
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

/** @brief	Task: Row shift register.
 *  Only 1 row is active at the given moment.
 */
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

    ESP_LOGI(TAG1, "Start keyboard_driver");
    int row = 0;
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
        keyboard_row = row;
        vTaskDelay(10 / portTICK_RATE_MS);
        if (row == 9)
            row = 0;
        else 
            row++;
    }
}

// TODO: has to send char to memory. Find char in row x column map. 
/** @brief	Task: Handles keyboard button press.
 */
void keyboard_press(void * pvParameters){
    ESP_LOGI(TAG1, "Start keyboard_press");
    uint32_t io_num;
    while (true) {
        if (uxQueueMessagesWaiting(col_isr_queue) > 0) {
            xQueueReceive(col_isr_queue, &io_num, portMAX_DELAY);

            switch((int) io_num)
            {
                case KEYBOARD_COL0:
                    ESP_LOGI(TAG1, "COL0");
                    keyboard_col = 0;
                    break;

                case KEYBOARD_COL1:
                    ESP_LOGI(TAG1, "COL1");
                    keyboard_col = 1;
                    break;

                case KEYBOARD_COL2:
                    ESP_LOGI(TAG1, "COL2");
                    keyboard_col = 2;
                    break;

                case KEYBOARD_COL3:
                    ESP_LOGI(TAG1, "COL3");
                    keyboard_col = 3;
                    break;

                case KEYBOARD_COL4:
                    ESP_LOGI(TAG1, "COL4");
                    keyboard_col = 4;
                    break;

                default:
                    ESP_LOGI(TAG1, "Error: Column undefined.");
            }

            msg_len++;
            char *msg2;
            msg2 = (char *)realloc(msg, msg_len*sizeof(char));
            msg = msg2;
            msg[msg_len-2] = keymap1[keyboard_row][keyboard_col];
            msg[msg_len-1] = 0;

            ESP_LOGI(TAG1, "msg = %s", msg);

            // Fixing bug for multiple presses by disabling interrupt temporary
            vTaskDelay(100 / portTICK_RATE_MS);
            gpio_intr_enable(KEYBOARD_COL0);
            gpio_intr_enable(KEYBOARD_COL1);
            gpio_intr_enable(KEYBOARD_COL2);
            gpio_intr_enable(KEYBOARD_COL3);
            gpio_intr_enable(KEYBOARD_COL4);
        }
        else {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}



/** @brief	Task: Display driver.
 */
void display_driver(void * pvParameters)
{
    ESP_LOGI(TAG1, "Starting display driver.");
    display_init();
    display_clear();
    char row = 4;
    while(true)
    {
        int msg_size = strlen(msg); // TODO: remove this
        display_contrast(54);
        for (int xad = 0; xad <= 7; xad++){
            for(int page = 0; page <= msg_size-1; page++){
                display_counter(1, 1);
                display_set_x(row+xad);
                display_counter(0, 1);
                display_set_y(page);
                display_word((int) msg[page], xad);
            }
        }
        vTaskDelay(100 / portTICK_RATE_MS); // Added to avoid triggering task watchdog
    }
}

/** @brief	Convert part of a message to 6 bits bitmap and send to display
 */
void display_word(int letter_index, int letter_row)
{   
    char word = 0x00;
    for (int letter_col = 0; letter_col < 5; letter_col++){
        word |= (((font[5*(letter_index-32)+letter_col] & (1 << letter_row)) > 0) ? 1 : 0) << (4-letter_col);
    }
    display_data(word);
}

/** @brief	Initialize communication with display.
 */
void display_init(void)
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

    display_word_length(0);
    display_counter(0, 1);
    display_set_z(0);
    display_onoff(1);
    display_opamp_level(7);
    display_opamp_strength(3);
    display_contrast(54);
}

/** @brief	Word Length. (86E)
 *	@param	word_length 8 bits = 1, 6 bits = 0
 */
void display_word_length(char word_length)
{
    display_instruction((char) (word_length << 0));
}

/** @brief	Turn on/off display. (DPE)
 *	@param	onoff ON = 1 , OFF = 0
 */
void display_onoff(char onoff)
{
    display_instruction((char) ((1 << 1) | (onoff << 0)));
}

/** @brief	Display counter and up/down mode. (UDE)
 *	@param	xy      X = 0 , Y = 1
 *  @param	updown  Down = 0 , Up = 1
 */
void display_counter(char xy, char updown)
{
    display_instruction((char) ((1 << 2) | (xy << 1) | (updown << 0)));
}

/** @brief	Power supply level of opamp. (OPA1)
 *	@param	level Range: 4 to 7
 */
void display_opamp_level(char level)
{
    display_instruction((char) ((1 << 4) | level));
}

/** @brief	Power supply strength of opamp. (OPA2)
 *	@param	duty Range: 0 to 3
 */
void display_opamp_strength(char duty)
{
    display_instruction((char) ((1 << 3) | duty));
}

/** @brief	Set display's Y-address. (SYE)
 *	@param	y Y-address (Range: 0 to 21).
 */
void display_set_y(char y)
{
    display_instruction((char) ((1 << 5) | y));
}

/** @brief	Set display's Z-address. (SZE)
 *	@param	z Z-address (Range: 0 to 63).
 */
void display_set_z(char z)
{
    display_instruction((char) ((1 << 6) | z));
}

/** @brief	Set display's X-address. (SXE)
 *	@param	x X-address (Range: 0 to 63).
 */
void display_set_x(char x)
{
    display_instruction((char) ((1 << 7) | x));
}

/** @brief	Set display's contrast. (SCE)
 *	@param	contrast Contrast (0 to 63)
 */
void display_contrast(char contrast)
{
    display_instruction((char) ((1 << 7) | (1 << 6) | contrast));
}

/** @brief	Send instruction to T6K04 display.
 *	@param	disp_instr Instruction.
 */
void display_instruction(char disp_instr)
{
    gpio_set_level(DISPLAY_DI, 0);
    mcp23s17_write_to_reg(GPIOB, disp_instr);
    gpio_set_level(DISPLAY_WR, 0);
    ets_delay_us(1);
    gpio_set_level(DISPLAY_CE, 0);
    ets_delay_us(1);
    gpio_set_level(DISPLAY_CE, 1);
}

/** @brief	Write data to T6K04 display. (DAWR)
 *	@param	disp_data Data.
 */
void display_data(char disp_data)
{
    gpio_set_level(DISPLAY_DI, 1);
    mcp23s17_write_to_reg(GPIOB, disp_data);
    gpio_set_level(DISPLAY_WR, 0);
    ets_delay_us(1);
    gpio_set_level(DISPLAY_CE, 0);
    ets_delay_us(1);
    gpio_set_level(DISPLAY_CE, 1);
}

/** @brief Clears display.
 */
void display_clear(void)
{
    for (int xad = 0; xad <= display_res[1]; xad++) {
        for (int page = 0; page < 22; page++){
            display_counter(1, 1);
            display_set_x(xad);
            display_counter(0, 1);
            display_set_y(page);
            display_data(0x00);
        }
    }
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
    ets_delay_us(EXPANDER_WRITE_DELAY_US);
}

void app_main(void)
{
    msg_len = 2;
    msg = (char *) malloc(msg_len);
    snprintf(msg, msg_len, "_");

    expander_spi_init();
    mcp23s17_init();
    keyboard_isr_init();
    col_isr_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreatePinnedToCore(keyboard_driver, "keyboard_driver", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(keyboard_press, "keyboard_press", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(display_driver, "display_driver", 4096, NULL, 10, NULL, 1);
    // TODO: WiFi task;
}
