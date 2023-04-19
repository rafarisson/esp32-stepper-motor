
#include <esp_types.h>

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#include "soc/ledc_struct.h"
#include "esp_log.h"
#include "esp_err.h"

#include "stepper_motor.h"

static const char TAG[] = "APP";

#define UART_CH			UART_NUM_0
#define UART_TX_PIN		1 //2
#define UART_RX_PIN		3
#define UART_BUF_MAX	128

static void app_uart_init(void);

void app_main(void) {
	stepper_motor_t motor = {
		.pcnt_unit = NULL,
		.pcnt_ch = NULL,
		.en_io = 21,
		.dir_io = 23,
		.step_io = 22,
		.num_of_steps = 200,
	};

	motor_init(&motor);
	app_uart_init();

	ESP_LOGI(TAG, "wait cmd..");

	while (1) {
		char rx_buf[UART_BUF_MAX];
		int rx_cnt = uart_read_bytes(UART_CH, rx_buf, UART_BUF_MAX, pdMS_TO_TICKS(100));
		
		if (rx_cnt > 0) {
			int rpm = 0;
			int steps = 0;
			if (2 == sscanf(rx_buf, "steps:%d;rpm:%d", &steps, &rpm)) {
				if (!motor_start(&motor, steps, rpm))
					motor_stop(&motor);
			}
		}
	}
}

static void app_uart_init(void) {
	const uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
    
	ESP_ERROR_CHECK(uart_driver_install(UART_CH, UART_BUF_MAX * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_CH, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_CH, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}