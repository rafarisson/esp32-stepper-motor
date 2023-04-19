#include "soc/gpio_sig_map.h"
#include "esp32/rom/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "stepper_motor.h"

#define LEDC_SPEED_MODE		LEDC_HIGH_SPEED_MODE
#define LEDC_CH				LEDC_CHANNEL_0
#define LEDC_TMR			LEDC_TIMER_0
#define LEDC_INIT_BITS		LEDC_TIMER_7_BIT
#define LEDC_DUTY			((1ULL << LEDC_INIT_BITS) / 2 - 1)
#define LEDC_INIT_FREQ		1000

#define PCNT_MAX			10000
#define PCNT_MIN			-10000

static const char TAG[] = "MOTOR";

static bool motor_pcnt_watch_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

void motor_init(stepper_motor_t *motor) {
	if (!motor)
		return;

	const gpio_config_t gpio_dir = {
		.pin_bit_mask = ((1LL << motor->dir_io) | (1LL << motor->en_io)),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&gpio_dir));
	ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)motor->en_io, !MOTOR_EN_LEVEL));

	const ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_SPEED_MODE,
        .timer_num        = LEDC_TMR,
        .duty_resolution  = LEDC_INIT_BITS,
        .freq_hz          = LEDC_INIT_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_SPEED_MODE,
        .channel        = LEDC_CH,
        .timer_sel      = LEDC_TMR,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = motor->step_io,
        .duty           = 0,
        .hpoint         = 0
    };
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

	const pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = PCNT_MAX,
        .low_limit = PCNT_MIN,
    };
	ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &motor->pcnt_unit));

	const pcnt_chan_config_t pcnt_chan_0_config = {
        .edge_gpio_num = motor->step_io,
        .level_gpio_num = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(motor->pcnt_unit, &pcnt_chan_0_config, &motor->pcnt_ch));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor->pcnt_ch, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor->pcnt_ch, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

	const pcnt_event_callbacks_t pcnt_cbs = {
        .on_reach = motor_pcnt_watch_cb,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(motor->pcnt_unit, &pcnt_cbs, motor));
	
	// Necessário para conectar o mesmo pino como saída e entrada
	// Sendo saída no LEDC e entrada no PCNT
	// TODO: alterar as constantes LEDC_HS_SIG_OUT0_IDX e PCNT_SIG_CH0_IN0_IDX
	// para valores correspondentes a outros canais LEDC/PCNT
	gpio_set_direction((gpio_num_t)motor->step_io, GPIO_MODE_INPUT_OUTPUT);
	gpio_matrix_out(motor->step_io, LEDC_HS_SIG_OUT0_IDX + LEDC_CH, 0, 0);
    gpio_matrix_in(motor->step_io, PCNT_SIG_CH0_IN0_IDX, 0);
}

void motor_stop(stepper_motor_t *motor) {
	if (!motor)
		return;
	
	ledc_stop(LEDC_SPEED_MODE, LEDC_CH, 0);
	pcnt_unit_remove_watch_point(motor->pcnt_unit, motor->steps);
	pcnt_unit_stop(motor->pcnt_unit);
	pcnt_unit_disable(motor->pcnt_unit);
	
	ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)motor->en_io, !MOTOR_EN_LEVEL));

	motor->steps = 0;
	motor->steps_left = 0;
}

bool motor_start(stepper_motor_t *motor, int steps, int rpm) {
	if (!motor || steps == 0 || rpm == 0)
		return false;

	int freq = rpm * motor->num_of_steps / 60;
	if (freq <= 0) {
		ESP_LOGE(TAG, "invalid freq %d (rpm %d)", freq, rpm);
		return false;
	}

	const int direction = steps >= 0;

	if (motor->steps > 0) 
		motor_stop(motor);

	motor->steps_left = steps >= 0 ? steps : (-1 * steps);
	motor->steps = motor->steps_left;
    if (motor->steps > PCNT_MAX) {
        motor->steps = PCNT_MAX - 1;
    }

	ESP_LOGI(TAG, "start direction %d rpm %d freq %d steps %d/%d", direction, rpm, freq, motor->steps, motor->steps_left);

	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor->pcnt_unit, motor->steps));
	ESP_ERROR_CHECK(pcnt_unit_enable(motor->pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_start(motor->pcnt_unit));
	
	ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)motor->dir_io, direction));
	ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)motor->en_io, MOTOR_EN_LEVEL));

	ESP_ERROR_CHECK(ledc_set_freq(LEDC_SPEED_MODE, LEDC_TMR, freq));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_SPEED_MODE, LEDC_CH, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, LEDC_CH));

	return true;
}

static bool motor_pcnt_watch_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
	BaseType_t high_task_wakeup = pdFALSE;
	stepper_motor_t *motor = (stepper_motor_t*)user_ctx;

	if (!motor || motor->pcnt_unit != unit) {
		ESP_EARLY_LOGE(TAG, "pcnt unit %p is invalid (%p)", unit, motor->pcnt_unit);
		return false;
	}

	motor->steps_left -= edata->watch_point_value;
	ESP_EARLY_LOGI(TAG, "pcnt steps %d/%d", edata->watch_point_value, motor->steps_left);

	if (motor->steps_left <= 0) {
		motor_stop(motor);
	}
	else {
		ESP_ERROR_CHECK(pcnt_unit_remove_watch_point(unit, motor->steps));
		motor->steps = motor->steps_left >= PCNT_MAX ? PCNT_MAX - 1 : motor->steps_left;
		ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, motor->steps));
		ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
		ESP_EARLY_LOGI(TAG, "next steps %d", motor->steps);
	}
	
	return high_task_wakeup == pdTRUE;
}
