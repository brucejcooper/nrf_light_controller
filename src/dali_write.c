#include <zephyr/logging/log.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/openthread.h>
#include <openthread/coap.h>
#include <openthread/srp_client.h>
#include <openthread/ip6.h>
#include <openthread/message.h>
#include <openthread/thread.h>
#include <coap_server_client_interface.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <nrfx_timer.h>
#include <nrfx_pwm.h>
#include "srp_utils.h"
#include "dali.h"

LOG_MODULE_DECLARE(dali, CONFIG_DALI_LOG_LEVEL);


#define NRFX_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        __ASSERT((LOCAL_ERR_CODE) == NRFX_SUCCESS, "Call at %s:%d returned %d", (const char*) __FILE__, __LINE__, LOCAL_ERR_CODE); \
    } while (0)


#define PWM_INVERT 0x8000
#define MAX_DALI_BITS 32


static uint16_t outputSequenceValues[MAX_DALI_BITS+3];
static nrf_pwm_sequence_t outputSequence = {
	.values = { .p_common =  outputSequenceValues },
	.length = 1,
	.repeats = 0,
	.end_delay = 0
};

static nrfx_pwm_t pwm = NRFX_PWM_INSTANCE(1);
static nrfx_pwm_config_t pwmConfig = {
	.output_pins = { DALI_OUTPUT_PIN, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED },
	.irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
	.base_clock = NRF_PWM_CLK_1MHz,
	.count_mode = NRF_PWM_MODE_UP,
	.top_value = 833,
	.step_mode = NRF_PWM_STEP_AUTO,
	.load_mode = NRF_PWM_LOAD_COMMON,
	.skip_gpio_cfg = true,
	.skip_psel_cfg = false
};



static void pwm_callback(nrfx_pwm_evt_type_t event_type, void *p_context) {
	if (event_type == NRFX_PWM_EVT_FINISHED) {
		LOG_DBG("PWM send finished");
	}
}


nrfx_err_t dali_write_init() {
	// Turn on GPIOTE
	LOG_DBG("Enabling DALI module");


	// We initialise the pin ourselves, as we need to set it to a different drive level
	nrf_gpio_pin_write(DALI_OUTPUT_PIN, 0);
    nrf_gpio_cfg(
        DALI_OUTPUT_PIN,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0H1,
        NRF_GPIO_PIN_NOSENSE);


	// NRFX is not setting up the IRQ properly, so we use our own
	IRQ_DIRECT_CONNECT(PWM1_IRQn, 0, nrfx_pwm_1_irq_handler, 0);
	NRFX_ERROR_CHECK(nrfx_pwm_init(&pwm, &pwmConfig, pwm_callback, NULL));

	return 0;
}


void dali_abort_write() {
	nrfx_pwm_stop(&pwm, true);	
}



void dali_send(uint32_t output, size_t numBits) {
	int bitPos = 0;

	__ASSERT(numBits <= MAX_DALI_BITS, "Attempt to send more than 32 bits via dali");

	// First bit is a start bit (a logical 1)
	outputSequenceValues[bitPos++] = PWM_INVERT | HALF_BIT_WIDTH_US;

	// Loop through each of the bits, starting with the MSB
	for (uint32_t bitMask = 1 << (numBits-1); bitMask != 0; bitMask >>=1) {
		// We always have the same duty, but we flip the polarity depending on if the bit is set or not.
		outputSequenceValues[bitPos++] = ((output & bitMask) ? 0x8000 : 0) | HALF_BIT_WIDTH_US;
	}
	// Add the stop bits (2 bit periods where it is just high)
	outputSequenceValues[bitPos++] = PWM_INVERT;
	outputSequenceValues[bitPos++] = PWM_INVERT;
	outputSequence.length = bitPos;

	nrfx_pwm_simple_playback(&pwm, &outputSequence, 1, 0);
}