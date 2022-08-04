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

LOG_MODULE_REGISTER(dali, CONFIG_DALI_LOG_LEVEL);


#define NRFX_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        __ASSERT((LOCAL_ERR_CODE) == NRFX_SUCCESS, "Call at %s:%d returned %d", (const char*) __FILE__, __LINE__, LOCAL_ERR_CODE); \
    } while (0)


#define PULSEWIDTH_TOLERANCE 20
#define DISABLE_TIMEOUT UINT32_MAX-1



typedef enum pulsewidth_type {
	PW_INVALID,
	PW_SHORT,
	PW_LONG,
	PW_TIMEOUT,
} pulsewidth_type_t;

typedef struct input_state_t *(state_callback_t)(enum pulsewidth_type pulse_width);


struct input_state_t {
	state_callback_t *handler;
	uint32_t timeout;
};



nrfx_gpiote_in_config_t inputGpioteConfig = {
	.sense = NRF_GPIOTE_POLARITY_TOGGLE,
	.pull = NRF_GPIO_PIN_PULLUP,
	.is_watcher = false,
	.hi_accuracy = true,
	.skip_gpio_setup = false,
};


static nrfx_timer_t timer = NRFX_TIMER_INSTANCE(2);
static nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
struct input_state_t;
static volatile uint32_t lastInputToggle;
// We start in the ERROR state, until we see an idle line for at least x Usec. 
static volatile struct input_state_t *currentInputState = NULL;
static uint32_t inputShiftReg = 0;
static size_t inputBitCount = 0;
static uint32_t lastInputBit;
static nrf_ppi_channel_t inputTogglePPIchannel;



static struct input_state_t *idle_handleTransition(pulsewidth_type_t pulse_width);
static struct input_state_t *inStartBit_handleTransition(pulsewidth_type_t pulse_width);
static struct input_state_t *inResettingCurrentlyLow_handleTransition(pulsewidth_type_t pulse_width);
static struct input_state_t *inResettingCurrentlyHigh_handleTransition(pulsewidth_type_t pulse_width);
static struct input_state_t *inHalfBit_handleTransition(pulsewidth_type_t pulse_width);
static struct input_state_t *atStartOfBit_handleTransition(pulsewidth_type_t pulse_width);

static struct input_state_t idleState = {
	.handler = idle_handleTransition,
	.timeout = DISABLE_TIMEOUT, // We never timeout.
};

static struct input_state_t inStartBitState = {
	.handler = inStartBit_handleTransition,
	.timeout = HALF_BIT_WIDTH_US*3/2,
};

static struct input_state_t resettingCurrentlyLowState = {
	.handler = inResettingCurrentlyLow_handleTransition,
	.timeout = BIT_WIDTH_US*2, // After this time, we transition back out
};

static struct input_state_t resettingCurrentlyHighState = {
	.handler = inResettingCurrentlyHigh_handleTransition,
	.timeout = BIT_WIDTH_US*2, // If its held high for this long, then that is bad.
};

static struct input_state_t inHalfBitState = {
	.handler = inHalfBit_handleTransition,
	.timeout = BIT_WIDTH_US*3/2,
};

static struct input_state_t atStartOfBitState = {
	.handler = atStartOfBit_handleTransition,
	.timeout = BIT_WIDTH_US*2,
};



static void processInput() {
	uint32_t value = inputShiftReg;
	uint32_t numBits = inputBitCount;

	LOG_INF("Received 0x%02x, size %d", value, numBits);

	inputShiftReg = 0;
	inputBitCount = 0;
}

static inline void pushBit(uint32_t value) {
	inputShiftReg = (inputShiftReg << 1) | (value ? 1 : 0);
	inputBitCount++;
}

static struct input_state_t *getResetState() {
	uint32_t currentVal = nrf_gpio_pin_read(DALI_INPUT_PIN);

	// Which reset state we go into depends on which level we are currently at
	return currentVal ? &resettingCurrentlyHighState : &resettingCurrentlyLowState ;
}

static struct input_state_t *cancelTransmitThenGetResetState() {
	dali_abort_write();
	return getResetState();
}



static struct input_state_t *idle_handleTransition(pulsewidth_type_t pulse_width) {
	return &inStartBitState;
}


static struct input_state_t *inStartBit_handleTransition(pulsewidth_type_t pulse_width) {
	if (pulse_width != PW_SHORT) {
		// Wasn't a valid first half of a start bit, but we were high
		return &resettingCurrentlyHighState;
	} else {
		lastInputBit = 1;
		inputShiftReg = 0;
		inputBitCount = 0;
		return &inHalfBitState;
	}
}


static struct input_state_t *inResettingCurrentlyLow_handleTransition(pulsewidth_type_t pulse_width) {
	if (pulse_width == PW_TIMEOUT) {
		// We're out of purgatory, and now back in idle.
		return &idleState;
	}
	
	// If we get here, we've gone back high again, which is bad
	return getResetState();
}


static struct input_state_t *inResettingCurrentlyHigh_handleTransition(pulsewidth_type_t pulse_width) {
	// A timeout here indicates the bus has been held for too long a time.  
	// Somebody has done something illegal. 
	if (pulse_width == PW_TIMEOUT) {
		// Double check it isn't us by releasing the output

		nrf_gpio_pin_clear(DALI_OUTPUT_PIN);
		LOG_ERR("Resetting output, as its stuck in a shorted state.");
		// But we stay in the current state.
		return getResetState();
	}
	// Otherwise, we're returning back to reset low
	return &resettingCurrentlyLowState;
}



static struct input_state_t *inHalfBit_handleTransition(pulsewidth_type_t pulse_width) {
	switch (pulse_width) {
		case PW_SHORT:
			// short pulse takes us to the start of the next bit, and the next pulse _must_ also be a short, 
			// and it will be the same bit as we just had.
			return &atStartOfBitState;

		case PW_LONG:
			// It was a long pulse, which means a new bit of the opposite type, but afterwards we're still in a half bit.
			lastInputBit = lastInputBit ? 0 : 1;
			pushBit(lastInputBit);
			return &inHalfBitState;

		case PW_TIMEOUT:
			if (lastInputBit) {
				// The line is now idle, and has been for 2.5 bit lengths.  That mean's we're done with the transfer.
				processInput();
				return &idleState;
			} else {
				// We've been stuck low for too long.  This is bad.
				return cancelTransmitThenGetResetState();
			}
			break;

		default:
			// All others are invlaid, so we reset. 
			return cancelTransmitThenGetResetState();
	}
}


static struct input_state_t *atStartOfBit_handleTransition(pulsewidth_type_t pulse_width) {
	// After a short pulse (from a half bit), the only valid transition is another short pulse, 
	// unless we are ending (had a timeout and the last bit was a 0) 
	switch (pulse_width) {
		case PW_SHORT:
			// We're done processing the last bit, and have started the next one (but its the same bit orientation than the last one.)
			pushBit(lastInputBit);
			return &inHalfBitState;

		case PW_TIMEOUT:
			if (!lastInputBit) {
				// We've received 2 stop bits, and the line is idle, so its an end to the receive..
				processInput();
				return &idleState;
			} 
			return cancelTransmitThenGetResetState();

		default:
			// all other transitions are invalid.
			return cancelTransmitThenGetResetState();
	}	
}




pulsewidth_type_t getPulseWidthType(uint32_t pulseWidth) {
	if (pulseWidth < HALF_BIT_WIDTH_US-PULSEWIDTH_TOLERANCE) {
		return PW_INVALID;
	} else if (pulseWidth < HALF_BIT_WIDTH_US+PULSEWIDTH_TOLERANCE) {
		return PW_SHORT;
	} else if (pulseWidth < BIT_WIDTH_US-PULSEWIDTH_TOLERANCE) {
		return PW_INVALID;
	} else if (pulseWidth < BIT_WIDTH_US+PULSEWIDTH_TOLERANCE) {
		return PW_LONG;
	} else {
		return PW_INVALID;
	}
}


void processStateMachine(pulsewidth_type_t pulse_width, uint32_t ticks) {
	struct input_state_t *newState = currentInputState->handler(pulse_width);

	// In some modes (IDLE), we don't have a timeout
	if (newState != currentInputState) {
		nrf_timer_cc_set(timer.p_reg, NRF_TIMER_CC_CHANNEL1, newState->timeout);
		if (currentInputState->timeout == DISABLE_TIMEOUT && newState->timeout != DISABLE_TIMEOUT) {
			// Switch our PPI second task to do a capture upon next toggle. 
	        nrf_ppi_fork_endpoint_setup(NRF_PPI, inputTogglePPIchannel, nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_CAPTURE0));
		} else if (currentInputState->timeout != DISABLE_TIMEOUT && newState->timeout == DISABLE_TIMEOUT) {
			nrfx_timer_pause(&timer);
			nrfx_timer_clear(&timer);

			// Switch our PPI second task to start the timer upon next toggle
	        nrf_ppi_fork_endpoint_setup(NRF_PPI, inputTogglePPIchannel, nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_START));
		}		
	}
	currentInputState = (struct input_state_t *) newState;
}

/**
 * @brief interrupt callback for level changes on the input pin
 * 
 * @param pin 
 * @param _action 
 */
void inputChanged(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t _action) {
	// Each time our input level changes, we use our state machine to work out what happens next.
	if (pin == DALI_INPUT_PIN) {
		uint32_t cc = nrfx_timer_capture_get(&timer, NRF_TIMER_CC_CHANNEL0);
		uint32_t pulseWidth = cc;
		pulsewidth_type_t pwt = getPulseWidthType(pulseWidth);
		processStateMachine(pwt, cc);
	}
}


/**
 */
static void timer_callback(nrf_timer_event_t event_type, void* p_context) {
	if (event_type == NRF_TIMER_EVENT_COMPARE1) {
		// Called when a timeout occurs. 
		processStateMachine(PW_TIMEOUT, 0);
	}
}



nrfx_err_t dali_read_init() {
	// Turn on GPIOTE
	LOG_DBG("Enabling DALI module");

	// Configure timer
	timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_cfg.mode = NRF_TIMER_MODE_TIMER;

	// For some bizarre reason, nrfx doesn't enable its own IRQ, so we do it for them.
	IRQ_DIRECT_CONNECT(TIMER2_IRQn, 0, nrfx_timer_2_irq_handler, 0);
    NRFX_ERROR_CHECK(nrfx_timer_init(&timer, &timer_cfg, timer_callback));

	// Configure Input for GPIOTE, on either edge
	nrfx_gpiote_in_init(DALI_INPUT_PIN, &inputGpioteConfig, inputChanged);
	nrfx_gpiote_in_event_enable(DALI_INPUT_PIN, true);

	// Set up PPI to capture the current time whenever the input toggles, then clear the timer (ready for the next transition)
	NRFX_ERROR_CHECK(nrfx_ppi_channel_alloc(&inputTogglePPIchannel));
	NRFX_ERROR_CHECK(nrfx_ppi_channel_assign(
						inputTogglePPIchannel,
						nrfx_gpiote_in_event_addr_get(DALI_INPUT_PIN),
						nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_CLEAR)
					));
	NRFX_ERROR_CHECK(nrfx_ppi_channel_fork_assign(inputTogglePPIchannel, nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_CAPTURE0)));
    NRFX_ERROR_CHECK(nrfx_ppi_channel_enable(inputTogglePPIchannel));

	// Also set a timer that goes off if we've stayed in any one value for too long (resetting us back to a different state.)
	nrfx_timer_extended_compare(&timer, NRF_TIMER_CC_CHANNEL1, 3*BIT_WIDTH_US, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
	nrfx_timer_clear(&timer);

	// Our initial state depends on what value the bus is currently at.  It will be one of the reset states, which means we will wait
	// until the bus is idle before startng to read.
	currentInputState = getResetState();
	nrfx_timer_enable(&timer);
	return 0;
}