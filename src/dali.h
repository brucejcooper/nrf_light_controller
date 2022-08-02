#ifndef __DALI_H__
#define __DALI_H__

#include <inttypes.h>
#include <strings.h>

#define DALI_INPUT_PIN 4
#define DALI_OUTPUT_PIN 3
#define BIT_WIDTH_US 833
#define HALF_BIT_WIDTH_US (BIT_WIDTH_US/2) 


nrfx_err_t dali_read_init();
nrfx_err_t dali_write_init();

void dali_abort_write();
void dali_send(uint32_t output, size_t numBits);

#endif