#ifndef __DALI_H__
#define __DALI_H__

#include <inttypes.h>
#include <strings.h>

nrfx_err_t dali_init();
void dali_send(uint32_t output, size_t numBits);

#endif