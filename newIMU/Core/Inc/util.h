#ifndef __UTIL_H
#define __UTIL_H

#include "main.h"
extern I2C_HandleTypeDef hi2c1;

int i2c_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf, uint16_t size);
int i2c_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf, uint16_t size);

/**
 * Write num_bits bits to a single byte register at mem_addr.
 */
int i2c_mask_write(uint8_t dev_addr, uint8_t mem_addr, int num_bits, int shift, uint8_t data);

#endif
