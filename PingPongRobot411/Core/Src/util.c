#include "util.h"

int i2c_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf, uint16_t size) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c3, dev_addr << 1, mem_addr, 1, buf, size, 100);

	return status == HAL_OK;
}

int i2c_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf, uint16_t size) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, dev_addr << 1, mem_addr, 1, buf, size, 100);

	return status == HAL_OK;
}

int i2c_mask_write(uint8_t dev_addr, uint8_t mem_addr, int num_bits, int shift, uint8_t data) {
	uint8_t buf[1];
	uint8_t mask = (~(0xff << num_bits)) << shift;

	int err = i2c_read(dev_addr, mem_addr, buf, 1);

	if (err) {
		return 1;
	}

	*buf &= ~mask;
	*buf |= (data << shift);

	err = i2c_write(dev_addr, mem_addr, buf, 1);
	return err;
}
