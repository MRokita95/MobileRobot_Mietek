/*
 * ICM20600.c
 *
 *  Created on: Aug 25, 2022
 *      Author: micha
 */


#include "ICM20600.h"
#include <stdint.h>


/********************************************
 *				PRIVATE DATA				*
 ********************************************/



/********************************************
 *			PRIVATE FUNCTIONS				*
 ********************************************/

int16_t ICM_getRawGyroscopeX(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint8_t _buffer[16] = {0};
    icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_GYRO_XOUT_H, 2, _buffer);
    return ((int16_t)_buffer[0] << 8) + _buffer[1];
}

int16_t ICM_getRawGyroscopeY(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint8_t _buffer[16] = {0};
    icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_GYRO_YOUT_H, 2, _buffer);
    return ((int16_t)_buffer[0] << 8) + _buffer[1];
}

int16_t ICM_getRawGyroscopeZ(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint8_t _buffer[16] = {0};
    icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_GYRO_ZOUT_H, 2, _buffer);
    return ((int16_t)_buffer[0] << 8) + _buffer[1];
}


int16_t ICM_getGyroscopeX(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t raw_data = ICM_getRawGyroscopeX(icmInstance, i2cInstance);
    raw_data = (raw_data * icmInstance->gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t ICM_getGyroscopeY(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t raw_data = ICM_getRawGyroscopeY(icmInstance, i2cInstance);
    raw_data = (raw_data * icmInstance->gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t ICM_getGyroscopeZ(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t raw_data = ICM_getRawGyroscopeZ(icmInstance, i2cInstance);
    raw_data = (raw_data * icmInstance->gyro_scale) >> 16;
    return (int16_t)raw_data;
}


int16_t ICM_getRawAccelerationX(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint8_t _buffer[16] = {0};
    icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_ACCEL_XOUT_H, 2, _buffer);
    return ((int16_t)_buffer[0] << 8) + _buffer[1];
}

int16_t ICM_getRawAccelerationY(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint8_t _buffer[16] = {0};
    icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_ACCEL_YOUT_H, 2, _buffer);
    return ((int16_t)_buffer[0] << 8) + _buffer[1];
}

int16_t ICM_getRawAccelerationZ(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
	uint8_t _buffer[16] = {0};
	icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_ACCEL_ZOUT_H, 2, _buffer);
    return ((int16_t)_buffer[0] << 8) + _buffer[1];
}

int16_t ICM_getAccelerationX(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t raw_data = ICM_getRawAccelerationX(icmInstance, i2cInstance);
    raw_data = (raw_data * icmInstance->acc_scale) >> 16;
    return (int16_t)raw_data;
}
int16_t ICM_getAccelerationY(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t raw_data = ICM_getRawAccelerationY(icmInstance, i2cInstance);
    raw_data = (raw_data * icmInstance->acc_scale) >> 16;
    return (int16_t)raw_data;
}
int16_t ICM_getAccelerationZ(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t raw_data = ICM_getRawAccelerationZ(icmInstance, i2cInstance);
    raw_data = (raw_data * icmInstance->acc_scale) >> 16;
    return (int16_t)raw_data;
}



uint8_t ICM_getDeviceID(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance)
{
	uint8_t _buffer[16] = {0};
	I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_WHO_AM_I, _buffer);
	return _buffer[0];
}
/********************************************
 *			PUBLIC FUNCTIONS				*
 ********************************************/


void ICM_initialize(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance)
{
	// configuration
	icmInstance->status = I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_CONFIG, 0x00);


    // disable fifo
	icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_FIFO_EN, 0x00);

    // set default power mode
    ICM_setPowerMode(icmInstance, i2cInstance, ICM_6AXIS_LOW_POWER);

    // gyro config
    ICM_setGyroScaleRange(icmInstance, i2cInstance, RANGE_2K_DPS);
    ICM_setGyroOutputDataRate(icmInstance, i2cInstance, GYRO_RATE_1K_BW_176);
    ICM_setGyroAverageSample(icmInstance, i2cInstance, GYRO_AVERAGE_1);

    // accel config
    ICM_setAccScaleRange(icmInstance, i2cInstance, RANGE_16G);
    ICM_setAccOutputDataRate(icmInstance, i2cInstance, ACC_RATE_1K_BW_420);
    ICM_setAccAverageSample(icmInstance, i2cInstance, ACC_AVERAGE_4);

    icmInstance->device_ID = ICM_getDeviceID(icmInstance, i2cInstance);
}

void ICM_setPowerMode(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, icm20600_power_type_t mode) {
    uint8_t data_pwr1;
    uint8_t data_pwr2 = 0x00;
    uint8_t data_gyro_lp;


    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_PWR_MGMT_1, icmInstance->buffer);
    data_pwr1 = icmInstance->buffer[0];
    data_pwr1 &= 0x8f;                  // 0b10001111

    icmInstance->status |= I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_GYRO_LP_MODE_CFG,icmInstance->buffer);
    data_gyro_lp = icmInstance->buffer[0];

    // When set to ‘1’ low-power gyroscope mode is enabled. Default setting is 0
    data_gyro_lp &= 0x7f;               // 0b01111111
    switch (mode) {
        case ICM_SLEEP_MODE:
            data_pwr1 |= 0x40;          // set 0b01000000
            break;

        case ICM_STANDYBY_MODE:
            data_pwr1 |= 0x10;          // set 0b00010000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_ACC_LOW_POWER:
            data_pwr1 |= 0x20;          // set bit5 0b00100000
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_ACC_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_GYRO_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00000000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            data_gyro_lp |= 0x80;
            break;

        case ICM_GYRO_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_6AXIS_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00100000
            data_gyro_lp |= 0x80;
            break;

        case ICM_6AXIS_LOW_NOISE:
            data_pwr1 |= 0x00;
            break;

        default:
            break;
    }
    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_PWR_MGMT_1, &data_pwr1);
    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_PWR_MGMT_2, &data_pwr2);
    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_GYRO_LP_MODE_CFG, &data_gyro_lp);
}

// SAMPLE_RATE = 1KHz / (1 + div)
// work for low-power gyroscope and low-power accelerometer and low-noise accelerometer
void ICM_setSampleRateDivier(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, uint8_t div) {
	icmInstance->status = I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_SMPLRT_DIV, &div);
}

void ICM_setAccScaleRange(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, acc_scale_type_t range) {
    uint8_t data;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_ACCEL_CONFIG, icmInstance->buffer);
    data = icmInstance->buffer[0];
    data &= 0xe7; // 0b 1110 0111

    switch (range) {
        case RANGE_2G:
            data |= 0x00;   // 0bxxx00xxx
            icmInstance->acc_scale = 4000;
            break;

        case RANGE_4G:
            data |= 0x08;   // 0bxxx01xxx
            icmInstance->acc_scale = 8000;
            break;

        case RANGE_8G:
            data |= 0x10;   // 0bxxx10xxx
            icmInstance->acc_scale = 16000;
            break;

        case RANGE_16G:
            data |= 0x18;   // 0bxxx11xxx
            icmInstance->acc_scale = 32000;
            break;

        default:
            break;
    }

    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_ACCEL_CONFIG, &data);
}

// for low power mode only
void ICM_setAccAverageSample(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, acc_averaging_sample_type_t sample) {
    uint8_t data = 0;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_ACCEL_CONFIG2, icmInstance->buffer);
    data = icmInstance->buffer[0];

    data &= 0xcf; // & 0b11001111
    switch (sample) {
        case ACC_AVERAGE_4:
            data |= 0x00; // 0bxx00xxxx
            break;

        case ACC_AVERAGE_8:
            data |= 0x10; // 0bxx01xxxx
            break;

        case ACC_AVERAGE_16:
            data |= 0x20; // 0bxx10xxxx
            break;

        case ACC_AVERAGE_32:
            data |= 0x30; // 0bxx11xxxx
            break;

        default:
            break;
    }

    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_ACCEL_CONFIG2, &data);
}

void ICM_setAccOutputDataRate(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, acc_lownoise_odr_type_t odr) {
    uint8_t data;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_ACCEL_CONFIG2, icmInstance->buffer);
    data = icmInstance->buffer[0];
    data &= 0xf0;  // 0b11110000

    switch (odr) {
        case ACC_RATE_4K_BW_1046:
            data |= 0x08;
            break;

        case ACC_RATE_1K_BW_420:
            data |= 0x07;
            break;

        case ACC_RATE_1K_BW_218:
            data |= 0x01;
            break;

        case ACC_RATE_1K_BW_99:
            data |= 0x02;
            break;

        case ACC_RATE_1K_BW_44:
            data |= 0x03;
            break;

        case ACC_RATE_1K_BW_21:
            data |= 0x04;
            break;

        case ACC_RATE_1K_BW_10:
            data |= 0x05;
            break;

        case ACC_RATE_1K_BW_5:
            data |= 0x06;
            break;

        default:
            break;
    }

    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_ACCEL_CONFIG2, &data);
}

void ICM_setGyroScaleRange(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, gyro_scale_type_t range) {
    uint8_t data = 0;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_GYRO_CONFIG, icmInstance->buffer);
    data = icmInstance->buffer[0];
    data &= 0xe7; // 0b11100111

    switch (range) {
        case RANGE_250_DPS:
            data |= 0x00;   // 0bxxx00xxx
            icmInstance->gyro_scale = 500;
            break;

        case RANGE_500_DPS:
            data |= 0x08;   // 0bxxx00xxx
            icmInstance->gyro_scale = 1000;
            break;

        case RANGE_1K_DPS:
            data |= 0x10;   // 0bxxx10xxx
            icmInstance->gyro_scale = 2000;
            break;

        case RANGE_2K_DPS:
            data |= 0x18;   // 0bxxx11xxx
            icmInstance->gyro_scale = 4000;
            break;

        default:
            break;
    }

    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_GYRO_CONFIG, &data);
}

// for low power mode only
void ICM_setGyroAverageSample(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, gyro_averaging_sample_type_t sample) {
    uint8_t data = 0;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_GYRO_LP_MODE_CFG, icmInstance->buffer);
    data = icmInstance->buffer[0];

    data &= 0x8f;           // 0b10001111
    switch (sample) {
        case GYRO_AVERAGE_1:
            data |= 0x00; // 0bx000xxxx
            break;

        case GYRO_AVERAGE_2:
            data |= 0x10; // 0bx001xxxx
            break;

        case GYRO_AVERAGE_4:
            data |= 0x20; // 0bx010xxxx
            break;

        case GYRO_AVERAGE_8:
            data |= 0x30; // 0bx011xxxx
            break;

        case GYRO_AVERAGE_16:
            data |= 0x40; // 0bx100xxxx
            break;

        case GYRO_AVERAGE_32:
            data |= 0x50; // 0bx101xxxx
            break;

        case GYRO_AVERAGE_64:
            data |= 0x60;
            break;

        case GYRO_AVERAGE_128:
            data |= 0x70;
            break;


        default:
            break;
    }

    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_GYRO_LP_MODE_CFG, &data);
}

void ICM_setGyroOutputDataRate(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, gyro_lownoise_odr_type_t odr) {
    uint8_t data;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_CONFIG, icmInstance->buffer);
    data = icmInstance->buffer[0];
    data &= 0xf8;  // DLPF_CFG[2:0] 0b11111000

    switch (odr) {
        case GYRO_RATE_8K_BW_3281:
            data |= 0x07;
            break;
        case GYRO_RATE_8K_BW_250:
            data |= 0x00;
            break;
        case GYRO_RATE_1K_BW_176:
            data |= 0x01;
            break;
        case GYRO_RATE_1K_BW_92:
            data |= 0x02;
            break;
        case GYRO_RATE_1K_BW_41:
            data |= 0x03;
            break;
        case GYRO_RATE_1K_BW_20:
            data |= 0x04;
            break;
        case GYRO_RATE_1K_BW_10:
            data |= 0x05;
            break;
        case GYRO_RATE_1K_BW_5:
            data |= 0x06;
            break;
    }

    icmInstance->status |= I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_CONFIG, &data);
}

void ICM_getAcceleration(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, int16_t* x, int16_t* y, int16_t* z) {
    *x = ICM_getAccelerationX(icmInstance, i2cInstance);
    *y = ICM_getAccelerationY(icmInstance, i2cInstance);
    *z = ICM_getAccelerationZ(icmInstance, i2cInstance);
}

void ICM_getGyroscope(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance, int16_t* x, int16_t* y, int16_t* z) {
    *x = ICM_getGyroscopeX(icmInstance, i2cInstance);
    *y = ICM_getGyroscopeY(icmInstance, i2cInstance);
    *z = ICM_getGyroscopeZ(icmInstance, i2cInstance);
}

int16_t ICM_getTemperature(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint16_t rawdata;
    icmInstance->status = I2C_readBytes(i2cInstance, icmInstance->addr, ICM20600_TEMP_OUT_H, 2, icmInstance->buffer);
    rawdata = (((uint16_t)icmInstance->buffer[0]) << 8) + icmInstance->buffer[1];
    return (int16_t)(rawdata / 327 + 25);
}

void ICM_reset(ICM_Handle_t *icmInstance, I2C_HandleTypeDef *i2cInstance) {
    uint8_t data;
    icmInstance->status = I2C_readByte(i2cInstance, icmInstance->addr, ICM20600_USER_CTRL, icmInstance->buffer);
    data = icmInstance->buffer[0];
    data &= 0xfe;  // ICM20600_USER_CTRL[0] 0b11111110
    data |= ICM20600_RESET_BIT;
    icmInstance->status = I2C_writeByte(i2cInstance, icmInstance->addr, ICM20600_USER_CTRL, &data);
}

