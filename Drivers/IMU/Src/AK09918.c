/*
 * AK09918.c
 *
 *  Created on: Sep 3, 2022
 *      Author: micha
 */
#include "AK09918.h"

/********************************************
 *			PRIVATE VARIABLES				*
 ********************************************/



/********************************************
 *			PRIVATE FUNCTIONS				*
 ********************************************/
static uint8_t _getRawMode(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance) {
    if (I2C_AKreadByte(i2cInstance, akInstance->addr, AK09918_CNTL2, akInstance->buffer) != HAL_OK) {
        return 0xFF;
    } else {
        return akInstance->buffer[0];
    }
}

AK09918_err_type_t AK_getRawData(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance, int32_t* axis_x, int32_t* axis_y, int32_t* axis_z) {
    if (akInstance->mode == AK09918_NORMAL) {
        AK_switchMode(akInstance, i2cInstance, AK09918_NORMAL);
        bool is_end = false;
        int count = 0;
        while (!is_end) {
            if (_getRawMode(akInstance, i2cInstance) == 0x00) {
                is_end = true;
            }
            if (count >= 15) {
                return AK09918_ERR_TIMEOUT;
            }
            count ++;
            HAL_Delay(1);
        }
    }


    if (I2C_AKreadBytes(i2cInstance, akInstance->addr, AK09918_HXL, 8, akInstance->buffer) != HAL_OK) {
        return AK09918_ERR_READ_FAILED;
    } else {
        *axis_x = (int32_t)((((int16_t)akInstance->buffer[1]) << 8) | akInstance->buffer[0]);
        *axis_y = (int32_t)((((int16_t)akInstance->buffer[3]) << 8) | akInstance->buffer[2]);
        *axis_z = (int32_t)((((int16_t)akInstance->buffer[5]) << 8) | akInstance->buffer[4]);
        if (akInstance->buffer[7] & AK09918_HOFL_BIT) {
            return AK09918_ERR_OVERFLOW;
        }
        return AK09918_ERR_OK;
    }
}

uint16_t AK_getDeviceID(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance) {
    I2C_AKreadBytes(i2cInstance, akInstance->addr, AK09918_WIA1, 2, akInstance->buffer);
    return (((uint16_t)akInstance->buffer[0]) << 8) | akInstance->buffer[1];
}


/********************************************
 *			PUBLIC FUNCTIONS				*
 ********************************************/
AK09918_err_type_t AK_initialize(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance, AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        mode = AK09918_POWER_DOWN;
    }
    akInstance->mode = mode;

    //if (mode == AK09918_NORMAL) {
      //  return AK09918_ERR_OK;
    //} else {
        return AK_switchMode(akInstance, i2cInstance, akInstance->mode);
    //}
}

AK09918_err_type_t AK_isDataReady(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance) {
    if (I2C_AKreadByte(i2cInstance, akInstance->addr, AK09918_ST1, akInstance->buffer) != HAL_OK) {
        return AK09918_ERR_READ_FAILED;
    } else {
        if (akInstance->buffer[0] & AK09918_DRDY_BIT) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_NOT_RDY;
        }
    }
}

AK09918_err_type_t AK_getData(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance, int32_t* axis_x, int32_t* axis_y, int32_t* axis_z) {
    AK09918_err_type_t err = AK_getRawData(akInstance, i2cInstance, axis_x, axis_y, axis_z);
    (*axis_x) = (*axis_x) * 15 / 100;
    (*axis_y) = (*axis_y) * 15 / 100;
    (*axis_z) = (*axis_z) * 15 / 100;

    return err;
}


AK09918_mode_type_t AK_getMode(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance) {
    return akInstance->mode;
}

AK09918_err_type_t AK_switchMode(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance, AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        return AK09918_ERR_WRITE_FAILED;
    }
    akInstance->mode = mode;
    if (I2C_AKwriteByte(i2cInstance, akInstance->addr, AK09918_CNTL2, &akInstance->mode) != HAL_OK) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

// 1.Set Power-down mode. (MODE[4:0] bits = “00000”)
// 2.Set Self-test mode. (MODE[4:0] bits = “10000”)
// 3.Check Data Ready or not by polling DRDY bit of ST1 register.
// 4.When Data Ready, proceed to the next step. Read measurement data. (HXL to HZH)
AK09918_err_type_t AK_selfTest(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance) {
    int32_t axis_x, axis_y, axis_z;
    bool is_end = false;
    AK09918_err_type_t err;
    uint8_t *test_mode;
    *test_mode = AK09918_POWER_DOWN;
    if (I2C_AKwriteByte(i2cInstance, akInstance->addr, AK09918_CNTL2, *test_mode) != HAL_OK) {
        return AK09918_ERR_WRITE_FAILED;
    }

    *test_mode = AK09918_SELF_TEST;
    if (I2C_AKwriteByte(i2cInstance, akInstance->addr, AK09918_CNTL2, *test_mode) != HAL_OK) {
        return AK09918_ERR_WRITE_FAILED;
    }

    while (!is_end) {
        err = AK_isDataReady(akInstance, i2cInstance);
        if (err == AK09918_ERR_OK) {
            is_end = true;
        } else if (err == AK09918_ERR_READ_FAILED) {
            return AK09918_ERR_READ_FAILED;
        }
    }

    // read data and check
    if (I2C_AKwriteBytes(i2cInstance, akInstance->addr, AK09918_HXL, 8, akInstance->buffer) != HAL_OK) {
        return AK09918_ERR_READ_FAILED;
    } else {
        axis_x = (int32_t)((((int16_t)akInstance->buffer[1]) << 8) | akInstance->buffer[0]);
        axis_y = (int32_t)((((int16_t)akInstance->buffer[3]) << 8) | akInstance->buffer[2]);
        axis_z = (int32_t)((((int16_t)akInstance->buffer[5]) << 8) | akInstance->buffer[4]);

        if ((axis_x >= -200) && (axis_x <= 200) && (axis_y >= -200) && (axis_y <= 200) && \
                (axis_z >= -1000) && (axis_z <= -150)) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_SELFTEST_FAILED;
        }

    }
}

AK09918_err_type_t AK_reset(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance) {
    if (I2C_AKwriteByte(i2cInstance, akInstance->addr, AK09918_CNTL3, AK09918_SRST_BIT) != HAL_OK) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

char* AK_strError(AK0_Handle_t *akInstance, I2C_HandleTypeDef *i2cInstance, AK09918_err_type_t err) {
    char* result;
    switch (err) {
        case AK09918_ERR_OK:
            result = "AK09918_ERR_OK: OK";
            break;

        case AK09918_ERR_DOR:
            result = "AK09918_ERR_DOR: Data skipped";
            break;

        case AK09918_ERR_NOT_RDY:
            result = "AK09918_ERR_NOT_RDY: Not ready";
            break;

        case AK09918_ERR_TIMEOUT:
            result = "AK09918_ERR_TIMEOUT: Timeout";
            break;

        case AK09918_ERR_SELFTEST_FAILED:
            result = "AK09918_ERR_SELFTEST_FAILED: Self test failed";
            break;

        case AK09918_ERR_OVERFLOW:
            result = "AK09918_ERR_OVERFLOW: Sensor overflow";
            break;

        case AK09918_ERR_WRITE_FAILED:
            result = "AK09918_ERR_WRITE_FAILED: Fail to write";
            break;

        case AK09918_ERR_READ_FAILED:
            result = "AK09918_ERR_READ_FAILED: Fail to read";
            break;

        default:
            result = "Unknown Error";
            break;
    }
    return result;
}



