// VL53L0X control hpp by caleb
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0



#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <driver/i2c.h>
#pragma once

typedef enum
{ VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
typedef struct
{
    uint8_t tcc : 1;
    uint8_t msrc : 1;
    uint8_t dss : 1;
    uint8_t pre_range : 1;
    uint8_t final_range : 1;
} SequenceStepEnables;

typedef struct
{
    uint16_t pre_range_vcsel_period_pclks,
        final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks,
        pre_range_mclks,
        final_range_mclks;
    uint32_t msrc_dss_tcc_us,
        pre_range_us,
        final_range_us;
} SequenceStepTimeouts;
class VL53L0X {
    // Functions returning const char * are OK for NULL, else error string

    public:
        // Set up I2C and create the vl53l0x structure, NULL means could not see device on I2C
        esp_err_t config (i2c_port_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8);
        // Initialise the VL53L0X
        const char *init (uint8_t address);
        // End I2C and free the structure
        void end ();

        void setAddress (uint8_t new_addr);
        uint8_t getAddress ();


        const char *setSignalRateLimit (float limit_Mcps);
        float getSignalRateLimit ();

        const char *setMeasurementTimingBudget (uint32_t budget_us);
        uint32_t getMeasurementTimingBudget ();

        const char *setVcselPulsePeriod (vcselPeriodType type, uint8_t period_pclks);
        uint8_t getVcselPulsePeriod (vcselPeriodType type);

        void startContinuous (uint32_t period_ms);
        void stopContinuous ();
        uint16_t readRangeContinuousMillimeters ();
        uint16_t readRangeSingleMillimeters ();

        void setTimeout (uint16_t timeout);
        uint16_t getTimeout ();
        int timeoutOccurred ();
        int i2cFail ();

    private:
        void writeReg8Bit (uint8_t reg, uint8_t value);
        void writeReg16Bit (uint8_t reg, uint16_t value);
        void writeReg32Bit (uint8_t reg, uint32_t value);
        uint8_t readReg8Bit (uint8_t reg);
        uint16_t readReg16Bit (uint8_t reg);
        uint32_t readReg32Bit (uint8_t reg);

        void writeMulti (uint8_t reg, uint8_t const *src, uint8_t count);
        void readMulti (uint8_t reg, uint8_t * dst, uint8_t count);

        i2c_cmd_handle_t Read (uint8_t reg);
        i2c_cmd_handle_t Write (uint8_t reg);
        esp_err_t Done (i2c_cmd_handle_t i);
        const char * getSpadInfo(uint8_t *count, int *type_is_aperture);
        const char * performSingleRefCalibration(uint8_t vhv_init_byte);
        void getSequenceStepEnables(SequenceStepEnables *enables);
        void getSequenceStepTimeouts(SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts);
        i2c_port_t port;
        uint8_t address;
        int8_t xshut;
        uint16_t io_timeout;
        uint8_t io_2v8 : 1;
        uint8_t did_timeout : 1;
        uint8_t i2c_fail : 1;
        uint8_t stop_variable;
        uint16_t timeout_start_ms;
        uint32_t measurement_timing_budget_us;

};