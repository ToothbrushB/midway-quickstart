#include "i2c.h"
#include <driver/i2c.h>

_i2c i2c;

void _i2c::init(int scl, int sda) {
    i2c_config_t conf;
    const int I2C_MASTER_SCL_IO = scl;       /*!< GPIO number used for I2C master clock */
    const int I2C_MASTER_SDA_IO = sda;       /*!< GPIO number used for I2C master data  */
    const int I2C_MASTER_NUM = 0;            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
    const int I2C_MASTER_FREQ_HZ = 100000;   /*!< I2C master clock frequency */
    const int I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    const int I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;  // select GPIO specific to your project
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;  // select GPIO specific to your project
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;  // select frequency specific to your project
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void _i2c::write(uint8_t addr, uint8_t* data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, size, 0);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void _i2c::write(uint8_t addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, data, 0);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void _i2c::read(uint8_t addr, uint8_t* data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, data, size, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
