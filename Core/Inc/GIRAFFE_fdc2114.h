/*
 * GIRAFFE_fdc2114.h
 *
 *  Created on: Jan 5, 2021
 *      Author: AUSTINLOCAL
 */

#ifndef GIRAFFE_FDC2114_H_
#define GIRAFFE_FDC2114_H_

#include <stdint.h>

enum fdc2114_config_registers
{
  rcount_ch0=0, rcount_ch1, rcount_ch2, rcount_ch3,
  offset_ch0, offset_ch1, offset_ch2, offset_ch3,
  settlecount_ch0, settlecount_ch1, settlecount_ch2, settlecount_ch3,
  clock_dividers_ch0, clock_dividers_ch1, clock_dividers_ch2, clock_dividers_ch3,
  mux_config,
  drive_current_ch0, drive_current_ch1, drive_current_ch2, drive_current_ch3,
  error_config,
  config, //config must be last so it will be written last
  NUM_CONFIG
};

enum fdc2114_data_registers
{
  data_ch0=0, data_ch1, data_ch2, data_ch3,
  status, manufacturer_id, device_id,
  NUM_DATA
};

enum fdc2114_special_registers
{
  reset_dev=0,
  NUM_SPECIAL
};

uint8_t  CONFIG_ADDR[NUM_CONFIG];
uint16_t CONFIG_VALS[NUM_CONFIG];
uint8_t  SPECIAL_ADDR[NUM_SPECIAL];
uint16_t SPECIAL_VALS[NUM_SPECIAL];
uint8_t  DATA_ADDR[NUM_DATA];


typedef enum i2c_status
{
	I2C_OK=0,
	I2C_ERROR
} GIRAFFE_i2c_status_t;

typedef enum i2c_error_type
{
	NONE=0,
	BUS_ERROR,
	OTHER_ERROR
} GIRAFFE_i2c_error_type_t;

typedef struct channel_data_s
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
} GIRAFFE_FDC2114_channel_data_t;

typedef struct i2c_fdc_it_state_s
{
	uint8_t read_data[8];
	uint8_t num_reads_remaining;
	uint8_t read_order[4];
	GIRAFFE_i2c_error_type_t error;
} GIRAFFE_FDC2114_IT_state_t;

struct i2c_rw_funcs_s
{
	GIRAFFE_i2c_status_t (*i2c_write)(uint16_t, uint8_t *, uint16_t); // dev_address, data pointer, size
	GIRAFFE_i2c_status_t (*i2c_read)(uint16_t, uint8_t*, uint16_t); // dev_address, data pointer, size
} i2c_rw_funcs;

struct i2c_rw_funcs_IT_s
{
	GIRAFFE_i2c_status_t (*i2c_write_it)(uint16_t, uint8_t *, uint16_t); // dev_address, data pointer, size
	GIRAFFE_i2c_status_t (*i2c_read_it)(uint16_t, uint8_t*, uint16_t); // dev_address, data pointer, size
} i2c_rw_funcs_IT;

void GIRAFFE_FDC2114_initialize_lib(GIRAFFE_i2c_status_t (*i2c_read)(uint16_t, uint8_t *, uint16_t),
		            GIRAFFE_i2c_status_t (*i2c_write)(uint16_t, uint8_t *, uint16_t));

void GIRAFFE_FDC2114_initialize_IT(GIRAFFE_i2c_status_t (*i2c_read_it)(uint16_t, uint8_t *, uint16_t),
		                           GIRAFFE_i2c_status_t (*i2c_write_it)(uint16_t, uint8_t *, uint16_t));

void GIRAFFE_FDC2114_set_single_ended(); // Call between initialize_lib and init_dev to set single ended config bits

GIRAFFE_i2c_status_t GIRAFFE_FDC2114_init_dev();

void GIRAFFE_FDC2114_read_channels(GIRAFFE_FDC2114_channel_data_t* data);

void GIRAFFE_FDC2114_read_channels_IT(GIRAFFE_FDC2114_IT_state_t *state);

void GIRAFFE_FDC2114_get_chan_data_from_state(GIRAFFE_FDC2114_IT_state_t *state, GIRAFFE_FDC2114_channel_data_t *data);

void GIRAFFE_FDC2114_write_clbk(GIRAFFE_FDC2114_IT_state_t *state);
void GIRAFFE_FDC2114_read_clbk(GIRAFFE_FDC2114_IT_state_t *state);
uint8_t GIRAFFE_is_error_severe(GIRAFFE_i2c_error_type_t err);



//void set_addr(); //set register addresses
//void set_default_config_vals(uint16_t* vals);

//void write_register_list(int fd, const uint8_t* registers, const uint16_t* values, int number);
//void read_register_list(int fd, const uint8_t* registers, int number);

#endif /* GIRAFFE_FDC2114_H_ */
