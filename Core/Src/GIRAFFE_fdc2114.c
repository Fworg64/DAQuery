/*
 * GIRAFFE_fdc2114.c
 *
 *  Created on: Jan 5, 2021
 *      Author: AUSTINLOCAL
 */

#include <GIRAFFE_fdc2114.h>

uint16_t byteswap(uint16_t swapme)
{
  uint16_t hibyte = (swapme & 0xFF00) >> 8;
  uint16_t lowbyte = (swapme & 0x00FF);
  return (lowbyte << 8 | hibyte);
}

void set_addr()
{
DATA_ADDR[data_ch0] = 0x00;
DATA_ADDR[data_ch1] = 0x02;
DATA_ADDR[data_ch2] = 0x04;
DATA_ADDR[data_ch3] = 0x06;

CONFIG_ADDR[rcount_ch0] = 0x08;
CONFIG_ADDR[rcount_ch1] = 0x09;
CONFIG_ADDR[rcount_ch2] = 0x0A;
CONFIG_ADDR[rcount_ch3] = 0x0B;
CONFIG_ADDR[offset_ch0] = 0x0C;
CONFIG_ADDR[offset_ch1] = 0x0D;
CONFIG_ADDR[offset_ch2] = 0x0E;
CONFIG_ADDR[offset_ch3] = 0x0F;

CONFIG_ADDR[settlecount_ch0] = 0x10;
CONFIG_ADDR[settlecount_ch1] = 0x11;
CONFIG_ADDR[settlecount_ch2] = 0x12;
CONFIG_ADDR[settlecount_ch3] = 0x13;
CONFIG_ADDR[clock_dividers_ch0] = 0x14;
CONFIG_ADDR[clock_dividers_ch1] = 0x15;
CONFIG_ADDR[clock_dividers_ch2] = 0x16;
CONFIG_ADDR[clock_dividers_ch3] = 0x17;

DATA_ADDR[status] = 0x18;

CONFIG_ADDR[error_config]= 0x19;
CONFIG_ADDR[config] = 0x1A;
CONFIG_ADDR[mux_config] = 0x1B;

SPECIAL_ADDR[reset_dev] = 0x1C;

CONFIG_ADDR[drive_current_ch0] = 0x1E;
CONFIG_ADDR[drive_current_ch1] = 0x1F;
CONFIG_ADDR[drive_current_ch2] = 0x20;
CONFIG_ADDR[drive_current_ch3] = 0x21;

DATA_ADDR[manufacturer_id] = 0x7E;
DATA_ADDR[device_id] = 0x7F;

}

void set_default_config_vals(uint16_t* vals)
{
vals[rcount_ch0] = 0x0050;
vals[rcount_ch1] = 0x0050;
vals[rcount_ch2] = 0x0050;
vals[rcount_ch3] = 0x0050;

vals[offset_ch0] = 0x0000;
vals[offset_ch1] = 0x0000;
vals[offset_ch2] = 0x0000;
vals[offset_ch3] = 0x0000;

vals[settlecount_ch0] = 0x0032;
vals[settlecount_ch1] = 0x0032;
vals[settlecount_ch2] = 0x0032;
vals[settlecount_ch3] = 0x0032;

vals[clock_dividers_ch0] = 0x1001;
vals[clock_dividers_ch1] = 0x1001;
vals[clock_dividers_ch2] = 0x1001;
vals[clock_dividers_ch3] = 0x1001;

vals[error_config] = 0x0001; //was 0, added 1 for DRDY to be on INTB
vals[config] = 0x1E01;
vals[mux_config] = 0xC20C;

vals[drive_current_ch0] = 0xA000;
vals[drive_current_ch1] = 0xA000;
vals[drive_current_ch2] = 0xA000;
vals[drive_current_ch3] = 0xA000;
}

void set_default_special_vals(uint16_t* vals)
{
	vals[reset_dev] = 0x0600; 	//set gain w/ MSB 0x06 => 16, 0x04 => 8, 0x02 => 4, 0x00 => 1
}

void GIRAFFE_FDC2114_initialize_lib(GIRAFFE_i2c_status_t (*i2c_read)(uint16_t, uint8_t *, uint16_t),
		                            GIRAFFE_i2c_status_t (*i2c_write)(uint16_t, uint8_t *, uint16_t))
{
	i2c_rw_funcs.i2c_read = i2c_read;
	i2c_rw_funcs.i2c_write = i2c_write;
	set_addr();
	set_default_special_vals(SPECIAL_VALS);
	set_default_config_vals(CONFIG_VALS);
}

void GIRAFFE_FDC2114_initialize_IT(GIRAFFE_i2c_status_t (*i2c_read_it)(uint16_t, uint8_t *, uint16_t),
		                           GIRAFFE_i2c_status_t (*i2c_write_it)(uint16_t, uint8_t *, uint16_t))
{
	i2c_rw_funcs_IT.i2c_read_it = i2c_read_it;
	i2c_rw_funcs_IT.i2c_write_it = i2c_write_it;
}

void write_register_list(const uint8_t* registers, const uint16_t* values, int number)
{
  GIRAFFE_i2c_status_t result;
  uint8_t bytes[2];
//  uint16_t addr;
  for (uint8_t a=0; a<number; a++)
  {
	bytes[0] = registers[a]; // first byte is address
	bytes[1] = (values[a] & 0xFF00) >> 8; // MSB is first
	bytes[2] = values[a] & 0x00FF;
    result = i2c_rw_funcs.i2c_write(0x2b << 1, bytes, 3);
    if (result == I2C_ERROR) break;
  }
}

void read_register_list(const uint8_t* registers, uint16_t* values, int number)
{
  GIRAFFE_i2c_status_t result;
  uint8_t bytes[2];
  //uint16_t addr;
  for (uint8_t a=0; a<number; a++)
  {
	//bytes[0] = values[a] & 0xFF;
	//bytes[1] = values[a] >> 8;
	//addr = registers[a];
	i2c_rw_funcs.i2c_write((0x2b << 1), registers + a, 1);
    result = i2c_rw_funcs.i2c_read((0x2b << 1)+1, bytes, 2);
    values[a] = bytes[1] + (bytes[0] << 8);
    if (result == I2C_ERROR) break;
  }
}

GIRAFFE_i2c_status_t GIRAFFE_FDC2114_init_dev()
{
	uint16_t check_vals[NUM_CONFIG];

	write_register_list(SPECIAL_ADDR, SPECIAL_VALS, NUM_SPECIAL);
	write_register_list(CONFIG_ADDR, CONFIG_VALS, NUM_CONFIG);
	read_register_list(CONFIG_ADDR, check_vals, NUM_CONFIG);
	for (uint8_t a=0; a<17 /*NUM_CONFIG*/; a++)
	{
		if (CONFIG_VALS[a] != check_vals[a])
			return I2C_ERROR;
	}
	return I2C_OK;
}

void GIRAFFE_FDC2114_read_channels(GIRAFFE_FDC2114_channel_data_t* data)
{
	uint8_t read_data[8];
	uint8_t read_order[] = {data_ch0, data_ch1, data_ch2, data_ch3};
	for (uint8_t a = 0; a < 4; a++)
	{
	i2c_rw_funcs.i2c_write(0x2b<< 1, DATA_ADDR + read_order[a], 1);
	i2c_rw_funcs.i2c_read(0x2b << 1, read_data + 2*a, 2);
	}

	data->ch0 = (read_data[1] <<8) + read_data[0];
	data->ch1 = (read_data[3] <<8) + read_data[2];
	data->ch2 = (read_data[5] <<8) + read_data[4];
	data->ch3 = (read_data[7] <<8) + read_data[6];
}

void GIRAFFE_FDC2114_read_channels_IT(GIRAFFE_FDC2114_IT_state_t *state)
{
	// Initialize State
	state->num_reads_remaining = 4;
	memset(state->read_data, 0, sizeof state->read_data);
	state->read_order[0] = data_ch0; state->read_order[1] = data_ch1; //read order is fixed for read -> data conversion
	state->read_order[2] = data_ch2; state->read_order[3] = data_ch3;
	// Start write
	i2c_rw_funcs_IT.i2c_write_it(0x2b << 1, DATA_ADDR + state->read_order[0], 1);
}

uint8_t GIRAFFE_is_error_severe(GIRAFFE_i2c_error_type_t err)
{
	switch (err)
	{
	  case BUS_ERROR:
		  return 1;
	  case OTHER_ERROR:
		  return 0;
	}
	return 0;
}

void GIRAFFE_FDC2114_write_clbk(GIRAFFE_FDC2114_IT_state_t *state)
{
    if (GIRAFFE_is_error_severe(state->error))
		{
    	  state->num_reads_remaining = 0;
    	  return;
		}
	if ((1 <= state->num_reads_remaining) && (state->num_reads_remaining  <= 4))
	  i2c_rw_funcs_IT.i2c_read_it(0x2b << 1, state->read_data + 2*(4-state->num_reads_remaining), 2);
	else
		return;
}

void GIRAFFE_FDC2114_read_clbk(GIRAFFE_FDC2114_IT_state_t *state)
{
    if (GIRAFFE_is_error_severe(state->error))
		{
    	  state->num_reads_remaining = 0;
    	  return;
		}
	if (--state->num_reads_remaining != 0)
		i2c_rw_funcs_IT.i2c_write_it(0x2b << 1, DATA_ADDR + state->read_order[4-state->num_reads_remaining], 1);
}

void GIRAFFE_FDC2114_get_chan_data_from_state(GIRAFFE_FDC2114_IT_state_t *state, GIRAFFE_FDC2114_channel_data_t *data)
{
	// only works for fixed read order
	data->ch0 = (state->read_data[1] <<8) + state->read_data[0];
	data->ch1 = (state->read_data[3] <<8) + state->read_data[2];
	data->ch2 = (state->read_data[5] <<8) + state->read_data[4];
	data->ch3 = (state->read_data[7] <<8) + state->read_data[6];
}


/*
void read_register_list(int fd, const uint8_t* registers, int number)
{
  int result;
  for (int a=0; a<number; a++)
  {
    result = byteswap(wiringPiI2CReadReg16(fd, registers[a]));
    cout << "Reading register " << hex << (int) registers[a] << ". Data = " << hex << result << dec << endl;
  }
}
*/
