#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/*   CONFIGURATION   */
#define BME_280_ADDR 0x76 
#define BME_280_ID_REGISTER 0xD0 
#define BME_280_CTRL_HUM 0xF2
#define BME_280_STATUS 0xF3
#define BME_280_CTRL_MEAS 0xF4
#define BME_280_CONFIG 0xF5
#define BME_280_TEMPERATURE_COMPENSATION_REG 0x88
#define BME_280_PRESSURE_COMPENSATION_REG 0x8E
#define BME_280_HUMIDITY_COMPENSATION_REG1 0xA1
#define BME_280_HUMIDITY_COMPENSATION_REG2 0xE1

/*   TEMPERATURE   */
#define BME_280_READ_DATA 0xF7 /* 0xF7 to 0xFE burst mode*/

/* The devicetree node identifier for the "led0" alias. */

#define I2C_NODE DT_NODELABEL(i2c1)

struct mbe280{
	uint16_t temp_compensation_t1;
	int16_t temp_compensation_t2;
	int16_t temp_compensation_t3;

	uint16_t pressure_compensation_p1;
	int16_t pressure_compensation_p2;
	int16_t pressure_compensation_p3;
	int16_t pressure_compensation_p4;
	int16_t pressure_compensation_p5;
	int16_t pressure_compensation_p6;
	int16_t pressure_compensation_p7;
	int16_t pressure_compensation_p8;
	int16_t pressure_compensation_p9;

	uint16_t humidity_compensation_h1;
	int16_t humidity_compensation_h2;
	uint16_t humidity_compensation_h3;
	int16_t humidity_compensation_h4;
	int16_t humidity_compensation_h5;
	int16_t humidity_compensation_h6;


	float temperature;
	float pressure;
	float humidity;
};

struct mbe280 bme280_data;


static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);
static uint8_t i2c_buff_write[4] = {0};
static uint8_t i2c_buff_read[20] = {0};
int temperature = 0;
int humidity = 0;
int presure = 0;
int t_fine;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
int bmp280_compensate_temperature_int32(int adc_T)
{
	int var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int)bme280_data.temp_compensation_t1 << 1))) * 
		((int)bme280_data.temp_compensation_t2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int)bme280_data.temp_compensation_t1)) * 
		((adc_T >> 4) - ((int)bme280_data.temp_compensation_t1))) >> 12) *
			((int)bme280_data.temp_compensation_t3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;

	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t bmp280_compensate_pressure_int32(uint32_t adc_P)
{
	int64_t var1, var2, p;	
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_data.pressure_compensation_p6;
	var2 = var2 + ((var1*(int64_t)bme280_data.pressure_compensation_p5)<<17);
	var2 = var2 + (((int64_t)bme280_data.pressure_compensation_p4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280_data.pressure_compensation_p3)>>8) +
	  ((var1 * (int64_t)bme280_data.pressure_compensation_p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_data.pressure_compensation_p1)>>33;	
	if (var1 == 0) {
	  return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)bme280_data.pressure_compensation_p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bme280_data.pressure_compensation_p8) * p) >> 19;	
	p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_data.pressure_compensation_p7)<<4);
	return p;
}

float bmp280_compensate_humidity_int32(uint32_t adc_H) {
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_data.humidity_compensation_h4) << 20) -
      (((int32_t)bme280_data.humidity_compensation_h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
         (((((((v_x1_u32r * ((int32_t)bme280_data.humidity_compensation_h6)) >> 10) *
        (((v_x1_u32r * ((int32_t)bme280_data.humidity_compensation_h3)) >> 11) + ((int32_t)32768))) >> 10) +
      ((int32_t)2097152)) * ((int32_t)bme280_data.humidity_compensation_h2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
           ((int32_t)bme280_data.humidity_compensation_h1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r>>12);
  return  h;
}

int main(void)
{
	int status = 0;

	if(!device_is_ready(i2c1_dev)){
		printk("i2c1_dev not ready\n");
		return -1;
	}

	i2c_buff_write[0] = BME_280_ID_REGISTER; // read ID
	status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 1); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");

/*
	i2c_buff_write[0] = BME_280_CTRL_HUM; 
	i2c_buff_write[1] = 0x01;					// CONFIGURE CONTROL HUM OVERSAMPLING 1 H
	status = i2c_write(i2c1_dev, i2c_buff_write, 2, BME_280_ADDR); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");


	i2c_buff_write[0] = BME_280_CTRL_MEAS; 
	i2c_buff_write[1] = 3 | 1 << 2 | 1 << 5;	// CONFIGURE CONTROL MEAS OVERSAMPLING 1 T P NORMAL MODE
	status = i2c_write(i2c1_dev, i2c_buff_write, 2, BME_280_ADDR); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");

	i2c_buff_write[0] = BME_280_CONFIG; 
	i2c_buff_write[1] = 0x00;					// CONFIGURE CONFIG 0.5 FILTER OFF
	status = i2c_write(i2c1_dev, i2c_buff_write, 2, BME_280_ADDR); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");
*/
	// read temperature compensation reg
	i2c_buff_write[0] = BME_280_TEMPERATURE_COMPENSATION_REG; 
	status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 6); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");
	bme280_data.temp_compensation_t1 = (uint16_t)i2c_buff_read[0] | (uint16_t)i2c_buff_read[1] << 8;
	bme280_data.temp_compensation_t2 = (uint16_t)i2c_buff_read[2] | (uint16_t)i2c_buff_read[3] << 8;
	bme280_data.temp_compensation_t3 = (uint16_t)i2c_buff_read[4] | (uint16_t)i2c_buff_read[5] << 8;

		// read pressure compensation reg
	i2c_buff_write[0] = BME_280_PRESSURE_COMPENSATION_REG; 
	status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 18); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");
	bme280_data.pressure_compensation_p1 = (uint16_t)i2c_buff_read[0]  | (uint16_t)i2c_buff_read[1]  << 8;
	bme280_data.pressure_compensation_p2 = (uint16_t)i2c_buff_read[2]  | (uint16_t)i2c_buff_read[3]  << 8;
	bme280_data.pressure_compensation_p3 = (uint16_t)i2c_buff_read[4]  | (uint16_t)i2c_buff_read[5]  << 8;
	bme280_data.pressure_compensation_p4 = (uint16_t)i2c_buff_read[6]  | (uint16_t)i2c_buff_read[7]  << 8;
	bme280_data.pressure_compensation_p5 = (uint16_t)i2c_buff_read[8]  | (uint16_t)i2c_buff_read[9]  << 8;
	bme280_data.pressure_compensation_p6 = (uint16_t)i2c_buff_read[10] | (uint16_t)i2c_buff_read[11] << 8;
	bme280_data.pressure_compensation_p7 = (uint16_t)i2c_buff_read[12] | (uint16_t)i2c_buff_read[13] << 8;
	bme280_data.pressure_compensation_p8 = (uint16_t)i2c_buff_read[14] | (uint16_t)i2c_buff_read[15] << 8;
	bme280_data.pressure_compensation_p9 = (uint16_t)i2c_buff_read[16] | (uint16_t)i2c_buff_read[17] << 8;


	// read HUMIDITY compensation reg
	i2c_buff_write[0] = BME_280_HUMIDITY_COMPENSATION_REG1; 
	status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 1); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");
	bme280_data.humidity_compensation_h1 = (uint16_t)i2c_buff_read[0];
	i2c_buff_write[0] = BME_280_HUMIDITY_COMPENSATION_REG2; 
	status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 7); 
	if(status < 0)
		printk("ERROR i2c_write_read in BME_280\n");
	bme280_data.humidity_compensation_h2 = (uint16_t)i2c_buff_read[0] | (uint16_t)i2c_buff_read[1] << 8;
	bme280_data.humidity_compensation_h3 = (uint16_t)i2c_buff_read[2];
	bme280_data.humidity_compensation_h4 = (uint16_t)i2c_buff_read[4] & 0x0f | (uint16_t)i2c_buff_read[3] << 4;
	bme280_data.humidity_compensation_h5 = (uint16_t)((i2c_buff_read[4] >> 4) & 0x0f) | (uint16_t)i2c_buff_read[5] << 4;
	bme280_data.humidity_compensation_h6 = (uint16_t)i2c_buff_read[6];


	while (1) {

		i2c_buff_write[0] = BME_280_STATUS; 
		status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 1); 
		if(status < 0)
			printk("ERROR i2c_write_read in BME_280\n");

		i2c_buff_write[0] = BME_280_READ_DATA; 
		status = i2c_write_read(i2c1_dev, BME_280_ADDR, &i2c_buff_write, 1, i2c_buff_read, 8); 
		if(status < 0)
			printk("ERROR i2c_write_read in BME_280\n");

		presure		= i2c_buff_read[2] >> 4 | i2c_buff_read[1] << 4 | i2c_buff_read[0] << 12;
		temperature	= i2c_buff_read[5] >> 4 | i2c_buff_read[4] << 4 | i2c_buff_read[3] << 12;
		humidity	= i2c_buff_read[7] | i2c_buff_read[6] << 8;

		bme280_data.temperature = (float)bmp280_compensate_temperature_int32(temperature) / 100;
		bme280_data.pressure = (float)bmp280_compensate_pressure_int32(presure) / 256;
		bme280_data.humidity = (float)bmp280_compensate_humidity_int32(humidity) / 1024;

		
		k_msleep(SLEEP_TIME_MS);
	} 
	return 0;
}