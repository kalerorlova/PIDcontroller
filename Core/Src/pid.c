/*
 * pid.c
 *
 *  Created on: Nov 13, 2026
 *      Author: Lera
 */

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ssd1306_conf.h"
#include "bme280.h"
#include "bme280_defs.h"
#include <stdio.h>
#include <stdbool.h>

#define INIT_TEMP	39.0
#define SETPOINT	30.0
#define OPENLOOP_P	6.4475
#define PIDLOOP		1

I2C_HandleTypeDef i2c1;
TIM_HandleTypeDef* tim3;
UART_HandleTypeDef uart2;

typedef struct {
	double p;
	double i;
	double d;
	double errCum;
	double valPrev;
	double peak;
	bool peakDetected;
} pidData;

pidData pid = {
		.p = 70, .i = 5, .d = 15,
		.errCum = 0, .valPrev = INIT_TEMP, .peak = 0,
		.peakDetected = false
};


// *********************			BME280			*************************************************

// bme280 HAL interface code below is taken from https://github.com/eziya/STM32_HAL_BME280
// modified for the updated bme280 driver (reused from HW1)

uint8_t bme280Addr = 0x76;
struct bme280_dev dev;
struct bme280_settings dev_settings;
struct bme280_data comp_data;
int8_t rslt;

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  uint8_t dev_id = *(uint8_t *)intf_ptr;
  return (HAL_I2C_Mem_Read(&i2c1, dev_id << 1, reg_addr,
		  I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK) ? 0 : 1;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  uint8_t dev_id = *(uint8_t *)intf_ptr;
  return (HAL_I2C_Mem_Write(&i2c1, dev_id << 1, reg_addr,
		  I2C_MEMADD_SIZE_8BIT, (uint8_t*) data, len, 100) == HAL_OK) ? 0 : 1;
}

// Receives microsec from the new driver, Hal expects to millisec -> convert
void user_delay_us(uint32_t period, void* intf_ptr) {
	  HAL_Delay((period + 999) / 1000);
}

int8_t bme_init() {
	dev.intf_ptr = &bme280Addr;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_us = user_delay_us;

	rslt = bme280_init(&dev);
	if (rslt != 0) return rslt;

	/* BME280 settings */
	dev_settings.osr_h = BME280_OVERSAMPLING_1X;
	dev_settings.osr_p = BME280_OVERSAMPLING_1X;
	dev_settings.osr_t = BME280_OVERSAMPLING_16X;
	dev_settings.filter = BME280_FILTER_COEFF_OFF;
	rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &dev_settings, &dev);
	return rslt;
}

// *********************			DISPLAY			*************************************************

float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;

char hum_str[50];
char temp_str[50];
char pr_str[50];

void ssd_printHello() {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Hello!", Font_11x18, White);
	ssd1306_UpdateScreen();
}

void ssd_printBME() {
	/* Forced mode setting, switched to SLEEP mode after measurement */
	rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev);
	dev.delay_us(50000, &bme280Addr);
	/*Get Data */
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

	if (rslt == BME280_OK)
		{
			temperature = comp_data.temperature / 100.0;
		  	humidity = comp_data.humidity / 1024.0;
		  	pressure = comp_data.pressure / 100.0;

		  	snprintf(hum_str, sizeof(hum_str),"Hum: %03.1f %% ", humidity);
		  	snprintf(temp_str, sizeof(temp_str), "Temp: %03.1f C ", temperature);
		  	//snprintf(pr_str, sizeof(pr_str), "P: %03.1f hPa ", pressure);
		  	if (pid.peakDetected) snprintf(pr_str, sizeof(pr_str), "Peak: %03.1f C", pid.peak);

		  	ssd1306_Fill(Black);
		  	ssd1306_SetCursor(0, 0);
		  	ssd1306_WriteString(hum_str, Font_7x10, White);
		  	ssd1306_SetCursor(0, 20);
		  	ssd1306_WriteString(temp_str, Font_7x10, White);
		  	ssd1306_SetCursor(0, 40);
		  	ssd1306_WriteString(pr_str, Font_7x10, White);
		  	ssd1306_UpdateScreen();
		}
}

void fan_set_duty(float duty)          /* 0..100 % */
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(tim3) + 1;
    __HAL_TIM_SET_COMPARE(tim3, TIM_CHANNEL_1, (uint32_t)(duty * period / 100.0f));
}

double pidUpdate(pidData* pid, double curr, double sp) {
	double pTerm, iTerm, dTerm;
	double err, diff;
	err = curr - sp;
	pTerm = pid->p * err;
	/*
	pid->errCum += err;
	if (pid->i > 0) {
			double lim = 100.0 / pid->i;
			if (pid->errCum >  lim) pid->errCum =  lim;
			if (pid->errCum < -lim) pid->errCum = -lim;
		}*/
	diff = curr - pid->valPrev;
	if (!pid->peakDetected || curr < pid->peak) {   // min temp for the overshoot value
		pid->peak = curr;
		pid->peakDetected = true;
	}
	pid->valPrev = curr;
	dTerm = pid->d * diff;
	double u = pTerm + pid->i * (pid->errCum + err) + dTerm;
	if (u > 0 && u < 100) pid->errCum += err;
	iTerm = pid->i * pid ->errCum;
	return pTerm + iTerm + dTerm;
}

void pidRun(I2C_HandleTypeDef* hpi2c, TIM_HandleTypeDef* hptim3, UART_HandleTypeDef* hpuart2) {
	i2c1 = *hpi2c;
	tim3 = hptim3;
	uart2 = *hpuart2;
	ssd1306_Init();
	bme_init();
	HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_1);
	// Open Loop
	if (!PIDLOOP) {
		fan_set_duty(52.0);		// find P empirically
		float openLoopDuty = (INIT_TEMP - SETPOINT) * OPENLOOP_P;
		fan_set_duty(openLoopDuty);		// Check control law & P for a random setpoint
	}
	while (1) {
		ssd_printBME();
		if (PIDLOOP) {
			double duty = pidUpdate(&pid, comp_data.temperature / 100.0, SETPOINT);
			if (duty < 10) duty = 10;	// the fan stops rotating if duty < 10
			else if (duty > 100) duty = 100; // max
			fan_set_duty(duty);
			HAL_Delay(1000);
		}
	}
}

