#include <stdio.h>
#include <stdint.h>
#include "libschsat.h"
#define DEFAULT_MIN_OUT  0
#define DEFAULT_MAX_OUT  255
#define SAMPLE_TIME 2e-3
#define INITIAL_INTEGRAL_TERM 0
#define INITIAL_PROCESS_VALUE 0
#define MAX_RPM 5000


void control(void){
	float gyro_x = 0; 
	float gyro_y = 0;
	float gyro_z = 0;
	uint16_t tmp = 0;
	
	gyro_set_offset(0, 4.65, -2.0, 0.712);
	
	// set telemetry perio 50ms...
	uint16_t period = 50;
	// ...for gyro...
	send_unican_message(0x19, 0x0A63, (uint8_t*)&period, 2);
	// ...and wheel...
	send_unican_message(0x0A,0x0A63, (uint8_t*)&period, 2);
	
	// set required rmp orbicraft, proportional koefficient and wheel rmp.
	float rpm_set = 0.0;
	float Kp = 1.233;  //1.5243  //5             //Спутник стабилизируется, но возникает импульс (1) 
	float Ki = 0.5071;	//0.59798  //0.2 При данных коэффициентах (2) ПИД-регулятор работает стабильно
	float Kd = 0.7496;	//0.97135  //1 //5 3 3 неудача // 2 0.2 1.2 неудача
	float error = 0;
	float iError = 0;
	float dError = 0;
	float last_error = 0;
	float motor_set = 1;
	//pid_Initialization(5, 3, 3);
	//pid_setParams(5, 3, 3);
	for(float i = 0; i < 300; i++)
	{
		gyro_request_raw(tmp, &gyro_x, &gyro_y, &gyro_z); // gyro returns degrees per sec!
		printf("Gyro rpm: %f\n rpm: %10f\t error: %10f\t time: %f\t sec", (gyro_z / 360.0 * 60.0), motor_set, error, (i/10));
		
		/*P = derror;
		I = I + derror * dt;
		D = (derror - last_derror) / dt;*/
		//res = p_koeff * P + i_koeff * I + d_koeff * D;
		// simple PID-controller
		float delta_motor_set = (rpm_set - gyro_z / 360.0 * 60) ; // delta_motor_set
		if (fabs(delta_motor_set) > 0.01)
		{
			error = delta_motor_set;
			iError = iError + error * (i/10); //Изменил значение времени. Попытка уменьшить шум. Изначально было деление на i, но так как это время, для большей точности сделал деление на 10.
			dError = (error - last_error) / (i/10);
			motor_set = Kp * error + Ki * iError + Kd * dError;
			last_error = error;
			
		} 
		motor_set_speed(0, motor_set);
		mSleep(100);
		//if (delta_motor_set > MAX_RPM)
			//motor_set_speed(0, 0);
	}
	motor_set_speed(0, 0);
	// set telemetry period 1 sec
	period = 1000;
	send_unican_message(0x19, 0x0A63, (uint8_t*)&period, 2);
	send_unican_message(0x0A, 0x0A63, (uint8_t*)&period, 2);
	puts("Job done!");
}