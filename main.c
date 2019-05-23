/*
 * Seguidor_Linea_prototipo.c
 *
 * Created: 26/03/2019 13:45:19
 * Author : Enrique
 */ 

#include <avr/io.h>
#include "Motors.h"

int main(void)
{
	int left_sensor;
	int right_sensor;
	DDRB &=(~(1<<DDB3))&(~(1<<DDB4));
	left_sensor =0;
	right_sensor=0;

	Motors_init();
	

while(1)
{
		left_sensor =PINB&0b00001000;
		right_sensor=PINB&0b00010000;
	
	if((left_sensor==0b00000000) & (right_sensor==0b00000000)) //if both sensors "off"
	{
		Motor_RIGHT(MOTOR_FORWARD,120); //Go Forth, no matter!
		Motor_LEFT(MOTOR_FORWARD,120);
	}
	if((left_sensor==0b00001000) & (right_sensor==0b00010000)) //if both sensors "on"
	{
		Motor_RIGHT(MOTOR_FORWARD,120); //Go Forth!
		Motor_LEFT(MOTOR_FORWARD,120);
	}
	if((left_sensor==0b00000000) & (right_sensor==0b00010000))
	{
		Motor_RIGHT(MOTOR_BACKWARD,60); //Left turn!
		Motor_LEFT(MOTOR_FORWARD,120);
	}
	if((left_sensor==0b00001000) & (right_sensor==0b00000000))
	{
		Motor_RIGHT(MOTOR_FORWARD,120); //Right turn!
		Motor_LEFT(MOTOR_BACKWARD,60);
	}
}

}

