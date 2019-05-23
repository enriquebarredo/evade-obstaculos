/*
 * Motors.h
 *
 * Created: 04/11/2015 06:48:04 a. m.
 *  Author: JLB
 */ 


#ifndef MOTORS_H_
#define MOTORS_H_

#define MOTOR_STOP 	0
#define MOTOR_BACKWARD	1
#define MOTOR_FORWARD	2

#define ROBO_SPEED 150 //0-255

void Motors_init();
void Motor_RIGHT(uint8_t dir,uint8_t speed);
void Motor_LEFT(uint8_t dir,uint8_t speed);

#endif /* MOTORS_H_ */