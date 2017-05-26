/*
 * Proto.h
 *
 *  Created on: Dec 6, 2016
 *      Author: leenovoz510
 */

#ifndef PROTO_H_
#define PROTO_H_

#define LCD_RS_PIN		  6
#define LCD_RW_PIN		  5
#define LCD_ENABLE_PIN	  7

#define Left_Sensor 0
#define Right_Sensor 1



void EF_void_LCD_goto(char y, char x);
void EF_Send_char(char Value);
void EF_send_command(char comand);
void EF_send_Data(char data);
void EF_CLEAR_LCD();
void EF_void_LCD_print(char *string);
void EF_send_a_string(char *string_of_characters);

volatile uint32_t overFlowCounter = 0;
volatile uint32_t trig_counter = 0;
volatile uint32_t no_of_ticks = 0;


#define TRIG_BIT    2             // Trigger Pin
#define ECHO_BIT    3             // Echo Pin

// Speed of sound
// Default: 343 meters per second in dry air at room temperature (~20C)
#define SPEED_OF_SOUND  343
#define MAX_SONAR_RANGE 10          // This is trigger + echo range (in meters) for SR04
#define DELAY_BETWEEN_TESTS 500     // Echo canceling time between sampling. Default: 500us
#define TIMER_MAX 65535             // 65535 for 16 bit timer and 255 for 8 bit timer

/* ...- . . .-. --- -... --- -
 * Do not change anything further unless you know what you're doing
 * */
#define TRIG_ERROR -1
#define ECHO_ERROR -2

#define CYCLES_PER_US (F_CPU/1000000)// instructions per microsecond
#define CYCLES_PER_MS (F_CPU/1000)      // instructions per millisecond
// Timeout. Decreasing this decreases measuring distance
// but gives faster sampling
#define SONAR_TIMEOUT ((F_CPU*MAX_SONAR_RANGE)/SPEED_OF_SOUND)

#define CONVERT_TO_CM ((10000*2)/SPEED_OF_SOUND)    // or simply 58


void init_sonar();
void intiate_Sensor();
void trigger_sonar();
void EF_Move_Backward();
void EF_Move_Forward();
void EF_Move_Right();
void EF_Move_Left();

unsigned int read_sonar();

void EF_Move_Backward(){
	PORTA = 0b01000010;
}

void EF_Move_Forward(){
	PORTA = 0b10000001;
}

void EF_Move_Right(){
	PORTA = 0b10000000;
}
void EF_Move_Left(){
	PORTA = 0b00000001;
}
void EF_stop(){
	PORTA = 0b00000000;
}

#endif /* PROTO_H_ */
