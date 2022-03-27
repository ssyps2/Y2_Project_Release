#include <stdio.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include <lcd.h>
#include <wiringPi.h>
#include <sys/time.h>

#define TRIG_PIN 20
#define ECHO_PIN 21
#define LCD_RS	 22
#define LCD_E	 27
#define LCD_D4	 18
#define LCD_D5   23
#define LCD_D6	 24
#define LCD_D7   25

#define DIS_STOP 30

//prototype of the functions
void robot_init();
void distance_detect();
void lcd_display();


//global variables
int robot;
int lcd;
float distance;

int main( )
{
	robot_init();
	
	serialPrintf(robot, "#Bafrfr 030 030 030 030");

	while(1){
		distance_detect();
		lcd_display();
		
		if (distance < DIS_STOP){
			serialPrintf(robot, "#ha");
			delay(1000);
			
			//play music
			system("play /home/pi/Music/LOSER.mp3");
			exit(0);
		}
		else {
			serialPrintf(robot, "#Bafrfr 030 030 030 030");
		}
	}//end of while
}

void robot_init(){
	//serial init
	wiringPiSetupGpio();
	
	robot = serialOpen("/dev/ttyAMA0", 57600); // returns int, -1 for error

	if (robot == -1)
	{
		puts("Error in openning robot\n");
		exit(-1);
	} // end if

	//lcd init
	lcdInit(2, 16, 4, LCD_RS, LCD_E, 
		LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
	
	lcdPosition(lcd, 0, 0);
	lcdPuts(lcd, "Distance");
	delay(50);
	lcdPosition(lcd, 8, 1);
	lcdPuts(lcd, "cm");
	delay(50);
	
	//ultrsonic init
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	
	puts("Robot Init Success\n");
}

void distance_detect(){
	struct timeval time_begin, time_end;
	
	float duration;
	
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);
	
	//while (!(digitalRead(ECHO_PIN) == 1) && puts("!"));
	while (!(digitalRead(ECHO_PIN) == 1));
	gettimeofday(&time_begin, NULL);
	//while (!(digitalRead(ECHO_PIN) == 0) && puts("@"));
	while (!(digitalRead(ECHO_PIN) == 0));
	gettimeofday(&time_end, NULL);
	
	duration = (time_end.tv_sec * 1000000 + time_end.tv_usec) 
		- (time_begin.tv_sec * 1000000 + time_begin.tv_usec);
		
	distance = (duration / 2) * 34300 / 1000000;
	printf("Distance: %4.2f cm\n", distance);
}

void lcd_display(){
	if (distance > 400 || distance <= 2){
		lcdPosition(lcd, 0, 1);
		lcdPuts(lcd, "OUT-VAL  ");
		delay(200);
	}
	else {
		lcdPosition(lcd, 0, 1);
		lcdPuts(lcd, "        ");
		delay(50);
		lcdPosition(lcd, 0, 1);
		lcdPrintf(lcd, "%4.2f", distance);
		delay(200);
	}
}
