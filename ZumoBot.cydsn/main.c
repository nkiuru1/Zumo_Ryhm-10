/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

#include <project.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "I2C_made.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "IR.h"
#include "Ambient.h"
#include "Beep.h"

#define MAX_SPEED 255
#define BASE_SPEED 255
#define MIN_SPEED 0
#define Kp 85
#define Kd 600

struct sensors_ ref;
int rread(void);

void motor_hard_turn_left(uint32 delay);
void motor_hard_turn_right(uint32 delay);
bool checkVoltage();
void flashLED();
void calibrate(struct sensors_ ref, float * result);
bool isOnBlackLine();
void rick_roll();
void stop();
float limitSpeed(float speed,int min,int max);

/**
 * @file    main.c
 * @brief   
 * @details  ** You should enable global interrupt for operating properly. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

int main()
{
    //Time 00:13:00. Somewhat reliable
    CyGlobalIntEnable; 
    UART_1_Start();
    ADC_Battery_Start();         
    printf("\nBoot\n");
    BatteryLed_Write(0); // Switch led off 
    uint8 button; //Button state
    uint8 leftMotor = 20; //LeftMotor Speed
    uint8 rightMotor = 20; //RightMotor Speed

    uint16 checkVoltageDelay = 5000; //Delay to check voltage every 5 seconds
    uint8 lineDelay = 0; //Delay to start checking if on black line
    uint16 l1W,l1B,l3W,l3B,r1W,r1B,r3W,r3B; //Reflectance sensor black and white values
    float result[5]; //Calibration results
    bool calibrated = false; //Calibration status
    uint8 leftDir = 0;//Direction of Left Motor, 0:forward 1:backward.
    uint8 rightDir = 0;//Direction of Right Motor, 0:forward 1:backward.
    unsigned int IR_val; //IR value
    
    l3B = 23999; //Black line sensor value
    l1B = 23999;
    r1B = 23999;
    r3B = 23999;
    
    CyGlobalIntEnable; 
    sensor_isr_StartEx(sensor_isr_handler);
    
    reflectance_start();
    IR_led_Write(1);
    
    float error = 0;
    float lastError = 0;
    /*
    Main loop
    
    Calibrates robot & positions it at start line & waits for remote to start
    */
    for(;;)
    {
        button = SW1_Read();
        if(button == 0){
            if(!calibrated){
                calibrate(ref, result);
                l1W = result[0];
                r1W = result[1];
                l3W = result[3];
                r3W = result[4];
                calibrated = true;
                button = 1;
            }
            //Robot placed on track & if calibrated, moves forward untill a perpendicular black line.
            if(button == 0 && calibrated){
                CyDelay(500);
                reflectance_read(&ref);
                if(!isOnBlackLine()){
                    for(;;){
                        motor_start();
                        motor_forward(100,1);
                        reflectance_read(&ref);
                        if(isOnBlackLine()){
                            motor_forward(0,0);
                            button = 1;
                            break;
                        }
                    }
                }
                IR_val = get_IR();
                if(IR_val != 1){
                    /*
                    Secondary Loop
                    PD Drive
                    
                    Using calibrated sensor values it Determines if it turns left or right.
                    If outer sensors detect a black line it changes the direction of one of the motors.
                    If values are either above 255 or below 0 they are set at 255 & 0 respectively.
                    Calls isOnBlackLine() to check if all sensors are on the line and stops it accordingly.
                    */
                    for(;;){
                        reflectance_read(&ref);
                        float r1Scale = (float)r1B/(ref.l1 - l1W);
                        float l1Scale = (float)l1B/(ref.r1 - r1W);
                        
                        error = (r1Scale) - (l1Scale);
                        float motorSpeed = Kp * error + Kd * (error - lastError);
                        lastError = error;
                        
                        float leftMotorSpeed = BASE_SPEED + motorSpeed;
                        float rightMotorSpeed = BASE_SPEED - motorSpeed;
                        
                        leftMotorSpeed = limitSpeed(leftMotorSpeed,MIN_SPEED,MAX_SPEED);
                        rightMotorSpeed = limitSpeed(rightMotorSpeed,MIN_SPEED,MAX_SPEED);
                        
                        if (rightMotorSpeed < leftMotorSpeed) leftMotorSpeed = MAX_SPEED; 
                        if (leftMotorSpeed < rightMotorSpeed) rightMotorSpeed = MAX_SPEED;
                        
                        rightMotor = rightMotorSpeed;
                        leftMotor = leftMotorSpeed;
                        
                        if(ref.r3 >= r3B-5000 && !isOnBlackLine()){
                            rightDir = 1;
                            leftDir= 0;
                            rightMotor = 255;
                            leftMotor= 255;
                        }
                        else if(ref.l3 >= l3B-5000 && !isOnBlackLine()){
                            leftDir = 1; 
                            rightDir = 0;
                            leftMotor = 255;
                            rightMotor = 255;
                        }
                        else{
                          leftDir = 0;
                          rightDir = 0;
                        }
                        
                        motor_drive(leftDir,rightDir,leftMotor,rightMotor,1);
                        
                        // Starting delay to begin to check for horizontal line.
                        if(lineDelay <= 100){
                            lineDelay++;
                        }
                     
                        //checks if passed black line every starting 100ms after starting
                        if(isOnBlackLine() && lineDelay > 100){
                            stop();
                        }
                    }
                }
            }
        }
        /*
        Checks voltage & if < 4.0 stops motors & flashes LED.
        */
        if(checkVoltageDelay >= 5000){
            if(checkVoltage()){
                break;
            }
            checkVoltageDelay = 0;
        }
        CyDelay(20);
        checkVoltageDelay +=20;
    }
    flashLED();
    motor_stop();
}
/*
Limits speed to min or max value
*/
float limitSpeed(float speed, int min, int max){
    int limitedSpeed = speed;
    if (limitedSpeed < min) limitedSpeed = min; 
    if(limitedSpeed > max) limitedSpeed = max;
    return limitedSpeed;
}

/*
Detects black line & stops on the second line and plays a tune
*/
void stop(){
    for(;;){
    reflectance_read(&ref);
    motor_forward(255,1);
    if(ref.l3 < 20000 && ref.r3 < 20000){
        for(;;){
            reflectance_read(&ref);
            motor_forward(255,1);
            if(isOnBlackLine()){
                motor_forward(0,0);
                motor_stop();
                rick_roll();
            }
        }
    }
}
}
/*
Plays a fun tune
*/
void rick_roll(){
        Beep(140,153);
        CyDelay(10);
        Beep(140,136);
        CyDelay(10);
        Beep(140,114);
        CyDelay(10);
        Beep(140,136);
        CyDelay(10);
        Beep(340,91);
        CyDelay(10);
        Beep(100,91);
        Beep(290,91);
        CyDelay(10);
        Beep(300,102);
        Beep(590,102);
        CyDelay(10);
        Beep(140,153);
        CyDelay(10);
        Beep(140,136);
        CyDelay(10);
        Beep(140,121);
        CyDelay(10);
        Beep(140,153);
        CyDelay(10);
        Beep(340,102);
        CyDelay(10);
        Beep(100,102);
        Beep(290,102);
        CyDelay(10);
        Beep(300,114);
        Beep(150,114);
        Beep(150,121);
        Beep(290,136);
        CyDelay(10);
        Beep(140,153);
        CyDelay(10);
        Beep(140,136);
        CyDelay(10);
        Beep(140,114);
        CyDelay(10);
        Beep(140,136);
        CyDelay(10);
        Beep(590,114);
        CyDelay(10);
        Beep(290,102);
        CyDelay(10);
        Beep(300,121);
        Beep(140,136);
        CyDelay(10);
        Beep(290,153);
        CyDelay(10);
        Beep(140,153);
        CyDelay(10);
        Beep(590,102);
        CyDelay(10);
        Beep(590,114);
        CyDelay(10);
        
        }
/*
 If all 4 sensors are mostly on black line, returns true
*/
bool isOnBlackLine(){
    if(ref.l3 > 22000 && ref.l1 > 22000 && ref.r1 > 22000 && ref.r3 > 22000){
        return true;
    }
    return false;
}
/*
Gets & converts the value from ADC into Volts & returns true if it is over 4.00
*/
bool checkVoltage(){
    uint16 adcresult = 0;
    float vbat = 0;
    ADC_Battery_StartConvert();
    if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for get ADC converted value
        adcresult = ADC_Battery_GetResult16();
        vbat = (float)adcresult/(float)819;
        vbat *=1.5;
        printf("Vbat: %.6f\n", vbat);
    }
    return vbat < 4.00;
 }   
/*
 +Flashes LED at increasing or decreasing intervals.
*/
void flashLED(){
    bool on = false;  
    int delay = 475;
    int delaySubtract = 10;
    for(;;){
        if(!on){
            BatteryLed_Write(1);
            on = true;
        }else{
            BatteryLed_Write(0);
            on = false;
        }
        
        if(delay == 0){
            delaySubtract = -1;
        }
        if(delay > 475){
            delaySubtract = 10;
        }
        if(delay < 100 && delaySubtract > 0){
            delaySubtract = 1;
        }
        else if(delay < 200 && delaySubtract > 0){
            delaySubtract = 3;
        }
        if(delay > 200 && delaySubtract < 0){
            delaySubtract = -10;
        }
        else if(delay > 100 && delaySubtract < 0){
            delaySubtract = -5;
        }
        delay-=delaySubtract;
        CyDelay(delay);
    }  
}
/*
Gets reflectance value from all 4 sensors over 1 second & returns the average.
*/
void calibrate(struct sensors_ ref, float *result)
{
    float l1[10]={};
    float r1[10]={};
    float l3[10]={};
    float r3[10]={};
    int timer = 0;
    int i = 0;
    
    while(timer <= 1100)
    {
        if(i < 10){
        reflectance_read(&ref);
        l1[i] += ref.l1;
        r1[i] += ref.r1;
        l3[i] += ref.l3;
        r3[i] += ref.r3;
        result[0] += l1[i];
        result[1] += r1[i];
        result[2] += l3[i];
        result[3] += r3[i];
        //printf("line: %i    ", i);
        printf("l:%f r:%f\n",l3[i],r3[i]);
        i++;
        }
        
        timer += 100;
        CyDelay(100);
        
    }
    result[0] /= 10;
    result[1] /= 10;
    result[2] /= 10;
    result[3] /= 10;
    Beep(25,200);
    Beep(25,255);
    Beep(25,10);
    Beep(25,50);
    Beep(25,100);
    Beep(25,150);
}

#if 0
int rread(void)
{
    SC0_SetDriveMode(PIN_DM_STRONG);
    SC0_Write(1);
    CyDelayUs(10);
    SC0_SetDriveMode(PIN_DM_DIG_HIZ);
    Timer_1_Start();
    uint16_t start = Timer_1_ReadCounter();
    uint16_t end = 0;
    while(!(Timer_1_ReadStatusRegister() & Timer_1_STATUS_TC)) {
        if(SC0_Read() == 0 && end == 0) {
            end = Timer_1_ReadCounter();
        }
    }
    Timer_1_Stop();
    
    return (start - end);
}
#endif

/* Don't remove the functions below */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Parameter is not used, suppress unused argument warning */
	int n;
	for(n = 0; n < len; n++) {
        if(*ptr == '\n') UART_1_PutChar('\r');
		UART_1_PutChar(*ptr++);
	}
	return len;
}

int _read (int file, char *ptr, int count)
{
    int chs = 0;
    char ch;
 
    (void)file; /* Parameter is not used, suppress unused argument warning */
    while(count > 0) {
        ch = UART_1_GetChar();
        if(ch != 0) {
            UART_1_PutChar(ch);
            chs++;
            if(ch == '\r') {
                ch = '\n';
                UART_1_PutChar(ch);
            }
            *ptr++ = ch;
            count--;
            if(ch == '\n') break;
        }
    }
    return chs;
}
/* [] END OF FILE */
