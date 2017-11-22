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
#define Kp 45
#define Kd 128

struct sensors_ ref;
int rread(void);

void motor_hard_turn_left(uint32 delay);
void motor_hard_turn_right(uint32 delay);
bool checkVoltage();
void flashLED();
void calibrate(struct sensors_ ref, float * result);
bool isOnBlackLine();
void rick_roll();
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
    uint8 button;
    uint8 leftMotor = 20;
    uint8 rightMotor = 20;

    uint16 checkVoltageDelay = 5000;
    uint8 blackLine = 0;
    uint8 lineDelay = 0;
    uint16 l1W,l1B,l3W,l3B,r1W,r1B,r3W,r3B;
    float result[5];
    bool calibrated = false;
    uint8 leftDir = 0;
    uint8 rightDir = 0;
    
    l3W = 4500;
    l3B = 23999;
    l1W = 3000;
    l1B = 23999;
    r1W = 3300;
    r1B = 23999;
    r3W = 8300;
    r3B = 23999;
    
    CyGlobalIntEnable; 
    sensor_isr_StartEx(sensor_isr_handler);
    
    reflectance_start();
    IR_led_Write(1);
    
    float error = 0;
    float lastError = 0;
    for(;;)
    {
        button = SW1_Read();
        //Calibrated on white line
        if(button == 0){
            if(!calibrated){
                calibrate(ref, result);
                l1W = result[0];
                r1W = result[1];
                l3W = result[3];
                r3W = result[4];
                calibrated = true;
                button = 1;
                //printf("Left 1: %i     Right 1: %i\n", l1W,r1W);
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
                if(button == 0){
                    for(;;){
                        reflectance_read(&ref);
                        //printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);  
                        //float l3Scale = (float)l3B/(ref.l3 - l3W);
                        float r1Scale = (float)r1B/(ref.l1 - l1W);
                        float l1Scale = (float)l1B/(ref.r1 - r1W);
                        //float r3Scale = (float)r3B/(ref.r3 - r3W);
                        
                        error = (r1Scale) - (l1Scale);
                        float motorSpeed = Kp * error + Kd * (error - lastError);
                        lastError = error;
                        
                        float leftMotorSpeed = BASE_SPEED + motorSpeed;
                        if(leftMotorSpeed > MAX_SPEED) leftMotorSpeed = MAX_SPEED;
        
                        float rightMotorSpeed = BASE_SPEED - motorSpeed;
                        if(rightMotorSpeed > MAX_SPEED) rightMotorSpeed = MAX_SPEED;
                
                        //if (rightMotorSpeed < -255) rightMotorSpeed = -255; 
                        //if (leftMotorSpeed < -255) leftMotorSpeed = -255;
                    
                        rightMotor = rightMotorSpeed;
                        leftMotor = leftMotorSpeed;
                        if(ref.r3 >= r3B-r3W && !isOnBlackLine()){
                            rightDir = 1;
                            rightMotor = 125;
                        }
                        if(ref.l3 >= l3B-l3W && !isOnBlackLine()){
                            leftDir = 1; 
                            leftMotor = 125;
                        }
                        if(ref.r3 < r3B-r3W && !isOnBlackLine())
                        {
                            rightDir = 0;
                        }
                        if(ref.l3 < l3B-l3W && !isOnBlackLine())
                        {
                            leftDir = 0;
                        }
                        motor_drive(leftDir,rightDir,leftMotor,rightMotor,1);

                        lineDelay ++;
                        //printf("left : %f   %f    right : %f    %f\n",l1Scale ,l3Scale,r1Scale,r3Scale);
                        //printf("error: %f       motorSpeed: %f\n", error, motorSpeed);
                        //printf("L:%f R:%f\n", leftMotorSpeed,rightMotorSpeed);
                     
                        //checks if passed black line every 20ms
                        if(lineDelay > 20){
                            if(isOnBlackLine()){
                                blackLine++;
                            }
                            if(blackLine > 3){
                                break;
                            }
                            lineDelay = 0;
                        }
                    }
                    motor_forward(0,0);
                    rick_roll();
                    }
            }
        }
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

bool isOnBlackLine(){
    if(ref.l3 > 20000 && ref.l1 > 20000 && ref.r1 > 20000 && ref.r3 > 20000){
        return true;
    }
    return false;
}

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
        printf("line: %i    ", i);
        printf("l:%f r:%f\n",l1[i],r1[i]);
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
}
//*/


/*//ultra sonic sensor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Ultra_Start();                          // Ultra Sonic Start function
    while(1) {
        //If you want to print out the value  
        printf("distance = %5.0f\r\n", Ultra_GetDistance());
        CyDelay(1000);
    }
}   
//*/


/*//nunchuk//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    nunchuk_start();
    nunchuk_init();
    
    for(;;)
    {    
        nunchuk_read();
    }
}   
//*/


/*//IR receiver//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    
    unsigned int IR_val; 
    
    for(;;)
    {
       IR_val = get_IR();
       printf("%x\r\n\n",IR_val);
    }    
 }   
//*/


/*//Ambient light sensor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    
    I2C_Start();
    
    I2C_write(0x29,0x80,0x00);          // set to power down
    I2C_write(0x29,0x80,0x03);          // set to power on
    
    for(;;)
    {    
        uint8 Data0Low,Data0High,Data1Low,Data1High;
        Data0Low = I2C_read(0x29,CH0_L);
        Data0High = I2C_read(0x29,CH0_H);
        Data1Low = I2C_read(0x29,CH1_L);
        Data1High = I2C_read(0x29,CH1_H);
        
        uint8 CH0, CH1;
        CH0 = convert_raw(Data0Low,Data0High);      // combine Data0
        CH1 = convert_raw(Data1Low,Data1High);      // combine Data1

        double Ch0 = CH0;
        double Ch1 = CH1;
        
        double data = 0;
        data = getLux(Ch0,Ch1);
        
        // If you want to print out data
        //printf("%lf\r\n",data);    
    }    
 }   
//*/


/*//accelerometer//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    I2C_Start();
  
    uint8 X_L_A, X_H_A, Y_L_A, Y_H_A, Z_L_A, Z_H_A;
    int16 X_AXIS_A, Y_AXIS_A, Z_AXIS_A;
    
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL1_REG, 0x37);           // set accelerometer & magnetometer into active mode
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL7_REG, 0x22);
    
    
    for(;;)
    {
        //print out accelerometer output
        X_L_A = I2C_read(ACCEL_MAG_ADDR, OUT_X_L_A);
        X_H_A = I2C_read(ACCEL_MAG_ADDR, OUT_X_H_A);
        X_AXIS_A = convert_raw(X_L_A, X_H_A);
        
        Y_L_A = I2C_read(ACCEL_MAG_ADDR, OUT_Y_L_A);
        Y_H_A = I2C_read(ACCEL_MAG_ADDR, OUT_Y_H_A);
        Y_AXIS_A = convert_raw(Y_L_A, Y_H_A);
        
        Z_L_A = I2C_read(ACCEL_MAG_ADDR, OUT_Z_L_A);
        Z_H_A = I2C_read(ACCEL_MAG_ADDR, OUT_Z_H_A);
        Z_AXIS_A = convert_raw(Z_L_A, Z_H_A);
        
        printf("ACCEL: %d %d %d %d %d %d \r\n", X_L_A, X_H_A, Y_L_A, Y_H_A, Z_L_A, Z_H_A);
        value_convert_accel(X_AXIS_A, Y_AXIS_A, Z_AXIS_A);
        printf("\n");
        
        CyDelay(50);
    }
}   
//*/


/*//reflectance//
int main()
{
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
  
    sensor_isr_StartEx(sensor_isr_handler);
    
    reflectance_start();

    IR_led_Write(1);
    for(;;)
    {
        reflectance_read(&ref);
        printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);        //print out 0 or 1 according to results of reflectance period
        
        CyDelay(500);
    }
}   
//*/

 /* //motor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();

    motor_start();              // motor start

    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // movinb backward
       
    motor_stop();               // motor stop
    
    for(;;)
    {

    }
}
//*/
    

/*//gyroscope//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    I2C_Start();
  
    uint8 X_L_G, X_H_G, Y_L_G, Y_H_G, Z_L_G, Z_H_G;
    int16 X_AXIS_G, Y_AXIS_G, Z_AXIS_G;
    
    I2C_write(GYRO_ADDR, GYRO_CTRL1_REG, 0x0F);             // set gyroscope into active mode
    I2C_write(GYRO_ADDR, GYRO_CTRL4_REG, 0x30);             // set full scale selection to 2000dps    
    
    for(;;)
    {
        //print out gyroscope output
        X_L_G = I2C_read(GYRO_ADDR, OUT_X_AXIS_L);
        X_H_G = I2C_read(GYRO_ADDR, OUT_X_AXIS_H);
        X_AXIS_G = convert_raw(X_H_G, X_L_G);
        
        
        Y_L_G = I2C_read(GYRO_ADDR, OUT_Y_AXIS_L);
        Y_H_G = I2C_read(GYRO_ADDR, OUT_Y_AXIS_H);
        Y_AXIS_G = convert_raw(Y_H_G, Y_L_G);
        
        
        Z_L_G = I2C_read(GYRO_ADDR, OUT_Z_AXIS_L);
        Z_H_G = I2C_read(GYRO_ADDR, OUT_Z_AXIS_H);
        Z_AXIS_G = convert_raw(Z_H_G, Z_L_G);
     
        // If you want to print value
        printf("%d %d %d \r\n", X_AXIS_G, Y_AXIS_G, Z_AXIS_G);
        CyDelay(50);
    }
}   
//*/


/*//magnetometer//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    I2C_Start();
   
    uint8 X_L_M, X_H_M, Y_L_M, Y_H_M, Z_L_M, Z_H_M;
    int16 X_AXIS, Y_AXIS, Z_AXIS;
    
    I2C_write(GYRO_ADDR, GYRO_CTRL1_REG, 0x0F);             // set gyroscope into active mode
    I2C_write(GYRO_ADDR, GYRO_CTRL4_REG, 0x30);             // set full scale selection to 2000dps
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL1_REG, 0x37);           // set accelerometer & magnetometer into active mode
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL7_REG, 0x22);
    
    
    for(;;)
    {
        X_L_M = I2C_read(ACCEL_MAG_ADDR, OUT_X_L_M);
        X_H_M = I2C_read(ACCEL_MAG_ADDR, OUT_X_H_M);
        X_AXIS = convert_raw(X_L_M, X_H_M);
        
        Y_L_M = I2C_read(ACCEL_MAG_ADDR, OUT_Y_L_M);
        Y_H_M = I2C_read(ACCEL_MAG_ADDR, OUT_Y_H_M);
        Y_AXIS = convert_raw(Y_L_M, Y_H_M);
        
        Z_L_M = I2C_read(ACCEL_MAG_ADDR, OUT_Z_L_M);
        Z_H_M = I2C_read(ACCEL_MAG_ADDR, OUT_Z_H_M);
        Z_AXIS = convert_raw(Z_L_M, Z_H_M);
        
        heading(X_AXIS, Y_AXIS);
        printf("MAGNET: %d %d %d %d %d %d \r\n", X_L_M, X_H_M, Y_L_M, Y_H_M, Z_L_M, Z_H_M);
        printf("%d %d %d \r\n", X_AXIS,Y_AXIS, Z_AXIS);
        CyDelay(50);      
    }
}   
//*/


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
