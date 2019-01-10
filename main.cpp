#include "mbed.h"
#include "TextLCD.h"
#include "ultrasonic.h"
#include "ledControl.h"

#define led_cmd     1
#define buzzer_cmd  2
#define motor_cmd   3
#define ON          1
#define OFF         0

void buzzer();
void RGB_LED(void);
void analog_sen();
void digital_sen();
void PIR_sen();
void PSD_sen();
int dht_read(void);
void dth11();
//void dist(int distance);
void ultrasonic_sensor();
void DC_motor(void);
void callback();        // uart receive
void sensordata(void);

void dist(int distance)
{
    //put code here to happen when the distance is changed
    printf("Distance %dmm\r\n", distance);
}

DigitalOut led(LED1);
// buzzer
DigitalOut mybuzzer(PC_4); 
// led
DigitalOut myled [4] = { PC_9, PB_2, PC_2, PC_3};  
// switch
DigitalIn mysw[4] = {PC_10,PC_11, PC_12, PA_13 }; 
// RGB LED
PwmOut mypwmR(PB_5);
PwmOut mypwmG(PB_3);
PwmOut mypwmB(PA_10);
// 2x16 Text LCD 

TextLCD lcd(PC_6,PC_8,PC_5,PC_0,PB_7,PC_13,PB_12);

// analog sensor
AnalogIn analog_value(PC_1);
// digital sensor
DigitalIn Sensor_DIN(PA_6);
// PIR sensor 
DigitalIn pir(PD_2);
// PSD sensor
AnalogIn psd(PB_0);   
// DHT11 
#define DHTLIB_OK                0
#define DHTLIB_ERROR_CHECKSUM   -1
#define DHTLIB_ERROR_TIMEOUT    -2
Timer tmr;
DigitalInOut data_pin(PB_10);
//ultra sonic 
ultrasonic mu(PC_7, PA_9, .1, 1, &dist);    //Set the trigger pin to D8 and the echo pin to D9 have updates every .1 seconds and a timeout after 1 second, and call dist when the distance changes
//DC motor
DigitalOut mo_in1(PB_14);
DigitalOut mo_in2(PB_15);
DigitalOut mo_en(PB_1);

char buffer[17];
int humidity;
int temperature;
float analog_meas; // analog sensor input data 
char din_dect;      // digital sensor input data
char pir_dect;      // pir sensor detect
float psd_meas;     // psd sensor data
//long dist_cal;      // psd sensor distance
//int distance;

// serial comm
Serial pi(PA_11, PA_12);

bool receive_flag = 0;
unsigned int buf_cnt = 0;
char uartBuff[1000];
char uartRcev[1000];
char sensor_data[6];
int main(void)
{
    lcd.gotoxy(1,1);
    lcd.printf("Core-1000");
    lcd.gotoxy(1,2);
    lcd.printf("IoT Practice");
 
    buzzer();
    pi.baud(115200);
    pi.attach(&callback);
    
    while (1) 
    {
        if(receive_flag == 1)   // pi -> nucleo cmd 
        {
     
            printf("%d %d \n\r", uartRcev[0], uartRcev[1]);
            // receive command -- actuators
            if(uartRcev[0] == led_cmd) // RGB LED ON/OFF
            {
       
                if(uartRcev[1] == ON) // RGB ON
                {
                    mypwmR.period_ms(10);
                    mypwmR.pulsewidth_ms(5);
                }
                else 
                {
                    led = !led;
                    mypwmR.pulsewidth_ms(0);
                }
            }
            if(uartRcev[0] == buzzer_cmd) // Buzzer ON/OFF
            {
                if(uartRcev[1] == ON) // buzzer ON
                {
                    mybuzzer = 1; 
                }
                else 
                {
                    mybuzzer = 0; 
                }
            }
            if(uartRcev[0] == motor_cmd) // Motor ON/OFF
            {
                if(uartRcev[1] == ON) // motor ON
                {
                    mo_in1 = 0;
                    mo_in2 = 1;
                    mo_en = 1;  
                }
                else 
                {
                    mo_in1 = 0;
                    mo_in2 = 0;
                    mo_en = 0;  
                }
            }
            receive_flag = 0;
        }
        sensordata();
        wait(1);
        
        // send sensor data
    } 
}

void sensordata(void)
{
    char analog_val[10];
    int analog;
    analog_sen();   //analog_meas
    digital_sen();   //din_dect
    PIR_sen();      //pir_dect
//  PSD_sen();      //psd_meas
    dht_read();     //humidity, temperature
//  ultrasonic_sensor();   //distance

    analog = analog_meas;   // float -> int
    analog_val[0] = (char)(analog >> 8);
    analog_val[1] = (char)(analog);
    pi.putc(analog_val[0]);
    pi.putc(analog_val[1]);
    pi.putc(din_dect);
    pi.putc(pir_dect);
    pi.putc(temperature);
    pi.putc(humidity);
    
//    printf("%d %d %d %d %d %d\r\n", analog_val[0], analog_val[1], din_dect, pir_dect, temperature, humidity);  // analog, digital, pir, temp, humi // use windows debug 

}

void callback() {
    char buf = 0;
    myled[0] =1;
    buf = pi.getc();
   
    uartBuff[buf_cnt] = buf;
    if(uartBuff[buf_cnt] == '\r' )
    {
   
        //printf("%c", buf);
        uartBuff[buf_cnt+1] = 0;
        memcpy(uartRcev,uartBuff,sizeof(uartRcev) );
        memset(uartBuff,0,sizeof(uartBuff));
        buf_cnt = 0;
        receive_flag = 1;
    }
    else
    {
        buf_cnt++;
    }
}

////////////////////////////////////////////////////////////////////////////////
// sensors /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void analog_sen()
{
        analog_meas = analog_value.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
        analog_meas = analog_meas * 3300; // Change the value to be in the 0 to 3300 range
        //data char type --> send 
//        printf("measure = %.0f mV\n\r", analog_meas);
}
void digital_sen()
{
        if (Sensor_DIN == 0)    din_dect = 1; // detected 
        else                    din_dect = 0; // not detected           
}
void PIR_sen()
{
    if(pir)
    {
        pir_dect = 1; 
        wait(0.25);
    }
    else    pir_dect = 0;       
}
void PSD_sen()
{
    psd_meas = psd.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
    //psd_meas = psd_meas * 3300; // Change the value to be in the 0 to 3300 range
    psd_meas = (psd_meas * 3300); 
}

 

// DHT11 Library
int dht_read(void){
    // BUFFER TO RECEIVE
    uint8_t bits[5];
    uint8_t cnt = 7;
    uint8_t idx = 0;
    
    tmr.stop();
    tmr.reset();

    // EMPTY BUFFER
    for(int i=0; i< 5; i++) bits[i] = 0;

    // REQUEST SAMPLE
    data_pin.output();
    data_pin.write(0);
    wait_ms(18);
    data_pin.write(1);
    wait_us(40);
    data_pin.input();

    // ACKNOWLEDGE or TIMEOUT
    unsigned int loopCnt = 10000;
    
    while(!data_pin.read())if(!loopCnt--)return DHTLIB_ERROR_TIMEOUT;

    loopCnt = 10000;
    
    while(data_pin.read())if(!loopCnt--)return DHTLIB_ERROR_TIMEOUT;

    // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
    for(int i=0; i<40; i++){      
        loopCnt = 10000;      
        while(!data_pin.read())if(loopCnt-- == 0)return DHTLIB_ERROR_TIMEOUT;
        //unsigned long t = micros();
        tmr.start();
        loopCnt = 10000;    
        while(data_pin.read())if(!loopCnt--)return DHTLIB_ERROR_TIMEOUT;
        if(tmr.read_us() > 40) bits[idx] |= (1 << cnt);    
        tmr.stop();
        tmr.reset();      
        if(cnt == 0){   // next byte?     
            cnt = 7;    // restart at MSB
            idx++;      // next byte!       
        }else cnt--;   
    }
    // WRITE TO RIGHT VARS
    // as bits[1] and bits[3] are allways zero they are omitted in formulas.
    humidity    = bits[0]; 
    temperature = bits[2]; 

    uint8_t sum = bits[0] + bits[2];  
    if(bits[4] != sum)return DHTLIB_ERROR_CHECKSUM;  
    return DHTLIB_OK;
}

void dth11()
{
        if(!dht_read())
        {
            printf("Hum %2d%%  Tmp %2dc\n\r", humidity, temperature);
            wait(0.5);  
        }
        else
        {
            printf("Sensor Error !!!\n\r");
        }        
}     


void ultrasonic_sensor()
{
    mu.startUpdates();//start mesuring the distance
    //Do something else here
    mu.checkDistance();     //call checkDistance() as much as possible, as this is where the class checks if dist needs to be called.
}

////////////////////////////////////////////////////////////////////////////////
// actuators  //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void buzzer()
{
    mybuzzer = 1; // LED is ON
    wait(0.2); // 200 ms
    mybuzzer = 0; // LED is OFF
    wait(1.0); // 1 sec
}
void RGB_LED(void)
{
    mypwmR.period_ms(10);
    mypwmR.pulsewidth_ms(5);
    wait(1);
    mypwmR.pulsewidth_ms(0);
    
    mypwmG.period_ms(10);
    mypwmG.pulsewidth_ms(5);
    wait(1);
    mypwmG.pulsewidth_ms(0);
    
    mypwmB.period_ms(10);
    mypwmB.pulsewidth_ms(5);
    wait(1);
    mypwmB.pulsewidth_ms(0);     
}

void DC_motor(void)
{
      mo_in1 = 0;
      mo_in2 = 1;
      mo_en = 1;  
/*
      wait_ms( 2000 );
      mo_en = 0; 
      wait_ms( 2000 );
      mo_in1 = 1;
      mo_in2 = 0;
      mo_en = 1; 
      wait_ms( 2000 );
      mo_en = 0; 
      wait_ms( 2000 );
*/
}