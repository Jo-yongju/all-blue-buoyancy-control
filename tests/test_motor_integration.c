#include "main.h"
#include "motor_test.h"
#include "control_app.h"
#include "app_transport.h"
#include "app_inputs.h"
#include "slider_sensor.h"
#include "runtime_port.h"
#include "live_io.h"
#include "trial_receiver.h"
#include "team_link.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
RCC_Type test_rcc;TIM_Type test_tim;GPIO_Type test_gpioa,test_gpiob;
static USART_Type serial;
UART_HandleTypeDef hcom_uart[1]={{&serial}};
static uint32_t now=1000;static const char *input="";static char output[4096];
uint32_t HAL_GetTick(void){return now;}
uint32_t HAL_RCC_GetPCLK1Freq(void){return 170000000;}
void HAL_GPIO_Init(GPIO_Type *p,GPIO_InitTypeDef *g){(void)p;(void)g;}
void Error_Handler(void){abort();}
int HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *b,uint16_t n,uint32_t t){
    (void)u;(void)n;(void)t;if(!*input)return 1;*b=(uint8_t)*input++;return 0;
}
int HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *s,uint16_t n,uint32_t t){
    (void)u;(void)t;size_t old=strlen(output);assert(old+n<sizeof(output));memcpy(output+old,s,n);output[old+n]=0;return 0;
}

TIM_Type test_tim4;GPIO_Type test_gpioc;
void Hardware_Off(void);
void Hardware_InitOutputs(void);
void Hardware_Write(float,float,uint32_t);
static unsigned live_inits,live_writes;
static float live_a,live_b;
static bool mock_radio;
static unsigned mock_switch=1500;
bool RuntimePort_ReadByte(uint8_t *b){if(!*input)return false;*b=(uint8_t)*input++;return true;}
bool RuntimePort_WriteByte(uint8_t b){size_t n=strlen(output);assert(n+1<sizeof output);output[n]=(char)b;output[n+1]=0;return true;}
void RuntimePort_InitClock(void){}
bool RuntimePort_ClockReady(void){return true;}
uint32_t RuntimePort_Cycles(void){return 0;}
uint32_t RuntimePort_ElapsedUs(uint32_t v){(void)v;return 0;}
void LiveIO_Off(void){Hardware_Off();live_a=live_b=0;}
void LiveIO_InitOutputs(void){Hardware_InitOutputs();++live_inits;}
void LiveIO_Write(float a,float b,uint32_t t){Hardware_Write(a,b,t);live_a=a;live_b=b;++live_writes;}
static bool imu_init_ok, imu_sample_ok, cancel_imu_init;
static unsigned imu_init_calls;
bool LiveIMU_Init(void){++imu_init_calls;if(cancel_imu_init)AppInputs_RequestImu(false);return imu_init_ok;}
bool LiveIMU_Sample(float *p){*p=12.5f;return imu_sample_ok;}
void AppTasks_FormatStats(char *p,size_t n){snprintf(p,n,"RTOS mock statistics\r\n");}
bool TrialReceiver_Init(void){return true;}
static unsigned receiver_channel;
static bool receiver_reverse_sw,receiver_reverse_ch4;
void TrialReceiver_Clear(void){receiver_channel=0;}
void TrialReceiver_Configure(unsigned ch,bool sw,bool axis){receiver_channel=ch;receiver_reverse_sw=sw;receiver_reverse_ch4=axis;}
void AppTasks_NotifyImu(void){}
bool LiveIMU_Pending(void){return false;}
bool TrialReceiver_ReadCommand(ReceiverCommand *c){
 if(!mock_radio)return false;
 uint16_t ch[14]; for(unsigned i=0;i<14;i++)ch[i]=1500;
 ch[4]=mock_switch;
 *c=ReceiverCommand_Decode(ch,now,receiver_channel,receiver_reverse_sw,receiver_reverse_ch4);
 return true;
}
static bool distinct_axes, axis2_ok=true;
static uint16_t sensor2_raw=529;
static uint32_t adc_delay_ms;
bool SliderSensor_ReadAxis(unsigned a,uint16_t *r){
    now+=adc_delay_ms;
    if (!SliderSensor_Read(r) || (a==1 && !axis2_ok)) return false;
    if (distinct_axes && a==1) *r=sensor2_raw;
    return true;
}
static void drain(void){for(int i=0;i<100;++i)MotorTest_Task();}
static void send(const char *s){input=s;output[0]=0;do {MotorTest_Task();}while(*input);drain();}

static void off(void){assert(TIM3->CCR1==0&&!(TIM3->CCER&TIM_CCER_CC1E));}
static bool sensor_ok=true;
static bool mdd_plant;
static uint16_t sensor1_raw;
bool SliderSensor_Read(uint16_t *raw){*raw=mdd_plant?sensor1_raw:2048;return sensor_ok;}
bool SliderSensor_Init(void){return true;}
/* Closed-loop test plant driven by the actual fake TIM3/GPIO registers, not
 * forced target readings. Model: no motion below 90% duty, no inertia.
 * This tests software behavior; it does not predict the real mechanics. */
static uint16_t plant_adc(float mm) {
    float adc=mm<=29.24f ? mm*529/29.24f : 529+(mm-29.24f)*3566/29.24f;
    if(adc<0)adc=0;
    if(adc>4095)adc=4095;
    return (uint16_t)lroundf(adc);
}
static void run_continuous_plant(void) {
    distinct_axes=true;sensor_ok=true;send("BENCH2 ON\n");
    assert(strstr(output,"drive=CONTINUOUS100"));
    assert(strstr(output,"stop=ARRIVAL_OR_CROSSING"));
    const char *commands[]={"BUOY HOVER\n","BUOY DOWN\n","BUOY HOVER\n","BUOY UP\n"};
    const unsigned goals[]={529,4095,529,0};
    for(unsigned repeat=0;repeat<2;repeat++) {
        float mm=0;
        float speed=repeat?6.0f:2.5f;
        for(unsigned move=0;move<4;move++) {
            sensor2_raw=plant_adc(mm);now+=200;
            send("ARM\n");send(commands[move]);assert(strstr(output,"OK MOVE SENSOR2"));
            unsigned on=0,off_count=0,continuous=0,max_on=0;
            for(unsigned i=0;i<6000;i++) {
                bool enabled=(TIM3->CCER&TIM_CCER_CC1E)!=0;
                if(enabled && TIM3->CCR1) {
                    assert(TIM3->CCR1==1000); // Same register duty as JOG 100.
                    int sign=GPIOB->BSRR==GPIO_PIN_1 ? 1 : -1;
                    assert(GPIOB->BSRR==GPIO_PIN_0 || GPIOB->BSRR==GPIO_PIN_1);
                    mm+=sign*speed*.01f;
                    if(mm<0)mm=0;
                    if(mm>58.48f)mm=58.48f;
                    on++;continuous++;
                    if(continuous>max_on)max_on=continuous;
                } else {off_count++;continuous=0;}
                sensor2_raw=plant_adc(mm);
                now+=10;MotorTest_Tick(now);MotorTest_Task();
                assert(!strstr(output,"ERR MOVE"));
                if(strstr(output,"DONE MOVE DONE"))break;
            }
            assert(strstr(output,"DONE MOVE DONE"));off();
            assert(on>0 && max_on==on); // No off intervals once motion starts.
            int delta=(int)sensor2_raw-(int)goals[move];
            assert(move==1 || move==3 ? delta==0 : abs(delta)<=5);
            printf("PASS closed-loop plant speed=%.1f move=%u adc=%u on=%u off=%u\n",speed,move,sensor2_raw,on,off_count);
        }
    }
    // Startup-delay model is a software test assumption, not measured hardware data.
    for(unsigned side=0;side<2;side++) {
        float mm=side?30.5f:28.0f;unsigned powered_ms=0;
        sensor2_raw=plant_adc(mm);now+=200;send("ARM\n");send("BUOY HOVER\n");
        for(unsigned i=0;i<2000;i++) {
            bool powered=(TIM3->CCER&TIM_CCER_CC1E)&&TIM3->CCR1;
            powered_ms=powered?powered_ms+10:0;
            if(powered_ms>=40) mm+=(GPIOB->BSRR==GPIO_PIN_1?1:-1)*1.0f*.01f;
            sensor2_raw=plant_adc(mm);now+=10;MotorTest_Tick(now);MotorTest_Task();
            assert(!strstr(output,"ERR MOVE"));
            if(strstr(output,"DONE MOVE DONE"))break;
        }
        assert(strstr(output,"DONE MOVE DONE"));off();assert(abs((int)sensor2_raw-529)<=5);
        printf("PASS startup-delay 40ms HOVER side=%u adc=%u\n",side,sensor2_raw);
    }
    // Near target: actual PWM register stays full duty, without pulse gaps.
    sensor2_raw=550;now+=200;send("ARM\n");send("BUOY HOVER\n");
    bool started=false;
    for(unsigned i=0;i<100;i++) {
        now+=10;MotorTest_Tick(now);MotorTest_Task();
        if(TIM3->CCR1)started=true;
        if(started)assert(TIM3->CCR1==1000);
    }
    assert(started);
    send("!\n");off();
    for(unsigned i=0;i<30;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();off();}
    // Continuous power still stops and latches on no net progress.
    sensor2_raw=3900;now+=200;send("ARM\n");send("BUOY DOWN\n");
    for(unsigned i=0;i<850;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    off();assert(strstr(output,"ERR MOVE NO_MOVEMENT"));
    send("ARM\n");assert(strstr(output,"OK ARM"));off();
    puts("PASS continuous PWM, async STOP/no restart, stall watchdog/latch");
}
static void team_tick(unsigned sw,bool failsafe,bool publish) {
    now+=10;
    if(publish) {
        TeamLink_PublishRadio(1500,(uint16_t)sw,failsafe,now);
        TeamLink_PublishPitch(0,true,now);
    }
    MotorTest_Tick(now);MotorTest_Task();
}
static void run_team(void) {
    distinct_axes=true;mock_radio=true;mock_switch=1000; /* standalone path must be ignored */
    TeamLink_PublishRadio(1500,1000,false,now);
    assert(TeamLink_RequestObserve());MotorTest_Task();
    assert(TeamLink_ReadStatus().active && !TeamLink_Read().have_radio);
    assert(imu_init_calls==0 && receiver_channel==0); /* no duplicate peripheral owner */
    team_tick(1500,false,true);
    assert(TeamLink_ReadStatus().ready_to_arm);
    assert(TeamLink_RequestArm());MotorTest_Task();
    for(unsigned i=0;i<70;i++)team_tick(1500,false,true);
    assert(TeamLink_ReadStatus().armed);
    for(unsigned i=0;i<60;i++)team_tick(2000,false,true);
    assert(live_a>0 && live_b>0);
    team_tick(2000,true,true);
    assert(live_a==0 && live_b==0 && TeamLink_ReadStatus().fault==SY_RADIO);
    for(unsigned i=0;i<19;i++){team_tick(2000,false,true);assert(live_a==0 && live_b==0);}
    for(unsigned i=0;i<20;i++)team_tick(2000,false,true);
    assert(live_a>0 && live_b>0); /* held DOWN, no reset/ARM */
    for(unsigned i=0;i<32;i++)team_tick(2000,false,false);
    assert(TeamLink_ReadStatus().fault==SY_RADIO && live_a==0 && live_b==0);
    for(unsigned i=0;i<40;i++)team_tick(2000,false,true);
    assert(live_a>0 && live_b>0);
    uint32_t stamp=now;
    TeamLink_PublishRadio(1000,1000,false,stamp-1);
    assert(TeamLink_Read().radio.channels[5]==2000);
    TeamLink_PublishPitch(NAN,true,stamp+1);
    assert(!TeamLink_Read().imu.valid);
    TeamLink_PublishPitch(50,true,stamp-1);assert(!TeamLink_Read().imu.valid);
    send("STOP\n");
    for(unsigned i=0;i<40;i++)team_tick(2000,false,true);
    assert(!TeamLink_ReadStatus().active && live_a==0 && live_b==0);
    now+=200;send("ARM\n");send("JOG 100 300\n");
    assert(strstr(output,"OK JOG"));assert(TIM3->CCR1==1000);
    send("STOP\n");off();
    puts("PASS TEAM task API -> shared LIVE controller, no UART/IMU init, failsafe/stale stop, held switch recovery, timestamp ordering, invalid IMU, STOP isolation, legacy JOG restored");
}

static void all_off(void) {off();assert(TIM4->CCR1==0&&TIM4->CCR4==0&&TIM4->CCER==0);}
static void run_mdd(void) {
    send("DRIVER MDD10A 1\n");assert(strstr(output,"OK DRIVER MDD10A 1 JOG_ONLY"));all_off();
    assert(TIM4->PSC==169&&TIM4->ARR==999);
    send("JOG 50 200\n");assert(strstr(output,"ARM first"));all_off();
    for(unsigned channel=1;channel<=2;channel++) for(int sign=-1;sign<=1;sign+=2) {
        char cmd[64];snprintf(cmd,sizeof cmd,"DRIVER MDD10A %u\n",channel);send(cmd);all_off();
        now+=200;send("ARM\n");snprintf(cmd,sizeof cmd,"JOG %d 200\n",sign*50);send(cmd);
        assert(strstr(output,"OK JOG"));off();
        assert(TIM4->CCR1==(channel==1?500u:0u));assert(TIM4->CCR4==(channel==2?500u:0u));
        assert(TIM4->CCER==(channel==1?TIM_CCER_CC1E:TIM_CCER_CC4E));
        uint32_t pin=channel==1?GPIO_PIN_4:GPIO_PIN_7;
        assert(GPIOC->BSRR==(sign>0?pin:pin<<16));
        for(unsigned i=0;i<19;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();assert(TIM4->CCER);}
        now+=10;MotorTest_Tick(now);all_off();drain();assert(strstr(output,"DONE STOP"));
    }
    now+=200;send("ARM\n");send("JOG 100 20000\n");assert(TIM4->CCR4==1000);
    send("DRIVER MDD10A 1\n");all_off();send("JOG 50 200\n");all_off();assert(strstr(output,"ARM first"));
    now+=200;send("ARM\n");send("JOG 100 20000\n");assert(TIM4->CCR1==1000);
    send("STOP\n");all_off();send("STATUS\n");assert(strstr(output,"driver=MDD10A channel=1"));
    now+=200;send("ARM\n");send("JOG -100 20000\n");send("!\n");all_off();
    now+=200;send("ARM\n");send("JOG 50 20000\n");now+=101;MotorTest_Tick(now);all_off();drain();
    now+=200;send("ARM\n");send("JOG 50 20000\n");MotorTest_CommunicationStop(true);drain();all_off();send("\n");
    send("DRIVER MDD10A 3\n");assert(strstr(output,"ERR DRIVER"));all_off();
    send("BENCH2 ON\n");now+=200;send("ARM\n");send("BUOY HOVER\n");assert(strstr(output,"ERR "));all_off();
    send("ARM\n");send("MOVE 20\n");assert(strstr(output,"ERR "));all_off();
    send("MODE SIM\n");send("DRIVER MDD10A 1\n");assert(strstr(output,"ERR send STOP"));all_off();
    send("STOP\n");send("DRIVER L298N\n");assert(strstr(output,"OK DRIVER L298N"));
    now+=200;send("ARM\n");send("JOG 50 200\n");assert(TIM3->CCR1==500&&TIM4->CCER==0);send("STOP\n");all_off();
    puts("PASS MDD10A actual output registers: both channels/signs/duty, ARM, deadline, STOP/!, channel switch, watchdog/UART stop, mode exclusion, L298N regression");
}


static uint16_t mdd_adc(float mm) {
    float a=mm<=29.24f?mm*522/29.24f:522+(mm-29.24f)*3573/29.24f;
    return (uint16_t)lroundf(fmaxf(0,fminf(a,4095)));
}
static void mdd_tick(void) {now+=10;MotorTest_Tick(now);MotorTest_Task();drain();}
static void run_mdd_position(void) {
    mdd_plant=true;distinct_axes=true;sensor2_raw=4095;sensor1_raw=0;
    send("BENCH1 ON\n");assert(strstr(output,"ERR BENCH1"));all_off();
    send("DRIVER MDD10A 1\n");send("BENCH1 ON\n");
    assert(strstr(output,"OK BENCH1 MDD10A1 PA0"));
    send("BENCH1 STATUS\n");assert(strstr(output,"HOVER_adc=522"));
    const char *cmd[]={"BUOY HOVER\n","BUOY DOWN\n","BUOY HOVER\n","BUOY UP\n"};
    unsigned goal[]={522,4095,522,0};
    for(unsigned repeat=0;repeat<2;repeat++) {
        float mm=0;float speed=repeat?6:2.5f;
        for(unsigned move=0;move<4;move++) {
            sensor1_raw=mdd_adc(mm);now+=200;send("ARM\n");send(cmd[move]);
            assert(strstr(output,"OK MOVE MDD10A1 SENSOR1"));
            bool powered=false;unsigned cycles=0;
            for(unsigned n=0;n<6000;n++) {
                if(TIM4->CCR1) {
                    assert(TIM4->CCR1==500 && TIM4->CCER==TIM_CCER_CC1E);
                    assert(GPIOC->BSRR==GPIO_PIN_4 || GPIOC->BSRR==(GPIO_PIN_4<<16));
                    mm+=(GPIOC->BSRR==(GPIO_PIN_4<<16)?1:-1)*speed*.01f;
                    mm=fmaxf(0,fminf(58.48f,mm));powered=true;cycles++;
                }
                sensor1_raw=mdd_adc(mm);mdd_tick();off();assert(TIM4->CCR4==0);
                assert(!strstr(output,"ERR MOVE"));
                if(strstr(output,"DONE MOVE DONE"))break;
                if(powered)assert(TIM4->CCR1==500); /* no pulses or deceleration */
            }
            assert(strstr(output,"DONE MOVE DONE"));all_off();assert(powered);
            assert(move==1||move==3?sensor1_raw==goal[move]:abs((int)sensor1_raw-(int)goal[move])<=5);
            printf("PASS MDD1+PA0 position repeat=%u goal=%u adc=%u powered_cycles=%u\n",repeat,goal[move],sensor1_raw,cycles);
        }
    }
    // Shared MOVE uses the same MDD route; STOP and changing profile revoke it.
    sensor1_raw=100;now+=200;send("ARM\n");send("MOVE 20\n");for(unsigned n=0;n<5;n++)mdd_tick();
    assert(TIM4->CCR1==500);send("!\n");all_off();
    for(unsigned n=0;n<30;n++){mdd_tick();all_off();}
    now+=200;send("ARM\n");send("BUOY DOWN\n");for(unsigned n=0;n<5;n++)mdd_tick();assert(TIM4->CCR1);
    send("BENCH1 OFF\n");all_off();
    send("BUOY DOWN\n");assert(strstr(output,"ERR BENCH1"));all_off();
    send("BENCH1 ON\n");send("DRIVER MDD10A 2\n");send("BENCH1 ON\n");
    assert(strstr(output,"ERR BENCH1"));all_off();
    now+=200;send("ARM\n");send("MOVE 20\n");assert(strstr(output,"ERR position routing"));all_off();
    // New profile must not affect the already working timed JOG channel 2.
    now+=200;send("ARM\n");send("JOG -30 200\n");assert(TIM4->CCR4==300&&TIM4->CCR1==0);
    send("STOP\n");all_off();
    // Sensor loss stops the position route, rather than leaving hardware PWM on.
    send("DRIVER MDD10A 1\n");send("BENCH1 ON\n");now+=200;
    send("ARM\n");send("BUOY DOWN\n");for(unsigned n=0;n<5;n++)mdd_tick();assert(TIM4->CCR1);
    sensor_ok=false;for(unsigned n=0;n<100;n++)mdd_tick();all_off();
    assert(strstr(output,"ERR MOVE SENSOR"));sensor_ok=true;
    puts("PASS MDD position: PA0 exclusively, direction, all goals both ways, MOVE, unused motor off, STOP, profile change, motor2 guard, JOG retained, sensor fault");
}

static void run_mdd2_position(void) {
    mdd_plant=true;distinct_axes=true;sensor1_raw=4095;sensor2_raw=0;
    send("DRIVER MDD10A 1\n");send("BENCH2 ON\n");assert(strstr(output,"ERR BENCH1"));all_off();
    send("DRIVER MDD10A 2\n");send("BENCH2 ON\n");
    assert(strstr(output,"OK BENCH2 MDD10A2 PA1"));
    send("BENCH2 STATUS\n");assert(strstr(output,"HOVER_adc=529"));
    const char *cmd[]={"BUOY HOVER\n","BUOY DOWN\n","BUOY HOVER\n","BUOY UP\n"};
    unsigned goal[]={529,4095,529,0};
    for(unsigned repeat=0;repeat<2;repeat++) {
        float mm=0;float speed=repeat?6:2.5f;
        for(unsigned move=0;move<4;move++) {
            sensor2_raw=plant_adc(mm);now+=200;send("ARM\n");send(cmd[move]);
            assert(strstr(output,"OK MOVE MDD10A2 SENSOR2"));
            bool powered=false;unsigned cycles=0;
            for(unsigned n=0;n<6000;n++) {
                if(TIM4->CCR4) {
                    assert(TIM4->CCR4==500 && TIM4->CCER==TIM_CCER_CC4E);
                    assert(GPIOC->BSRR==GPIO_PIN_7 || GPIOC->BSRR==(GPIO_PIN_7<<16));
                    mm+=(GPIOC->BSRR==(GPIO_PIN_7<<16)?1:-1)*speed*.01f;
                    mm=fmaxf(0,fminf(58.48f,mm));powered=true;cycles++;
                }
                sensor2_raw=plant_adc(mm);mdd_tick();off();assert(TIM4->CCR1==0);
                assert(!strstr(output,"ERR MOVE"));
                if(strstr(output,"DONE MOVE DONE"))break;
                if(powered)assert(TIM4->CCR4==500); /* no pulses or deceleration */
            }
            assert(strstr(output,"DONE MOVE DONE"));all_off();assert(powered);
            assert(move==1||move==3?sensor2_raw==goal[move]:abs((int)sensor2_raw-(int)goal[move])<=5);
            printf("PASS MDD2+PA1 position repeat=%u goal=%u adc=%u powered_cycles=%u\n",repeat,goal[move],sensor2_raw,cycles);
        }
    }
    // Shared MOVE uses the same MDD route; STOP and changing profile revoke it.
    sensor2_raw=100;now+=200;send("ARM\n");send("MOVE 20\n");for(unsigned n=0;n<5;n++)mdd_tick();
    assert(TIM4->CCR4==500);send("!\n");all_off();
    for(unsigned n=0;n<30;n++){mdd_tick();all_off();}
    now+=200;send("ARM\n");send("BUOY DOWN\n");for(unsigned n=0;n<5;n++)mdd_tick();assert(TIM4->CCR4);
    send("BENCH2 OFF\n");all_off();
    send("BUOY DOWN\n");assert(strstr(output,"ERR BENCH1"));all_off();
    send("BENCH2 ON\n");send("DRIVER MDD10A 1\n");send("BENCH2 ON\n");
    assert(strstr(output,"ERR BENCH1"));all_off();
    now+=200;send("ARM\n");send("MOVE 20\n");assert(strstr(output,"ERR position routing"));all_off();
    // New profile must not affect the already working timed JOG channel 2.
    now+=200;send("ARM\n");send("JOG -30 200\n");assert(TIM4->CCR1==300&&TIM4->CCR4==0);
    send("STOP\n");all_off();
    // Sensor loss stops the position route, rather than leaving hardware PWM on.
    send("DRIVER MDD10A 2\n");send("BENCH2 ON\n");now+=200;
    send("ARM\n");send("BUOY DOWN\n");for(unsigned n=0;n<5;n++)mdd_tick();assert(TIM4->CCR4);
    sensor_ok=false;for(unsigned n=0;n<100;n++)mdd_tick();all_off();
    assert(strstr(output,"ERR MOVE SENSOR"));sensor_ok=true;
    puts("PASS MDD position: PA0 exclusively, direction, all goals both ways, MOVE, unused motor off, STOP, profile change, motor2 guard, JOG retained, sensor fault");
}

static void run_dual_position(void)
{
    MotorTest_Init();distinct_axes=true;mdd_plant=true;axis2_ok=true;sensor_ok=true;
    send("DUAL ON\n");assert(strstr(output,"OK DUAL ON"));
    float mm[2]={5,10};
    const char *cmd[]={"DUAL HOVER\n","DUAL DOWN\n","DUAL HOVER\n","DUAL UP\n"};
    const unsigned goals[4][2]={{522,529},{4095,4095},{522,529},{0,0}};
    for(unsigned move=0;move<4;move++) {
        sensor1_raw=mdd_adc(mm[0]);sensor2_raw=plant_adc(mm[1]);
        now+=300;send("ARM\n");assert(strstr(output,"OK ARM"));
        send(cmd[move]);assert(strstr(output,"OK DUAL MOVE"));
        bool together=false,one_finished=false,seen_done[2]={false,false};
        for(unsigned tick=0;tick<6000;tick++) {
            unsigned duty[]={TIM4->CCR1,TIM4->CCR4};
            unsigned bit[]={TIM_CCER_CC1E,TIM_CCER_CC4E};
            float command[]={live_a,live_b};
            bool power[2];
            for(unsigned a=0;a<2;a++) {
                power[a]=duty[a] && (TIM4->CCER&bit[a]);
                if(power[a]) {
                    assert(!seen_done[a]);assert(duty[a]==500);
                    // Two independent speeds; neither sensor is faked at the goal.
                    mm[a]-=command[a]*(a?2.5f:6.f)*.01f;
                    if(mm[a]<0)mm[a]=0;
                    if(mm[a]>58.48f)mm[a]=58.48f;
                }
            }
            if(power[0] && power[1])together=true;
            sensor1_raw=mdd_adc(mm[0]);sensor2_raw=plant_adc(mm[1]);
            unsigned adc[]={sensor1_raw,sensor2_raw};
            for(unsigned a=0;a<2;a++) {
                bool arrived=(goals[move][a]==0 || goals[move][a]==4095)?adc[a]==goals[move][a]:abs((int)adc[a]-(int)goals[move][a])<=5;
                if(!power[a] && arrived)seen_done[a]=true;
            }
            if((seen_done[0] && power[1]) || (seen_done[1] && power[0]))one_finished=true;
            now+=10;MotorTest_Tick(now);MotorTest_Task();
            if(strstr(output,"ERR")) {puts(output);assert(0);}
            if(strstr(output,"DONE DUAL DONE"))break;
        }
        assert(strstr(output,"DONE DUAL DONE"));all_off();
        assert(together && one_finished);
        printf("PASS dual unequal-speed target=%u adc=%u,%u independent arrival\n",move,sensor1_raw,sensor2_raw);
    }
    // Already arrived: neither motor receives a pulse.
    sensor1_raw=522;sensor2_raw=529;now+=300;send("ARM\n");send("DUAL HOVER\n");
    for(unsigned i=0;i<30;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();all_off();}
    assert(strstr(output,"DONE DUAL DONE"));
    now+=300;send("ARM\n");send("DUAL DOWN\n");
    for(unsigned i=0;i<5;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(TIM4->CCR1&&TIM4->CCR4);
    send("!\n");all_off();
    for(unsigned i=0;i<100;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();all_off();}
    puts("PASS dual async STOP, no automatic restart, already-at-goal no pulse");
    // One sensor fails: both outputs off; recovery requires both healthy + ARM.
    now+=300;send("ARM\n");send("DUAL DOWN\n");axis2_ok=false;
    for(unsigned i=0;i<70;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(strstr(output,"ERR DUAL SENSOR_INVALID fault_axis=2") || strstr(output,"ERR DUAL SENSOR fault_axis=2"));all_off();
    send("ARM\n");assert(strstr(output,"ERR RETRY_WAIT"));
    axis2_ok=true;
    for(unsigned i=0;i<40;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    send("ARM\n");assert(strstr(output,"OK ARM"));send("DUAL DOWN\n");assert(strstr(output,"OK DUAL MOVE"));
    for(unsigned i=0;i<850;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(strstr(output,"ERR DUAL NO_MOVEMENT"));all_off();
    puts("PASS dual sensor failure / explicit retry / no-progress stops both");
    for(unsigned i=0;i<40;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    send("ARM\n");send("DUAL DOWN\n");now+=25;MotorTest_Tick(now);all_off();
    drain();assert(strstr(output,"ERR CONTROL_TIMEOUT"));
    puts("PASS dual watchdog stops both without ControlTask");
    send("DRIVER MDD10A 1\n");send("BENCH1 ON\n");send("DUAL STATUS\n");assert(strstr(output,"enabled=0"));
}

int main(void){
    AppTransport_Init();MotorTest_Init();off();assert(TIM3->PSC==169&&TIM3->ARR==999);
    if(getenv("TEST_DUAL")){run_dual_position();return 0;}
    if(getenv("TEST_MDD2_POSITION")){run_mdd2_position();return 0;}
    if(getenv("TEST_MDD_POSITION")){run_mdd_position();return 0;}
    if(getenv("TEST_MDD10A")){run_mdd();return 0;}
    if(getenv("TEST_TEAM")){run_team();return 0;}
    if(getenv("TEST_LIVE_RECOVERY")) {
        mock_radio=true;distinct_axes=true;send("MODE LIVE 5\n");
        now+=10;MotorTest_Task();send("LIVE ARM\n");assert(strstr(output,"OK LIVE ARM"));
        for(unsigned i=0;i<70;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        mock_switch=2000;
        for(unsigned i=0;i<100;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(live_a>0 && live_b>0);
        for(unsigned i=0;i<760;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        send("STATUS\n");assert(strstr(output,"fault=STALL"));assert(live_a==0&&live_b==0);
        for(unsigned i=0;i<50;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(live_a==0&&live_b==0); // repeated DOWN never retries
        mock_switch=1000;
        for(unsigned i=0;i<80;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(live_a<0 && live_b<0); // changed switch, no MCU reset / LIVE ARM
        now+=300;MotorTest_Tick(now);assert(live_a==0&&live_b==0);
        MotorTest_Task();send("STATUS\n");assert(strstr(output,"fault=TIMING"));
        mock_switch=2000;
        for(unsigned i=0;i<80;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(live_a>0 && live_b>0);
        send("STOP\n");
        for(unsigned i=0;i<80;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(live_a==0&&live_b==0);
        puts("PASS physical LIVE application path with mock hardware: stall recovery, timing ISR recovery, repeated switch stays stopped, STOP cancels");
        return 0;
    }
    if(getenv("TEST_CONTINUOUS_MOTOR")){run_continuous_plant();return 0;}
    now+=200;send("JOG 30 300\n");off();assert(strstr(output,"ARM first"));
    send("ARM\n");send("JOG 30 300\n");assert(TIM3->CCR1==300);assert(GPIOB->BSRR==GPIO_PIN_0);
    send("ENC ON\n");assert(strstr(output,"stopped motors"));assert(TIM3->CCR1==300);
    send("ENC STATUS 1\n");assert(strstr(output,"OBSERVE_ONLY"));assert(TIM3->CCR1==300);
    now+=99;MotorTest_Tick(now);assert(TIM3->CCR1==300);MotorTest_Task();
    now+=99;MotorTest_Tick(now);MotorTest_Task();now+=99;MotorTest_Tick(now);MotorTest_Task();
    now+=3;MotorTest_Tick(now);off();drain();assert(strstr(output,"DONE STOP"));
    now+=200;send("ARM\n");send("JOG -50 1000\n");assert(TIM3->CCR1==500&&GPIOB->BSRR==GPIO_PIN_1);
    now+=101;MotorTest_Tick(now);off();send("STATUS\n");assert(strstr(output,"LOOP TIMEOUT"));
    now+=200;send("ARM\n");now+=10000;MotorTest_Tick(now);send("JOG 20 200\n");off();
    send("ARM\n");send("JOG 100 1000\n");assert(TIM3->CCR1==1000);send("!\n");off();
    now+=200;send("ARM\n");send("JOG 50 300\n");send("JOG -50 300\n");off();assert(strstr(output,"BUSY"));
    now+=200;send("ARM\n");send("JOG 20 200\n");MotorTest_CommunicationStop(true);drain();off();send("\n");
    now=UINT32_MAX-100;MotorTest_Stop();now+=200;send("ARM\n");send("JOG 20 50\n");assert(TIM3->CCR1==200);
    now+=50;MotorTest_Tick(now);off();
    now=UINT32_MAX-200;MotorTest_Stop();now+=150;send("ARM\n");send("JOG 20 80\n");assert(TIM3->CCR1==200);
    now+=80;MotorTest_Tick(now);off();
    now+=200;send("ARM\n");send("JOG 100 1000\n");assert(TIM3->CCR1==1000);
    send("SENSOR\n");off();assert(strstr(output,"SENSOR raw=2048 mv_est=1650"));
    send("JOG 100 300\n");off();assert(strstr(output,"ARM first"));
    now+=200;send("ARM\n");send("JOG 100 1000\n");sensor_ok=false;
    send("SENSOR\n");off();assert(strstr(output,"ERR SENSOR"));
    send("JOG 100 300\n");off();assert(strstr(output,"ARM first"));
    now+=200;send("ARM\n");send("JOG 100 20000\n");
    for(unsigned i=0;i<199;i++){now+=100;MotorTest_Tick(now);MotorTest_Task();assert(TIM3->CCR1==1000);}
    now+=99;MotorTest_Tick(now);MotorTest_Task();assert(TIM3->CCR1==1000);
    now+=1;MotorTest_Tick(now);off();drain();assert(strstr(output,"DONE STOP"));
    now+=200;send("ARM\n");send("JOG 100 20001\n");off();assert(strstr(output,"ERR command"));
    send("JOG 100 3000\n");off();assert(strstr(output,"ARM first"));
    now+=200;send("ARM\n");send("JOG -100 20000\n");now+=101;MotorTest_Tick(now);off();
    send("STATUS\n");assert(strstr(output,"LOOP TIMEOUT"));
    send("HELP\n");assert(strstr(output,"20..20000ms"));
    send("HELP\n");
    assert(!strcmp(output,"STOP | ARM | JOG signed_percent milliseconds | STATUS | SENSOR\r\n"
                         "DRIVER L298N | DRIVER MDD10A 1 | DRIVER MDD10A 2 (timed JOG only)\r\n"
                         "JOG: +/-1..100%, 20..20000ms; ARM once per JOG; ! stops immediately\r\n"));
    send("ARM\n");assert(!strcmp(output,"OK ARM (10s, one JOG)\r\n"));send("STOP\n");
    assert(live_inits==0 && live_writes==0); // Legacy never claims MDD10A pins.
    now+=200;send("ARM\n");send("JOG -100 20000\n");
    for(unsigned i=0;i<199;i++){now+=100;MotorTest_Tick(now);MotorTest_Task();assert(TIM3->CCR1==1000);assert(GPIOB->BSRR==GPIO_PIN_1);}
    now+=100;MotorTest_Tick(now);off();drain();assert(strstr(output,"DONE STOP"));
    send("MODE SIM\n");off();send("ARM\n");off();assert(live_writes==0);
    send("STOP\n");now+=200;send("ARM\nJOG 30 300\n");assert(TIM3->CCR1==300);send("STOP\n");
    send("MODE LIVE 7\n");off();assert(live_inits==1);
    send("LIVE ARM\n");off();assert(strstr(output,"unconfirmed"));assert(live_writes==0);
    send("STOP\n");now+=200;send("ARM\nJOG -30 300\n");assert(TIM3->CCR1==300&&GPIOB->BSRR==GPIO_PIN_1);send("STOP\n");
    send("ARM\n");send("MOVE 20\n");off();assert(strstr(output,"CALIBRATION required"));
    send("HELP ALL\n");assert(strstr(output,"MODE LIVE"));off();
    // Queue is owned by console/control; commands queued before ! must not restart.
    send("STOP\n");now+=200;
    input="ARM\nJOG 100 20000\n";
    while(*input)ControlApp_ConsoleStep();
    MotorTest_CommunicationStop(false);
    for(unsigned i=0;i<8;i++)ControlApp_Step();
    off();send("STATUS\n");assert(strstr(output,"armed=0"));
    // STOP bypasses even a saturated queue and cancels queued commands.
    input="ARM\nARM\nARM\nARM\nARM\n";
    while(*input)ControlApp_ConsoleStep();
    off();for(unsigned i=0;i<8;i++)ControlApp_Step();
    send("STATUS\n");assert(strstr(output,"armed=0"));
    // Partial line assembled before asynchronous stop is discarded to newline.
    input="AR";ControlApp_ConsoleStep();MotorTest_CommunicationStop(false);
    input="M\n";ControlApp_ConsoleStep();ControlApp_Step();off();
    send("STATUS\n");assert(strstr(output,"armed=0"));
    // Explicit two-channel observation is still available with motion inhibited.
    sensor_ok=true;send("MODE LIVE 5\n");
    assert(imu_init_calls==0); // Mode command never blocks on IMU initialization.
    AppInputs_Step();assert(imu_init_calls==1);
    now+=10;send("STATUS\n");assert(strstr(output,"adc_ok=1 raw=2048,2048"));off();
    send("RTOS\n");assert(strstr(output,"RTOS mock"));
    send("STOP\n");
    // InputTask publishes only completed samples and discards obsolete requests.
    imu_init_ok=true;imu_sample_ok=true;cancel_imu_init=true;
    AppInputs_RequestImu(true);AppInputs_Step();
    assert(!AppInputs_ReadImu().valid);
    cancel_imu_init=false;AppInputs_Step();
    AppInputs_RequestImu(true);AppInputs_Step();
    AppImuSnapshot snapshot=AppInputs_ReadImu();
    assert(snapshot.initialized && snapshot.valid && snapshot.pitch_deg==12.5f && snapshot.sampled_ms==now);
    AppInputs_RequestImu(false);assert(!AppInputs_ReadImu().valid);
    send("STOP\n");send("ENC ON\n");off();assert(strstr(output,"OK ENC"));
    send("ENC SCALE 1 100\n");send("ENC REF 1 29.24\n");off();
    send("ENC STATUS 1\n");assert(strstr(output,"pos_ok=1"));off();
    send("MOVE 20\n");off();assert(strstr(output,"ERR"));
    send("MODE LIVE 5\n");send("LIVE ARM\n");off();assert(strstr(output,"ERR"));
    send("STOP\n");send("ENC OFF\n");off();
    send("STOP\n");send("MODE SIM\n");send("PITCH STATUS\n");
    assert(strstr(output,"control=ANGLE_PWM")&&strstr(output,"HW=OFF"));off();
    send("STOP\n");send("MODE LIVE 5\n");send("PITCH STATUS\n");
    assert(strstr(output,"control=ANGLE_PWM")&&strstr(output,"permit=0"));off();
    send("STOP\n");
    distinct_axes=true;sensor_ok=true;
    unsigned before_init=live_inits;
    send("SENSOR BOTH\n");off();assert(strstr(output,"SENSORS raw1=2048 raw2=529"));
    assert(live_inits==before_init);
    now+=200;send("ARM\n");send("JOG 100 1000\n");assert(TIM3->CCR1==1000);
    send("SENSOR BOTH\n");off();assert(strstr(output,"SENSORS raw1=2048 raw2=529"));
    send("JOG 100 300\n");off();assert(strstr(output,"ARM first"));
    axis2_ok=false;send("SENSOR BOTH\n");off();assert(strstr(output,"ERR SENSORS axis=2"));
    axis2_ok=true;sensor_ok=false;send("SENSOR BOTH\n");off();assert(strstr(output,"ERR SENSORS axis=1"));
    sensor_ok=true;send("SENSOR\n");assert(strstr(output,"SENSOR raw=2048 mv_est=1650"));
    puts("PASS SENSOR BOTH: distinct channels, stop/disarm, per-axis failure, no LIVE init, legacy unchanged");
    send("BUOY HOVER\n");off();assert(strstr(output,"ERR BENCH1 ON"));
    send("BENCH2 ON\n");off();assert(strstr(output,"OK BENCH2 PA1"));
    send("BUOY HOVER\n");off();assert(strstr(output,"ARM first"));
    sensor2_raw=800;now+=200;send("ARM\n");send("BUOY HOVER\n");
    assert(strstr(output,"OK MOVE SENSOR2"));
    for(unsigned i=0;i<10;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(TIM3->CCR1>0 && GPIOB->BSRR==GPIO_PIN_0); // Positive reduces ADC.
    for(unsigned i=0;i<271;i++){--sensor2_raw;now+=10;MotorTest_Tick(now);MotorTest_Task();}
    for(unsigned i=0;i<25;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    off();assert(strstr(output,"DONE MOVE DONE"));
    send("SENSOR BOTH\n");assert(strstr(output,"raw2=529"));
    now+=200;send("ARM\n");send("BUOY DOWN\n");
    for(unsigned i=0;i<10;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(TIM3->CCR1>0 && GPIOB->BSRR==GPIO_PIN_1); // Negative increases ADC/fills.
    send("STOP\n");off();now+=200;send("ARM\n");send("BUOY UP\n");
    for(unsigned i=0;i<10;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(TIM3->CCR1>0 && GPIOB->BSRR==GPIO_PIN_0);
    send("STOP\n");off();
    sensor2_raw=4095;now+=200;send("ARM\n");send("BUOY DOWN\n");
    now+=10;MotorTest_Tick(now);MotorTest_Task();drain();off();assert(strstr(output,"DONE MOVE DONE"));
    sensor2_raw=4094;for(unsigned i=0;i<25;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}off();
    sensor2_raw=0;now+=200;send("ARM\n");send("BUOY UP\n");
    now+=10;MotorTest_Tick(now);MotorTest_Task();drain();off();assert(strstr(output,"DONE MOVE DONE"));
    sensor2_raw=1;now+=10;MotorTest_Tick(now);MotorTest_Task();off();
    now+=200;send("ARM\n");send("BUOY HOVER\n");assert(strstr(output,"OK MOVE SENSOR2"));
    for(unsigned i=0;i<10;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    assert(TIM3->CCR1>0 && GPIOB->BSRR==GPIO_PIN_1);send("STOP\n");off();
    sensor2_raw=529;send("BENCH2 OFF\n");now+=200;send("ARM\n");send("MOVE 20\n");off();assert(strstr(output,"ERR CALIBRATION"));
    // Reproduce real ADC completing 1 ms after the RTOS release.
    send("BENCH2 ON\n");now+=200;send("ARM\n");sensor2_raw=529;
    input="BUOY DOWN\n";output[0]=0;
    while(*input)ControlApp_ConsoleStep();
    adc_delay_ms=1;uint32_t release=now;
    ControlApp_Step(); // Start sample completes at release+1.
    assert(now==release+1);off();
    for(unsigned i=1;i<=5;i++) {
        now=release+i*10;MotorTest_Tick(now);ControlApp_Step();
        assert(now==release+i*10+1);
        if(i>=4) assert(TIM3->CCR1>0 && GPIOB->BSRR==GPIO_PIN_1);
    }
    adc_delay_ms=0;send("STATUS\n");assert(strstr(output,"fault=0"));
    send("STOP\n");off();
    puts("PASS 1ms ADC phase regression: updates at each RTOS release; no 19ms skip or false timing fault");
    if (getenv("TEST_SENSOR_JUMP")) {
        send("STOP\n");sensor2_raw=0;
        send("BENCH2 ON\n");now+=200;send("ARM\n");send("BUOY HOVER\n");
        for(unsigned i=0;i<10;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(TIM3->CCR1>0);
        sensor2_raw=11;
        for(unsigned i=0;i<30;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(!strstr(output,"ERR MOVE"));assert(TIM3->CCR1==1000);
        // Outlier far beyond measured noise must be withheld, then fault if sustained.
        sensor2_raw=2000;
        for(unsigned i=0;i<60;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        off();assert(strstr(output,"ERR MOVE SENSOR_JUMP"));
        // Reacquire stable values while off. New ARM recovers without RESET.
        sensor2_raw=11;
        for(unsigned i=0;i<25;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        send("ARM\n");assert(strstr(output,"OK ARM"));off();
        send("BUOY HOVER\n");assert(strstr(output,"OK MOVE SENSOR2"));
        for(unsigned i=0;i<20;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
        assert(TIM3->CCR1==1000);send("STOP\n");off();
        puts("PASS measured ADC noise accepted; persistent large jump stops; fresh stable samples + ARM recover without reset");
        return 0;
    }
    send("BENCH2 ON\n");now+=200;send("ARM\n");send("BUOY DOWN\n");
    for(unsigned i=0;i<850;i++){now+=10;MotorTest_Tick(now);MotorTest_Task();}
    off();assert(strstr(output,"ERR MOVE NO_MOVEMENT"));
    send("ARM\n");assert(strstr(output,"OK ARM"));off();
    puts("PASS BENCH2: opt-in/ARM, PA1 feedback, HOVER convergence, UP/DOWN direction, STOP, endpoint exact stop/no restart, stall latch, legacy disabled default");
    puts("PASS RTOS application boundaries: FIFO saturation stops, stop generations cancel queued/partial commands, IMU init asynchronous, snapshot cancellation, two ADC observation");
    puts("PASS integration: exact legacy HELP/ARM, both 20s directions, SIM/LIVE isolation and STOP return, uncalibrated MOVE/LIVE blocked");
    puts("PASS long JOG: full 20000ms run, deadline stop, 20001 rejection, loop-stall stop, HELP limit");
    puts("PASS sensor command: interrupts motion, disarms, reports value, ADC failure remains stopped");
    puts("PASS runtime: boot OFF, ARM, duty/direction, deadline, loop stall, expiry, STOP, busy/error stop, tick wrap");
}
