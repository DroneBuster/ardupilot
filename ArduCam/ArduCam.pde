#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

//INPUTS
#define ON_OFF CH_1
#define CAM_TRIG CH_2
//OUTPUTS
#define ON_OFF_SW CH_1

#define IR_LED 54 //A0

#define RADIO_MIN 1100
#define RADIO_MAX 1900
#define RADIO_TRIM 1500

#define CAM_OFF_DELAY_MS 4000

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Math.h>
#include <AP_Scheduler.h>       // main loop scheduler

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_Scheduler scheduler;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of start of main control loop
static uint32_t fast_loopTimer_us;

// Number of milliseconds used in last main loop cycle
static uint32_t delta_us_fast_loop;

// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;

// The maximum main loop execution time recorded in the current performance monitoring interval
static uint32_t G_Dt_max = 0;

// The main loop execution time.  Seconds
static float G_Dt = 0.02f;

static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { read_rc,             1,    700},
};


void setup()
{
    hal.console->println("ArduPilot RC Channel test");
    hal.scheduler->delay(100);
    hal.rcout->enable_ch(ON_OFF_SW);
    hal.rcout->write(ON_OFF_SW, 1900);
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

void loop()
{
    hal.scheduler->delay_microseconds(fast_loopTimer_us + 20000 - hal.scheduler->micros());
    uint32_t timer = hal.scheduler->micros();

    delta_us_fast_loop  = timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = timer;

    if (delta_us_fast_loop > G_Dt_max)
        G_Dt_max = delta_us_fast_loop;

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t remaining = (timer + 20000) - hal.scheduler->micros();
    if (remaining > 19500) {
        remaining = 19500;
    }
    scheduler.run(remaining);
}

void debug_rcin() {
    uint16_t channels[8];
    hal.rcin->read(channels, 8);
    hal.console->printf_P(
        PSTR("rcin: %u %u %u %u %u %u %u %u\r\n"),
        channels[0],
        channels[1],
        channels[2],
        channels[3],
        channels[4],
        channels[5],
        channels[6],
        channels[7]);
    hal.console->printf_P(PSTR("gdt: %u\r\n"), G_Dt_max);
}

void high(unsigned int time, int freq, int pin){
    int pause = (1000/freq/2)-4;
    unsigned long start = hal.scheduler->micros();
    while(hal.scheduler->micros()-start<=time){
        hal.gpio->write(pin, 1);
        hal.scheduler->delay_microseconds(pause);
        hal.gpio->write(pin,0);
        hal.scheduler->delay_microseconds(pause);
    }
}

class Sony{
public:
  Sony(int pin);
  void shutterNow();
private:
  int _pin;
  int _freq;
};

Sony::Sony(int pin)
{
   hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
   _pin = pin;
   _freq = 40;
}

void Sony::shutterNow()
{
    bool _seq[] = {
        1,0,1,1,0,1,0,0,1,0,1,1,1,0,0,0,1,1,1,1    };
    for (int j=0;j<3;j++) {
        high(2320,_freq,_pin);
        hal.scheduler->delay_microseconds(650);
        for (int i=0;i<sizeof(_seq);i++){
            if (_seq[i]==0){
                high(575,_freq,_pin);
                hal.scheduler->delay_microseconds(650);
            }
            else{
                high(1175,_freq,_pin);
                hal.scheduler->delay_microseconds(650);
            }
        }
        hal.scheduler->delay_microseconds(10000);
    }
}

Sony camera(IR_LED);

bool cam_trig = false;
bool cam_on = false;
bool cam_on1 = false;
uint32_t delay_time;
uint16_t read_delay = 0;

void read_rc()
{
    uint16_t on_pwm = hal.rcin->read(ON_OFF);
    uint16_t cam_pwm = hal.rcin->read(CAM_TRIG);

    if(cam_pwm >= 1400){
        if(!cam_trig){
            cam_trig = true;
            camera.shutterNow();
        }
    }
    else if(cam_pwm < 1400)
        cam_trig = false;

    if(on_pwm < 1400){
        if(cam_on && read_delay >= 200){
            if(cam_on1){
                hal.rcout->write(ON_OFF_SW, 1100);
                delay_time = hal.scheduler->millis() + 500;
                cam_on1 = false;
            }
            if(delay_time <= hal.scheduler->millis() && !cam_on1){
                hal.rcout->write(ON_OFF_SW, 1900);
                delay_time = hal.scheduler->millis() + CAM_OFF_DELAY_MS;
                cam_on = false;
            }
        }
        else if(cam_on)
            read_delay+=1;
        else if(delay_time <= hal.scheduler->millis())
            hal.rcout->write(ON_OFF_SW, 1100);
    }
    else if(on_pwm >= 1400){
        cam_on = true;
        cam_on1 = true;
        read_delay = 0;
        hal.rcout->write(ON_OFF_SW, 1900);
    }
}
AP_HAL_MAIN();
