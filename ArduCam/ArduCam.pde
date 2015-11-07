#define THISFIRMWARE "ArduCam 0.1-alpha"

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

#define CAM_OFF_DELAY_MS 5000

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL.h>
#include <AP_Common.h>
#include <Filter.h>                     // Filter library
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay.h>       // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed.h>
#include <AP_Terrain.h>

#include <APM_OBC.h>
#include <APM_Control.h>
#include <AP_AutoTune.h>
#include <GCS.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash.h>
#include <SITL.h>
#include <AP_Scheduler.h>       // main loop scheduler

#include <AP_Navigation.h>
#include <AP_L1_Control.h>
#include <AP_RCMapper.h>        // RC input mapping library

#include <AP_Vehicle.h>
#include <AP_SpdHgtControl.h>
#include <AP_TECS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>     // Mission command library

#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library

#include <AP_Arming.h>
#include <AP_BoardConfig.h>
#include <AP_Frsky_Telem.h>
#include <AP_ServoRelayEvents.h>

#include <AP_Rally.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_Scheduler scheduler;

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif defined(HAL_BOARD_LOG_DIRECTORY)
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);
#else
// no dataflash driver
DataFlash_Empty DataFlash;
#endif

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
    { write_log,          10,    700},
//    { serial_manage,       1,    700},
    { debug_rcin,          100,    700},
};

static void run_cli(AP_HAL::UARTDriver *port);

void setup()
{
    hal.console->println("ArduPilot RC Channel test");
    hal.scheduler->delay(100);
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);
    // turn all lights off
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
    hal.rcout->enable_ch(ON_OFF_SW);
    hal.rcout->write(ON_OFF_SW, 1900);
    log_init();
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

void loop()
{
    hal.scheduler->delay(20);
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

uint8_t crlf_count = 0;

static void serial_manage(void)
{
    uint8_t c = hal.console->read();
    if (c == '\n' || c == '\r') {
          crlf_count++;
      } else {
          crlf_count = 0;
      }
      if (crlf_count == 3) {
          run_cli(hal.console);
      }
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

    if(cam_pwm >= 1600){
        if(!cam_trig){
            cam_trig = true;
            camera.shutterNow();
            DataFlash.Log_Write_Message("Taking picture!");
        }
    }
    else if(cam_pwm < 1400)
        cam_trig = false;

    if(on_pwm < 1400 && on_pwm != 900){
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
                DataFlash.Log_Write_Message("Turning camera off!");
            }
        }
        else if(cam_on)
            read_delay+=1;
        else if(delay_time <= hal.scheduler->millis())
            hal.rcout->write(ON_OFF_SW, 1100);
    }
    else if(on_pwm >= 1400 || on_pwm == 900){
        cam_on = true;
        cam_on1 = true;
        read_delay = 0;
        hal.rcout->write(ON_OFF_SW, 1900);
    }

    if(cam_on)
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
    else
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
}

static void write_log(void) {
    //Log_Write_Performance();
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();

}
AP_HAL_MAIN();
