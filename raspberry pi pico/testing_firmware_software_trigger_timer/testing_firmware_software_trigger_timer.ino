#include <HoneywellTruStabilitySPI.h>
#include "sfm3019.h"

// timing
static const unsigned long TIME_INTERVAL_US = 5000;
unsigned long timestamp_last_cycle = 0;

// pin assignment
static const int pin_SS = 17; 
static const int pin_valve = 15; 
static const int pin_led = 3; 

// sensors
TruStabilityPressureSensor sensor( pin_SS, -60.0, 60.0 );
SfmConfig sfm3019;

// state variables
int i = 0;
int _error = 0;
bool pulse_started = false;
int _pressure;

void setup() 
{
  // SFM3019
  sensirion_i2c_init();
  _error = sensirion_i2c_general_call_reset();
  sensirion_sleep_usec(SFM3019_SOFT_RESET_TIME_US);
  while (sfm3019_probe()) 
    sensirion_sleep_usec(100000);
  sfm3019 = sfm3019_create();
  _error = sfm_common_start_continuous_measurement(&sfm3019, SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_O2);
  sensirion_sleep_usec(SFM3019_MEASUREMENT_INITIALIZATION_TIME_US);

  pinMode(pin_valve,OUTPUT);
  pinMode(pin_led,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  SPI.begin(); // start SPI communication
  sensor.begin(); // run sensor initialization

  Serial.begin(20000000); // start Serial communication

  timestamp_last_cycle = micros();

}

void loop() 
{

  if( micros() - timestamp_last_cycle >= TIME_INTERVAL_US )
  {
    timestamp_last_cycle = micros();

    int16_t flow_raw;
    int16_t temperature_raw;
    uint16_t status;
    float flow;
    float temperature;
    _error = sfm_common_read_measurement_raw(&sfm3019, &flow_raw, &temperature_raw, &status);
    sfm_common_convert_flow_float(&sfm3019, flow_raw, &flow);
  
    i = i+1;
    if(i==1000*1000/TIME_INTERVAL_US)
    {
      _pressure = 0;
      digitalWrite(LED_BUILTIN,LOW);
      digitalWrite(pin_led,LOW);
      digitalWrite(pin_valve,HIGH);
    }
      
    if(i==3000*1000/TIME_INTERVAL_US)
    {
      _pressure = -1;
      digitalWrite(LED_BUILTIN,HIGH);
      digitalWrite(pin_led,HIGH);
      digitalWrite(pin_valve,LOW);
      i = 0;
    }

    if( _error == 0 ) 
    {
      // float t = sensor.temperature();
      // float p = sensor.pressure();
      Serial.print(flow);
      Serial.print('\t');
      // Serial.println( min(5,10*sensor.pressure()) );
      Serial.println(_pressure);
    }
    
  }
}
