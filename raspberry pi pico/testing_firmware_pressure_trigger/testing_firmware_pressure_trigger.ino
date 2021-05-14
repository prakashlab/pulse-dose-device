#include <HoneywellTruStabilitySPI.h>
#include "sfm3019.h"

// pin assignment
static const int pin_SS = 17; 
static const int pin_valve = 15; 
static const int pin_led = 3; 

// sensors
TruStabilityPressureSensor sensor( pin_SS, -60.0, 60.0 );
SfmConfig sfm3019;

// variables
int _error = 0;

bool flag_pulse_started = false;
bool flag_ready_to_stop_flow = false; 

unsigned long timestamp_trigger = 0;
unsigned long time_since_pulse_started = 0;
unsigned long time_since_inhalation_started = 0;

float pressuer_sensor_offset = 0;
static const float dp_start_flow = -0.05;
static const float dp_stop_flow = -0.3; // dp_stop_flow should be smaller than dp_start_flow

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

  delay(100);

  while(sensor.readSensor() != 0)
    delay(1);
  pressuer_sensor_offset = sensor.pressure();

  Serial.begin(20000000); // start Serial communication

  timestamp_trigger = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  int16_t flow_raw;
  int16_t temperature_raw;
  uint16_t status;
  float flow;
  float temperature;
  _error = sfm_common_read_measurement_raw(&sfm3019, &flow_raw, &temperature_raw, &status);
  sfm_common_convert_flow_float(&sfm3019, flow_raw, &flow);
  
  if( _error == 0 && sensor.readSensor() == 0) 
  {
    // float t = sensor.temperature();
    // float p = sensor.pressure();
    Serial.print(flow);
    Serial.print('\t');
    // Serial.println( min(5,10*sensor.pressure()) );
    Serial.println(10*sensor.pressure());
  }
  
  delay(20); // to change to timer-based operation instead of using delay

  if( sensor.pressure() < dp_start_flow && flag_pulse_started == false && time_since_inhalation_started > 1500 )
  {
    flag_pulse_started = true;
    flag_ready_to_stop_flow = false;
    time_since_pulse_started = millis() - timestamp_trigger;
    time_since_inhalation_started = millis() - timestamp_trigger;
    digitalWrite(pin_valve, HIGH);
    digitalWrite(pin_led,HIGH);
    digitalWrite(LED_BUILTIN,HIGH);
    timestamp_trigger = millis();
  }

  if( flag_pulse_started && sensor.pressure() < dp_stop_flow)
    flag_ready_to_stop_flow = true;

  if (flag_ready_to_stop_flow && sensor.pressure() > dp_stop_flow
  {
    digitalWrite(pin_valve, LOW);
    digitalWrite(pin_led, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    flag_pulse_started = false;
  }

}
