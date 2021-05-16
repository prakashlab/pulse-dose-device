#include <Wire.h>
#include "sfm3019.h"

#define TESTING_MODE true

// pin assignment
static const int pin_SS = 17;
static const int pin_valve = 15;
static const int pin_led = 3;

// pressure sensors
const uint16_t _output_min = 1638; // 10% of 2^14
const uint16_t _output_max = 14745; // 90% of 2^14
const float _p_min = -4*2.54; // psi
const float _p_max = 4*2.54; // cmH2O
static const int TE_sensor_addr = 0x46;

double measured_pressure = 0;
double pressuer_sensor_offset = 0;

// flow sensor
SfmConfig sfm3019;

// variables
int _error = 0;

// flags
bool flag_pulse_started = false;
bool flag_ready_to_stop_flow = false;

// timing
unsigned long timestamp_trigger = 0;
unsigned long timestamp_flow_stopped = 0;
unsigned long time_since_flow_started = 0;
unsigned long time_since_flow_stopped = 0;

// trigger settings
static const float dp_start_flow = -0.05;
static const float dp_stop_flow = -0.1; // dp_stop_flow should be smaller than dp_start_flow

// maxt rr setting
static const float rr_max = 40;

// setup
void setup()
{
  if(TESTING_MODE)
  {
    // SFM3019 - for testing only
    sensirion_i2c_init();
    _error = sensirion_i2c_general_call_reset();
    sensirion_sleep_usec(SFM3019_SOFT_RESET_TIME_US);
    while (sfm3019_probe())
      sensirion_sleep_usec(100000);
    sfm3019 = sfm3019_create();
    _error = sfm_common_start_continuous_measurement(&sfm3019, SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_O2);
    sensirion_sleep_usec(SFM3019_MEASUREMENT_INITIALIZATION_TIME_US);
  }
 
  // i2c
  Wire.begin();
  delay(1000);

  // pins
  pinMode(pin_valve, OUTPUT);
  pinMode(pin_led, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(pin_valve, LOW);
  digitalWrite(pin_led, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  if(TESTING_MODE)
    Serial.begin(20000000); // start Serial communication - for testing only

  // initialize the timestamp variable
  timestamp_trigger = millis();

  // determine the offset pressure at startup
  int N = 16;
  int count = 0;
  double pressure_measurement_sum = 0;
  for(int i = 0; i<=16; i++)
  {
    Wire.requestFrom(TE_sensor_addr,0);
    Wire.requestFrom(TE_sensor_addr,2);
    uint8_t byte1 = Wire.read();
    uint8_t byte2 = Wire.read();
    if ( (byte1 >> 6) != 0)
      continue; // pressure sensor reading error
    uint16_t raw_data = (byte1 << 8 | byte2);
    measured_pressure = float(constrain(raw_data, _output_min, _output_max) - _output_min) * (_p_max - _p_min) / (_output_max - _output_min) + _p_min;
    pressure_measurement_sum = pressure_measurement_sum + measured_pressure;
    count = count + 1;
    delay(1);
  }
  pressuer_sensor_offset = pressure_measurement_sum/count;

}

void loop() 
{
  
  // measurement interval - to change to timer-based operation instead of using delay
  delay(20);

  // timing updates
  time_since_flow_started = millis() - timestamp_trigger;
  time_since_flow_stopped = millis() - timestamp_flow_stopped;

  // read pressure sensor
  Wire.requestFrom(TE_sensor_addr,0);
  Wire.requestFrom(TE_sensor_addr,2);
  uint8_t byte1 = Wire.read();
  uint8_t byte2 = Wire.read();
  if ( (byte1 >> 6) != 0)
    return; // pressure sensor reading error
  uint16_t raw_data = (byte1 << 8 | byte2);
  measured_pressure = float(constrain(raw_data, _output_min, _output_max) - _output_min) * (_p_max - _p_min) / (_output_max - _output_min) + _p_min;

  if(TESTING_MODE)
  {
    int16_t flow_raw;
    int16_t temperature_raw;
    uint16_t status;
    float flow;
    float temperature;
    _error = sfm_common_read_measurement_raw(&sfm3019, &flow_raw, &temperature_raw, &status);
    sfm_common_convert_flow_float(&sfm3019, flow_raw, &flow);
    
    Serial.print(flow);
    Serial.print('\t');
    Serial.println(10*(measured_pressure-pressuer_sensor_offset));
  }

  // starting the flow
  if ( measured_pressure - pressuer_sensor_offset < dp_start_flow && flag_pulse_started == false && time_since_flow_started >= 1000*(60/rr_max) )
  {
    flag_pulse_started = true;
    flag_ready_to_stop_flow = false;
    digitalWrite(pin_valve, LOW);
    digitalWrite(pin_led, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    timestamp_trigger = millis();
  }

  // arming for stopping the flow - needed if dp_stop_flow < dp_start_flow
  if ( flag_pulse_started && measured_pressure - pressuer_sensor_offset < dp_stop_flow * 1.05)
    flag_ready_to_stop_flow = true;

  // stop the flow
  if (flag_ready_to_stop_flow && measured_pressure - pressuer_sensor_offset > dp_stop_flow)
  {
    digitalWrite(pin_valve, HIGH);
    digitalWrite(pin_led, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    timestamp_flow_stopped = millis();
    flag_pulse_started = false;
    flag_ready_to_stop_flow = false;
  }

  //  if (time_since_flow_stopped > 3000 && flag_pulse_started == false)
  //  {
  //    digitalWrite(pin_valve, LOW);
  //    digitalWrite(pin_led,HIGH);
  //    digitalWrite(LED_BUILTIN,HIGH);
  //  }

}
