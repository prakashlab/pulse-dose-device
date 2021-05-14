#include <HoneywellTruStabilitySPI.h>
//#include <sfm3x00.h>
//#include <Wire.h>
#include "sfm3019.h"

int error = 0;
SfmConfig sfm3019;

static const int pin_SS = 10; 
static const int pin_valve = 0;
TruStabilityPressureSensor sensor( pin_SS, -60.0, 60.0 );
//SFM3000 sfm3000;

bool pulse_started = false;
elapsedMillis time_since_pulse_started = 0;
elapsedMillis time_since_inhalation_started = 0;

void setup() 
{
  pinMode(pin_valve,OUTPUT);
  digitalWrite(pin_valve,LOW);
  
  Serial.begin(20000000); // start Serial communication

  /*
  Wire.setClock(100000);
  Wire.begin();

  delay(200);
  
  while(true) 
  {
    int ret_sfm3000 = sfm3000.init();
    if (ret_sfm3000 == 0) 
    break;
  }
  sfm3000.get_scale_offset();
  sfm3000.start_continuous();
  */

  // SFM3019
  sensirion_i2c_init();
  error = sensirion_i2c_general_call_reset();
  sensirion_sleep_usec(SFM3019_SOFT_RESET_TIME_US);
  while (sfm3019_probe()) 
    sensirion_sleep_usec(100000);
  sfm3019 = sfm3019_create();
  error = sfm_common_start_continuous_measurement(&sfm3019, SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_O2);
  sensirion_sleep_usec(SFM3019_MEASUREMENT_INITIALIZATION_TIME_US);

  // pressure sensor
  SPI.begin(); // start SPI communication
  sensor.begin(); // run sensor initialization
}

void loop() 
{
  // if( sensor.readSensor() == 0 && sfm3000.read_sample() == 0) 

  int16_t flow_raw;
  int16_t temperature_raw;
  uint16_t status;
  float flow;
  float temperature;
  error = sfm_common_read_measurement_raw(&sfm3019, &flow_raw, &temperature_raw, &status);
  sfm_common_convert_flow_float(&sfm3019, flow_raw, &flow);
  
  if( error == 0 && sensor.readSensor() == 0) 
  {
    // float t = sensor.temperature();
    // float p = sensor.pressure();
    Serial.print(flow);
    Serial.print('\t');
    Serial.println( min(5,10*sensor.pressure()) );
  }
  delay(20);

  if( sensor.pressure() < -0.25 && pulse_started == false && time_since_inhalation_started > 2000 )
  {
    pulse_started = true;
    time_since_pulse_started = 0;
    time_since_inhalation_started = 0;
    digitalWrite(pin_valve, HIGH);
  }

  if (time_since_pulse_started >= 900)
  {
    digitalWrite(pin_valve, LOW);
    pulse_started = false;
  }

}
