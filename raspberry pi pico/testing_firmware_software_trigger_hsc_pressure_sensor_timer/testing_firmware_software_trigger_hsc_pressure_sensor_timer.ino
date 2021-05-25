#include <Wire.h>
#include "sfm3019.h"
#include <HoneywellTruStabilitySPI.h>

#define TESTING_MODE true
#define MEASURE_FLOW true
#define USE_PYTHON_GUI true

// timing
static const unsigned long TIME_INTERVAL_US = 5000;
unsigned long timestamp_last_cycle = 0;
int i = 0;

// pin assignment
static const int pin_SS = 17;
static const int pin_valve = 15;
static const int pin_led = 3;

// HSC sensor
TruStabilityPressureSensor sensor( pin_SS, -60.0, 60.0 );

//// pressure sensors
//const uint16_t _output_min = 1638; // 10% of 2^14
//const uint16_t _output_max = 14745; // 90% of 2^14
//const float _p_min = -4*2.54; // psi
//const float _p_max = 4*2.54; // cmH2O
//static const int TE_sensor_addr = 0x46;

double measured_pressure = 0;
double pressuer_sensor_offset = 0;

// flow sensor
SfmConfig sfm3019;
int16_t flow_raw;
int16_t temperature_raw;
uint16_t status;
float flow;
float temperature;

// variables
int _error = 0;

// flags
bool flag_pulse_started = false;
bool flag_ready_to_stop_flow = false;
int buffer_rx_ptr;
int buffer_tx_ptr;

// timing
unsigned long timestamp_trigger = 0;
unsigned long timestamp_flow_stopped = 0;
unsigned long time_since_flow_started = 0;
unsigned long time_since_flow_stopped = 0;

// communication
static const int MSG_LENGTH = 8*5; // send message every 5*5 ms = 25 ms
static const int CMD_LENGTH = 3;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
uint16_t temp;
float _comm_pressure_min = -60;
float _comm_pressure_max = 60;
float _comm_flow_min = -10;
float _comm_flow_max = 50;

// system timestamp
uint32_t timestamp = 0;

// trigger settings
float dp_start_flow = -0.07;
float dp_stop_flow = -0.12; // dp_stop_flow should be smaller than dp_start_flow

// protection against falsely turning of the flow when inhalation starts and falsely turning on the flow when inhalation ends
static const unsigned long flow_on_time_min_us = 50*1000;
static const unsigned long flow_off_time_min_us = 50*1000;

// add protection against false triggering: trade off between delay and sensitivity

// maxt rr setting
static const float rr_max = 40;

// setup
void setup()
{ 
  if(TESTING_MODE)
  {
    if(MEASURE_FLOW)
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
  }
 
  // i2c
  Wire.begin();
  delay(1000);

  // HSC
  SPI.begin(); // start SPI communication
  sensor.begin(); // run sensor initialization

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
  timestamp_trigger = micros();

  // determine the offset pressure at startup
  pressuer_sensor_offset = 0;

  timestamp_last_cycle = micros();

}

void loop() 
{

  // read serial
  while (Serial.available()) 
  {
    buffer_rx[buffer_rx_ptr] = Serial.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) 
    {
      buffer_rx_ptr = 0;
      if(buffer_rx[0]==0)
        dp_start_flow = (double(uint16_t(buffer_rx[1])*256 + uint16_t(buffer_rx[2]))/65535)*(_comm_pressure_max-_comm_pressure_min) + _comm_pressure_min;
      if(buffer_rx[0]==1)
        dp_stop_flow = (double(uint16_t(buffer_rx[1])*256 + uint16_t(buffer_rx[2]))/65535)*(_comm_pressure_max-_comm_pressure_min) + _comm_pressure_min;
    }
  }
  
  if( micros() - timestamp_last_cycle >= TIME_INTERVAL_US )
  {
    timestamp = timestamp + 1;
    timestamp_last_cycle = micros();
  
    // timing updates
    time_since_flow_started = micros() - timestamp_trigger;
    time_since_flow_stopped = micros() - timestamp_flow_stopped;
  
    // read pressure sensor
    sensor.readSensor();
    measured_pressure = sensor.pressure();
    
    i = i+1;
    if(i==1000*1000/TIME_INTERVAL_US)
    {
      digitalWrite(LED_BUILTIN,LOW);
      digitalWrite(pin_led,LOW);
      digitalWrite(pin_valve,HIGH);
    }
      
    if(i==3000*1000/TIME_INTERVAL_US)
    {
      digitalWrite(LED_BUILTIN,HIGH);
      digitalWrite(pin_led,HIGH);
      digitalWrite(pin_valve,LOW);
      i = 0;
    }

    if(TESTING_MODE)
    {
      if(MEASURE_FLOW)
      {
        _error = sfm_common_read_measurement_raw(&sfm3019, &flow_raw, &temperature_raw, &status);
        sfm_common_convert_flow_float(&sfm3019, flow_raw, &flow);
      }
    }

    if(TESTING_MODE)
    {
      if(USE_PYTHON_GUI)
      {
        buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 24);
        buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 16);
        buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(timestamp % 256);

        temp = ((flow-_comm_flow_min)/(_comm_flow_max-_comm_flow_min))*65535;
        buffer_tx[buffer_tx_ptr++] = byte(temp >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(temp % 256);

        temp = ((measured_pressure-_comm_pressure_min)/(_comm_pressure_max-_comm_pressure_min))*65535;
        buffer_tx[buffer_tx_ptr++] = byte(temp >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(temp % 256);

        if (buffer_tx_ptr == MSG_LENGTH)
        {
          buffer_tx_ptr = 0;
          SerialUSB.write(buffer_tx, MSG_LENGTH);
        }
      }
      else
      {
        Serial.print(flow);
        Serial.print('\t');
        Serial.println(10*(measured_pressure-pressuer_sensor_offset));
      }
    }
  
    //  if (time_since_flow_stopped > 3000 && flag_pulse_started == false)
    //  {
    //    digitalWrite(pin_valve, LOW);
    //    digitalWrite(pin_led,HIGH);
    //    digitalWrite(LED_BUILTIN,HIGH);
    //  }
  }
}
