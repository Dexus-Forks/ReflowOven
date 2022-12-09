#ifndef _HEATER_H_
#define _HEATER_H_

#include <Arduino.h>

#define dWidth 10 // the error scale in 'c

class Heater
{

private:
  uint8_t PWMcnt = 0;
  uint16_t timeE = 0;
  uint8_t prevTemp = 0;

  ///////////PID copied in
  //	float temperature_read = 0.0;
  //	float PID_error = 0;
  //	float previous_error = 0;
  //	float elapsedTime, Time, timePrev;
  //	float PID_value = 0;
  //	int button_pressed = 0;
  //	int menu_activated = 0;
  //	float last_set_temperature = 0;
  ////
  ////	//PID constants
  //////////////////////////////////////////////////////////////
  //	int kp = 90;
  //	int ki = 10;
  //	int kd = 10;
  //////////////////////////////////////////////////////////////
  ////
  //	int PID_p = 0;
  //	int PID_i = 0;
  //	int PID_d = 0;
  //	float last_kp = 0;
  //	float last_ki = 0;
  //	float last_kd = 0;
  //
  //	int PID_values_fixed = 0;
  ////////////// PID copied in^^^^

public:
  uint8_t hpin;
  uint8_t Duty = 0;
  uint16_t target = 0;
  uint16_t current = 0;
  float dutyTrig = 0; // 0-100% dimmer value

  bool heating = false;
  bool running = false;

  Heater(uint8_t input) : hpin(input) {}

  void runheaterPWM();
  void run();
  void stop();
  void setDelta();
};

#endif