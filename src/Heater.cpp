#include "Heater.h"

void Heater::runheaterPWM()
{
  if ((timeE + 100) < millis())
  {
    setDelta();
    timeE = millis();
  }

  if (running == true)
  {
    //debugSerial.println(dutyTrig);
    //	dutyTrig=Duty;
    if (dutyTrig < Duty)
    {
      dutyTrig++;
    }
    else
    {
      dutyTrig--;
    }

    if (dutyTrig < 0)
    {
      dutyTrig = 0;
    }
    if (dutyTrig > 100)
    {
      dutyTrig = 100;
    }

    //dutyTrig=dutyTrig+(Duty-dutyTrig/10);

    //					debugSerial.println(dutyTrig);

    //					if (Duty > PWMcnt)
    //						{
    //							digitalWrite(hpin, HIGH);
    //							heating = true;
    //						}
    //					else
    //						{
    //							digitalWrite(hpin, LOW);
    //							heating = false;
    //						}
    //
    //					PWMcnt++;
    //					if (PWMcnt >= 100)
    //						{
    //							PWMcnt = 0;
    //						}
  }
  else
  {

    Duty = 0;
    dutyTrig = 0;
    //	PWMcnt = 0;
  }
}

void Heater::run()
{
  running = true;
}

void Heater::stop()
{
  Duty = 0;
  dutyTrig = 0;
  digitalWrite(hpin, LOW);
  heating = false;
  running = false;
}

void Heater::setDelta()
{
  //			// First we read the real value of temperature
  //							temperature_read = current;
  //
  //							//Next we calculate the error between the setpoint and the real value
  //							PID_error = target - temperature_read + 3;
  //
  //							//Calculate the P value
  //							PID_p = 0.01 * kp * PID_error;
  //
  //							//Calculate the I value in a range on +-3
  //							PID_i = 0.01 * PID_i + (ki * PID_error);
  //
  //							//For derivative we need real time to calculate speed change rate
  //							timePrev = Time; // the previous time is stored before the actual time read
  //							Time = millis();                            // actual time read
  //
  //							elapsedTime = (Time - timePrev) / 1000;
  //							//Now we can calculate the D calue
  //
  //							PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);
  //
  //							//Final total PID value is the sum of P + I + D
  //							PID_value = PID_p + PID_i + PID_d;
  //
  //							//We define PWM range between 0 and 255
  //							if (PID_value < 0)
  //								{
  //									PID_value = 0;
  //								}
  //							if (PID_value > 255)
  //								{
  //									PID_value = 255;
  //								}
  //							Duty=PID_value;
  ////							//Now we can write the PWM signal to the mosfet on digital pin D3
  ////
  ////							//Since we activate the MOSFET with a 0 to the base of the BJT, we write 255-PID value (inverted)
  ////							Duty=map(PID_value,0,255,0,100);//analogWrite(PWM_pin, 255 - PID_value);
  //						previous_error = PID_error; //Remember to store the previous error for next loop.

  int d = target + 3 - current;

  if (target + 3 > current) //for the offeset observed when holding temp
  {
    if (d > dWidth)
    {
      Duty = 100; // in %
    }
    else
    {
      Duty = (100 / dWidth) * d;
    }
  }
  else
  {
    Duty = 0;
  }
}