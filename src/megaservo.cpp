#include "megaservo.h"

#include <Arduino.h>

MegaServo::MegaServo()
{
  if (s_index == MAX_SERVOS)
  {
    return;
  }
  
  index = s_index;
  
  _m_servos[s_index++] = this;

  tolerance = 15;
}

void MegaServo::attach(int _cw_pin, int _ccw_pin, int _enablerPin, uint8_t _sensor_pin)
{
  attach(_cw_pin, _ccw_pin, _enablerPin, _sensor_pin, 0);
}

void MegaServo::attach(int _cw_pin, int _ccw_pin, int _enablerPin, uint8_t _sensor_pin, int _offset)
{
  cw_pin = _cw_pin;  
  ccw_pin =  _ccw_pin;  
  enabler_pin = _enablerPin;  
  sensor_pin = _sensor_pin;  
  offset = _offset;

  active = true;

  pinMode(cw_pin, OUTPUT);  
  pinMode(ccw_pin, OUTPUT);

  digitalWrite(cw_pin, LOW);  
  digitalWrite(ccw_pin, LOW);

  // shamelessly hijack timer1...
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1041;            // compare match register 16MHz/256/60Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

// timer interrupt routine
ISR(TIMER1_COMPA_vect)          
{
  for (int i = 0; i < s_index; i++)
  {
    if (_m_servos[i]->active)
    {
      _m_servos[i]->update();
    }
  }
}

void MegaServo::detach()
{
  stop();

  active = false;
}

void MegaServo::write(int degrees)
{
  // map degrees to potentiometer value
  int pot_value = map(degrees, MIN_ANGLE, MAX_ANGLE, MIN_POT_VALUE, MAX_POT_VALUE);  

  write_pot_value(pot_value);
}

void MegaServo::writeVolume(int vol)
{
  // map mL to potentiometer value
  int pot_value = map(vol, MIN_VOLUME, MAX_VOLUME, MIN_POT_VALUE, MAX_POT_VALUE);  

  write_pot_value(pot_value);
}

void MegaServo::setSpeed(int _speed)
{
  speed  = _speed;
}
void MegaServo::setTol(int _tolerance)
{
  tolerance  = _tolerance;
}
void MegaServo::setOffset(int _offset)
{
  offset  = _offset;
}

void MegaServo::update()
{
  int current_position = get_position();

  // Guard servo's min and max bounds
  if (current_position < MIN_POT_VALUE)
  {
    //write(MIN_ANGLE);
    write(MIN_VOLUME);
  }

  if (current_position > MAX_POT_VALUE)
  {
//    write(MAX_ANGLE);
    write(MAX_VOLUME);
  }

  // Gap between curent and wanted positions
  int gap = abs(current_position - wanted_position);

  // If gap is within tolerance then stop
  if (gap < tolerance)
  {
    stop();
    return;
  }

  // set speed according to gap remaining 
/*  speed = 125;

  if (gap > 50)
  {
    speed = 165;
  }
  if (gap > 75)
  {
    speed = 205;
  }
  if (gap > 100)
  {
    speed = 255;
  }
*/
  // Determine direction
  if (current_position < wanted_position)
  {
    run_cw();
  }

  if (current_position > wanted_position)
  {
    run_ccw();
  }
}

servo_status MegaServo::read()
{
  // Get current postion, wanted position and speed
  servo_status current_status;
  
  int current_angle = map(get_position(), MIN_POT_VALUE, MAX_POT_VALUE, MIN_ANGLE, MAX_ANGLE);
  int wanted_angle = map(wanted_position, MIN_POT_VALUE, MAX_POT_VALUE, MIN_ANGLE, MAX_ANGLE);

  current_status.current_angle = current_angle;
  current_status.wanted_angle = wanted_angle;
  current_status.speed = speed;

  return current_status;
}

servo_status MegaServo::readVolume()
{
  // Get current postion, wanted position and speed
  servo_status current_status;
  
  int current_volume = map(get_position(), MIN_POT_VALUE, MAX_POT_VALUE, MIN_VOLUME, MAX_VOLUME);
  int wanted_volume = map(wanted_position, MIN_POT_VALUE, MAX_POT_VALUE, MIN_VOLUME, MAX_VOLUME);

  current_status.current_volume = current_volume;
  current_status.wanted_volume = wanted_volume;
  current_status.speed = speed;

  return current_status;
}

/*************** Private stuff **************/

void MegaServo::write_pot_value(int value)
{
  // Constrain value to servo's min and max bounds
  int constrained_value = constrain(value, MIN_POT_VALUE + tolerance, MAX_POT_VALUE - tolerance);

  // set destination
  wanted_position = constrained_value;
}

// Run motor clockwise at current speed
void MegaServo::run_cw()
{
  digitalWrite(cw_pin, HIGH);
  digitalWrite(ccw_pin, LOW);

  analogWrite(enabler_pin, speed);
}

// Run motor counter clockwise at current speed
void MegaServo::run_ccw()
{
  digitalWrite(cw_pin, LOW);
  digitalWrite(ccw_pin, HIGH);

  analogWrite(enabler_pin, speed);
}

// Stop the motor, set speed to 0
void MegaServo::stop()
{
  digitalWrite(cw_pin, LOW);
  digitalWrite(ccw_pin, LOW);

  analogWrite(enabler_pin, 0); 
}

// Get current offset potentiometer position
int MegaServo::get_position()
{
  return analogRead(sensor_pin) + offset;
}