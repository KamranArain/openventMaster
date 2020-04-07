#ifndef __MEGA_SERVO_H
#define __MEGA_SERVO_H

#define MAX_SERVOS 2

#define MIN_POT_VALUE 180
#define MAX_POT_VALUE 800

#define MIN_ANGLE 5
#define MAX_ANGLE 355

#define MIN_VOLUME 5    //mL
#define MAX_VOLUME 1000 //mL

typedef unsigned char uint8_t;

typedef struct
{
  int wanted_angle;
  int current_angle;
  int wanted_volume;
  int current_volume;
  uint8_t speed;
} servo_status;

class MegaServo
{
public:
  MegaServo();
  void attach(int _cw_pin, int _ccw_pin, int _enablerPin, uint8_t _sensor_pin);
  void attach(int _cw_pin, int _ccw_pin, int _enablerPin, uint8_t _sensor_pin, int _offset);
  void detach();
  void write(int degrees);
  void writeVolume(int vol);
  void setSpeed(int _speed);
  void setTol(int _tolerance);
  void setOffset(int _offset);
  void update();
  void run_cw();  //Function Made Public: To Run Motor in Open Loop without Pot
  void run_ccw(); //Function Made Public: To Run Motor in Open Loop without Pot
  servo_status read();
  servo_status readVolume();
  void stop(); //Function Made Public: To Run Motor in Open Loop without Pot

  bool active;
  int index;

private:
  int offset;
  int cw_pin;
  int ccw_pin;
  int enabler_pin;
  uint8_t sensor_pin;
  uint8_t speed;
  int wanted_position;
  int tolerance;

  void write_pot_value(int degrees);
  int get_position();
};

static int s_index;                      // number of registered servo's
static MegaServo *_m_servos[MAX_SERVOS]; // array containing registered servo's

#endif
