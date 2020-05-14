#ifndef __ALARMS_H
#define __ALARMS_H
#include "header.h"


// pins for testing
#define BATTERY_ALARM           37
#define CKT_INTEGRITY_ALARM     36
#define OXYGEN_ALARM            35
#define VENT_DIS_ALARM          34
#define PRESSURE_DIS_ALARM      33
#define MECHANICAL_INT_ALARM    32
#define HOMING_NOT_DONE_ALARM   31
#define HOURS_96_ALARM          30
#define FLOW_SENSOR_DIS_ALARM   12
#define O2_SENSOR_DIS_ALARM     13


// Define Priority of Alarms
#define PEAK_ALARM_PRIORITY              1
#define PLATEAU_PRESSURE_ALARM_PRIORITY  2
#define PEEP_ALARM_PRIORITY              3
#define VENT_DIS_ALARM_PRIORITY          4
#define OXYGEN_ALARM_PRIORITY            5
#define TIDAL_VOLUME_ALARM_PRIORITY      6
#define FIO2_ALARM_PRIORITY              7
#define RR_RATE_ALARM_PRIORITY           8
#define MINUTE_VOLUME_ALARM_PRIORITY     9
#define BATTERY_ALARM_PRIORITY           10
#define PRESSURE_DIS_ALARM_PRIORITY      11
#define MECHANICAL_INT_ALARM_PRIORITY    12
#define HOMING_NOT_DONE_ALARM_PRIORITY   13
#define HOURS_96_ALARM_PRIORITY          14
#define FLOW_SENSOR_DIS_ALARM_PRIORITY   15
#define O2_SENSOR_DIS_ALARM_PRIORITY     16
#define CKT_INTEGRITY_ALARM_PRIORITY     17

// Snooze timeout values - Time in seconds
#define TIMEOUT_BATTERY_ALARM           10
#define TIMEOUT_CKT_INTEGRITY_ALARM     10
#define TIMEOUT_OXYGEN_ALARM            10
#define TIMEOUT_VENT_DIS_ALARM          10
#define TIMEOUT_PRESSURE_DIS_ALARM      10
#define TIMEOUT_MECHANICAL_INT_ALARM    10
#define TIMEOUT_HOMING_NOT_DONE_ALARM   10
#define TIMEOUT_HOURS_96_ALARM          10
#define TIMEOUT_FLOW_SENSOR_DIS_ALARM   10
#define TIMEOUT_O2_SENSOR_DIS_ALARM     10
#define TIMEOUT_RR_RATE_ALARM           10
#define TIMEOUT_PEAK_ALARM              10
#define TIMEOUT_FIO2_ALARM              10
#define TIMEOUT_TIDAL_VOLUME_ALARM      10
#define TIMEOUT_MINUTE_VOLUME_ALARM     10
#define TIMEOUT_PEEP_ALARM              10
#define TIMEOUT_PLATEAU_PRESSURE_ALARM  10

// Wait duration after values are out of range
#define WAIT_MAX_RR                    5
#define WAIT_MAX_PEAK                  5
#define WAIT_MAX_FIO2                  5
#define WAIT_MAX_TIDAL_VOL             5
#define WAIT_MAX_MIN_VOLUME            5
#define WAIT_MAX_PEEP                  5
#define WAIT_MAX_PLATEAU               5

// Used for showing sensor reading are high or low
#define VALUE_HIGH                     true
#define VALUE_LOW                      false

#define ALARM_SET                      2

struct ALARMS
{
    // Thresholds for checking sensor values
    const float THRESHOLD_RR                 =  1.0;
    const float THRESHOLD_PEAK               =  5.0;
    const float THRESHOLD_FIO2               =  2.0;
    const float THRESHOLD_TIDAL_VOL          =  15.0;  // %
    const float THRESHOLD_MIN_VOLUME         =  1.0;  //  %
    const float THRESHOLD_PEEP_HIGH          =  21.0;
    const float THRESHOLD_PEEP_LOW           =  4.0;
    const float THRESHOLD_HIGH_PLATEAU       =  2.0;

    // Sensor reading is High or Low
    bool Alarm_type_RR               = VALUE_LOW;
    bool Alarm_type_PEAK             = VALUE_LOW;
    bool Alarm_type_FIO2             = VALUE_LOW;
    bool Alarm_type_TIDAL_VOL        = VALUE_LOW;
    bool Alarm_type_MIN_VOLUME       = VALUE_LOW;
    bool Alarm_type_PEEP             = VALUE_LOW;
    bool Alarm_type_PLATEAU          = VALUE_LOW;

    // Wait counter
    long Wait_count_RR            = 0;
    long Wait_count_PEAK          = 0;
    long Wait_count_FIO2          = 0;
    long Wait_count_TIDAL_VOL     = 0;
    long Wait_count_MIN_VOLUME    = 0;
    long Wait_count_PEEP          = 0;
    long Wait_count_PLATEAU       = 0;

    bool Flag_init_RR            = true;
    bool Flag_init_PEAK          = true;
    bool Flag_init_FIO2          = true;
    bool Flag_init_TIDAL_VOL     = true;
    bool Flag_init_MIN_VOLUME    = true;
    bool Flag_init_PEEP          = true;
    bool Flag_init_PLATEAU       = true;

    // Alarm Flags
    bool Flag_battery_alarm           = true;
    bool Flag_ckt_integrity_alarm     = true;
    bool Flag_oxygen_alarm            = true;
    bool Flag_vent_dis_alarm          = true;
    bool Flag_pressure_dis_alarm      = true;
    bool Flag_mech_int_alarm          = true;
    bool Flag_homing_alarm            = true;
    bool Flag_96hours_alarm           = true;
    bool Flag_flow_sensor_dis_alarm   = true;
    bool Flag_O2_dis_alarm            = true;
    bool Flag_RR_Rate_alarm           = true;
    bool Flag_PEAK_alarm              = true;
    bool Flag_FiO2_alarm              = true;
    bool Flag_Tidal_volume_alarm      = true;
    bool Flag_Minute_volume_alarm     = true;
    bool Flag_peep_alarm              = true;
    bool Flag_Plateau_alarm           = true;

    //generated alarm
    bool Status_System_on_battery      = false;
    bool Status_ckt_integrity          = false;
    bool Status_oxygen_failure         = false;
    bool Status_vent_dis               = false;
    bool Status_pressure_dis           = false;
    bool Status_mech_int               = false;
    bool Status_homing                 = false;
    bool Status_96hours                = false;
    bool Status_flow_sensor_dis        = false;
    bool Status_O2_dis                 = false;

    // Alarm Snooze flag
    bool Flag_Snooze_battery_alarm          = false;
    bool Flag_Snooze_ckt_integrity_alarm    = false;
    bool Flag_Snooze_oxygen_alarm           = false;
    bool Flag_Snooze_vent_dis_alarm         = false;
    bool Flag_Snooze_pressure_dis_alarm     = false;
    bool Flag_Snooze_mech_int_alarm         = false;
    bool Flag_Snooze_homing_alarm           = false;
    bool Flag_Snooze_96hours_alarm          = false;
    bool Flag_Snooze_flow_sensor_dis_alarm  = false;
    bool Flag_Snooze_O2_dis_alarm           = false;
    bool Flag_Snooze_RR_Rate_alarm          = false;
    bool Flag_Snooze_PEAK_alarm             = false;
    bool Flag_Snooze_FiO2_alarm             = false;
    bool Flag_Snooze_Tidal_volume_alarm     = false;
    bool Flag_Snooze_Minute_volume_alarm    = false;
    bool Flag_Snooze_peep_alarm             = false;
    bool Flag_Snooze_Plateau_alarm          = false;

    // Alarm timer counter
    long Snooze_Count_battery_alarm         = 0;
    long Snooze_Count_ckt_integrity_alarm   = 0;
    long Snooze_Count_oxygen_alarm          = 0;
    long Snooze_Count_vent_dis_alarm        = 0;
    long Snooze_Count_pressure_dis_alarm    = 0;
    long Snooze_Count_mech_int_alarm        = 0;
    long Snooze_Count_homing_alarm          = 0;
    long Snooze_Count_96hours_alarm         = 0;
    long Snooze_Count_flow_sensor_dis_alarm = 0;
    long Snooze_Count_O2_dis_alarm          = 0;
    long Snooze_Count_RR_Rate_alarm         = 0;
    long Snooze_Count_PEAK_alarm            = 0;
    long Snooze_Count_FiO2_alarm            = 0;
    long Snooze_Count_Tidal_volume_alarm    = 0;
    long Snooze_Count_Minute_volume_alarm   = 0;
    long Snooze_Count_peep_alarm            = 0;
    long Snooze_Count_Plateau_alarm         = 0;

    bool Flag_alarm = false;

    unsigned int alarms_priority_table[20][1] ;
    unsigned int set_alarm = 0;
    unsigned int previous_alarm = 0;

    // param from the sensors
    float Sensor_val_RR = defaultBPM;
    float Sensor_val_PEAK = defaultPressure;
    float Sensor_val_FiO = 55.0;
    float Sensor_val_TV = defaultVolume;
    float Sensor_val_Min_vol = defaultVolume * defaultBPM;
    float Sensor_PEEP = 15;
    float Sensor_val_High_Plateau = defaultPressure;

    byte Error_status_byte_1 = 0;
    byte Error_status_byte_2 = 0;
    byte Error_status_byte_3 = 0;
    byte Error_status_byte_4 = 0;

    };

void alarmsSetup();
void alarmControl();
#endif