#include <Arduino.h>
#include "alarms.h"
#include "sensors.h"
#include "userInterface.h"
#include "TimerThree.h"


extern bool key_int_flag;

extern uint8_t   FiO2_Value[4][2];

extern uint16_t breathLength;

extern struct STATUS_FLAGS status;
extern struct setpointStatus setpoint;
extern struct MONITORING_PARAMS monitoring;
extern struct TidalVolume TV;
extern struct O2_Sensor O2;

struct ALARMS Alarms;
#ifdef Beeper
struct Buzzer buzzer;
#endif


// for testing
uint8_t Set_Param_PEEP = 0;
int LED_STATE = HIGH;

void timer3Isr();
void Snooze_alarm();
void CheckSensorData(void);
void CheckAlarms(void);
void AlarmsGen(void);
void AlarmsTelemetry(void);
#ifdef Beeper //Function Needs Revision
void beep();
#endif

void CheckSensorData(void)
{

    // Check FiO2
    if((Alarms.Sensor_val_FiO > ((float) FiO2_Value[setpoint.curFiO2][1] + Alarms.THRESHOLD_FIO2)) && status.FIO2Valid)
    {
        if(Alarms.Flag_init_FIO2)
        {
          Alarms.Alarm_type_FIO2 = VALUE_HIGH;
          Alarms.Flag_init_FIO2 = false;
          Alarms.Wait_count_FIO2 = 0;
        }
    }
    else if((Alarms.Sensor_val_FiO < ((float) FiO2_Value[setpoint.curFiO2][0] - Alarms.THRESHOLD_FIO2)) && status.FIO2Valid)
    {
        if(Alarms.Flag_init_FIO2)
        {
          Alarms.Alarm_type_FIO2 = VALUE_LOW;
          Alarms.Flag_init_FIO2 = false;
          Alarms.Wait_count_FIO2 = 0;
        }
    }
    else
    {
      Alarms.Flag_init_FIO2 = true;
      Alarms.Flag_FiO2_alarm = 1;
    }

  if (status.VentilatorOperationON) {
    // Check RR
    if((Alarms.Sensor_val_RR >  ((float) setpoint.curBPM + Alarms.THRESHOLD_RR)) && status.RRValid)
    {

      // if (!Alarms.Flag_init_RR && Alarms.Alarm_type_RR == VALUE_LOW)
      // {
      //   Alarms.Flag_init_RR = true;
      // }

      if(Alarms.Flag_init_RR)
      {
        Alarms.Alarm_type_RR = VALUE_HIGH;
        Alarms.Flag_init_RR = false;
        Alarms.Wait_count_RR = 0;
      }
    }
    else if((Alarms.Sensor_val_RR < ((float) setpoint.curBPM - Alarms.THRESHOLD_RR)) && status.RRValid)
    {
      // if (!Alarms.Flag_init_RR && Alarms.Alarm_type_RR == VALUE_HIGH)
      // {
      //   Alarms.Flag_init_RR = true;
      // }

      if(Alarms.Flag_init_RR)
      {
        Alarms.Alarm_type_RR = VALUE_LOW;
        Alarms.Flag_init_RR = false;
        Alarms.Wait_count_RR = 0;
      }
    }
    else
    {
      Alarms.Flag_init_RR = true;
      Alarms.Flag_RR_Rate_alarm = 1;
    }

    // Check Tidal Volume
    if(Alarms.Sensor_val_TV > (float) setpoint.curVolume + ((Alarms.THRESHOLD_TIDAL_VOL/100.0) * (float) setpoint.curVolume)   && ( setpoint.curVentMode == VENT_MODE_VCV || setpoint.curVentMode == VENT_MODE_AC_VCV) && status.TidalVolValid)
    {
      if(Alarms.Flag_init_TIDAL_VOL)
      {
        Alarms.Alarm_type_TIDAL_VOL = VALUE_HIGH;
        Alarms.Flag_init_TIDAL_VOL = false;
        Alarms.Wait_count_TIDAL_VOL = 0;
      }
    }
    else if(Alarms.Sensor_val_TV < (float) setpoint.curVolume - ((Alarms.THRESHOLD_TIDAL_VOL/100.0) * (float) setpoint.curVolume)   && ( setpoint.curVentMode == VENT_MODE_VCV || setpoint.curVentMode == VENT_MODE_AC_VCV) && status.TidalVolValid)
    {
      if(Alarms.Flag_init_TIDAL_VOL)
      {
        Alarms.Alarm_type_TIDAL_VOL = VALUE_LOW;
        Alarms.Flag_init_TIDAL_VOL = false;
        Alarms.Wait_count_TIDAL_VOL = 0;
      }
    }
    else
    {
      Alarms.Flag_init_TIDAL_VOL = true;
      Alarms.Flag_Tidal_volume_alarm = 1;
    }
      
    // Check Peak inspiratory pressure
    if((Alarms.Sensor_val_PEAK > ((float) setpoint.curPressure + Alarms.THRESHOLD_PEAK) ) && status.PeakPrsValid)
    {
      if(Alarms.Flag_init_PEAK)
      {
        Alarms.Alarm_type_PEAK = VALUE_HIGH;
        Alarms.Flag_init_PEAK = false;
        Alarms.Wait_count_PEAK = 0;
      }
    }
    else if(Alarms.Sensor_val_PEAK < (float) setpoint.curPressure - Alarms.THRESHOLD_PEAK  && ( setpoint.curVentMode == VENT_MODE_PCV || setpoint.curVentMode == VENT_MODE_AC_PCV) && status.PeakPrsValid)
    {
      if(Alarms.Flag_init_PEAK)
      {
        Alarms.Alarm_type_PEAK = VALUE_LOW;
        Alarms.Flag_init_PEAK = false;
        Alarms.Wait_count_PEAK = 0;
      }
    }
    else
    {
      Alarms.Flag_init_PEAK = true;
      Alarms.Flag_PEAK_alarm = 1;
    }

    // check Minute Volume 
    if(Alarms.Sensor_val_Min_vol >  ((float) setpoint.curBPM * (float) setpoint.curVolume) + ((Alarms.THRESHOLD_MIN_VOLUME/100.0) * ((float) setpoint.curBPM * (float) setpoint.curVolume)) && ( setpoint.curVentMode == VENT_MODE_VCV || setpoint.curVentMode == VENT_MODE_AC_VCV) && status.MinVolValid)
    {
        if(Alarms.Flag_init_MIN_VOLUME)
        {
          Alarms.Alarm_type_MIN_VOLUME = VALUE_HIGH;
          Alarms.Flag_init_MIN_VOLUME = false;
          Alarms.Wait_count_MIN_VOLUME = 0;
        }
    }
    else if(Alarms.Sensor_val_Min_vol < ((float) setpoint.curBPM * (float) setpoint.curVolume) - ((Alarms.THRESHOLD_MIN_VOLUME/100.0) * ((float) setpoint.curBPM * (float) setpoint.curVolume)) && ( setpoint.curVentMode == VENT_MODE_VCV || setpoint.curVentMode == VENT_MODE_AC_VCV) && status.MinVolValid)
    {
        if(Alarms.Flag_init_MIN_VOLUME)
        {
          Alarms.Alarm_type_MIN_VOLUME = VALUE_LOW;
          Alarms.Flag_init_MIN_VOLUME = false;
          Alarms.Wait_count_MIN_VOLUME = 0;
        }
    }
    else
    {
      Alarms.Flag_init_MIN_VOLUME = true;
      Alarms.Flag_Minute_volume_alarm = 1;
    }

  // check PEEP 
    if((Alarms.Sensor_val_PEEP >= Alarms.THRESHOLD_PEEP_HIGH) && status.PeepValid)
    {
        if(Alarms.Flag_init_PEEP)
        {
          Alarms.Alarm_type_PEEP = VALUE_HIGH;
          Alarms.Flag_init_PEEP = false;
          Alarms.Wait_count_PEEP = 0;
        }
    }
    else if((Alarms.Sensor_val_PEEP <= Alarms.THRESHOLD_PEEP_LOW) && status.PeepValid)
    {
        if(Alarms.Flag_init_PEEP)
        {
            Alarms.Alarm_type_PEEP = VALUE_LOW;
            Alarms.Flag_init_PEEP = false;
            Alarms.Wait_count_PEEP = 0;
        }
    }
    else
    {
      Alarms.Flag_init_PEEP = true;
      Alarms.Flag_peep_alarm = 1;
    }

    // check High Plateau Pressure 
    if((Alarms.Sensor_val_Plateau > ((float) setpoint.curPressure + Alarms.THRESHOLD_PLATEAU)) && status.PltPrsValid )
    {
        if(Alarms.Flag_init_PLATEAU)
        {
          Alarms.Alarm_type_PLATEAU = VALUE_HIGH;
          Alarms.Flag_init_PLATEAU = false;
          Alarms.Wait_count_PLATEAU = 0;
        }
    }
    else if(Alarms.Sensor_val_Plateau < (float) setpoint.curPressure - Alarms.THRESHOLD_PLATEAU && ( setpoint.curVentMode == VENT_MODE_PCV || setpoint.curVentMode == VENT_MODE_AC_PCV) && status.PltPrsValid )
    {
        if(Alarms.Flag_init_PLATEAU)
        {
            Alarms.Alarm_type_PLATEAU = VALUE_LOW;
            Alarms.Flag_init_PLATEAU = false;
            Alarms.Wait_count_PLATEAU = 0;
        }
    }
    else
    {
      Alarms.Flag_init_PLATEAU = true;
      Alarms.Flag_Plateau_alarm = 1;
    }
  }
  else {
//      Alarms.Flag_init_FIO2 = true;
//      Alarms.Wait_count_FIO2 = 0;
//      status.FIO2Valid = false;

      Alarms.Flag_init_RR = true;
      Alarms.Wait_count_RR = 0;

      Alarms.Flag_init_TIDAL_VOL = true;
      Alarms.Wait_count_TIDAL_VOL = 0;

      Alarms.Flag_init_MIN_VOLUME = true;
      Alarms.Wait_count_MIN_VOLUME = 0;

      Alarms.Flag_init_PEAK = true;
      Alarms.Wait_count_PEAK = 0;

      Alarms.Flag_init_PEEP = true;
      Alarms.Wait_count_PEEP = 0;

      Alarms.Flag_init_PLATEAU = true;
      Alarms.Wait_count_PLATEAU = 0;      
  }

}

void timer3Isr()
{
   digitalWrite(pin_LED1, LED_STATE);
   LED_STATE = !LED_STATE;
  // delay logic for sensor readings
  if(!Alarms.Flag_init_RR)
  {
    Alarms.Wait_count_RR++;
    if( Alarms.Wait_count_RR > (uint32_t)Alarms.WAIT_MAX_RR)
    {
      Alarms.Flag_init_RR = true;
      Alarms.Flag_RR_Rate_alarm = 0;
    }
  }
  
  if(!Alarms.Flag_init_PEAK)
  {
    Alarms.Wait_count_PEAK++;
    if( Alarms.Wait_count_PEAK > (uint32_t)Alarms.WAIT_MAX_PEAK)
    {
      Alarms.Flag_init_PEAK = true;
      Alarms.Flag_PEAK_alarm = 0;
    }
  }
 
  if(!Alarms.Flag_init_TIDAL_VOL)
  {
    Alarms.Wait_count_TIDAL_VOL++;
    if( Alarms.Wait_count_TIDAL_VOL > (uint32_t)Alarms.WAIT_MAX_TIDAL_VOL)
    {
      Alarms.Flag_init_TIDAL_VOL = true;
      Alarms.Flag_Tidal_volume_alarm = 0;
    }
  }

  if(!Alarms.Flag_init_FIO2)
  {
    Alarms.Wait_count_FIO2++;
    if( Alarms.Wait_count_FIO2 > (uint32_t)Alarms.WAIT_MAX_FIO2)
    {
      Alarms.Flag_init_FIO2 = true;
      Alarms.Flag_FiO2_alarm = 0;
    }
  }

  if(!Alarms.Flag_init_MIN_VOLUME)
  {
    Alarms.Wait_count_MIN_VOLUME++;
    if( Alarms.Wait_count_MIN_VOLUME > (uint32_t)Alarms.WAIT_MAX_MIN_VOLUME)
    {
      Alarms.Flag_init_MIN_VOLUME = true;
      Alarms.Flag_Minute_volume_alarm = 0;
    }
  }

  if(!Alarms.Flag_init_PEEP)
  {
    Alarms.Wait_count_PEEP++;
    if( Alarms.Wait_count_PEEP > (uint32_t)Alarms.WAIT_MAX_PEEP)
    {
      // Serial.println("PEEP ALARM 1");
      Alarms.Flag_init_PEEP = true;
      Alarms.Flag_peep_alarm = 0;
    }
  }

  if(!Alarms.Flag_init_PLATEAU)
  {
    Alarms.Wait_count_PLATEAU++;
    if( Alarms.Wait_count_PLATEAU > (uint32_t)Alarms.WAIT_MAX_PLATEAU)
    {
      Alarms.Flag_init_PLATEAU = true;
      Alarms.Flag_Plateau_alarm = 0;
    }
  }

  // Snooze logic
  if(Alarms.Flag_Snooze_battery_alarm)
  {
      Alarms.Snooze_Count_battery_alarm++;
      if( Alarms.Snooze_Count_battery_alarm > (uint32_t)TIMEOUT_BATTERY_ALARM)
      {
          Alarms.Flag_Snooze_battery_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_ckt_integrity_alarm)
  {
      Alarms.Snooze_Count_ckt_integrity_alarm++;
      if( Alarms.Snooze_Count_ckt_integrity_alarm > (uint32_t)TIMEOUT_CKT_INTEGRITY_ALARM)
      {
          Alarms.Flag_Snooze_ckt_integrity_alarm = false;
      }
  }
  
  if(Alarms.Flag_Snooze_oxygen_alarm)
  {
      Alarms.Snooze_Count_oxygen_alarm++;
      if( Alarms.Snooze_Count_oxygen_alarm > (uint32_t)TIMEOUT_OXYGEN_ALARM)
      {
          Alarms.Flag_Snooze_oxygen_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_vent_dis_alarm)
  {
      Alarms.Snooze_Count_vent_dis_alarm++;
      if( Alarms.Snooze_Count_vent_dis_alarm > (uint32_t)TIMEOUT_VENT_DIS_ALARM)
      {
          Alarms.Flag_Snooze_vent_dis_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_pressure_dis_alarm)
  {
      Alarms.Snooze_Count_pressure_dis_alarm++;
      if( Alarms.Snooze_Count_pressure_dis_alarm > (uint32_t)TIMEOUT_PRESSURE_DIS_ALARM)
      {
          Alarms.Flag_Snooze_pressure_dis_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_mech_int_alarm)
  {
      Alarms.Snooze_Count_mech_int_alarm++;
      if( Alarms.Snooze_Count_mech_int_alarm > (uint32_t)TIMEOUT_MECHANICAL_INT_ALARM)
      {
          Alarms.Flag_Snooze_mech_int_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_homing_alarm)
  {
      Alarms.Snooze_Count_homing_alarm++;
      if( Alarms.Snooze_Count_homing_alarm > (uint32_t)TIMEOUT_HOMING_NOT_DONE_ALARM)
      {
          Alarms.Flag_Snooze_homing_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_96hours_alarm)
  {
      Alarms.Snooze_Count_96hours_alarm++;
      if( Alarms.Snooze_Count_96hours_alarm > (uint32_t)TIMEOUT_HOURS_96_ALARM)
      {
          Alarms.Flag_Snooze_96hours_alarm = false;
      }
  }
  
  if(Alarms.Flag_Snooze_flow_sensor_dis_alarm)
  {
      Alarms.Snooze_Count_flow_sensor_dis_alarm++;
      if( Alarms.Snooze_Count_flow_sensor_dis_alarm > (uint32_t)TIMEOUT_FLOW_SENSOR_DIS_ALARM)
      {
          Alarms.Flag_Snooze_flow_sensor_dis_alarm = false;
      }
  }
  
  if(Alarms.Flag_Snooze_O2_dis_alarm)
  {
      Alarms.Snooze_Count_O2_dis_alarm++;
      if( Alarms.Snooze_Count_O2_dis_alarm > (uint32_t)TIMEOUT_O2_SENSOR_DIS_ALARM)
      {
          Alarms.Flag_Snooze_O2_dis_alarm = false;
      }
  }

  if(Alarms.Flag_Snooze_RR_Rate_alarm)
  {
      Alarms.Snooze_Count_RR_Rate_alarm++;
      if( Alarms.Snooze_Count_RR_Rate_alarm > (uint32_t)TIMEOUT_RR_RATE_ALARM)
      {
          Alarms.Flag_Snooze_RR_Rate_alarm = false;
      }
  }
  if(Alarms.Flag_Snooze_PEAK_alarm)
  {
      Alarms.Snooze_Count_PEAK_alarm++;
      if( Alarms.Snooze_Count_PEAK_alarm > (uint32_t)TIMEOUT_PEAK_ALARM)
      {
          Alarms.Flag_Snooze_PEAK_alarm = false;
      }
  }
  if(Alarms.Flag_Snooze_FiO2_alarm)
  {
      Alarms.Snooze_Count_FiO2_alarm++;
      if( Alarms.Snooze_Count_FiO2_alarm > (uint32_t)TIMEOUT_FIO2_ALARM)
      {
          Alarms.Flag_Snooze_FiO2_alarm = false;
      }
  }
  if(Alarms.Flag_Snooze_Tidal_volume_alarm)
  {
      Alarms.Snooze_Count_Tidal_volume_alarm++;
      if( Alarms.Snooze_Count_Tidal_volume_alarm > (uint32_t)TIMEOUT_TIDAL_VOLUME_ALARM)
      {
          Alarms.Flag_Snooze_Tidal_volume_alarm = false;
      }
  }
  if(Alarms.Flag_Snooze_Minute_volume_alarm)
  {
      Alarms.Snooze_Count_Minute_volume_alarm++;
      if( Alarms.Snooze_Count_Minute_volume_alarm > MINUTE_VOLUME_ALARM_PRIORITY)
      {
          Alarms.Flag_Snooze_Minute_volume_alarm = false;
      }
  }
  if(Alarms.Flag_Snooze_peep_alarm)
  {
      Alarms.Snooze_Count_peep_alarm++;
      if( Alarms.Snooze_Count_peep_alarm > (uint32_t)TIMEOUT_PEEP_ALARM)
      {
          Alarms.Flag_Snooze_peep_alarm = false;
      }
  }
  if(Alarms.Flag_Snooze_Plateau_alarm)
  {
      Alarms.Snooze_Count_Plateau_alarm++;
      if( Alarms.Snooze_Count_Plateau_alarm > (uint32_t)TIMEOUT_PLATEAU_PRESSURE_ALARM)
      {
          Alarms.Flag_Snooze_Plateau_alarm = false;
      }
  }
}

void Snooze_alarm()
{
      for(int i = 0 ; i < 20 ; i++)
      {
        if(Alarms.alarms_priority_table[i][0] == 2)
        {
            Alarms.alarms_priority_table[i][0] = 0;
            switch(i)
            {
                case BATTERY_ALARM_PRIORITY:  
                      Alarms.Flag_battery_alarm = true; 
                      Alarms.Flag_Snooze_battery_alarm = true; 
                      Alarms.Snooze_Count_battery_alarm = 0;
                      break;
                case CKT_INTEGRITY_ALARM_PRIORITY:  
                      Alarms.Flag_ckt_integrity_alarm = true;
                      Alarms.Flag_Snooze_ckt_integrity_alarm = true; 
                      Alarms.Snooze_Count_ckt_integrity_alarm = 0;
                      break;
                case OXYGEN_ALARM_PRIORITY: 
                      Alarms.Flag_oxygen_alarm = true;
                      Alarms.Flag_Snooze_oxygen_alarm = true; 
                      Alarms.Snooze_Count_oxygen_alarm = 0;
                      break;
                case VENT_DIS_ALARM_PRIORITY: 
                      Alarms.Flag_vent_dis_alarm = true;
                      Alarms.Flag_Snooze_vent_dis_alarm = true; 
                      Alarms.Snooze_Count_vent_dis_alarm = 0;
                      break;
                case PRESSURE_DIS_ALARM_PRIORITY: 
                      Alarms.Flag_pressure_dis_alarm = true;
                      Alarms.Flag_Snooze_pressure_dis_alarm = true; 
                      Alarms.Snooze_Count_pressure_dis_alarm = 0;
                      break;
                case MECHANICAL_INT_ALARM_PRIORITY: 
                      Alarms.Flag_mech_int_alarm = true;
                      Alarms.Flag_Snooze_mech_int_alarm = true; 
                      Alarms.Snooze_Count_mech_int_alarm = 0;
                      break;
                case HOMING_NOT_DONE_ALARM_PRIORITY: 
                      Alarms.Flag_homing_alarm = true;
                      Alarms.Flag_Snooze_homing_alarm = true; 
                      Alarms.Snooze_Count_homing_alarm = 0;
                      break;
                case HOURS_96_ALARM_PRIORITY: 
                      Alarms.Flag_96hours_alarm = true;
                      Alarms.Flag_Snooze_96hours_alarm = true; 
                      Alarms.Snooze_Count_96hours_alarm = 0;
                      break;
                case FLOW_SENSOR_DIS_ALARM_PRIORITY:  
                      Alarms.Flag_flow_sensor_dis_alarm = true;
                      Alarms.Flag_Snooze_flow_sensor_dis_alarm = true; 
                      Alarms.Snooze_Count_flow_sensor_dis_alarm = 0;
                      break;
                case O2_SENSOR_DIS_ALARM_PRIORITY:  
                      Alarms.Flag_O2_dis_alarm= true;
                      Alarms.Flag_Snooze_O2_dis_alarm = true; 
                      Alarms.Snooze_Count_O2_dis_alarm = 0;
                      break;
                case RR_RATE_ALARM_PRIORITY: 
                      Alarms.Flag_RR_Rate_alarm = true;
                      Alarms.Flag_Snooze_RR_Rate_alarm = true; 
                      Alarms.Snooze_Count_RR_Rate_alarm = 0;
                      break;
                case PEAK_ALARM_PRIORITY: 
                      Alarms.Flag_PEAK_alarm = true;
                      Alarms.Flag_Snooze_PEAK_alarm = true; 
                      Alarms.Snooze_Count_PEAK_alarm = 0;
                      break;
                case FIO2_ALARM_PRIORITY: 
                      Alarms.Flag_FiO2_alarm = true;
                      Alarms.Flag_Snooze_FiO2_alarm = true; 
                      Alarms.Snooze_Count_FiO2_alarm = 0;
                      break;
                case TIDAL_VOLUME_ALARM_PRIORITY: 
                      Alarms.Flag_Tidal_volume_alarm = true;
                      Alarms.Flag_Snooze_Tidal_volume_alarm = true; 
                      Alarms.Snooze_Count_Tidal_volume_alarm = 0;
                      break;
                case MINUTE_VOLUME_ALARM_PRIORITY: 
                      Alarms.Flag_Minute_volume_alarm = true;
                      Alarms.Flag_Snooze_Minute_volume_alarm = true; 
                      Alarms.Snooze_Count_Minute_volume_alarm = 0;
                      break;
                case PEEP_ALARM_PRIORITY:  
                      Alarms.Flag_peep_alarm = true;
                      Alarms.Flag_Snooze_peep_alarm = true; 
                      Alarms.Snooze_Count_peep_alarm = 0;
                      break;
                case PLATEAU_PRESSURE_ALARM_PRIORITY:  
                      Alarms.Flag_Plateau_alarm= true;
                      Alarms.Flag_Snooze_Plateau_alarm = true; 
                      Alarms.Snooze_Count_Plateau_alarm = 0;
                      break;        
            }
          break;
        }
        key_int_flag=true;
        Alarms.previous_alarm = 99;
        Alarms.set_alarm = 98;
        
      }
}

void CheckAlarms(void)
{
  if(!Alarms.Flag_battery_alarm && Alarms.Flag_Snooze_battery_alarm == false)
  {
    Alarms.alarms_priority_table[BATTERY_ALARM_PRIORITY][0] = 2;    
  }
  if(!Alarms.Flag_ckt_integrity_alarm && Alarms.Flag_Snooze_ckt_integrity_alarm == false)
  {
    Alarms.alarms_priority_table[CKT_INTEGRITY_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_oxygen_alarm && Alarms.Flag_Snooze_oxygen_alarm == false)
  {
    Alarms.alarms_priority_table[OXYGEN_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_vent_dis_alarm && Alarms.Flag_Snooze_vent_dis_alarm == false)
  {
    Alarms.alarms_priority_table[VENT_DIS_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_pressure_dis_alarm && Alarms.Flag_Snooze_pressure_dis_alarm == false)
  {
    Alarms.alarms_priority_table[PRESSURE_DIS_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_mech_int_alarm && Alarms.Flag_Snooze_mech_int_alarm == false)
  {
    Alarms.alarms_priority_table[MECHANICAL_INT_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_homing_alarm && Alarms.Flag_Snooze_homing_alarm == false)
  {
    Alarms.alarms_priority_table[HOMING_NOT_DONE_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_96hours_alarm && Alarms.Flag_Snooze_96hours_alarm == false)
  {
    Alarms.alarms_priority_table[HOURS_96_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_flow_sensor_dis_alarm && Alarms.Flag_Snooze_flow_sensor_dis_alarm == false)
  {
    Alarms.alarms_priority_table[FLOW_SENSOR_DIS_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_O2_dis_alarm && Alarms.Flag_Snooze_O2_dis_alarm == false)
  {
    Alarms.alarms_priority_table[O2_SENSOR_DIS_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_RR_Rate_alarm && Alarms.Flag_Snooze_RR_Rate_alarm == false)
  {
    Alarms.alarms_priority_table[RR_RATE_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_PEAK_alarm && Alarms.Flag_Snooze_PEAK_alarm == false)
  {
    Alarms.alarms_priority_table[PEAK_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_FiO2_alarm && Alarms.Flag_Snooze_FiO2_alarm == false)
  {
    Alarms.alarms_priority_table[FIO2_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_Tidal_volume_alarm && Alarms.Flag_Snooze_Tidal_volume_alarm == false)
  {
    Alarms.alarms_priority_table[TIDAL_VOLUME_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_Minute_volume_alarm && Alarms.Flag_Snooze_Minute_volume_alarm == false)
  {
    Alarms.alarms_priority_table[MINUTE_VOLUME_ALARM_PRIORITY][0] = 2;
  }
  if(!Alarms.Flag_peep_alarm && Alarms.Flag_Snooze_peep_alarm == false)
  {
    Alarms.alarms_priority_table[PEEP_ALARM_PRIORITY][0] = 2;
          // Serial.println("PEEP ALARM 2");

  }
  if(!Alarms.Flag_Plateau_alarm && Alarms.Flag_Snooze_Plateau_alarm == false)
  {
    Alarms.alarms_priority_table[PLATEAU_PRESSURE_ALARM_PRIORITY][0] = 2;
  }

  // check priority
  for(int i = 0; i < 20 ; i++)
  {
    //Serial.print(i);
    //Serial.print("  ");
    //Serial.println(Alarms.alarms_priority_table[i][0]);
    if(Alarms.alarms_priority_table[i][0] == 2)
    {
      Alarms.set_alarm = i;
      Alarms.Flag_alarm = true;
      break;
    }
    else
    {
      Alarms.set_alarm = 0;
      Alarms.Flag_alarm = false;
    }
  }
}


void AlarmsTelemetry(void)
{
  Alarms.Error_status_byte_1 = 0;
  Alarms.Error_status_byte_2 = 0;
  Alarms.Error_status_byte_3 = 0;
  Alarms.Error_status_byte_4 = 0;
  
  Alarms.Error_status_byte_1 = (Alarms.Status_System_on_battery) | (Alarms.Status_ckt_integrity << 1);
  Alarms.Error_status_byte_2 = (Alarms.Status_oxygen_failure << 3) | (status.systemReset << 6);

  Alarms.Error_status_byte_3 = (Alarms.Status_vent_dis << 1) | (Alarms.Status_mech_int << 2) | (Alarms.Status_homing <<3) | 
                        (Alarms.Status_96hours << 4)  | (Alarms.Status_flow_sensor_dis << 5) | (Alarms.Status_pressure_dis  << 6)  |
                        (Alarms.Status_O2_dis << 7);

  // byte 1
  if(Alarms.alarms_priority_table[RR_RATE_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_RR == VALUE_HIGH )
  {
      Alarms.Error_status_byte_1 = Alarms.Error_status_byte_1 | 1 << 2;
  }

  if(Alarms.alarms_priority_table[FIO2_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_FIO2 == VALUE_HIGH )
  {
      Alarms.Error_status_byte_1 = Alarms.Error_status_byte_1 | 1 << 3;
  }

  if(Alarms.alarms_priority_table[PEEP_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_PEEP == VALUE_HIGH )
  {
      Alarms.Error_status_byte_1 = Alarms.Error_status_byte_1 | 1 << 4;
  }

  if(Alarms.alarms_priority_table[PLATEAU_PRESSURE_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_PLATEAU == VALUE_HIGH )
  {
      Alarms.Error_status_byte_1 = Alarms.Error_status_byte_1 | 1 << 5;
  }
  if(Alarms.alarms_priority_table[PEAK_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_PEAK == VALUE_HIGH )
  {
      Alarms.Error_status_byte_1 = Alarms.Error_status_byte_1 | 1 << 6;
  }
  if(Alarms.alarms_priority_table[PEAK_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_PEAK == VALUE_LOW )
  {
      Alarms.Error_status_byte_1 = Alarms.Error_status_byte_1 | 1 << 7;
  }


  // byte 2
  if(Alarms.alarms_priority_table[FIO2_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_FIO2 == VALUE_LOW )
  {
      Alarms.Error_status_byte_2 = Alarms.Error_status_byte_2 | 1 ;
  }

  if(Alarms.alarms_priority_table[PEEP_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_PEEP == VALUE_LOW )
  {
      Alarms.Error_status_byte_2 = Alarms.Error_status_byte_2 | 1 << 1;
  }

  if(Alarms.alarms_priority_table[PLATEAU_PRESSURE_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_PLATEAU == VALUE_LOW )
  {
      Alarms.Error_status_byte_2 = Alarms.Error_status_byte_2 | 1 << 2;
  }

  if(Alarms.alarms_priority_table[TIDAL_VOLUME_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_TIDAL_VOL == VALUE_LOW )
  {
      Alarms.Error_status_byte_2 = Alarms.Error_status_byte_2 | 1 << 4;
  }
  if(Alarms.alarms_priority_table[TIDAL_VOLUME_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_TIDAL_VOL == VALUE_HIGH )
  {
      Alarms.Error_status_byte_2 = Alarms.Error_status_byte_2 | 1 << 5;
  }
  if(Alarms.alarms_priority_table[MINUTE_VOLUME_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_MIN_VOLUME == VALUE_LOW )
  {
      Alarms.Error_status_byte_2 = Alarms.Error_status_byte_2 | 1 << 7;
  }


  // byte 3

  if(Alarms.alarms_priority_table[MINUTE_VOLUME_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_MIN_VOLUME == VALUE_HIGH )
  {
      Alarms.Error_status_byte_3 = Alarms.Error_status_byte_3 | 1 ;
  }


  // byte 4
  if(Alarms.alarms_priority_table[RR_RATE_ALARM_PRIORITY][0] == ALARM_SET && Alarms.Alarm_type_RR == VALUE_LOW )
  {
      Alarms.Error_status_byte_4 = Alarms.Error_status_byte_4 | 1;
  }


// for testing
  // Serial.print(" byte_1 BIN:");
  // Serial.print(Alarms.Error_status_byte_1,BIN);
  // Serial.print("  HEX:");
  // Serial.print(Alarms.Error_status_byte_1,HEX);
  

  // Serial.print("\tbyte_2 BIN:");
  // Serial.print(Alarms.Error_status_byte_2,BIN);
  // Serial.print("  HEX:");
  // Serial.print(Alarms.Error_status_byte_2,HEX);


  // Serial.print("\tbyte_3 BIN:");
  // Serial.print(Alarms.Error_status_byte_3,BIN);
  // Serial.print("  HEX:");
  // Serial.print(Alarms.Error_status_byte_3,HEX);

  // Serial.print(" byte_4 BIN:");
  // Serial.print(Alarms.Error_status_byte_4,BIN);
  // Serial.print("  HEX:");
  // Serial.println(Alarms.Error_status_byte_4,HEX);
  

}

void SetAlarmsWaitTimes()
{
    // Wait duration after values are out of range
  Alarms.WAIT_MAX_RR     =               ceil(1/TIMER3_PERIOD);
  Alarms.WAIT_MAX_PEAK    =              ceil(1*breathLength/(TIMER3_PERIOD*1000));
  // Alarms.WAIT_MAX_PEAK  =                12/TIMER3_PERIOD;
  Alarms.WAIT_MAX_FIO2      =            ceil(2*breathLength/(TIMER3_PERIOD*1000));
  Alarms.WAIT_MAX_TIDAL_VOL  =           ceil(3*breathLength/(TIMER3_PERIOD*1000));
  Alarms.WAIT_MAX_MIN_VOLUME  =          ceil(1/TIMER3_PERIOD);
  Alarms.WAIT_MAX_PEEP         =         ceil(1*breathLength/(TIMER3_PERIOD*1000));
  Alarms.WAIT_MAX_PLATEAU       =        ceil(1*breathLength/(TIMER3_PERIOD*1000));
}

void AlarmsGen(void)
{
//For Testing
  status.MinVolValid = false;
  status.RRValid = false;



  status.homingFailure = false;
  status.presSensorFailure = false;
  status.flowSensorFailure = false;
  status.compressionMechFailure = false;
   status.oxygenFailure = false;
 status.ventCktDisconnected = false;
  status.PeakPrsValid = false;
  status.PltPrsValid = false;
  status.PeepValid = false;
  status.TidalVolValid = false;
  status.FIO2Valid = false;
  
  if(status.battInUse)
  {
    if(Alarms.Status_System_on_battery == false)
    {
      Alarms.Status_System_on_battery = true;
      Alarms.Flag_battery_alarm = false;
    }
  }
  else
  {
    Alarms.Status_System_on_battery = false;
  }

  if(status.circuitIntegrityFailure)
  {
    if(Alarms.Status_ckt_integrity == false)
    {
      Alarms.Status_ckt_integrity = true;
      Alarms.Flag_ckt_integrity_alarm = false;
    }
  }
  else
  {
    Alarms.Status_ckt_integrity = false;
  }


  if(status.oxygenFailure)
  {      
    if(Alarms.Status_oxygen_failure == false)
    {
      Alarms.Status_oxygen_failure = true;
      Alarms.Flag_oxygen_alarm = false;
    }
  }
  else
  {
    Alarms.Status_oxygen_failure = false;
  }

  if(status.ventCktDisconnected)
  {      
    if(Alarms.Status_vent_dis == false && status.ventCktDisconnectedValid)
    {
      Alarms.Status_vent_dis = true;
      Alarms.Flag_vent_dis_alarm = false;
    }
  }
  else
  {
    Alarms.Status_vent_dis = false;
  }

  if(status.presSensorFailure)
  {      
    if(Alarms.Status_pressure_dis == false)
    {
      Alarms.Status_pressure_dis = true;
      Alarms.Flag_pressure_dis_alarm = false;
    }
  }
  else
  {
    Alarms.Status_pressure_dis = false;
  }


  // if(status.mechIntergrityFaiure)
  if(status.compressionMechFailure)
  {      
    if(Alarms.Status_mech_int == false)
    {
      Alarms.Status_mech_int = true;
      Alarms.Flag_mech_int_alarm = false;
    }
  }
  else
  {
    Alarms.Status_mech_int = false;
  }

  if(status.homingFailure)
  {      
    if(Alarms.Status_homing == false)
    {
      Alarms.Status_homing = true;
      Alarms.Flag_homing_alarm = false;
    }
  }
  else
  {
    Alarms.Status_homing = false;
  }

  if(status.hours96_complete)
  {      
    if(Alarms.Status_96hours == false)
    {
      Alarms.Status_96hours = true;
      Alarms.Flag_96hours_alarm = false;
    }
  }
  else
  {
    Alarms.Status_96hours = false;
  }

  if(status.flowSensorFailure)
  {      
    if(Alarms.Status_flow_sensor_dis == false)
    {
      Alarms.Status_flow_sensor_dis = true;
      Alarms.Flag_flow_sensor_dis_alarm = false;
    }
  }
  else
  {
    Alarms.Status_flow_sensor_dis = false;
  }

  if(status.o2SensorFailure)
  {      
    if(Alarms.Status_O2_dis == false)
    {
      Alarms.Status_O2_dis = true;
      Alarms.Flag_O2_dis_alarm = false;
    }
  }
  else
  {
    Alarms.Status_O2_dis = false;
  }

  // Alarms.Sensor_val_FiO = (float)FiO2_Value[setpoint.curFiO2][0] + 5.0;
  Alarms.Sensor_val_RR = monitoring.measuredRR;//setpoint.curBPM;

  Alarms.Sensor_val_FiO = O2.FIO2_conc;//(float)FiO2_Value[setpoint.reqFiO2][0] + 5.0;//O2.FIO2_conc;
  Alarms.Sensor_val_Min_vol = TV.minuteVentilation;
  Alarms.Sensor_val_TV = TV.maxInhale;
  Alarms.Sensor_val_PEAK = monitoring.peakInspPressure;
  Alarms.Sensor_val_Plateau = monitoring.plateauPressure;
  Alarms.Sensor_val_PEEP = monitoring.PEEPressure;
//  Serial.print("PEEP = ");Serial.println(Alarms.Sensor_val_PEEP);
//   Serial.print("PEEPValid = ");Serial.println(status.PeepValid);

}

void alarmsSetup()
{
    attachInterrupt(digitalPinToInterrupt(pin_SNOOZBTN), Snooze_alarm, RISING); 
    Timer3.initialize(TIMER3_PERIOD * 1000000UL); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
    Timer3.attachInterrupt( timer3Isr);
}

void alarmControl()
{
  AlarmsGen();
  CheckAlarms();
  CheckSensorData();
  AlarmsTelemetry();
  #ifdef Beeper //Function Needs Revision
    beep(); //alarmAction = RING_ALARM, SNOOZE_ALARM; alarmSeverity = SEVERITY_HIGH, SEVERITY_MED, SEVERITY_LOW, SEVERITY_MUTE
  #endif
}

#ifdef Beeper //Function Needs Revision

void beep() // Launch a beep
{
  static unsigned int currentToneFreq = SNOOZE_ALARM;
  static unsigned long t_millis = 0;

  if (buzzer.action == SNOOZE_ALARM)
  {
    noTone(pin_BUZZER);
    currentToneFreq = SNOOZE_ALARM;
    buzzer.toneFreq = SNOOZE_ALARM;
    buzzer.timePeriod = SNOOZE_ALARM;
  }
  else
  {
    if (buzzer.toneFreq > currentToneFreq)
    {
      currentToneFreq = buzzer.toneFreq; //High Severity Alarm Has priority
      t_millis = 0;
    }

    if ((millis() - t_millis) >= buzzer.timePeriod)
    {
      t_millis = millis();
      tone(pin_BUZZER, currentToneFreq, (int)(buzzer.timePeriod / 2)); //Duration in milliseconds
    }    
  }
}
#endif