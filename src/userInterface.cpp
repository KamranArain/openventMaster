#include "header.h"
#include "userInterface.h"
#include "alarms.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);

extern bool activateVentilatorOperation;
extern struct setpointStatus setpoint;
extern struct ALARMS Alarms;

void Display_menu_1(void);
void Display_menu_2(void);
void get_value(void);
void Input_Validation(void);
void clear_value(void);
void KeyPressed(); //ISR function

bool key_int_flag = false;
unsigned int key_value = 0;
unsigned int display_screen_state = DISPLAY_WELCOME;
unsigned int display_screen_Next_state = 0;
unsigned int new_value = 0;

/*
 * Default Parameters
 */

uint8_t IE_R_Value[3][2] = {{1, 1}, {1, 2}, {1, 3}};
uint8_t FiO2_Value[4][2] = {{30, 40}, {50, 60}, {70, 80}, {90, 100}};

char VentMode[5][7] = {"VCV   ", "PCV   ", "AC-VCV", "AC-PCV", "CPAP  "};

// For Initial Values See setup() function in main.cpp
uint8_t Set_Param_FiO2 = defaultFIO2Index; // FiO2
uint16_t Set_Param_TV = 200;               // Tidal Volume
uint8_t Set_Param_RR = 20;                 // Respiratory Rate
uint8_t Set_Param_PC = 20;                 // Pressure Control
uint8_t Set_Param_IE_R = 1;                // I:E Ratio
float Set_Param_TRIG = 0.5;                // Triggering
uint8_t Set_Param_vMode = VENT_MODE_VCV;

uint8_t Set_Param_New_vMode = 0;
float Set_Param_New_TRIG = 0;
uint8_t Set_Param_New_IE_R = 1;
uint8_t Set_Param_New_FiO2 = defaultFIO2Index; // FiO2

void LCD_setup()
{
  attachInterrupt(digitalPinToInterrupt(pin_KEYPAD_INTERRUPT), KeyPressed, FALLING);

  lcd.init();
  lcd.backlight();
  lcd.clear();
#if defined(__AVR__)
  DDRA = 0;
#endif
  lcd.blink();
  key_int_flag = true;
}

void InitializeParams()
{
  Set_Param_vMode = setpoint.reqVentMode;
  Set_Param_FiO2 = setpoint.reqFiO2;               // FiO2
  Set_Param_TV = setpoint.reqVolume;               // Tidal Volume
  Set_Param_RR = setpoint.reqBPM;                  // Respiratory Rate
  Set_Param_PC = setpoint.reqPressure;             // Pressure Control
  Set_Param_TRIG = setpoint.flowTriggerSenstivity; // Triggering
  Set_Param_IE_R = setpoint.reqI_E_Section;        // I:E Ratio
}

void Display_menu_1(void)
{
  char displayStr[21] = {};
  lcd.clear();

  // Display FiO2
  lcd.setCursor(0, 0);
  sprintf(displayStr, "FiO2 = %u-%u %%", FiO2_Value[Set_Param_FiO2][0], FiO2_Value[Set_Param_FiO2][1]);
  lcd.print(displayStr);

  // Display Tidal Volume
  lcd.setCursor(0, 1);
  displayStr[20] = {};
  sprintf(displayStr, "T.V = %u ml", Set_Param_TV);
  lcd.print(displayStr);

  // Display Respiratory Rate
  lcd.setCursor(0, 2);
  displayStr[20] = {};
  sprintf(displayStr, "RR= %u BPM", Set_Param_RR);
  lcd.print(displayStr);

  // Display Pressure Control
  lcd.setCursor(0, 3);
  displayStr[20] = {};
  sprintf(displayStr, "PC= %u cm H2O", Set_Param_PC);
  lcd.print(displayStr);
}

void Display_menu_2(void)
{
  char displayStr[21] = {};
  lcd.clear();

  // Display I:E Ratio
  lcd.setCursor(0, 0);
  sprintf(displayStr, "I.E = %u:%u", IE_R_Value[Set_Param_IE_R][0], IE_R_Value[Set_Param_IE_R][1]);
  lcd.print(displayStr);

  // Display Triggering
  lcd.setCursor(0, 1);
  lcd.print(F("Trig= "));
  lcd.print(Set_Param_TRIG);
  lcd.print(F(" SLPM"));

  // Display Vent mode
  lcd.setCursor(0, 2);
  lcd.print(F("Vent Mode = "));
  lcd.print(VentMode[Set_Param_vMode]);

  // for future values
  //lcd.setCursor(0, 3);
  //lcd.print(F("enter new parm"));
}

void get_value(void)
{
  // if the display state is edit mode
  if (display_screen_Next_state >= 2 && display_screen_Next_state <= 4) //Input not applicable on FiO2; IE Ratio; Vent Mode and Trigger Settings
  {
    // check the key pressed
    if (key_value >= 0 && key_value <= 9)
    {
      new_value = new_value * 10;
      new_value = new_value + key_value;
      lcd.print(key_value);
    }
  }
}

void Input_Validation(void) ///
{
  bool updateEEPROM = false;
  switch (display_screen_Next_state)
  {
  case DISPLAY_FIO2:
    Set_Param_FiO2 = Set_Param_New_FiO2;
    setpoint.reqFiO2 = Set_Param_FiO2;
    updateEEPROM = true;
    break;
  case DISPLAY_T_V:
  {
    if (new_value >= minVolume && new_value <= maxVolume)
    {
      Set_Param_TV = new_value;
      setpoint.reqVolume = Set_Param_TV;
      updateEEPROM = true;
      new_value = 0;
    }
    else
    {
      new_value = 0;
    }
  }
  break;
  case DISPLAY_RR:
    if (new_value >= minBPM && new_value <= maxBPM)
    {
      Set_Param_RR = new_value;
      setpoint.reqBPM = Set_Param_RR;
      updateEEPROM = true;
      new_value = 0;
    }
    else
    {
      new_value = 0;
    }
    break;
  case DISPLAY_PC:
  {
    uint8_t min = minPressure;
    uint8_t max = maxPressure;
    if (setpoint.reqEnableCPAP_F == 1)
    {
      min = minPressureCPAP;
      max = maxPressureCPAP;
    }
    if (new_value >= min && new_value <= max)
    {
      Set_Param_PC = new_value;
      setpoint.reqPressure = Set_Param_PC;
      updateEEPROM = true;
      new_value = 0;
    }
    else
    {
      new_value = 0;
    }
  }
  break;
  case DISPLAY_TRIG:
    Set_Param_TRIG = Set_Param_New_TRIG;
    setpoint.flowTriggerSenstivity = Set_Param_TRIG;
    updateEEPROM = true;
    break;
  case DISPLAY_I_E:
    Set_Param_IE_R = Set_Param_New_IE_R;
    setpoint.reqI_E_Section = Set_Param_IE_R;
    updateEEPROM = true;
    break;
  case DISPLAY_VMODE:
    Set_Param_vMode = Set_Param_New_vMode;
    setpoint.reqVentMode = Set_Param_vMode;
    V_Mode_Breakdown();
    updateEEPROM = true;
    break;
  default:
    new_value = 0;
    break;
  }
#ifdef E2PROM
  if (updateEEPROM)
    eeput();
#endif
}

// clear input value
void clear_value(void)
{
  if (display_screen_Next_state >= 1 && display_screen_Next_state <= 7)
  {
    lcd.setCursor(0, 3);
    lcd.print(F("New Value =        "));
    lcd.setCursor(12, 3);
    new_value = 0;
  }
}

void Display(void)
{
  bool flag_alarm = Alarms.Flag_alarm;
  if ((display_screen_state >= 9) && (display_screen_state <= 15))
  {
    flag_alarm = false;
  }

  // if(Alarms.Flag_alarm)
  if (flag_alarm)
  {
    Alarms.Flag_alarm = false;
    if (Alarms.previous_alarm != Alarms.set_alarm)
    {
      if (display_screen_state == DISPLAY_SET_FiO2)
        display_screen_Next_state = DISPLAY_SET_FiO2;
      else if (display_screen_state == DISPLAY_SET_I_E)
        display_screen_Next_state = DISPLAY_SET_I_E;
      else if (display_screen_state == DISPLAY_SET_TRIG)
        display_screen_Next_state = DISPLAY_SET_TRIG;
      else if (display_screen_state == DISPLAY_SET_VMODE)
        display_screen_Next_state = DISPLAY_SET_VMODE;

      lcd.clear();
      lcd.setCursor(6, 0);
      lcd.print(F("ALARM !!"));
      lcd.setCursor(0, 1);
      lcd.setCursor(0, 2);
      lcd.setCursor(0, 3);

      // ACTION BASED ON SET ALARM
      switch (Alarms.set_alarm)
      {
      case BATTERY_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 1"));
        break;
      case CKT_INTEGRITY_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 2"));
        break;
      case OXYGEN_ALARM_PRIORITY:
        lcd.setCursor(3, 2);
        lcd.print(F("OXYGEN FAILURE"));
        break;
      case VENT_DIS_ALARM_PRIORITY:
        lcd.setCursor(5, 1);
        lcd.print(F("VENTILATOR"));
        lcd.setCursor(7, 2);
        lcd.print(F("CIRCUIT"));
        lcd.setCursor(4, 3);
        lcd.print(F("DISCONNECTED"));
        break;
      case PRESSURE_DIS_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 3"));
        break;
      case MECHANICAL_INT_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 4"));
        break;
      case HOMING_NOT_DONE_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 5"));
        break;
      case HOURS_96_ALARM_PRIORITY:
        lcd.setCursor(6, 2);
        lcd.print(F("96 HOURS!!!"));
        lcd.setCursor(1, 3);
        lcd.print(F("REPLACE AMBU BAG"));
        break;
      case FLOW_SENSOR_DIS_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 6"));
        break;
      case O2_SENSOR_DIS_ALARM_PRIORITY:
        lcd.setCursor(1, 2);
        lcd.print(F("Error Code: 7"));
        break;

        //

      case RR_RATE_ALARM_PRIORITY:
        lcd.setCursor(2, 2);
        lcd.print(F("RESPIRATORY RATE"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_RR == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }

        break;
      case PEAK_ALARM_PRIORITY:
        lcd.setCursor(3, 2);
        lcd.print(F("PEAK PRESSURE"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_PEAK == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }
        break;

      case FIO2_ALARM_PRIORITY:
        lcd.setCursor(7, 2);
        lcd.print(F("FiO2"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_FIO2 == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }
        break;
      case TIDAL_VOLUME_ALARM_PRIORITY:
        lcd.setCursor(3, 2);
        lcd.print(F("TIDAL VOLUME"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_TIDAL_VOL == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }
        break;

      case MINUTE_VOLUME_ALARM_PRIORITY:
        lcd.setCursor(3, 2);
        lcd.print(F("MINUTE VOLUME"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_MIN_VOLUME == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }
        break;
      case PEEP_ALARM_PRIORITY:
        lcd.setCursor(7, 2);
        lcd.print(F("PEEP"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_PEEP == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }
        break;
      case PLATEAU_PRESSURE_ALARM_PRIORITY:
        lcd.setCursor(2, 2);
        lcd.print(F("PLATEAU PRESSURE"));
        lcd.setCursor(7, 3);
        if (Alarms.Alarm_type_PLATEAU == VALUE_HIGH)
        {
          lcd.print(F("HIGH"));
        }
        else
        {
          lcd.print(F("LOW"));
        }
        break;
      }
      Alarms.previous_alarm = Alarms.set_alarm;
    }
  }
  else if (key_int_flag || Alarms.Flag_alarm)
  {
    char displayStr[21] = {};
    static bool init1 = true;
    static bool init2 = true;
    static unsigned long timeStamp = 0;
    key_int_flag = false;
    switch (display_screen_state)
    {
    case DISPLAY_WELCOME: // welcome screen
      if (init1)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.setCursor(0, 1);
        lcd.print(F("Welcome"));
        lcd.setCursor(0, 2);
        lcd.print(F("OpenVentPK 1.0"));
        lcd.setCursor(0, 3);
        init1 = false;
        display_screen_state = DISPLAY_CAUTION;
      }
      key_int_flag = true;
      break;
    case DISPLAY_CAUTION:
      if (init2)
      {
        timeStamp = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Verify Following!!!!"));
        lcd.setCursor(0, 1);
        lcd.print(F("1.FiO2"));
        lcd.setCursor(0, 2);
        lcd.print(F("2.PEEP"));
        lcd.setCursor(0, 3);
        lcd.print(F("3.Minute Ventilation"));
        init2 = false;
      }
      key_int_flag = true;
      if ((millis() - timeStamp) >= 5000)
        display_screen_state = DISPLAY_FIO2;
      break;
    case DISPLAY_FIO2:
      Display_menu_1();
      display_screen_Next_state = DISPLAY_SET_FiO2;
      lcd.setCursor(19, 0);
      break;
    case DISPLAY_T_V:
      Display_menu_1();
      display_screen_Next_state = DISPLAY_SET_T_V;
      lcd.setCursor(19, 1);
      break;
    case DISPLAY_RR:
      Display_menu_1();
      display_screen_Next_state = DISPLAY_SET_RR;
      lcd.setCursor(19, 2);
      break;
    case DISPLAY_PC:
      Display_menu_1();
      display_screen_Next_state = DISPLAY_SET_PC;
      lcd.setCursor(19, 3);
      break;
    case DISPLAY_I_E:
      Display_menu_2();
      display_screen_Next_state = DISPLAY_SET_I_E;
      lcd.setCursor(19, 0);
      break;
    case DISPLAY_TRIG:
      Display_menu_2();
      display_screen_Next_state = DISPLAY_SET_TRIG;
      lcd.setCursor(19, 1);
      break;
    case DISPLAY_VMODE:
      Display_menu_2();
      display_screen_Next_state = DISPLAY_SET_VMODE;
      lcd.setCursor(19, 2);
      break;
    case 8:
      break;
    case DISPLAY_SET_FiO2:
      if (display_screen_Next_state != DISPLAY_FIO2)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("FiO2"));
        lcd.setCursor(0, 1);
        lcd.print(F("Range 30 to 100 %"));
        lcd.setCursor(0, 2);
        lcd.print(F("Set Value = "));
        lcd.print(FiO2_Value[Set_Param_FiO2][0]);
        lcd.print(F("-"));
        lcd.print(FiO2_Value[Set_Param_FiO2][1]);
        lcd.setCursor(0, 3);
        lcd.print(F("New Value = "));
        lcd.print(FiO2_Value[Set_Param_New_FiO2][0]);
        lcd.print(F("-"));
        lcd.print(FiO2_Value[Set_Param_New_FiO2][1]);
        display_screen_Next_state = DISPLAY_FIO2;
        Set_Param_New_FiO2 = Set_Param_FiO2;
      }
      else
      {
        lcd.setCursor(12, 3);
        // lcd.print(F("New Value = "));
        lcd.print(FiO2_Value[Set_Param_New_FiO2][0]);
        lcd.print(F("-"));
        lcd.print(FiO2_Value[Set_Param_New_FiO2][1]);
        lcd.print(F(" "));
      }
      break;
    case DISPLAY_SET_T_V:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Tidal Volume"));
      lcd.setCursor(0, 1);
      displayStr[20] = {};
      sprintf(displayStr, "Range %u to %u ml", minVolume, maxVolume);
      lcd.print(displayStr);
      lcd.setCursor(0, 2);
      displayStr[20] = {};
      sprintf(displayStr, "Set Value = %u", Set_Param_TV);
      lcd.print(displayStr);
      lcd.setCursor(0, 3);
      lcd.print(F("New Value = "));
      display_screen_Next_state = DISPLAY_T_V;
      break;
    case DISPLAY_SET_I_E:
      if (display_screen_Next_state != DISPLAY_I_E)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("I:E Ratio"));
        lcd.setCursor(0, 1);
        lcd.print(F("Range 1:1 to 1:3"));
        lcd.setCursor(0, 2);
        lcd.print(F("Set Value = "));
        lcd.print(IE_R_Value[Set_Param_IE_R][0]);
        lcd.print(F(":"));
        lcd.print(IE_R_Value[Set_Param_IE_R][1]);
        lcd.setCursor(0, 3);
        lcd.print(F("New Value = "));
        lcd.print(IE_R_Value[Set_Param_New_IE_R][0]);
        lcd.print(F(":"));
        lcd.print(IE_R_Value[Set_Param_New_IE_R][1]);
        display_screen_Next_state = DISPLAY_I_E;
        Set_Param_New_IE_R = Set_Param_IE_R;
      }
      else
      {
        lcd.setCursor(12, 3);
        // lcd.print(F("New Value = "));
        lcd.print(IE_R_Value[Set_Param_New_IE_R][0]);
        lcd.print(F(":"));
        lcd.print(IE_R_Value[Set_Param_New_IE_R][1]);
      }
      break;
    case DISPLAY_SET_TRIG:
      if (display_screen_Next_state != DISPLAY_TRIG)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Triggering"));
        lcd.setCursor(0, 1);
        lcd.print(F("Range -0.5 to 5 SLPM"));
        lcd.setCursor(0, 2);
        lcd.print(F("Set Value = "));
        lcd.print(Set_Param_TRIG);
        lcd.setCursor(0, 3);
        lcd.print(F("New Value = "));
        lcd.print(Set_Param_TRIG);
        lcd.setCursor(0, 3);
        lcd.print(F("New Value = "));
        lcd.print(Set_Param_TRIG);
        lcd.print(F(" "));
        display_screen_Next_state = DISPLAY_TRIG;
        Set_Param_New_TRIG = Set_Param_TRIG;
      }
      else
      {
        lcd.setCursor(12, 3);
        // lcd.print(F("New Value = "));
        lcd.print(Set_Param_New_TRIG);
        lcd.print(F(" "));
      }
      break;
    case DISPLAY_SET_RR:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Respiratory Rate"));
      lcd.setCursor(0, 1);
      lcd.print(F("Range 8 to 35 BPM"));
      lcd.setCursor(0, 2);
      displayStr[20] = {};
      sprintf(displayStr, "Set Value = %u", Set_Param_RR);
      lcd.print(displayStr);
      lcd.setCursor(0, 3);
      lcd.print(F("New Value = "));
      display_screen_Next_state = DISPLAY_RR;
      break;
    case DISPLAY_SET_PC:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Pressure Control"));
      lcd.setCursor(0, 1);
      if (setpoint.reqEnableCPAP_F == 1)
        lcd.print(F("Range 5 to 20 cm H2O"));
      else
        lcd.print(F("Range 0 to 40 cm H2O"));
      lcd.setCursor(0, 2);
      displayStr[20] = {};
      sprintf(displayStr, "Set Value = %u", Set_Param_PC);
      lcd.print(displayStr);
      lcd.setCursor(0, 3);
      lcd.print(F("New Value = "));
      display_screen_Next_state = DISPLAY_PC;
      break;
    case DISPLAY_SET_VMODE:
      if (display_screen_Next_state != DISPLAY_VMODE)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Vent Mode: VCV/PCV"));
        lcd.setCursor(0, 1);
        // lcd.print(F("AC-VCV/PCV | CPAP ");
        lcd.print(F(" CPAP | AC-VCV/PCV  "));
        lcd.setCursor(0, 2);
        lcd.print(F("Set Mode = "));
        lcd.print(VentMode[Set_Param_vMode]);
        lcd.setCursor(0, 3);
        lcd.print(F("New Mode = "));
        lcd.print(VentMode[Set_Param_vMode]);
        Set_Param_New_vMode = Set_Param_vMode;
        display_screen_Next_state = DISPLAY_VMODE;
      }
      else
      {
        lcd.setCursor(11, 3);
        // lcd.print(F("New Mode = "));
        lcd.print(VentMode[Set_Param_New_vMode]);
      }
      break;
    case DISPLAY_INPUT:
      get_value();
      break;
    case DISPLAY_CLEAR:
      clear_value();
      break;
    default:
      break;
    }
    // testing
    // Serial.print(new_value);
    // Serial.print("  ");
    // Serial.print(display_screen_state);
    // Serial.print("  ");
    // Serial.println(key_value);
  }
}

void KeyPressed() //ISR function
{
  if (key_int_flag == false)
  {
// byte val = (PINA & 0xF0) >> 4; // get PORTA value
#if defined(__AVR__)
    byte val = pins_KEYPAD;
#else
    byte val = 0;
#endif

    switch (val)
    {
    case KEY_0:
      key_value = 0;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_1:
      key_value = 1;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_2:
      key_value = 2;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_3:
      key_value = 3;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_4:
      key_value = 4;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_5:
      key_value = 5;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_6:
      key_value = 6;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_7:
      key_value = 7;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_8:
      key_value = 8;
      display_screen_state = DISPLAY_INPUT;
      break;
    case KEY_9:
      key_value = 9;
      display_screen_state = DISPLAY_INPUT;
      break;

    // char '+' (go up in main menu)
    case KEY_A:
      // Menu go up or change screen
      if (display_screen_state >= 2 && display_screen_state <= 7)
      {
        display_screen_state--;
      }
      else if (display_screen_state == 1)
      {
        display_screen_state = 7;
      }
      // Change Triggering
      else if (display_screen_Next_state == DISPLAY_TRIG)
      {
        if (Set_Param_New_TRIG < 5)
        {
          Set_Param_New_TRIG = Set_Param_New_TRIG + 0.5;
          display_screen_state = DISPLAY_SET_TRIG; // remove this after - function is made
        }
      }
      // Change I:E Ratio
      else if (display_screen_Next_state == DISPLAY_I_E)
      {
        if (Set_Param_New_IE_R < 2)
        {
          Set_Param_New_IE_R++;
          display_screen_state = DISPLAY_SET_I_E;
        }
      }
      // Change Vent mode
      else if (display_screen_Next_state == DISPLAY_VMODE)
      {
        if (Set_Param_New_vMode < 4)
        {
          Set_Param_New_vMode++;
          display_screen_state = DISPLAY_SET_VMODE;
        }
      }
      // Change FiO2
      else if (display_screen_Next_state == DISPLAY_FIO2)
      {
        if (Set_Param_New_FiO2 < 3)
        {
          Set_Param_New_FiO2++;
          display_screen_state = DISPLAY_SET_FiO2;
        }
      }
      else
      {
        display_screen_state = DISCARD_INPUT;
      }
      break;

    /*  Main Menu = Go Down, Setting Menu    */
    case KEY_B:
      // Menu go down or change screen
      if (display_screen_state >= 0 && display_screen_state <= 6)
      {
        display_screen_state++;
      }
      else if (display_screen_state == 7)
      {
        display_screen_state = 1;
      }
      // Change Triggering Value
      else if (display_screen_Next_state == DISPLAY_TRIG)
      {
        if (Set_Param_New_TRIG > 0.5)
        {
          Set_Param_New_TRIG = Set_Param_New_TRIG - 0.5;
          display_screen_state = DISPLAY_SET_TRIG;
        }
      }
      // Change I:E Ratio
      else if (display_screen_Next_state == DISPLAY_I_E)
      {
        if (Set_Param_New_IE_R > 0)
        {
          Set_Param_New_IE_R--;
          display_screen_state = DISPLAY_SET_I_E;
        }
      }
      // Change Vent mode
      else if (display_screen_Next_state == DISPLAY_VMODE)
      {
        if (Set_Param_New_vMode > 0)
        {
          Set_Param_New_vMode--;
          display_screen_state = DISPLAY_SET_VMODE;
        }
      }
      // Change FiO2
      else if (display_screen_Next_state == DISPLAY_FIO2)
      {
        if (Set_Param_New_FiO2 > 0)
        {
          Set_Param_New_FiO2--;
          display_screen_state = DISPLAY_SET_FiO2;
        }
      }
      else
      {
        display_screen_state = DISCARD_INPUT;
      }
      break;

    /* char 'x'   clear Input */
    case KEY_C:
      if (display_screen_Next_state >= 2 && display_screen_Next_state <= 4)
      {
        display_screen_state = DISPLAY_CLEAR;
      }
      break;

    /* key '/'  NOT USED     */
    case KEY_D:
      if (display_screen_Next_state >= 1 && display_screen_Next_state <= 7)
      {
        display_screen_state = DISCARD_INPUT;
      }
      break;

    /*  edit settings in main menu, save settings in settings menu  (in simulation key '=') */
    case KEY_HASH:
      Input_Validation();
      display_screen_state = display_screen_Next_state;
      display_screen_Next_state = 0;
      new_value = 0;
      break;

    /*  Discard settings in settings menu  (in simulation key 'on/c') */
    case KEY_STAR:
      if (display_screen_Next_state >= 1 && display_screen_Next_state <= 7)
      {
        display_screen_state = display_screen_Next_state;
        new_value = 0;
      }
      break;
    }
    key_int_flag = true;
  }
}

void readSwitches()
{
  if (digitalRead(pin_Switch_START) == HIGH)
    activateVentilatorOperation = true;
  else
    activateVentilatorOperation = false;
}

void V_Mode_Breakdown()
{
  switch (setpoint.reqVentMode)
  {
  case VENT_MODE_VCV:
    setpoint.reqControlVariable = VOL_CONT_MODE;
    setpoint.reqAssistMode_F = 0;
    setpoint.reqEnableCPAP_F = 0;
    break;
  case VENT_MODE_PCV:
    setpoint.reqControlVariable = PRESS_CONT_MODE;
    setpoint.reqAssistMode_F = 0;
    setpoint.reqEnableCPAP_F = 0;
    break;
  case VENT_MODE_AC_VCV:
    setpoint.reqControlVariable = VOL_CONT_MODE;
    setpoint.reqAssistMode_F = 1;
    setpoint.reqEnableCPAP_F = 0;
    break;
  case VENT_MODE_AC_PCV:
    setpoint.reqControlVariable = PRESS_CONT_MODE;
    setpoint.reqAssistMode_F = 1;
    setpoint.reqEnableCPAP_F = 0;
    break;
  case VENT_MODE_CPAP:
    setpoint.reqControlVariable = PRESS_CONT_MODE;
    setpoint.reqAssistMode_F = 0;
    setpoint.reqEnableCPAP_F = 1;
    setpoint.reqPressure = constrain(setpoint.reqPressure, minPressureCPAP, maxPressureCPAP);
    Set_Param_PC = setpoint.reqPressure;
    break;
  default:
    break;
  }
}