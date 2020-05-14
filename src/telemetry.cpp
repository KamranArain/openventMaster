#include "header.h"
#include "flowSensor.h"
#include "alarms.h"

#ifdef TX_SERIAL_TELEMETRY

#include "telemetry.h"

extern struct Flow_Sensor FS;
extern struct P_Sensor p_sensor;
extern struct STATUS_FLAGS status;
extern struct setpointStatus setpoint;
extern struct TidalVolume TV;
extern struct MONITORING_PARAMS monitoring;
extern struct ALARMS Alarms;


struct TEL_TYPE TEL;

extern uint8_t FiO2_Value[4][2];
extern uint8_t IE_R_Value[3][2];

void Prepare_Tx_Telemetry()
{
    unsigned char TEL_BUFF[TEL_PACKET_LENGTH+1];
    static unsigned long milli_old = 0;
    static uint8_t init = 1;
    static byte ctr = 0;

    if (init == 1)
    {
        milli_old = millis();
        init = 0;
    }
    
    unsigned char checksum = 0x00;
    unsigned int uInt = 0;
    

    if (TEL.FDCB == 0xCC)
    {

        // TEL_BUFF[0] = "$";
        // TEL_BUFF[1] = "O";
        // TEL_BUFF[2] = "V";
        // TEL_BUFF[3] = "P";

        TEL_BUFF[0] = 36;
        TEL_BUFF[1] = 79;
        TEL_BUFF[2] = 86;
        TEL_BUFF[3] = 80;

        

        TEL_BUFF[4] = (TEL.Time & 0x000000FF);
        TEL_BUFF[5] = ((TEL.Time & 0x0000FF00) >> 8);
        TEL_BUFF[6] = ((TEL.Time & 0x00FF0000) >> 16);
        TEL_BUFF[7] = ((TEL.Time & 0xFF000000) >> 24);

        uInt = (unsigned int)((TEL.mTV + 2000.0) * (65535.0 / 4000.0));
        TEL_BUFF[8] = (uInt & 0x00FF);
        TEL_BUFF[9] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.mPressure + 30.0) * (65535.0 / 90.0));
        TEL_BUFF[10] = (uInt & 0x00FF);
        TEL_BUFF[11] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.mFlowRate + 200.0) * (65535.0 / 400.0));//SLPM - Standard Litre Per Minuted
        TEL_BUFF[12] = (uInt & 0x00FF);
        TEL_BUFF[13] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.mPEEP + 10.0) * (65535.0 / 40.0));
        TEL_BUFF[14] = (uInt & 0x00FF);
        TEL_BUFF[15] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.mPltPress + 30.0) * (65535.0 / 90.0));
        TEL_BUFF[16] = (uInt & 0x00FF);
        TEL_BUFF[17] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.mFiO2) * (65535.0 / 100.0));
        TEL_BUFF[18] = (uInt & 0x00FF);
        TEL_BUFF[19] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)(TEL.spTV);
        TEL_BUFF[20] = (uInt & 0x00FF);
        TEL_BUFF[21] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)(TEL.spInsPressure + 30.0);
        TEL_BUFF[22] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.spBPM);
        TEL_BUFF[23] = (uInt & 0x00FF);

        uInt = 0x00;
        uInt = TEL.spIE_Inhale & 0x0F;
        uInt |= ((TEL.spIE_Exhale & 0x0F) << 4);
        TEL_BUFF[24] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.spFiO2_LowerBound);
        TEL_BUFF[25] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.spExpPressure + 30.0);
        TEL_BUFF[26] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.ErrorStatusByte1);
        TEL_BUFF[27] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.patientWeight);
        TEL_BUFF[28] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.statusByte1);
        TEL_BUFF[29] = (uInt & 0x00FF);

        uInt = (unsigned int)((TEL.mTVinsp + 2000.0) * (65535.0 / 4000.0));
        TEL_BUFF[30] = (uInt & 0x00FF);
        TEL_BUFF[31] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.mTVexp + 2000.0) * (65535.0 / 4000.0));
        TEL_BUFF[32] = (uInt & 0x00FF);
        TEL_BUFF[33] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.minuteVentilation) * (65535.0 / 40.0));
        TEL_BUFF[34] = (uInt & 0x00FF);
        TEL_BUFF[35] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.staticCompliance) * (65535.0 / 400.0));
        TEL_BUFF[36] = (uInt & 0x00FF);
        TEL_BUFF[37] = ((uInt & 0xFF00) >> 8);

        uInt = (unsigned int)((TEL.spTrigger + 20.0) * (255.0 / 25.0));
        TEL_BUFF[38] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.mRR);
        TEL_BUFF[39] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.mPeakPressure + 30.0);
        TEL_BUFF[40] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.ErrorStatusByte2);
        TEL_BUFF[41] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.ErrorStatusByte3);
        TEL_BUFF[42] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.ErrorStatusByte4);
        TEL_BUFF[43] = (uInt & 0x00FF);

        uInt = (unsigned int)(TEL.spFiO2_UpperBound);
        TEL_BUFF[44] = (uInt & 0x00FF);

        TEL_BUFF[45] = 0x00;
        TEL_BUFF[46] = 0x00;

        for (int i = 0; i < (TEL_PACKET_LENGTH-1); i++) //Byte 0 to 46
            checksum ^= TEL_BUFF[i];

        TEL_BUFF[TEL_PACKET_LENGTH-1] = (checksum & 0xFF);
        TEL_BUFF[TEL_PACKET_LENGTH] = 13; //CR
#ifdef TEL_AT_UART0
        if (Serial.availableForWrite() >= (TEL_PACKET_LENGTH+1))
        {
            TEL.FDCB = 0xFF;
            Serial.write(TEL_BUFF, TEL_PACKET_LENGTH+1);
            ctr++;        
        }
#else
        if (Serial1.availableForWrite() >= (TEL_PACKET_LENGTH+1))
        {
            TEL.FDCB = 0xFF;
            Serial1.write(TEL_BUFF, TEL_PACKET_LENGTH+1);
            ctr++;        
        }
#endif
    }
/*  int NData = 12;
  String str_Payload;

//Crea un string con el formato: NData Data1 Data2 Data3 ... DataN
  str_Payload += NData;
  str_Payload += " " + ((String)(TEL.mTV));
  str_Payload += " " + ((String)(TEL.mPressure));
  str_Payload += " " + ((String)(TEL.mPEEP));
  str_Payload += " " + ((String)(TEL.mPltPress));
  str_Payload += " " + ((String)(TEL.mFlowRate));
  str_Payload += " " + ((String)(TEL.mTVinsp));
  str_Payload += " " + ((String)(TEL.mTVexp));
  str_Payload += " " + ((String)(TEL.spTV));
  str_Payload += " " + ((String)(TEL.spInsPressure));
  str_Payload += " " + ((String)(TEL.spBPM));
  str_Payload += " " + ((String)(TEL.spIE_Inhale));
  str_Payload += " " + ((String)(TEL.spIE_Exhale));
  str_Payload += " \n";
  
  byte Payload[str_Payload.length()];
  int LengthFrameAPI = 18 + sizeof(Payload);
  int LengthPayload = sizeof(Payload);
  
  Serial.print(str_Payload);
*/
    if ((millis() - milli_old) >= 1000)
    {
        milli_old = millis();
        TEL.txUpdateRate = ctr;
        //Serial.print(F("Tel Tx Rate: ")); Serial.print(TEL.txUpdateRate); Serial.println(F(" Hz"));
        ctr = 0;
    }
}
#endif

#ifdef TX_SERIAL_TELEMETRY
void GetTelData()
{

  static boolean init = true;
  byte TEL_BYTE = 0x00;
  if (init)
  {
    TEL.Time = 0;
    TEL.txUpdateRate = 0;
    TEL.FDCB = 0xFF;
    init = false;
  }

  TEL.Time += (samplePeriod1);

  if ((TEL.Time % 20) == 0)
  {
    TEL.mTV = TV.measured; //ml
    //TEL.mTV = constrain(TEL.mTV, 0.0, 1000.0);
    TEL.mTVinsp = TV.inspiration; //ml
    TEL.mTVexp = TV.expiration; //ml
    TEL.mPressure = p_sensor.pressure_gauge_CM; //cmH2O
    TEL.mFlowRate = FS.Q_SLM; //SLPM
    TEL.mPEEP = monitoring.PEEPressure;
    TEL.mPltPress = monitoring.plateauPressure;
    TEL.mFiO2 = 21.0;
    TEL.minuteVentilation = TV.minuteVentilation;
    TEL.mPeakPressure = monitoring.peakInspPressure;
    TEL.mRR = monitoring.measuredRR;
    TEL.staticCompliance = TV.staticCompliance;
    TEL.spTrigger = setpoint.flowTriggerSenstivity;

    TEL.spTV = setpoint.reqVolume;
    TEL.spInsPressure = setpoint.reqPressure;
    TEL.spExpPressure = 5; //0;

    TEL.spFiO2_LowerBound = FiO2_Value[setpoint.reqFiO2][0];
    TEL.spFiO2_UpperBound = FiO2_Value[setpoint.reqFiO2][1];
    TEL.spBPM = setpoint.reqBPM;
    TEL.spIE_Inhale = IE_R_Value[setpoint.reqI_E_Section][0];
    TEL.spIE_Exhale = IE_R_Value[setpoint.reqI_E_Section][1];
    TEL.patientWeight = setpoint.patientWeight;

    TEL.ErrorStatusByte1 = Alarms.Error_status_byte_1;
    TEL.ErrorStatusByte2 = Alarms.Error_status_byte_2;
    if (status.systemReset)
      TEL.ErrorStatusByte2 |= 0x40;
    TEL.ErrorStatusByte3 = Alarms.Error_status_byte_3;
    TEL.ErrorStatusByte4 = Alarms.Error_status_byte_4;

    TEL_BYTE = 0x00;
    TEL_BYTE |= status.breathPhase & 0x03;
    TEL_BYTE |= ((setpoint.reqVentMode & 0x07) << 2);
    if (status.VentilatorOperationON == 1)  TEL_BYTE |= 0x20;
    if (status.selfTestProg == ST_IN_PROG)
      TEL_BYTE |= 0x40;
    else if ((status.selfTestProg == ST_COMPLETE) && (status.selfTestStatus == ST_FAIL))
      TEL_BYTE |= 0x80;
    else if ((status.selfTestProg == ST_COMPLETE) && (status.selfTestStatus == ST_PASS))
      TEL_BYTE |= 0xC0;
    else //NOT INIT
      TEL_BYTE &= 0x3F; //Clear D6 and D7

    TEL.statusByte1 = TEL_BYTE;
    TEL.FDCB = 0xCC;
  }
}
#endif