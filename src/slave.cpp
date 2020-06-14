#include "header.h"

extern struct STATUS_FLAGS status;
extern struct Slave slave;
extern struct MONITORING_PARAMS monitoring;

void receiveSlaveTel() {
    while (Serial2.available()) {
      // get the new byte:
      char inChar = (char)Serial2.read();
      // add it to the inputString:
      slave.AckStr += inChar;
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      if (inChar == '\r') {
        #ifndef TEL_AT_UART0
      // Serial.print(F("Rcvd Ack Str: "));
      // Serial.print(F("B: "));
      Serial3.print(slave.AckStr);
      #endif
        slave.strComplete = true;
        break;
      }
    }
    if (slave.strComplete == true)
    {
      decodeSlaveTel();
      slave.AckStr = "";
      slave.strComplete = false;
    }
}


void decodeSlaveTel()
{
  String message = "";

    for (uint16_t i = 0; i < slave.AckStr.length(); i++)
    {
      if (slave.AckStr[i] == '#')
      {
        if ((slave.AckStr[i+1] == 'R') && (slave.AckStr[i+2] == 'U') && (slave.AckStr[i+3] == 'N'))
        {
          message = slave.AckStr[i+5];
          slave.runAck = message.toInt();
    //      Serial.print("runAck = ");Serial.println(slave.runAck);
          break;
        }
        else if ((slave.AckStr[i+1] == 'S') && (slave.AckStr[i+2] == 'T') && (slave.AckStr[i+3] == 'O') && (slave.AckStr[i+4] == 'P'))
        {
          message = slave.AckStr[i+6];
          slave.stopAck = message.toInt();
      //    Serial.print("stopAck = ");Serial.println(slave.stopAck);
          break;
        }
        else if ((slave.AckStr[i+1] == 'H') && (slave.AckStr[i+2] == 'O') && (slave.AckStr[i+3] == 'M') && (slave.AckStr[i+4] == 'E'))
        {
          message = slave.AckStr[i+6];
          slave.homeAck = message.toInt();  
         Serial.print(F("Decoded homeAck = "));Serial.println(slave.homeAck);
          break;        
        }
        else if ((slave.AckStr[i+1] == 'R') && (slave.AckStr[i+2] == 'R') )
        {
          char inChar;
          uint8_t j = i+4;
          inChar = slave.AckStr[j];
          while (inChar != '\r') {
            message += inChar;
            j++;
            inChar = slave.AckStr[j];
          }
          monitoring.measuredRR = message.toInt();
          status.RRValid = true;
          // Serial.print(F("Measured RR = ")); Serial.print(monitoring.measuredRR); Serial.println(F(" BPM"));
          break;
        }
      }
    }
}


void txSlaveCMD(int CMD_ID, unsigned int period=0, unsigned int pulses=0, String dir="0")
{
  String cmdString = "";
  switch (CMD_ID)
  {
  case RUN:
    cmdString = "#RUN " + String(pulses) + " " + String(period) + " " + dir + "\r";
    break;
  case HOME:
    cmdString = "#HOME " + String(period) + "\r";
    break;
  case STOP:
    cmdString = "#STOP\r";
    break;
  default:
    break;
  }
  Serial2.print(cmdString);
#ifndef TEL_AT_UART0
  Serial3.print(F("M: "));Serial3.println(cmdString);
  #endif
}