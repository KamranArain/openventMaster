#include "header.h"

#ifdef TX_SERIAL_TELEMETRY

#include "telemetry.h"

struct TEL_TYPE TEL;

void Prepare_Tx_Telemetry()
{
    unsigned char TEL_BUFF[TEL_PACKET_LENGTH+1];
    static unsigned long milli_old = 0;
    static unsigned int init = 1;
    if (init == 1)
    {
        milli_old = millis();
        init = 0;
    }
    
    unsigned char checksum = 0x00;
    unsigned int uInt = 0;
    
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

    uInt = (unsigned int)(TEL.mTV * (65535.0 / 1000.0));
    TEL_BUFF[8] = (uInt & 0x00FF);
    TEL_BUFF[9] = ((uInt & 0xFF00) >> 8);

    uInt = (unsigned int)((TEL.mPressure + 30.0) * (255.0 / 90.0));
    TEL_BUFF[10] = (uInt & 0x00FF);

    uInt = (unsigned int)((TEL.mFlowRate + 200.0) * (65535.0 / 400.0));//SLPM - Standard Litre Per Minuted
    TEL_BUFF[11] = (uInt & 0x00FF);
    TEL_BUFF[12] = ((uInt & 0xFF00) >> 8);

    uInt = (unsigned int)((TEL.mPEEP + 10.0) * (255.0 / 40.0));
    TEL_BUFF[13] = (uInt & 0x00FF);

    uInt = (unsigned int)((TEL.mPltPress + 30.0) * (255.0 / 90.0));
    TEL_BUFF[14] = (uInt & 0x00FF);

    uInt = (unsigned int)((TEL.mFiO2) * (255.0 / 100.0));
    TEL_BUFF[15] = (uInt & 0x00FF);

    uInt = (unsigned int)(TEL.spTV * (65535.0 / 1000.0));
    TEL_BUFF[16] = (uInt & 0x00FF);
    TEL_BUFF[17] = ((uInt & 0xFF00) >> 8);

    uInt = (unsigned int)((TEL.spPressure + 30.0) * (255.0 / 90.0));
    TEL_BUFF[18] = (uInt & 0x00FF);

    uInt = (unsigned int)((TEL.spBPM) * (255.0 / 40.0));
    TEL_BUFF[19] = (uInt & 0x00FF);

//    uInt = (unsigned int)((TEL.spIE) * (255.0 / 4.0));
         uInt = TEL.spIE_Inhale & 0x0F;
    uInt |= ((TEL.spIE_Exhale & 0x0F) << 4);
    TEL_BUFF[20] = (uInt & 0x00FF);

    uInt = (unsigned int)((TEL.spFiO2) * (255.0 / 100.0));
    TEL_BUFF[21] = (uInt & 0x00FF);

    uInt = (unsigned int)((TEL.spPEEP + 10.0) * (255.0 / 40.0));
    TEL_BUFF[22] = (uInt & 0x00FF);

    uInt = (unsigned int)(TEL.statusByteError);
    TEL_BUFF[23] = (uInt & 0x00FF);

    uInt = (unsigned int)(TEL.patientWeight);
    TEL_BUFF[24] = (uInt & 0x00FF);

    uInt = (unsigned int)(TEL.statusByte1);
    TEL_BUFF[25] = (uInt & 0x00FF);

    for (int i = 26; i <= 33; i++) //Byte 0 to 33
        TEL_BUFF[i] = 0x00;

    for (int i = 0; i < (TEL_PACKET_LENGTH-1); i++) //Byte 0 to 33
        checksum ^= TEL_BUFF[i];

    TEL_BUFF[TEL_PACKET_LENGTH-1] = (checksum & 0xFF);
    TEL_BUFF[TEL_PACKET_LENGTH] = 13;

    if (Serial.availableForWrite() >= TEL_PACKET_LENGTH)
    {
        Serial.write(TEL_BUFF, TEL_PACKET_LENGTH);
        TEL.txPktCtr++;        
    }

    if ((millis() - milli_old) >= 1000)
    {
        milli_old = millis();
        TEL.txUpdateRate = TEL.txPktCtr;
    //    Serial.print("Telemetry Update Rate: "); Serial.print(TEL.txUpdateRate); Serial.println(" Hz");
        TEL.txPktCtr = 0;
    }
}
#endif