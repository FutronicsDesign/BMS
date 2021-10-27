/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#define nDev_ID 0
unsigned char command[8];
int UART_RX_RDY = 0;
int RTI_TIMEOUT = 0;


/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

// Contactor Operation

void ContactorOFF(void){
    if(ContState == 1){
        gioSetBit(gioPORTA, 3, 1); // Positive path off
        gioSetBit(gioPORTA, 4, 1); // Pre-charged path off
        delayms(500);
        gioSetBit(gioPORTA, 5, 1); // Nagative path off
        ContState = 0;
        printf("CONTACTOR STATUS: OFF\n");
    }
}

void ContactorON(void){
    if(ContState == 0){
        gioSetBit(gioPORTA, 5, 0);          // Nagative path on
        delayms(1000);                      // 1sec delay
        gioSetBit(gioPORTA, 4, 0);          // Pre-charged path on
        delayms(2000);                      // 2sec delay
        gioSetBit(gioPORTA, 3, 0);          // Positive path on
        delayms(5000);                      // 5sec delay
        gioSetBit(gioPORTA, 4, 1);          // Pre-charged path off
        ContState = 1;
        printf("CONTACTOR STATUS: ON\n");
    }
}

// bframe one byte header then cell voltage, aux channel values, die temperature and vpack.

float getCellVoltage(BYTE *bFrame, struct cell c[]){
//    int i, nSent;
    int Verror = 0;
    float packV = 0;
    bool flg = false;
    for(i=1; i<=noSCell; i++){
        c[noSCell-i].voltage = (bFrame[i*2-1]<<8|bFrame[i*2]) * 0.000076295; // recv slave address stores in bframe sep data . internal temprature
        packV+=c[noSCell-i].voltage;

        TX_Voltage_Master[1+i*2] = bFrame[i*2-1];
        TX_Voltage_Master[2+i*2] = bFrame[i*2];

//    for protection of cell against over and under voltage
        if((c[noSCell-i].voltage < Cuv || c[noSCell-i].voltage > Cov) && !flg ){
            flg=true;
            Verror = 1;
            nSent = WriteReg(nDev_ID, 12, 0x40, 1, FRMWRT_SGL_NR);  // send out broadcast pwrdown command
            break;
        }
    }
    if(Verror == 1){
        printf("Error in Cell Voltage\n");
    }
    return packV;
}

float getCellCurrent(struct cell c[]){
//    int i;
    float pack_I, Ih;
//    pack_I = 0.00;
    pack_I = (ADC_Data[1] - ADC_Data[0])*20;                   //Iline = (Vish-Viref)/((X)*0.0005); X = 20/50/100/200
    Ih = (ADC_Data[2] - 2.505)*40;
//    printf("Is = %f & Ih = %f\n",pack_I,Ih);
    for(i = 0; i < noSCell; i++){
        c[i].current = (pack_I/noPCell);
    }
    return (pack_I);
 }

float getCellTemp(BYTE *bFrame, struct cell c[]){
//    int i;
    int Terror = 0;
    float v, j;
    float TempCr = 20;
    float TempTolal = 0;
    bool flg=false;
    for(i = 1; i<=noTemp; i++){
        v  = ((bFrame[2*i+15]<<8|bFrame[2*i+16])*0.000076295);// directly conn to adc
        j = log(Rf*v/(R0*(Vf-v)));

        float dummyvalue = T0*beta/(beta + T0*j);               // SPI frame
        uint16 temprature_16Value = dummyvalue*150;
        TX_Aux_Master[5+i*2] = temprature_16Value>>8;
        TX_Aux_Master[6+i*2] = temprature_16Value;

        c[8-i].temp = T0*beta/(beta + T0*j) - 273.15;
//        c[2].temp = c[5].temp+0.0111;

        TempTolal+=c[8-i].temp;
        if((c[8-i].temp>=55 || c[8-i].temp <= -20) && !flg){
            flg=true;
            Terror = 1;
            break;
        }
    }
    if(Terror == 1){
        printf("Error in Temperature\n");
    }
    if ((TempTolal/noTemp)<TempCr){
        gioSetBit(gioPORTA, 6, 1);
        gioSetBit(gioPORTA, 7, 1);
    }
    else{
        gioSetBit(gioPORTA, 6, 0);
        gioSetBit(gioPORTA, 7, 0);
    }
    return (TempTolal/noTemp);
}

float getInitialSOC(struct cell c[]){
//    int i;
    float Z, zTotal=0;
    for(i = 0; i < noSCell; i++){
        if (c[i].voltage >= Vno){
            Z = OCV10 + OCV11*pow(c[i].voltage,1) + OCV12*pow(c[i].voltage,2) + OCV13*pow(c[i].voltage,3) + OCV14*pow(c[i].voltage,4); //
            if (Z > 1){
              c[i].z = .99;
            }
            else{
              c[i].z = Z;
            }
        }
        else if (c[i].voltage < Vno){
            Z = OCV20 + OCV21*pow(c[i].voltage,1) + OCV22*pow(c[i].voltage,2) + OCV23*pow(c[i].voltage,3) + OCV24*pow(c[i].voltage,4);
            if (Z < 0.01){
              c[i].z = .02;
            }
            else{
              c[i].z = Z;
            }
        }
        else{
            Z = 0;
            c[i].z = Z;
        }
        zTotal+=c[i].z;
        uint16 soc_value = ((c[i].z)*50000);
        TX_Soc_Master[3+2*i] = soc_value>>8;
        TX_Soc_Master[4+2*i] = soc_value;
    }
    return (zTotal/noSCell);
}

void adcRead(){
    uint16 ADCvalue;
    int j,Avg=25;
        float ADCavg[3] = {0, 0, 0};
        adcData_t adc_data[4];                                      // ADC Data Structure
        for(j=1;j<=Avg;j++){                                        // Moving Avarage
        adcStartConversion(adcREG1, adcGROUP1);                     // Start ADC conversion
        while((adcIsConversionComplete(adcREG1, adcGROUP1))==0);    // Wait for ADC conversion
        adcGetData(adcREG1, adcGROUP1,&adc_data[0]);                // Store conversion into ADC pointer
        for(i=0;i<ADC_pin;i++){
            ADC_value[i] = adc_data[i].value;                       // ADC pointer to ADC bit frame
        }
        for(i=0;i<ADC_pin;i++){
    //        ADC_Data[i]=(((ADC_value[i]+0.5)*(3.3-0.1))/4095)+0.1;
            ADC_Data[i]=(((ADC_value[i])*(3.3))/4096);              // Actual ADC conversion
        }
        for(i=0;i<ADC_pin;i++){
            ADCavg[i]=ADCavg[i]+ADC_Data[i];                        // Moving Avarage
        }
        }
        for(i=0;i<ADC_pin;i++){
                ADC_Data[i]=ADCavg[i]/Avg;                          // Avarage Value
        }
    ADCvalue = ADC_Data[0]*1000;                                    // Current referance to SPI frame
    TX_Aux_Master[CURRENT_STARTING_INDEX] = ADCvalue>>8;
    TX_Aux_Master[CURRENT_STARTING_INDEX+1] = ADCvalue;
    ADCvalue = ADC_Data[1]*1000;                                    // Shunt based Current value to SPI frame
    TX_Aux_Master[CURRENT_STARTING_INDEX+2] = ADCvalue>>8;
    TX_Aux_Master[CURRENT_STARTING_INDEX+3] = ADCvalue;
}

float calculateCellSOC(struct cell c[], float deltaT){
//    int i;
    int zerror = 0;
    float zTotal=0;
    bool flg=false;
    for(i = 0; i< noSCell; i++){
        c[i].z -= ((c[i].current*deltaT*eta)/Q);
        zTotal += c[i].z;

        uint16 soc_value = ((c[i].z)*50000);
        TX_Soc_Master[3+2*i] = soc_value>>8;
        TX_Soc_Master[4+2*i] = soc_value;

        if((c[i].z>=1 || c[i].z <= .01) && !flg){
            zerror = 1;
            flg=true;
            break;
        }
    }
    if(zerror == 1){
        printf("Cell SOC Error");
    }
    return (zTotal/noSCell);
}

float calculateCellCapacity(struct cell c[], float deltaT){
    int i;
    float TotalCapacity=0;
    for(i = 0; i< noSCell; i++){
        c[i].Capacity += (c[i].current*deltaT);
        TotalCapacity += c[i].Capacity;
    }
    return (TotalCapacity);
}

float CalculateMinSOC(struct cell c[]){
    int i;
    float zMin;
    zMin = c[0].z;
    for(i = 0; i<noSCell; i++){
        if (c[i].z < zMin){
            zMin = c[i].z;
        }
    }
    return (zMin);
}

void Print(){
   int i;
   printf("Battery Pack Status\t\t\t\t\t\t\t\tSeries cell's Voltages\t\t\t\t\t\t\t\t\t\t\t\t\t\tSeries cell's SOC\t\t\t\t\t\t\t\t\t\t\t\t\t\tTemperature Sensors data\t\t\t\t\t\t\t\t\t\t\tBalancing Cell ID\n");
//   printf("SoC\t\tVoltage\t\tCurrent\t\tTemperature\tCapacity\t");
   printf("SoC\t\tVoltage\t\tCurrent\t\tTemperature\t");

//   printf("Cell_Voltage:\t");
   for(i = 0; i < noSCell; i++){
     printf("%d\t\t",i+1);  //
      }

//   printf("Cell_SOC:\t");
   for(i = 0; i < noSCell; i++){
     printf("%d\t\t",i+1);
      }

//   printf("Temp_Sensors:\t");
   for(i = 0; i < noTemp; i++){
     printf("%d\t\t",i+1);
      }

   for(i = 0; i < noSCell; i++){
     printf("%d\t",i+1);
      }
//   printf("\tBalancing Cell ID");
   printf("\n");
}

void showdata4(struct cell c[],float vPack, float iPack, float zAvg, float TempPack, float PCap){
   int i;
//   printf("%f\t%f\t%f\t%f\t%f\t",zAvg*100,vPack,iPack,TempPack,PCap);
   printf("%f\t%f\t%f\t%f\t",zAvg*100,vPack,iPack,TempPack);
//   printf("\t");
   for(i = 0; i < noSCell; i++){
     printf("%f\t",c[i].voltage);// c is structure voltage data
      }
//   printf("\t\t");
   for(i = 0; i < noSCell; i++){
     printf("%f\t",c[i].z*100);
      }
//   printf("\t\t");
    for(i = 0; i < noTemp; i++){
        printf("%f\t ",c[i].temp);
    }
}

void showdata5(struct cell c[],float vPack, float iPack, float zAvg, float TempPack, float PCap){

    showdata4(c,vPack, iPack, zAvg, TempPack, PCap);
        int i, j;
        j=cellBalID;
//     ("Balancing Cell ID: ");

        for(i=0;i < noSCell;++i)
        {
            if(j&1<<i){
//                printf("\t%d",i+1);
                printf("1\t");
            }
            else{
                printf("0\t");
            }
        }
    printf("\n");
}

void CAN_Pack(float vPack, float iPack, float zAvg, float TempPack){
    int i;
//    printf("Battery Pack Status: CAN Tx\n");
    uint16 vtx=vPack;
//    printf("vPack: %f   %d\n",vPack,vtx);
    uint16 itx=(iPack+20);
//    printf("iPack: %f   %d\n",iPack,itx);
    uint16 ztx=zAvg*100;
//    printf("zAvg: %f    %d\n",zAvg,ztx);
    uint16 ttx=(TempPack+30);
//    printf("tPack: %f   %d\n",TempPack,ttx);
    tx_data[0] = vtx;//vPack;
    tx_data[1] = (vPack*100 - tx_data[0]*100);//iPack;
    tx_data[2] = itx;//zAvg;
    tx_data[3] = ((iPack+20) *100 - tx_data[2]*100);
    tx_data[4] = ztx;//TempPack;
    tx_data[5] = (zAvg *10000 - tx_data[4]*100);
    tx_data[6] = ttx;
    tx_data[7] = ((TempPack+30) *100 - ttx*100);
    canTransmit(canREG1, canMESSAGE_BOX1, tx_data);
//    for(i=0;i<8;i++){
//        printf("%d\t",tx_data[i]);
//    }
}

void CAN_Cell(uint8 *bframe){
    int i;
    for(i=0;i<8;i++){
        tx_data1[i]=bframe[i+1];
    }
    for(i=8;i<16;i++){
        tx_data2[i-8]=bframe[i+1];
    }
    for(i=16;i<24;i++){
        tx_data3[i-16]=bframe[i+1];
    }
    for(i=24;i<32;i++){
        tx_data4[i-24]=bframe[i+1];
    }
    canTransmit(canREG1, canMESSAGE_BOX2, tx_data1);
    canTransmit(canREG1, canMESSAGE_BOX3, tx_data2);
    canTransmit(canREG1, canMESSAGE_BOX4, tx_data3);
    canTransmit(canREG1, canMESSAGE_BOX5, tx_data4);
}

void Floating(BYTE *bFrame, struct cell c[], float iref, float iPack){
//    int i,nSent;
    uint16 j=0;
//    float zAvg, zMin, vPack, TempPack, PCap;
    WakePL455();
    delayms(5); //~5ms

    nSent = WriteReg(nDev_ID, 20, 0, 2, FRMWRT_SGL_NR);  //Balancing
    nSent = WriteReg(nDev_ID, 2, 0x01, 1, FRMWRT_SGL_R); // send sync sample command
    nSent = WaitRespFrame(bFrame, 39, 0);                //1 header+  16(8 cell voltage) + 16(AUX)bytes data + 2 Analog die + 2 Vpack + 2 CRC

    adcRead();
    iPack = getCellCurrent(c);
    vPack = getCellVoltage(bFrame,c);
    TempPack = getCellTemp(bFrame,c);
    zMin = CalculateMinSOC(c);
//    printf("zMin=%f\n",zMin*100);
//    for(i = 0; i < noSCell; i++){
//        if(c[i].z-zMin > 0.01){ //check % or fractional value
//            j = j|1<<i;
//            c[i].current = c[i].current + (c[i].voltage/75);
//        }
//    }
    zAvg = calculateCellSOC(c, deltaT);
    PCap = calculateCellCapacity(c, deltaT);
//  Mask Customer Checksum Fault bit
//    nSent = WriteReg(nDev_ID, 107, 0x8000, 2, FRMWRT_SGL_NR);  // clear all fault summary flags
////  Clear all faults
//    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);   // clear all fault summary flags
//    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR);     // clear fault flags in the system status register
//    nSent = WriteReg(nDev_ID, 20, j, 2, FRMWRT_SGL_NR);        // Balancing
//    cellBalID=j;
    if(cycleCnt==0){
        showdata5(c,vPack, iPack, zAvg, TempPack, PCap);
    }
    else if(cycleCnt==5){
        cycleCnt=-1;
    }
    cycleCnt++;
}

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    systemInit();
    spiDAT1_t dataconfig1_t;

    dataconfig1_t.CS_HOLD = TRUE;
    dataconfig1_t.WDEL    = TRUE;
    dataconfig1_t.DFSEL   = SPI_FMT_0;
    dataconfig1_t.CSNR    = 0xFE;

    _enable_IRQ();                          /* Enable CPU Interrupt through CPSR */ //compiler intrinsic function. You need to be compiling in ARM mode in order to use this intrinsic
    gioInit();                              //Initializes the GPIO driver
    hetInit();                              //Initializes the HET driver
    sciInit();                              //Initializes the SCI (UART) driver
    sciSetBaudrate(scilinREG, BAUDRATE);
    adcInit();                              //Initializes the ADC driver
    canInit();                              //Initializes the CAN driver
    rtiInit();                              //Initializes the RTI driver
    vimInit();
    spiInit();
//    WakePL455();
    CommClear();
    CommReset();
    gioSetDirection(gioPORTA, 0xFF);        // pin Direction (7,6,5,4,3,2,1,0 as output)FF
    for (i=3; i<8; i++){
             gioSetBit(gioPORTA, i, 1);
         }
    gioSetDirection(hetPORT1, 0xFFFFFFFF);  // all HET pin Direction as out
//    gioSetBit(hetPORT1, 2, 1);
//    gioSetBit(hetPORT1, 0, 1);
    ContactorOFF();
//  initialize local variables
//    int cycles = 0;
    printf("BMS INITIALIZ...........\n");
//  Wake devices ID 0
//  The wake tone will awaken any device that is already in shutdown and the pwrdown will shutdown any device that is already awake.
//  The least number of times to sequence wake and pwrdown will be half the number of
//  Boards to cover the worst case combination of boards already awake or shutdown.
//    nSent = WriteReg(nDev_ID, 12, 0x40, 1, FRMWRT_SGL_NR);      // send out broadcast pwrdown command
    delayms(5); //~5ms
    WakePL455();
    delayms(5); //~5ms
//  Mask Customer Checksum Fault bit
    nSent = WriteReg(nDev_ID, 107, 0x8000, 2, FRMWRT_SGL_NR);   // clear all fault summary flags
//  Clear all faults
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);    // clear all fault summary flags
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR);      // clear fault flags in the system status register
//  Set addresses for all boards in daisy-chain (section 1.2.3)
    nSent = WriteReg(nDev_ID, 10, nDev_ID, 1, FRMWRT_SGL_NR);   // send address to each board
//  Enable/Disable communication interfaces on all/single boards in the stack (section 1.2.1)
//    nSent = WriteReg(nDev_ID, 16, 0x10F8, 2, FRMWRT_SGL_NR);    // set communications baud rate and enable all interfaces on all boards in stack
//    nSent = WriteReg(nDev_ID, 16, 0x1020, 2, FRMWRT_SGL_NR);    // Disable High side receiver on differential (1.2.5)
//    nSent = WriteReg(nDev_ID, 16, 0x10C0, 2, FRMWRT_SGL_NR);    // Disable low side receiver on differential (1.2.6)
    nSent = WriteReg(nDev_ID, 16, 0x1080, 2, FRMWRT_SGL_NR);    // only baud rate and uart on differential and all disable (1.2.6)
// Clear all faults (section 1.2.7)
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_ALL_NR);    // clear all fault summary flags
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_ALL_NR);      // clear fault flags in the system status register
    delayms(10);
// Configure AFE best setting
    nSent = WriteReg(0, 61, 0x00, 1, FRMWRT_ALL_NR);            // set 0 initial delay
// Configure cell voltage and internal temp sample period
    nSent = WriteReg(0, 62, 0xB4, 1, FRMWRT_ALL_NR);            // set (CC) 100us cell and 100us temp ADC sample period
// Configure AUX voltage sample period
    nSent = WriteReg(0, 63, 0x44444444, 4, FRMWRT_ALL_NR);      // set 12.6us AUX sample period
// Configure the oversampling rate
    nSent = WriteReg(0, 7, 0x7B, 1, FRMWRT_ALL_NR);             // set 8x oversampling, stay on channel for oversample and 12.6us oversample period for cell and AUX
// Set AFE_PCTL
    nSent = WriteReg(0, 15, 0x80, 1, FRMWRT_ALL_NR);            // set AFE_PCTL bit to on (only enable AFE when sampling)
//// Configure AFE to Recomended setting
  // {
//    nSent = WriteReg(0, 61, 0x00, 1, FRMWRT_ALL_NR);            // set 0 initial delay
//// Configure cell voltage and internal temp sample period
//    nSent = WriteReg(0, 62, 0xB4, 1, FRMWRT_ALL_NR);            // set 4.13us cell and 12.6us temp ADC sample period
//// Configure AUX voltage sample period
//    nSent = WriteReg(0, 63, 0x44444444, 4, FRMWRT_ALL_NR);      // set 12.6us AUX sample period
//// Configure the oversampling rate
//    nSent = WriteReg(0, 7, 0x7B, 1, FRMWRT_ALL_NR);             // set 0 oversampling
//// Set AFE_PCTL
//    nSent = WriteReg(0, 15, 0x80, 1, FRMWRT_ALL_NR);            // set AFE_PCTL bit to on (only enable AFE when sampling)
  // }
// Clear and check faults (section 2.2.4)
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR);      // clear fault flags in the system status register
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);    // clear all fault summary flags
// Select number of cells and Aux channels to sample (section 2.2.5.1)
    nSent = WriteReg(nDev_ID, 13, 0x08, 1, FRMWRT_SGL_NR);      // set number of cells to 8
    nSent = WriteReg(nDev_ID, 3, 0x00FFFF42, 4, FRMWRT_SGL_NR); // select 8 cell, 8 AUX channels, internal analog die temperature and vpack
// Developed by Gunit..........
    nSent = WriteReg(0, 67, 0x4900, 2, FRMWRT_ALL_NR);          // set vm sampling period
    nSent = WriteReg(0, 30, 0x0001, 2, FRMWRT_ALL_NR);          // enable vm
// Set cell over-voltage and cell under-voltage thresholds on a single board (section 2.2.6.1)
    nSent = WriteReg(nDev_ID, 144, 0xD998, 2, FRMWRT_SGL_NR);   // set OV threshold = 4.2500V
    nSent = WriteReg(nDev_ID, 142, 0x8A3C, 2, FRMWRT_SGL_NR);   // set UV threshold = 2.7000V
// Configure GPIO pin direction and set new pin values (section 5.2.1)
    nSent = WriteReg(nDev_ID, 123, 0x00, 1, FRMWRT_SGL_NR);     // turn off all GPIO pull downs
    nSent = WriteReg(nDev_ID, 122, 0x00, 1, FRMWRT_SGL_NR);     // turn off all GPIO pull ups
    nSent = WriteReg(nDev_ID, 120, 0x3F, 1, FRMWRT_SGL_NR);     // set GPIO[5:0] to output direction
    nSent = WriteReg(nDev_ID, 122, 0x3F, 1, FRMWRT_SGL_NR);     // turn on all GPIO pull ups
    nSent = WriteReg(nDev_ID, 121, 0x00, 1, FRMWRT_SGL_NR);     // set GPIO outputs (as 0)
    delayms(2500);
    nSent = WriteReg(nDev_ID, 2, 0x01, 1, FRMWRT_SGL_R);        // send sync sample command
    nSent = WaitRespFrame(bFrame, 39, 0);                       // 1+16(8 cell voltage) + 16(8temp) bytes data + packet header + CRC(2), 0ms timeout

    printf("Battery Initial Status:\n");
    adcRead();
    iPack = getCellCurrent(c);
    vPack = getCellVoltage(bFrame,c);
    TempPack = getCellTemp(bFrame,c);
    zAvg = getInitialSOC(c);
    PCap = calculateCellCapacity(c, deltaT);
    Print();
    showdata4(c,vPack,iPack,zAvg,TempPack, PCap);
    spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Voltage_Master);
    delayms(250);
    spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Soc_Master);
    delayms(250);
    spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Aux_Master);
    delayms(250);
    CAN_Cell(bFrame);
    CAN_Pack(vPack, iPack, zAvg, TempPack);
    delayms(250);

    ContactorON();
    printf("\n");
    delayms(10);
    while(1){
        gioToggleBit(gioPORTA,2);
        Floating(bFrame, c, iref, iPack);
        spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Voltage_Master);
        delayms(250);
        spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Soc_Master);
        delayms(250);
        spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Aux_Master);
        delayms(250);
        CAN_Cell(bFrame);
        CAN_Pack(vPack, iPack, zAvg, TempPack);
        delayms(250);
    }

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */
