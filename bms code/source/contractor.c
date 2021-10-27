/*
 * contractor.c
 *
 *  Created on: 18-Oct-2021
 *      Author: reliance
 */
#include <main.h>
#include "contractor.h"
#include "sys_common.h"
#include "can.h"
#include "gio.h"
#include "spi.h"
#include "pl455.h"
#include "sci.h"
#include "reg_het.h"
#include "reg_adc.h"
#include "adc.h"

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


void ContactorON(){
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


