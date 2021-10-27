/*
 * soc.c
 *
 *  Created on: 18-Oct-2021
 *      Author: reliance
 */


#include <main.h>
#include "contractor.h"
#include <math.h>

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

