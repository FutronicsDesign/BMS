/*
 * main.h
 *
 *  Created on: 26-Oct-2021
 *      Author: reliance
 */



#include "sys_common.h"
#include "can.h"
#include "gio.h"
#include "spi.h"
#include "pl455.h"
#include "sci.h"
#include "reg_het.h"
#include "reg_adc.h"
#include "adc.h"
#include "contractor.h"
#include <string.h>

/* USER CODE BEGIN (1) */
#define nDev_ID 0
// int UART_RX_RDY = 0;
// int RTI_TIMEOUT = 0;

//*Number of cells connected in Series & Parallel and Temp. Sensors
#define noSCell 8
#define noPCell 7
#define noTemp 8
//*Under and Over Voltage range for cells & pack and Voltage range for Initial SoC
#define Cuv 2.70
#define Cov 4.25
#define Puv 22.00
#define Pov 33.65
#define Vno 3.64
//*Constants for Initial SOC Vs OCV Equation
//*For OCV >= 3.64
#define OCV10  -277.55819
#define OCV11  278.20471
#define OCV12  -105.59373
#define OCV13  17.98312
#define OCV14  -1.15544
//*For OCV < 3.64
#define OCV20  -79003.66373
#define OCV21  89342.99753
#define OCV22  -37875.24943
#define OCV23  7133.61983
#define OCV24  -503.6491
//*Single Cell Capacity in Amp-Sec
static float Q = 2.6*3600;
//*Purturbation time(Delta_T) Decesion for SOC calculation
static  float deltaT=0.16445;
//* Variable for Contactor Status
 static int ContState = 1;
//*Other Variables
static int T0 = 298;           // used in Temp. calculation
static int beta = 3950; //4356;3950        // used in Temp. calculation
static int R0 = 10000; //25000;10000         // used in Temp. calculation
static int Rf = 10000;         // used in Temp. calculation
static float Vf = 5.30;        // used in Temp. calculation
static  float eta = 0.99;       // used in SOC calculation
static int cycleCnt = 0;       // used in Printing the data
static uint16 cellBalID = 0;   // used in Balancing

// ADC Setting
static int ADC_pin = 3;                // Number of ADC Pin
static adcData_t adc_data[4];    //ADC Data Structure
static uint32 ADC_value[4];
 static float ADC_Data[4];
static float pack_Is, pack_Ih;
/*
// Data Frame setting
// create a garbage value with struct */
static  int nSent, nRead = 0;
static  float zAvg , vPack, iPack, iref, TempPack, PCap, zMin;
 int i,m;

// CAN Fram
 uint8  tx_data[8];
 uint8  tx_data1[8];
 uint8  tx_data2[8];
 uint8  tx_data3[8];
 uint8  tx_data4[8];
 uint8  tx_data5[8];
 uint8  tx_data6[8];

// SPI Frame
//spiDAT1_t dataconfig1_t;
#define SPI_DATA_LENGTH 34
#define CURRENT_STARTING_INDEX 3
static uint16 TX_Voltage_Master[34] = {2,0,1,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0};
static uint16 TX_Soc_Master[34] = {2,0,2,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0};
static uint16 TX_Aux_Master[34] = {2,0,3,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0};

 static  uint16 actual_Current_ADC=0;
 // static float  getCellTemp(bFrame,c);
//static float getCellVoltage(BYTE *, struct cell *);
//static float getCellCurrent(struct cell *);
//static float getCellTemp(BYTE *, struct cell *);







