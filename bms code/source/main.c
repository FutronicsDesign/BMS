/*
/*
 * main.c
 *
 *  Created on:
 *      Author: reliance
 */


#include <string.h>
#include <main.h>

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


