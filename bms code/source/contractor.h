/*
 * contractor.h
 *
 *  Created on: 26-Oct-2021
 *      Author: reliance
 */
#include <datatypes.h>
#ifndef BMS_CODE_SOURCE_CONTRACTOR_H_
#define BMS_CODE_SOURCE_CONTRACTOR_H_

struct cell{
    float voltage;
    float current;
    float z;
    float temp;
    float Capacity;
};

  struct cell c[8];

 unsigned char command[8];

 BYTE  bFrame[132];


#endif /* BMS_CODE_SOURCE_CONTRACTOR_H_ */
