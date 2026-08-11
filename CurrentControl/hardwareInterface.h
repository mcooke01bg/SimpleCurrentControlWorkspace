#ifndef __HARDWARE_INTERFACE_H
#define __HARDWARE_INTERFACE_H
/*==================================================================================================
COPYRIGHT 2026 MATIMAR LTD. ALL RIGHTS RESERVED

This software is protected by copyright and intellectual property laws in Bulgaria and other
countries. The software may be licensed and is not sold to you. You may use the software only
in accordance with the terms of the license agreement you entered into with Matimar Ltd.

==================================================================================================*/

/*==================================================================================================
  FILE DESCRIPTION

  Contains a class definition for the HardwareInterface, which serves as an abstraction layer for 
  hardware-specific operations.

===================================================================================================*/

/*==================================================================================================
  DEPENDENCIES
===================================================================================================*/
#include "commonTypes.h"

/*==================================================================================================
  CLASS DEFINITION
===================================================================================================*/
class HardwareInterface {
    public:
    HardwareInterface() = default;
    virtual ~HardwareInterface() = default;
    /*
    * readCurrentsFromADC:
    *  Reads the current feedback from the ADC and stores the values in the provided pointers.
    *  NOTE: This is currently a placeholder function and should be implemented according to the specific hardware.
    * Parameters:
    *  int32_t *currADC1 - Pointer to store the ADC value for the first current feedback
    *  int32_t *currADC2 - Pointer to store the ADC value for the second current feedback
    * Side effects:
    *  None
    * Return value:
    *  None
    */
   virtual void readCurrentsFromADC ( int32_t *currADC1, int32_t *currADC2 ) = 0;

    /*
    * setDutyUVW:
    *  Sets the duty cycle for the three phases to the values provided in the arguments.
    *  NOTE: This is currently a placeholder function and should be implemented according to the specific hardware.
    * Parameters:
    *  uint32_t dutyPhaseA - Duty cycle for phase A
    *  uint32_t dutyPhaseB - Duty cycle for phase B
    *  uint32_t dutyPhaseC - Duty cycle for phase C
    * Side effects:
    *  Sets the duty cycles for the three phases.
    *  NOTE: The duty cycle setting is not implemented in this code and should be implemented according to the specific 
    *  requirements of the system.
    * Return value:
    *  None
    */
    virtual void setDutyUVW ( uint32_t dutyPhaseA, uint32_t dutyPhaseB, uint32_t dutyPhaseC ) = 0;

    /*
    * motorElectricalAngle:
    *  Returns the motor electrical angle of rotation.
    * Parameters:
    *  None
    * Return value:
    *  uint16_t - electrical angle, represented in an unsigned 16 bit value
    *                         where 0 = 0 degrees, and UINT16_MAX = ~360 degrees
    * Side effects:
    *  None
    */
    virtual uint16_t motorElectricalAngle (void) = 0;
};
#endif /* __HARDWARE_INTERFACE_H */
