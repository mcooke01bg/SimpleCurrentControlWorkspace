#ifndef __CURRENT_CONTROL_H
#define __CURRENT_CONTROL_H
/*==================================================================================================
COPYRIGHT 2026 MATIMAR LTD. ALL RIGHTS RESERVED

This software is protected by copyright and intellectual property laws in Bulgaria and other 
countries. The software may be licensed and is not sold to you. You may use the software only 
in accordance with the terms of the license agreement you entered into with Matimar Ltd.
===================================================================================================*/

/*==================================================================================================
  FILE DESCRIPTION

  Provides the interface for the current control module, which provides vector current control for 
  a brushless three-phase AC motor.

  The current module provides a class CurrentControl with a member function currentControlLoop with 
  an int32_t parameter demandedCurrent and void return value which sets the PWM outputs to cause the 
  motor torque to correspond to this parameter.

===================================================================================================*/

/*==================================================================================================
  DEPENDENCIES
===================================================================================================*/
#include "commonTypes.h"
#include "hardwareInterface.h"

/*==================================================================================================
  MACRO DEFINITIONS
===================================================================================================*/

/*==================================================================================================
   TYPE DEFINITIONS
===================================================================================================*/
/* Structure to hold the current and voltage values in different reference frames */
typedef struct
{
    int32_t iU;  /* Measured current on bridge leg U */
    int32_t iV;  /* Measured current on bridge leg V */
    int32_t iAlpha;  /* Park transformation of iU and iV */
    int32_t iBeta;  /* Park transformation of iU and iV */
    int32_t iQ;  /* Q-axis currrent - Clarke transformation of iAlpha and iBeta */
    int32_t iD;  /* D-axis current - Clarke transformation of iAlpha and iBeta */
    int32_t voltD;  /* D-axis voltage */
    int32_t voltQ;  /* Q-axis voltage */
    int32_t voltAlpha;  /* Alpha-axis voltage inverse Clarke transformation of voltD and voltQ */
    int32_t voltBeta;  /* Beta-axis voltage inverse Clarke transformation of voltD and voltQ*/
    int32_t sector;  /* Sector of the space vector modulation */
    int64_t wX;
    int64_t wY;
    int64_t wZ;
    int64_t wUAlpha;
    int64_t wUBeta;
    int32_t sineValue;  /* Sine of the electrical angle */

} ParkAndClarke_t;

/* Enumeration for current control mode */
typedef enum
{
    CC_MODE_Q_AXIS, /* Normal operation - current in Q-axis (90 degrees ahead of stator)*/
    CC_MODE_D_AXIS, /* Braking operation - current in D-axis (aligned with stator) */
    CC_MODE_RESET   /* Reset operation - initialization */
} currentControlMode_t;

/*==================================================================================================
  CLASS DECLARATION
===================================================================================================*/
class CurrentControl {

/*==================================================================================================
  GLOBAL DECLARATIONS
===================================================================================================*/
protected:
    /* Pointer to the hardware interface instance */
    HardwareInterface* g_hardwareInterface;  

public:

/*==================================================================================================
  FUNCTION DECLARATIONS
===================================================================================================*/
/* Constructor
 *  Initializes the current control module with a hardware interface.
 * Parameters:
 *  hardwareInterface - Pointer to an instance of a class derived from HardwareInterface, which provides 
 *  the necessary hardware-specific operations for the current control module.
 * Side effects:
 *  None
 * Return value:
 *  None
 */
 CurrentControl(HardwareInterface* hardwareInterface) {
    g_hardwareInterface = hardwareInterface;};

/* Destructor
 */
virtual ~CurrentControl() = default;

/*
 * currentControlLoop
 *  This function executes one iteration of the current control loop. It reads the current feedback, 
 *  performs Clarke and Park transformations, calculates the control output using a proportional-integral controller, 
 *  applies circle limitation, performs inverse Park transformation, and updates the PWM outputs based on the calculated voltages.
 * Parameters:
 *  demandedCurrent - the desired peak current in the motor windings, in ADC counts
 *  parkAndClarkeStruct - pointer to a structure containing the current and voltage values in different reference frames, 
 *  which is updated throughout the function
 *  mode - the current control mode, which determines whether the d-axis or q-axis current is being controlled, or the 
 *   control loop is being reset.
 * Side effects:
 *  Updates the PWM outputs to control the motor current.
 *  Updates the global variable g_peakCurrent with the current peak current.
 *  Updates the integrator values and stopIntegrator flags used in the proportional-integral controller.
 * Return value:
 *  None.
 * Global variables used:
 *  None.
 */
virtual void currentControlLoop 
              (int32_t demandCurrent, ParkAndClarke_t *parkAndClarkeStruct, currentControlMode_t mode) = 0;

};
#endif /* __CURRENT_CONTROL_H */
