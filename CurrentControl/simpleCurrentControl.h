#ifndef __SIMPLE_CURRENT_CONTROL_H
#define __SIMPLE_CURRENT_CONTROL_H
/*==================================================================================================
COPYRIGHT 2026 MATIMAR LTD. ALL RIGHTS RESERVED

This software is protected by copyright and intellectual property laws in Bulgaria and other
countries. The software may be licensed and is not sold to you. You may use the software only
in accordance with the terms of the license agreement you entered into with Matimar Ltd.

==================================================================================================*/

/*==================================================================================================
  FILE DESCRIPTION

  Definition of the SimpleCurrentControl class, which implements a simple current control algorithm 
  for motor control applications.
  
  The class inherits from the CurrentControl base class and provides implementations for the current 
  control loop, Clarke and Park transformations, proportional-integral control, and PWM output updates.

===================================================================================================*/

/*==================================================================================================
  DEPENDENCIES
===================================================================================================*/
#include "currentControl.h"
#include "commonTypes.h"

/*==================================================================================================
 TYPE DEFINITIONS
===================================================================================================*/
typedef struct
{
    uint32_t proportionalGain;
    uint32_t integralGain;
    uint32_t pwmFrequency;
    uint32_t clockFrequency;
    uint32_t pwmPrescaler;
} currentControlParameters_t;

/*==================================================================================================
  GLOBAL CONSTANT DECLARATIONS
===================================================================================================*/
/* Current control configuration parameters structure */
static const currentControlParameters_t g_configuration = 
{   
    .proportionalGain = 2000UL,
    .integralGain = 200UL,
    .pwmFrequency = 20161UL,
    .clockFrequency = 160000000UL, /* CPU running at 160MHz Resolution: 1Hz. */
    .pwmPrescaler = 1 /* no prescaling */
};

class SimpleCurrentControl: public CurrentControl {

private:
    
/*==================================================================================================
  FUNCTION DECLARATIONS
===================================================================================================*/

/*
 * clarkeTransform:
 *  Calculates the i_alpha and i_beta currents which are the two orthogonal components of the motor current in the 
 *  stationary reference frame. They are calculated from the phase currents i_u and i_v using the following equations:
 *  iα = iU
 *  iβ = ( iU + 2iV ) / √3
 * Parameters:
 *  ParkAndClarke_t *parkAndClarkeStruct - Pointer to structure containing U and V phase current values. 
 *  The iAlpha and iBeta values in this structure will be updated by this function.
 * Return value:
 *  None.
 * Side effects:
 *  None
 */
void clarkeTransform (ParkAndClarke_t *input);

/*
 * calculateSine:
 *  Calculates the sine value for a given angle. 
 * Parameters:
 *  uint16_t angle - The angle for which to calculate the sine value. Represented as an unsigned 16-bit value, 
 *   where 0 corresponds to 0 degrees and UINT16_MAX corresponds to approximately 360 degrees. 
 * Return value:
 *  int32_t - The sine value corresponding to the input angle.
 * Side effects:
 *  None.
 */
int32_t calculateSine (uint16_t angle);

/*
 * calculateCosine:
 *  Calculates the cosine value for a given angle. 
 * Parameters:
 *  uint16_t angle - The angle for which to calculate the cosine value. Represented as an unsigned 16-bit value, 
 *   where 0 corresponds to 0 degrees and UINT16_MAX corresponds to approximately 360 degrees. 
 * Return value:
 *  int32_t - The cosine value corresponding to the input angle.
 * Side effects:
 *  None.
 */
int32_t calculateCosine (uint16_t angle);

/*
 * parkTransform:
 *  Calculates the i_d and i_q currents which are the two orthogonal components of the motor current in the rotating 
 *  reference frame.
 *  i_d = i_alpha * cos(elec ang) + i_beta * sin (elec ang)
 *  i_q = - i_alpha * sin(elec ang) + i_beta * cos (elec ang)
 * Parameters:
 *  uint16_t electricalAngle - The electrical angle of the motor, represented as an unsigned 16-bit value, where 0 corresponds
 *   to 0 degrees and UINT16_MAX corresponds to approximately 360 degrees. 
 *  ParkAndClarke_t *parkAndClarkeStruct - Pointer to structure containing iAlpha and iBeta current values. The iD and iQ 
 *   values in this structure are updated by this function.
 * Return value:
 *  None.
 * Side effects:
 *  None.
 */
void parkTransform (ParkAndClarke_t *parkAndClarkeStruct, uint16_t electricalAngle);

/**
 * inverseParkTransform:
 * i_alpha = i_d * cos(elec ang) - i_q * sin (elec ang)
 * i_beta = i_d * sin(elec ang) + i_q * cos (elec ang)
 * \param[in] uint16_t pos -
 * \param[in,ou ( ParkAndClarke_t *parkAndClarkeStruct -
 */
void inverseParkTransform (ParkAndClarke_t *input, uint16_t electricalAngle);

/*
 * updatePWMOutputs:
 *  Calculates the duty cycle for each phase based on the voltAlpha and voltBeta values in the parkAndClarkeStruct structure,
 *  and updates the PWM outputs accordingly. 
 * Parameters:
 *  ParkAndClarke_t *parkAndClarkeStruct - Pointer to structure containing voltAlpha and voltBeta values. The voltAlpha 
 *  and voltBeta values in this structure are used to calculate the duty cycles for each phase.
 * Return value:
 *  None.
 * Side effects:
 *  Changes the PWM outputs by calling setDutyUVW with the calculated duty cycles for each phase.
 */
void updatePWMOutputs (ParkAndClarke_t *parkAndClarkeStruct);

/*
 * proportionalIntegralControl:
 *  Calculates the voltage output for given current error using a proportional-integral controller.
 *  Limits the output voltage to the range [-VOLT_LIMIT, VOLT_LIMIT]. If the output voltage exceeds this range, 
 *  does not update the integrator to prevent windup.
 * Parameters:
 *  demandedCurrent - The demanded current.
 *  actualCurrent - The actual current.
 *  *outputVoltage - Pointer to the calculated voltage output. 
 *  *stopIntegrator - Pointer to a boolean indicating whether to update the integrator term.
 *  *integrator - Pointer to the integrator term.
 * Side effects:
 *  None
 * Return value:
 *  None
 */
void proportionalIntegralControl (
    int32_t demandedCurrent,
    int32_t actualCurrent, 
    int32_t *outputVoltage,
    bool_t* stopIntegrator,
    int32_t* integrator);

/*
* readCurrentFeedback:
*  Reads the current feedback from the ADC for the U and V phases.
* parameters:
*  *iU - Pointer to the ADC value for the U phase. 
*  *iV - Pointer to the ADC value for the V phase
* side effects:
*  None
* return value:
*  None
*/
void readCurrentFeedback ( int32_t *iU, int32_t *iV );

public:
SimpleCurrentControl(HardwareInterface* hardwareInterface) : CurrentControl(hardwareInterface) {};
~SimpleCurrentControl() override = default;
void currentControlLoop 
              (int32_t demandCurrent, ParkAndClarke_t *parkAndClarkeStruct, currentControlMode_t mode);
};

#endif /* __SIMPLE_CURRENT_CONTROL_H */