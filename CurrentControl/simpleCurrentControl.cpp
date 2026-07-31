/*==================================================================================================
COPYRIGHT 2026 MATIMAR LTD. ALL RIGHTS RESERVED

This software is protected by copyright and intellectual property laws in Bulgaria and other
countries. The software may be licensed and is not sold to you. You may use the software only
in accordance with the terms of the license agreement you entered into with Matimar Ltd.

==================================================================================================*/

/*==================================================================================================
  FILE DESCRIPTION

  Contains a simplified implementation of current control, which implements vector 
  current control algorithms for a brushless three-phase AC motor, but without any additional features 
  necessary for a real application, that would increase the complexity of the code.

  The current control module carries out the following sequence of actions:
    Read the currents iu and iv of the phases of the motor.
    Apply the Clarke transform to derive iα and iβ.
    Apply the Park transform to derive id and iq.
    Apply a proportional plus integral control algorithm to determine the required voltages vd and vq.
    Apply the inverse Park transform to derive vα and vβ.
    Set the PWM duty cycle times hta, htb and htc.

  The current control module performs all calculations using an electrical angle θe with a precision 
  of 1/(216) revolutions (i.e. represented as 0 to 65535).

  The current control module uses a look up table with 4096 values representing 90º for calculating
  sines and cosines. 

  The resolution of the output duty cycle times is not lower that that determined by the resolutions 
  of the input currents and sine/cosine look up table.

===================================================================================================*/

/*==================================================================================================
  DEPENDENCIES
===================================================================================================*/
#include "simpleCurrentControl.h"
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
  GLOBAL VARIABLE DECLARATIONS
===================================================================================================*/
/* Current control configuration parameters structure */
static currentControlParameters_t g_configuration = 
{   
    .proportionalGain = 2000UL,
    .integralGain = 200UL,
    .pwmFrequency = 20161UL,
    .clockFrequency = 160000000UL, /* CPU running at 160MHz Resolution: 1Hz. */
    .pwmPrescaler = 1 /* no prescaling */
};

/*==================================================================================================
  FUNCTION DECLARATIONS
===================================================================================================*/
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
static void setDutyUVW ( uint32_t dutyPhaseA, uint32_t dutyPhaseB, uint32_t dutyPhaseC );

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
static void readCurrentsFromADC ( int32_t *currADC1, int32_t *currADC2 );

/**
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
static void clarkeTransform (ParkAndClarke_t *input);

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
static int32_t calculateSine (uint16_t angle);

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
static int32_t calculateCosine (uint16_t angle);

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
static void parkTransform (ParkAndClarke_t *parkAndClarkeStruct, uint16_t electricalAngle);

/**
 * inverseParkTransform:
 * i_alpha = i_d * cos(elec ang) - i_q * sin (elec ang)
 * i_beta = i_d * sin(elec ang) + i_q * cos (elec ang)
 * \param[in] uint16_t pos -
 * \param[in,ou ( ParkAndClarke_t *parkAndClarkeStruct -
 */
static void inverseParkTransform (ParkAndClarke_t *input, uint16_t electricalAngle);

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
static void updatePWMOutputs (ParkAndClarke_t *parkAndClarkeStruct);

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
static uint16_t motorElectricalAngle (void);

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
static void proportionalIntegralControl (
    int32_t demandedCurrent,
    int32_t actualCurrent, 
    int32_t *outputVoltage,
    bool_t* stopIntegrator,
    int32_t* integrator);

/*==================================================================================================
  FUNCTION DEFINITIONS
===================================================================================================*/
  
/**
 * clarkeTransform:
 *  q1.15 fixed-point implementation of the Clarke transformation. 
 *  Calculates the iα and iβ currents which are the two orthogonal components of the motor current in the 
 *  stationary reference frame. They are calculated from the phase currents i_u and i_v using the following equations:
 *  iα = iU
 *  iβ = ( iU + 2iV ) / √3
 */
void clarkeTransform (ParkAndClarke_t *parkAndClarkeStruct)
{
    const int64_t DIV_SQRT_3 = 0x49E6LL; /* 1/sqrt(3) in q1.15 format=0.5773315 (0x49E6 / 0x8000) */
    int32_t iBetaComponent1;
    int32_t iBetaComponent2;

    parkAndClarkeStruct->iAlpha = parkAndClarkeStruct->iU;

    iBetaComponent1 = (((int32_t)DIV_SQRT_3 * parkAndClarkeStruct->iU) / 32768);
    iBetaComponent2 = (((int32_t)DIV_SQRT_3 * parkAndClarkeStruct->iV) / 32768);

    parkAndClarkeStruct->iBeta = iBetaComponent1 + (2 * iBetaComponent2);
}

/*
 * calculateSine:
 *  Calculates the sine value for a given angle using a lookup table. 
 */
static int32_t calculateSine ( uint16_t angle )
{
    /* A position between 0 - 255 will give sin(0 -> pi/2) * 32767 */
    static const int32_t sineLookupTable[1024] =
        {0, 51, 101, 151, 202, 252, 302, 353, 403, 453, 504, 554, 604, 655,
         705, 755, 805, 856, 906, 956, 1007, 1057, 1107, 1157, 1208, 1258,
         1308, 1359, 1409, 1459, 1509, 1560, 1610, 1660, 1710, 1761, 1811,
         1861, 1911, 1962, 2012, 2062, 2112, 2162, 2213, 2263, 2313, 2363,
         2413, 2464, 2514, 2564, 2614, 2664, 2714, 2764, 2815, 2865, 2915,
         2965, 3015, 3065, 3115, 3165, 3215, 3265, 3315, 3366, 3416, 3466,
         3516, 3566, 3616, 3666, 3716, 3766, 3816, 3866, 3916, 3965, 4015,
         4065, 4115, 4165, 4215, 4265, 4315, 4365, 4415, 4464, 4514, 4564,
         4614, 4664, 4714, 4763, 4813, 4863, 4913, 4962, 5012, 5062, 5111,
         5161, 5211, 5261, 5310, 5360, 5409, 5459, 5509, 5558, 5608, 5657,
         5707, 5756, 5806, 5856, 5905, 5954, 6004, 6053, 6103, 6152, 6202,
         6251, 6300, 6350, 6399, 6449, 6498, 6547, 6596, 6646, 6695, 6744,
         6793, 6843, 6892, 6941, 6990, 7039, 7088, 7138, 7187, 7236, 7285,
         7334, 7383, 7432, 7481, 7530, 7579, 7628, 7677, 7726, 7774, 7823,
         7872, 7921, 7970, 8019, 8067, 8116, 8165, 8214, 8262, 8311, 8360,
         8408, 8457, 8506, 8554, 8603, 8651, 8700, 8748, 8797, 8845, 8894,
         8942, 8990, 9039, 9087, 9135, 9184, 9232, 9280, 9329, 9377, 9425,
         9473, 9521, 9569, 9618, 9666, 9714, 9762, 9810, 9858, 9906, 9954,
         10002, 10049, 10097, 10145, 10193, 10241, 10289, 10336, 10384, 10432,
         10479, 10527, 10575, 10622, 10670, 10718, 10765, 10813, 10860, 10908,
         10955, 11002, 11050, 11097, 11144, 11192, 11239, 11286, 11333, 11381,
         11428, 11475, 11522, 11569, 11616, 11663, 11710, 11757, 11804, 11851,
         11898, 11945, 11992, 12039, 12085, 12132, 12179, 12225, 12272, 12319,
         12365, 12412, 12458, 12505, 12552, 12598, 12644, 12691, 12737, 12784,
         12830, 12876, 12922, 12969, 13015, 13061, 13107, 13153, 13199, 13245,
         13291, 13337, 13383, 13429, 13475, 13521, 13567, 13612, 13658, 13704,
         13750, 13795, 13841, 13886, 13932, 13977, 14023, 14068, 14114, 14159,
         14205, 14250, 14295, 14340, 14386, 14431, 14476, 14521, 14566, 14611,
         14656, 14701, 14746, 14791, 14836, 14881, 14926, 14970, 15015, 15060,
         15105, 15149, 15194, 15238, 15283, 15327, 15372, 15416, 15461, 15505,
         15549, 15594, 15638, 15682, 15726, 15770, 15814, 15858, 15902, 15946,
         15990, 16034, 16078, 16122, 16166, 16209, 16253, 16297, 16340, 16384,
         16428, 16471, 16515, 16558, 16601, 16645, 16688, 16731, 16775, 16818,
         16861, 16904, 16947, 16990, 17033, 17076, 17119, 17162, 17205, 17248,
         17290, 17333, 17376, 17418, 17461, 17504, 17546, 17589, 17631, 17673,
         17716, 17758, 17800, 17842, 17885, 17927, 17969, 18011, 18053, 18095,
         18137, 18179, 18221, 18262, 18304, 18346, 18387, 18429, 18471, 18512,
         18554, 18595, 18637, 18678, 18719, 18761, 18802, 18843, 18884, 18925,
         18966, 19007, 19048, 19089, 19130, 19171, 19212, 19252, 19293, 19334,
         19374, 19415, 19455, 19496, 19536, 19577, 19617, 19657, 19697, 19738,
         19778, 19818, 19858, 19898, 19938, 19978, 20018, 20057, 20097, 20137,
         20177, 20216, 20256, 20295, 20335, 20374, 20414, 20453, 20492, 20531,
         20571, 20610, 20649, 20688, 20727, 20766, 20805, 20844, 20882, 20921,
         20960, 20998, 21037, 21076, 21114, 21153, 21191, 21229, 21268, 21306,
         21344, 21382, 21420, 21458, 21496, 21534, 21572, 21610, 21648, 21686,
         21723, 21761, 21798, 21836, 21874, 21911, 21948, 21986, 22023, 22060,
         22097, 22134, 22172, 22209, 22246, 22282, 22319, 22356, 22393, 22430,
         22466, 22503, 22539, 22576, 22612, 22649, 22685, 22721, 22758, 22794,
         22830, 22866, 22902, 22938, 22974, 23010, 23045, 23081, 23117, 23152,
         23188, 23224, 23259, 23294, 23330, 23365, 23400, 23436, 23471, 23506,
         23541, 23576, 23611, 23646, 23680, 23715, 23750, 23784, 23819, 23854,
         23888, 23922, 23957, 23991, 24025, 24059, 24094, 24128, 24162, 24196,
         24230, 24263, 24297, 24331, 24365, 24398, 24432, 24465, 24499, 24532,
         24565, 24599, 24632, 24665, 24698, 24731, 24764, 24797, 24830, 24863,
         24895, 24928, 24961, 24993, 25026, 25058, 25091, 25123, 25155, 25188,
         25220, 25252, 25284, 25316, 25348, 25380, 25411, 25443, 25475, 25506,
         25538, 25569, 25601, 25632, 25664, 25695, 25726, 25757, 25788, 25819,
         25850, 25881, 25912, 25943, 25973, 26004, 26035, 26065, 26096, 26126,
         26156, 26187, 26217, 26247, 26277, 26307, 26337, 26367, 26397, 26427,
         26456, 26486, 26515, 26545, 26574, 26604, 26633, 26662, 26692, 26721,
         26750, 26779, 26808, 26837, 26866, 26894, 26923, 26952, 26980, 27009,
         27037, 27066, 27094, 27122, 27151, 27179, 27207, 27235, 27263, 27291,
         27318, 27346, 27374, 27401, 27429, 27457, 27484, 27511, 27539, 27566,
         27593, 27620, 27647, 27674, 27701, 27728, 27755, 27781, 27808, 27835,
         27861, 27888, 27914, 27940, 27967, 27993, 28019, 28045, 28071, 28097,
         28123, 28148, 28174, 28200, 28225, 28251, 28276, 28302, 28327, 28352,
         28378, 28403, 28428, 28453, 28478, 28502, 28527, 28552, 28577, 28601,
         28626, 28650, 28675, 28699, 28723, 28747, 28771, 28795, 28819, 28843,
         28867, 28891, 28915, 28938, 28962, 28985, 29009, 29032, 29055, 29079,
         29102, 29125, 29148, 29171, 29194, 29217, 29239, 29262, 29285, 29307,
         29330, 29352, 29374, 29397, 29419, 29441, 29463, 29485, 29507, 29529,
         29551, 29572, 29594, 29615, 29637, 29658, 29680, 29701, 29722, 29743,
         29764, 29785, 29806, 29827, 29848, 29869, 29889, 29910, 29931, 29951,
         29971, 29992, 30012, 30032, 30052, 30072, 30092, 30112, 30132, 30152,
         30171, 30191, 30210, 30230, 30249, 30268, 30288, 30307, 30326, 30345,
         30364, 30383, 30402, 30420, 30439, 30458, 30476, 30495, 30513, 30531,
         30549, 30568, 30586, 30604, 30622, 30640, 30657, 30675, 30693, 30710,
         30728, 30745, 30763, 30780, 30797, 30814, 30831, 30848, 30865, 30882,
         30899, 30916, 30932, 30949, 30965, 30982, 30998, 31014, 31031, 31047,
         31063, 31079, 31095, 31110, 31126, 31142, 31158, 31173, 31189, 31204,
         31219, 31235, 31250, 31265, 31280, 31295, 31310, 31324, 31339, 31354,
         31368, 31383, 31397, 31412, 31426, 31440, 31454, 31468, 31482, 31496,
         31510, 31524, 31538, 31551, 31565, 31578, 31592, 31605, 31618, 31631,
         31644, 31657, 31670, 31683, 31696, 31709, 31721, 31734, 31747, 31759,
         31771, 31784, 31796, 31808, 31820, 31832, 31844, 31856, 31867, 31879,
         31891, 31902, 31914, 31925, 31936, 31948, 31959, 31970, 31981, 31992,
         32003, 32013, 32024, 32035, 32045, 32056, 32066, 32076, 32087, 32097,
         32107, 32117, 32127, 32137, 32146, 32156, 32166, 32175, 32185, 32194,
         32204, 32213, 32222, 32231, 32240, 32249, 32258, 32267, 32276, 32284,
         32293, 32301, 32310, 32318, 32326, 32334, 32343, 32351, 32359, 32366,
         32374, 32382, 32390, 32397, 32405, 32412, 32420, 32427, 32434, 32441,
         32448, 32455, 32462, 32469, 32476, 32482, 32489, 32495, 32502, 32508,
         32514, 32521, 32527, 32533, 32539, 32545, 32550, 32556, 32562, 32567,
         32573, 32578, 32584, 32589, 32594, 32599, 32604, 32609, 32614, 32619,
         32624, 32629, 32633, 32638, 32642, 32646, 32651, 32655, 32659, 32663,
         32667, 32671, 32675, 32679, 32682, 32686, 32689, 32693, 32696, 32699,
         32703, 32706, 32709, 32712, 32715, 32717, 32720, 32723, 32725, 32728,
         32730, 32733, 32735, 32737, 32739, 32741, 32743, 32745, 32747, 32749,
         32750, 32752, 32754, 32755, 32756, 32758, 32759, 32760, 32761, 32762,
         32763, 32764, 32764, 32765, 32766, 32766, 32767, 32767, 32767, 32767,
         32767, 32767};
    
    const int32_t DEG_90_INDEX = 1024;
    uint32_t tableIndex;
    uint32_t localTableIndex;
    int32_t returnValue;

    /* Shift angle represented as 16 bits to table represented as 12 bits */
    tableIndex = (uint32_t)angle >> 4U;

    if (tableIndex < DEG_90_INDEX)
    {
        returnValue = sineLookupTable[tableIndex];
    }
    else if (tableIndex < (uint32_t)2 * DEG_90_INDEX)
    {
        localTableIndex= (uint32_t)2 * DEG_90_INDEX - tableIndex - (uint32_t)1;
        returnValue = sineLookupTable[localTableIndex];
    }
    else if (tableIndex < (uint32_t)3 * DEG_90_INDEX)
    {
        localTableIndex= tableIndex - (uint32_t)2 * DEG_90_INDEX;
        returnValue = -sineLookupTable[localTableIndex];
    }
    else if (tableIndex < (uint32_t)4 * DEG_90_INDEX)
    {
        localTableIndex= (uint32_t)4 * DEG_90_INDEX - tableIndex - (uint32_t)1;
        returnValue = -sineLookupTable[localTableIndex];
    }
    else if (tableIndex == (uint32_t)4 * DEG_90_INDEX)
    {
        localTableIndex= (uint32_t)0;
        returnValue = -sineLookupTable[localTableIndex];
    }
    else
    {
        returnValue = (int32_t)0;
    }

    return (returnValue);
}

/*
 * calculateCosine:
 *  Calculates the cosine value for a given angle using a lookup table. 
 */
static int32_t calculateCosine ( uint16_t angle )
{
    /* A position between 0 - 255 will give cos(0 -> pi/2) * 32767 */
    static const int32_t cosineLookupTable[1024] =
        {32767, 32767, 32767, 32767, 32767, 32767, 32766, 32766, 32765, 32764,
         32764, 32763, 32762, 32761, 32760, 32759, 32758, 32756, 32755, 32754,
         32752, 32750, 32749, 32747, 32745, 32743, 32741, 32739, 32737, 32735,
         32733, 32730, 32728, 32725, 32723, 32720, 32717, 32715, 32712, 32709,
         32706, 32703, 32699, 32696, 32693, 32689, 32686, 32682, 32679, 32675,
         32671, 32667, 32663, 32659, 32655, 32651, 32646, 32642, 32638, 32633,
         32629, 32624, 32619, 32614, 32609, 32604, 32599, 32594, 32589, 32584,
         32578, 32573, 32567, 32562, 32556, 32550, 32545, 32539, 32533, 32527,
         32521, 32514, 32508, 32502, 32495, 32489, 32482, 32476, 32469, 32462,
         32455, 32448, 32441, 32434, 32427, 32420, 32412, 32405, 32397, 32390,
         32382, 32374, 32366, 32359, 32351, 32343, 32334, 32326, 32318, 32310,
         32301, 32293, 32284, 32276, 32267, 32258, 32249, 32240, 32231, 32222,
         32213, 32204, 32194, 32185, 32175, 32166, 32156, 32146, 32137, 32127,
         32117, 32107, 32097, 32087, 32076, 32066, 32056, 32045, 32035, 32024,
         32013, 32003, 31992, 31981, 31970, 31959, 31948, 31936, 31925, 31914,
         31902, 31891, 31879, 31867, 31856, 31844, 31832, 31820, 31808, 31796,
         31784, 31771, 31759, 31747, 31734, 31721, 31709, 31696, 31683, 31670,
         31657, 31644, 31631, 31618, 31605, 31592, 31578, 31565, 31551, 31538,
         31524, 31510, 31496, 31482, 31468, 31454, 31440, 31426, 31412, 31397,
         31383, 31368, 31354, 31339, 31324, 31310, 31295, 31280, 31265, 31250,
         31235, 31219, 31204, 31189, 31173, 31158, 31142, 31126, 31110, 31095,
         31079, 31063, 31047, 31031, 31014, 30998, 30982, 30965, 30949, 30932,
         30916, 30899, 30882, 30865, 30848, 30831, 30814, 30797, 30780, 30763,
         30745, 30728, 30710, 30693, 30675, 30657, 30640, 30622, 30604, 30586,
         30568, 30549, 30531, 30513, 30495, 30476, 30458, 30439, 30420, 30402,
         30383, 30364, 30345, 30326, 30307, 30288, 30268, 30249, 30230, 30210,
         30191, 30171, 30152, 30132, 30112, 30092, 30072, 30052, 30032, 30012,
         29992, 29971, 29951, 29931, 29910, 29889, 29869, 29848, 29827, 29806,
         29785, 29764, 29743, 29722, 29701, 29680, 29658, 29637, 29615, 29594,
         29572, 29551, 29529, 29507, 29485, 29463, 29441, 29419, 29397, 29374,
         29352, 29330, 29307, 29285, 29262, 29239, 29217, 29194, 29171, 29148,
         29125, 29102, 29079, 29055, 29032, 29009, 28985, 28962, 28938, 28915,
         28891, 28867, 28843, 28819, 28795, 28771, 28747, 28723, 28699, 28675,
         28650, 28626, 28601, 28577, 28552, 28527, 28502, 28478, 28453, 28428,
         28403, 28378, 28352, 28327, 28302, 28276, 28251, 28225, 28200, 28174,
         28148, 28123, 28097, 28071, 28045, 28019, 27993, 27967, 27940, 27914,
         27888, 27861, 27835, 27808, 27781, 27755, 27728, 27701, 27674, 27647,
         27620, 27593, 27566, 27539, 27511, 27484, 27457, 27429, 27401, 27374,
         27346, 27318, 27291, 27263, 27235, 27207, 27179, 27151, 27122, 27094,
         27066, 27037, 27009, 26980, 26952, 26923, 26894, 26866, 26837, 26808,
         26779, 26750, 26721, 26692, 26662, 26633, 26604, 26574, 26545, 26515,
         26486, 26456, 26427, 26397, 26367, 26337, 26307, 26277, 26247, 26217,
         26187, 26156, 26126, 26096, 26065, 26035, 26004, 25973, 25943, 25912,
         25881, 25850, 25819, 25788, 25757, 25726, 25695, 25664, 25632, 25601,
         25569, 25538, 25506, 25475, 25443, 25411, 25380, 25348, 25316, 25284,
         25252, 25220, 25188, 25155, 25123, 25091, 25058, 25026, 24993, 24961,
         24928, 24895, 24863, 24830, 24797, 24764, 24731, 24698, 24665, 24632,
         24599, 24565, 24532, 24499, 24465, 24432, 24398, 24365, 24331, 24297,
         24263, 24230, 24196, 24162, 24128, 24094, 24059, 24025, 23991, 23957,
         23922, 23888, 23854, 23819, 23784, 23750, 23715, 23680, 23646, 23611,
         23576, 23541, 23506, 23471, 23436, 23400, 23365, 23330, 23294, 23259,
         23224, 23188, 23152, 23117, 23081, 23045, 23010, 22974, 22938, 22902,
         22866, 22830, 22794, 22758, 22721, 22685, 22649, 22612, 22576, 22539,
         22503, 22466, 22430, 22393, 22356, 22319, 22282, 22246, 22209, 22172,
         22134, 22097, 22060, 22023, 21986, 21948, 21911, 21874, 21836, 21798,
         21761, 21723, 21686, 21648, 21610, 21572, 21534, 21496, 21458, 21420,
         21382, 21344, 21306, 21268, 21229, 21191, 21153, 21114, 21076, 21037,
         20998, 20960, 20921, 20882, 20844, 20805, 20766, 20727, 20688, 20649,
         20610, 20571, 20531, 20492, 20453, 20414, 20374, 20335, 20295, 20256,
         20216, 20177, 20137, 20097, 20057, 20018, 19978, 19938, 19898, 19858,
         19818, 19778, 19738, 19697, 19657, 19617, 19577, 19536, 19496, 19455,
         19415, 19374, 19334, 19293, 19252, 19212, 19171, 19130, 19089, 19048,
         19007, 18966, 18925, 18884, 18843, 18802, 18761, 18719, 18678, 18637,
         18595, 18554, 18512, 18471, 18429, 18387, 18346, 18304, 18262, 18221,
         18179, 18137, 18095, 18053, 18011, 17969, 17927, 17885, 17842, 17800,
         17758, 17716, 17673, 17631, 17589, 17546, 17504, 17461, 17418, 17376,
         17333, 17290, 17248, 17205, 17162, 17119, 17076, 17033, 16990, 16947,
         16904, 16861, 16818, 16775, 16731, 16688, 16645, 16601, 16558, 16515,
         16471, 16428, 16384, 16340, 16297, 16253, 16209, 16166, 16122, 16078,
         16034, 15990, 15946, 15902, 15858, 15814, 15770, 15726, 15682, 15638,
         15594, 15549, 15505, 15461, 15416, 15372, 15327, 15283, 15238, 15194,
         15149, 15105, 15060, 15015, 14970, 14926, 14881, 14836, 14791, 14746,
         14701, 14656, 14611, 14566, 14521, 14476, 14431, 14386, 14340, 14295,
         14250, 14205, 14159, 14114, 14068, 14023, 13977, 13932, 13886, 13841,
         13795, 13750, 13704, 13658, 13612, 13567, 13521, 13475, 13429, 13383,
         13337, 13291, 13245, 13199, 13153, 13107, 13061, 13015, 12969, 12922,
         12876, 12830, 12784, 12737, 12691, 12644, 12598, 12552, 12505, 12458,
         12412, 12365, 12319, 12272, 12225, 12179, 12132, 12085, 12039, 11992,
         11945, 11898, 11851, 11804, 11757, 11710, 11663, 11616, 11569, 11522,
         11475, 11428, 11381, 11333, 11286, 11239, 11192, 11144, 11097, 11050,
         11002, 10955, 10908, 10860, 10813, 10765, 10718, 10670, 10622, 10575,
         10527, 10479, 10432, 10384, 10336, 10289, 10241, 10193, 10145, 10097,
         10049, 10002, 9954, 9906, 9858, 9810, 9762, 9714, 9666, 9618, 9569,
         9521, 9473, 9425, 9377, 9329, 9280, 9232, 9184, 9135, 9087, 9039,
         8990, 8942, 8894, 8845, 8797, 8748, 8700, 8651, 8603, 8554, 8506,
         8457, 8408, 8360, 8311, 8262, 8214, 8165, 8116, 8067, 8019, 7970,
         7921, 7872, 7823, 7774, 7726, 7677, 7628, 7579, 7530, 7481, 7432,
         7383, 7334, 7285, 7236, 7187, 7138, 7088, 7039, 6990, 6941, 6892,
         6843, 6793, 6744, 6695, 6646, 6596, 6547, 6498, 6449, 6399, 6350,
         6300, 6251, 6202, 6152, 6103, 6053, 6004, 5954, 5905, 5856, 5806,
         5756, 5707, 5657, 5608, 5558, 5509, 5459, 5409, 5360, 5310, 5261,
         5211, 5161, 5111, 5062, 5012, 4962, 4913, 4863, 4813, 4763, 4714,
         4664, 4614, 4564, 4514, 4464, 4415, 4365, 4315, 4265, 4215, 4165,
         4115, 4065, 4015, 3965, 3916, 3866, 3816, 3766, 3716, 3666, 3616,
         3566, 3516, 3466, 3416, 3366, 3315, 3265, 3215, 3165, 3115, 3065,
         3015, 2965, 2915, 2865, 2815, 2764, 2714, 2664, 2614, 2564, 2514,
         2464, 2413, 2363, 2313, 2263, 2213, 2162, 2112, 2062, 2012, 1962,
         1911, 1861, 1811, 1761, 1710, 1660, 1610, 1560, 1509, 1459, 1409,
         1359, 1308, 1258, 1208, 1157, 1107, 1057, 1007, 956, 906, 856, 805,
         755, 705, 655, 604, 554, 504, 453, 403, 353, 302, 252, 202, 151, 101,
         51, 1};

    const int32_t DEG_90_INDEX = 1024;
    uint32_t tableIndex;
    uint32_t localTableIndex;
    int32_t returnValue;

    /* Shift angle represented as 16 bits to table represented as 12 bits */
    tableIndex = (uint32_t)angle >> 4U;

    if (tableIndex < DEG_90_INDEX)
    {
        returnValue = cosineLookupTable[tableIndex];
    }
    else if (tableIndex < (uint32_t)2 * DEG_90_INDEX)
    {
        localTableIndex= (uint32_t)2 * DEG_90_INDEX - tableIndex - (uint32_t)1;
        returnValue = -cosineLookupTable[localTableIndex];
    }
    else if (tableIndex < (uint32_t)3 * DEG_90_INDEX)
    {
        localTableIndex= tableIndex - (uint32_t)2 * DEG_90_INDEX;
        returnValue = -cosineLookupTable[localTableIndex];
    }
    else if (tableIndex < (uint32_t)4 * DEG_90_INDEX)
    {
        returnValue = cosineLookupTable[localTableIndex];
    }
    else if (tableIndex == (uint32_t)4 * DEG_90_INDEX)
    {
        localTableIndex= (uint32_t)0;
        returnValue = cosineLookupTable[localTableIndex];
    }
    else
    {
        returnValue = (int32_t)0;
    }

    return (returnValue);
}

/*
parkTransform:
 The Park transform uses a 360 degree motor electrical angle, θe, represented as 0 - 65535.
 The Park transform uses two values, iα &  iβ, which are the Clarke transforms of the U and V phase currents.
 The id current is calculated using the following equation.
    id = iα * cos( θe ) + iβ * sin ( θe )"
 The iq current is calculated using the following equation.
    iq =  − iα * sin( θe ) +  iβ * cos ( θe )
*/
void parkTransform (ParkAndClarke_t *parkAndClarkeStruct, uint16_t electricalAngle)
{
    int32_t sineValue;
    int32_t cosineValue;
    int32_t iQComponent1;
    int32_t iQComponent2;
    int32_t iDComponent1;
    int32_t iDComponent2;

    sineValue = calculateSine(electricalAngle);
    cosineValue = calculateCosine(electricalAngle);

    iQComponent1 = (parkAndClarkeStruct->iAlpha * sineValue) / 32768;
    iQComponent2 = (parkAndClarkeStruct->iBeta * cosineValue) / 32768;

    parkAndClarkeStruct->iQ = -iQComponent1 + iQComponent2;

    iDComponent1 = (parkAndClarkeStruct->iAlpha * cosineValue) / 32768;
    iDComponent2 = (parkAndClarkeStruct->iBeta * sineValue) / 32768;

    parkAndClarkeStruct->iD = iDComponent1 + iDComponent2;
}

/*
 * inverseParkTransform:
 */
void inverseParkTransform ( ParkAndClarke_t *parkAndClarkeStruct, uint16_t electricalAngle )
{
    int32_t sineValue;
    int32_t cosineValue;
    int32_t vAlphaComponent1;
    int32_t vAlphaComponent2;
    int32_t vBetaComponent1;
    int32_t vBetaComponent2;

    sineValue = calculateSine(electricalAngle);
    cosineValue = calculateCosine(electricalAngle);

    vAlphaComponent1 = (parkAndClarkeStruct->voltQ * sineValue) / 32768;
    vAlphaComponent2 = (parkAndClarkeStruct->voltD * cosineValue) / 32768;

    parkAndClarkeStruct->voltAlpha = -vAlphaComponent1 + vAlphaComponent2;

    vBetaComponent1 = (parkAndClarkeStruct->voltQ * cosineValue) / 32768;
    vBetaComponent2 = (parkAndClarkeStruct->voltD * sineValue) / 32768;

    parkAndClarkeStruct->voltBeta = vBetaComponent1 + vBetaComponent2;
}

/*
 * updatePWMOutputs:
 *  The function first calculates wX, wY and wZ and determines the sector of the voltage vector. Based on the sector, it then
 *  calculates the high time for each phase using the appropriate formula. Finally, it calls setDutyUVW() to update the PWM
 *  outputs with the calculated duty cycles.
 */
static void updatePWMOutputs ( ParkAndClarke_t *parkAndClarkeStruct )
{
    uint32_t sector = 0; /* sector number for PWM switching */
    int64_t wX;
    int64_t wY;
    int64_t wZ;
    int64_t wUAlpha;
    int64_t wUBeta;

    /* Variables for output PWM duty cycles */
    /* 500 = 50% duty cycle = no current to motor as the PWM is centre-aligned*/
    int32_t highTimePhaseA = 500;  
    int32_t highTimePhaseB = 500;
    int32_t highTimePhaseC = 500;

    uint32_t pwmPeriod = 0;

    /* constants used in the PWM calculation */
    static const int64_t SQRT_3 = 113512LL;        /* 1.732051 = 113512 / 65536 */
    static int64_t T = 0;
    static int64_t T_SQRT3 = 0;

    /* Calculate T and T_SQRT3 only once as they are constants for a given PWM frequency and timer configuration. */
    if (T == 0)
    {
        pwmPeriod = g_configuration.clockFrequency / (2U * g_configuration.pwmFrequency * g_configuration.pwmPrescaler);
        T = (int64_t)((int64_t)pwmPeriod * (int64_t)4);
        T_SQRT3 = (int64_t)((T * SQRT_3) / (int64_t)65536);
    }    
    wUAlpha = (int64_t)(parkAndClarkeStruct->voltAlpha * T_SQRT3);
    wUBeta = (int64_t)(parkAndClarkeStruct->voltBeta * T);

    wX = wUBeta;
    wY = (wUBeta + wUAlpha) / 2;
    wZ = (wUBeta - wUAlpha) / 2;

    /* Calculate the sector based on the wX and wZ values */
    if (wY < (int64_t)0)
    {
        if (wZ < (int64_t)0)
        {
            sector = 5;
        }
        else if (wX <= (int64_t)0)
        {
            sector = 4;
        }
        else
        {
            sector = 3;
        }
    }
    else
    {
        if (wZ >= (int64_t)0)
        {
            sector = 2;
        }
        else if (wX <= (int64_t)0)
        {
            sector = 6;
        }
        else
        {
            sector = 1;
        }
    }

    switch (sector)
    {
        case 1:
        case 4:
        {
            highTimePhaseA =
                (int32_t)((T / 8) + ((((T + wX) - wZ) / 2) / 131072));
            highTimePhaseB = (int32_t)(highTimePhaseA + wZ / 131072);
            highTimePhaseC = (int32_t)(highTimePhaseC - wX / 131072);
            break;
        }
        case 2:
        case 5:
        {
            highTimePhaseA =
                (int32_t)((T / 8) + ((((T + wY) - wZ) / 2) / 131072));
            highTimePhaseB = (int32_t)(highTimePhaseA + wZ / 131072);
            highTimePhaseC = (int32_t)(highTimePhaseA - wY / 131072);
            break;
        }
        case 3:
        case 6:
        {
            highTimePhaseA =
                (int32_t)((T / 8) + ((((T - wX) + wY) / 2) / 131072));
            highTimePhaseB = (int32_t)(highTimePhaseA - wY / 131072);
            highTimePhaseC = (int32_t)(highTimePhaseC + wX / 131072);
            break;
        }
        default: /* This is a fault and should never happen! */
        {
            /* report error */
            break;
        }
    }

    /* Set the duty cycles for the three phases */
    setDutyUVW ( (uint32_t)highTimePhaseA, (uint32_t)highTimePhaseB, (uint32_t)highTimePhaseC );
}

/*
 * proportionalIntegralControl:
 *  Simple implementation of a proportional-integral (PI) controller for current control. 
 */
static void proportionalIntegralControl (
    int32_t demandedCurrent,
    int32_t actualCurrent, 
    int32_t *outputVoltage,
    bool_t* stopIntegrator,
    int32_t* integrator)
{
    static const int32_t VOLT_LIMIT = 32767;
    int32_t currentError = 0; /* error between the demanded current and the actual current. */
    int32_t proportionalOutput = 0; /* output of the proportional term of the controller. */
    int32_t integralOutput = 0; /* output of the integral term of the controller. */
    int32_t temporaryIntegrator = 0;  /* temporary integrator value */
    int32_t voltageOutput = 0; /* used in calculating output voltage */
    bool_t upperLimitHit = boolFALSE; /* flag indicating whether the upper voltage limit has been hit */
    bool_t lowerLimitHit = boolFALSE; /* flag indicating whether the lower voltage limit has been hit */

    /* Calculate the unlimited output voltage*/
    currentError = demandedCurrent - actualCurrent;
    proportionalOutput = (int32_t)g_configuration.proportionalGain * currentError;
    temporaryIntegrator = *integrator + currentError;
    integralOutput = (int32_t)g_configuration.integralGain * temporaryIntegrator;
    voltageOutput = (proportionalOutput + integralOutput);
    voltageOutput = voltageOutput / 8192;  /* Scale voltageOutput to the correct range */

    /* Integrator anti-windup! Do not update integrator if output saturated and error increasing. */
    if ((voltageOutput >= VOLT_LIMIT) && (currentError > 0) && (temporaryIntegrator > *integrator))
    {
        upperLimitHit = boolTRUE;
    }
    else
    {
        upperLimitHit = boolFALSE;
    }
    if ((voltageOutput <= -VOLT_LIMIT) && (currentError < 0) && (temporaryIntegrator < *integrator))
    {
        lowerLimitHit = boolTRUE;
    }
    else
    {
        lowerLimitHit = boolFALSE;
    }

    if ((upperLimitHit == boolTRUE) || (lowerLimitHit == boolTRUE))
    {
        *stopIntegrator = boolTRUE;
    }
    else
    {
        *stopIntegrator = boolFALSE;
    }

    if (*stopIntegrator == boolFALSE)
    {
        *integrator = temporaryIntegrator;
    }
    /* Limit output. */
    if (voltageOutput >= VOLT_LIMIT)
    {
        voltageOutput = VOLT_LIMIT;
    }
    if (voltageOutput <= -VOLT_LIMIT)
    {
        voltageOutput = -VOLT_LIMIT;
    }

    *outputVoltage = voltageOutput;
}

/*
 * readCurrentFeedback:
 *  Reads the current feedback from the ADC for the U and V phases.
 * parameters:
 *  *iU - Pointer to the ADC value for the U phase
 *  *iV - Pointer to the ADC value for the V phase
 * global variables used:
 *  g_currentOffsetU - The offset value for the U phase current
 *  g_currentOffsetV - The offset value for the V phase current
 * side effects:
 *  None
 * return value:
 *  None
 */
static void readCurrentFeedback ( int32_t *iU, int32_t *iV )
{
    int32_t ADCUCurrent; /* ADC value for the U phase current */
    int32_t ADCVCurrent; /* ADC value for the V phase current */

    readCurrentsFromADC ( &ADCUCurrent, &ADCVCurrent );
    /* An ADC reading of 2048 is 0 amps, so remove this offset*/ 
    *iU = ADCUCurrent - 2048;   
    *iV = ADCVCurrent - 2048; 
}

/*
 * currentControlLoop
 */
void currentControlLoop (int32_t demandedCurrent, ParkAndClarke_t *parkAndClarkeStruct, currentControlMode_t mode)
{
    static const int32_t D_AXIS_CURRENT_DEMAND = 0; /* D-axis current set-point = 0 for maximum torque */
    static bool_t stopIntegratorA = boolFALSE; /* flag to stop integratorA from increasing to prevent windup */
    static int32_t integratorA = 0; /* integrator term A for Q-axis */
    static bool_t stopIntegratorB = boolFALSE; /* flag to stop integratorB from increasing to prevent windup */
    static int32_t integratorB = 0; /* integrator term B for D-axis */
    uint16_t electricalAngle; /* electrical angle of the motor */
    
    if (mode == CC_MODE_RESET)
    {  
        /* Read configuration parameters*/
        stopIntegratorA = boolFALSE;
        integratorA = 0;
        stopIntegratorB = boolFALSE;
        integratorB = 0;
        setDutyUVW(500, 500, 500); /* 50% duty cycle = no current to motor as the PWM is centre-aligned */
        demandedCurrent = 0;
    }

    /* read control inputs*/
    electricalAngle = motorElectricalAngle();
    readCurrentFeedback (&(parkAndClarkeStruct->iU), &(parkAndClarkeStruct->iV));
    
    /* Clarke and Park transformations */
    clarkeTransform (parkAndClarkeStruct);
    parkTransform (parkAndClarkeStruct, electricalAngle);

    /* Proportional-integral control */
    if (mode == CC_MODE_D_AXIS)
    {
        proportionalIntegralControl (D_AXIS_CURRENT_DEMAND, -(parkAndClarkeStruct->iQ), &(parkAndClarkeStruct->voltQ), &stopIntegratorA, &integratorA);
        proportionalIntegralControl (demandedCurrent, -(parkAndClarkeStruct->iD), &(parkAndClarkeStruct->voltD), &stopIntegratorB, &integratorB);
    }
    else
    {
        proportionalIntegralControl (demandedCurrent, -(parkAndClarkeStruct->iQ), &(parkAndClarkeStruct->voltQ), &stopIntegratorA, &integratorA);
        proportionalIntegralControl (D_AXIS_CURRENT_DEMAND, -(parkAndClarkeStruct->iD), &(parkAndClarkeStruct->voltD), &stopIntegratorB, &integratorB);
    }

    /* Apply Inverse Park transform and update PWM outputs */
    inverseParkTransform(parkAndClarkeStruct, electricalAngle);
    updatePWMOutputs (parkAndClarkeStruct);
}

/*
 * setDutyUVW:
 *  This is a placeholder. Implementation of this function will depend on the specific hardware, PWM generation method
 *  and driver. 
 */
static void setDutyUVW (uint32_t dutyPhaseA, uint32_t dutyPhaseB, uint32_t dutyPhaseC)
{
    ;
}

/*
 * readCurrentsFromADC:
 *  This is a placeholder. Implementation of this function will depend on the specific hardware and driver.
 *  being used. 
 */
static void readCurrentsFromADC ( int32_t *currADC1, int32_t *currADC2 )
{
    ;
}

/*
 * motorElectricalAngle:
 *  This is a placeholder. Implementation of this function will depend on the specific hardware and driver. 
 */
static uint16_t motorElectricalAngle (void)
{
    uint16_t electricalAngle;
    return electricalAngle;
}
