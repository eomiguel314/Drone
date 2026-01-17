//
// File: flightController.h
//
// Code generated for Simulink model 'flightController'.
//
// Model version                  : 1.125
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sat Jan 17 18:32:33 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Custom Processor->Custom
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_flightController_h_
#define RTW_HEADER_flightController_h_
#include "rtwtypes.h"
#include <cmath>
#ifndef flightController_COMMON_INCLUDES_
# define flightController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // flightController_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef DEFINED_TYPEDEF_FOR_CommandBus_
#define DEFINED_TYPEDEF_FOR_CommandBus_

typedef struct {
  boolean_T controlModePosVSOrient;
  real32_T pos_ref[3];
  boolean_T takeoff_flag;
  real32_T orient_ref[3];
  uint32_T live_time_ticks;
} CommandBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_statesEstim_t_
#define DEFINED_TYPEDEF_FOR_statesEstim_t_

typedef struct {
  real32_T X;
  real32_T Y;
  real32_T Z;
  real32_T yaw;
  real32_T pitch;
  real32_T roll;
  real32_T dx;
  real32_T dy;
  real32_T dz;
  real32_T p;
  real32_T q;
  real32_T r;
} statesEstim_t;

#endif

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  real32_T DiscreteTimeIntegrator_DSTATE[2];// '<S2>/Discrete-Time Integrator'
  real32_T Delay_DSTATE[2];            // '<S2>/Delay'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: TorqueTotalThrustToThrustPerMotor_Value
  //  Referenced by: '<S3>/TorqueTotalThrustToThrustPerMotor'

  real32_T TorqueTotalThrustToThrustPerMotor_Value[16];

  // Computed Parameter: MotorDirections_Gain
  //  Referenced by: '<S7>/MotorDirections'

  real32_T MotorDirections_Gain[4];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  CommandBus ReferenceValueServerBus;  // '<Root>/ReferenceValueServerBus'
  statesEstim_t states_estim;          // '<Root>/states_estim'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T motors_refout[4];           // '<Root>/motors_refout'
  real32_T pose_refout[8];             // '<Root>/pose_refout'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model flightController
class flightControllerModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  flightControllerModelClass();

  // Destructor
  ~flightControllerModelClass();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'flightController'
//  '<S1>'   : 'flightController/Flight Controller'
//  '<S2>'   : 'flightController/Flight Controller/Attitude'
//  '<S3>'   : 'flightController/Flight Controller/ControlMixer'
//  '<S4>'   : 'flightController/Flight Controller/XY-to-reference-orientation'
//  '<S5>'   : 'flightController/Flight Controller/Yaw'
//  '<S6>'   : 'flightController/Flight Controller/gravity feedforward//equilibrium thrust'
//  '<S7>'   : 'flightController/Flight Controller/thrustsToMotorCommands'

#endif                                 // RTW_HEADER_flightController_h_

//
// File trailer for generated code.
//
// [EOF]
//
