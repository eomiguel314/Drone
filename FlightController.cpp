//
// File: flightController.cpp
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
#include "flightController.h"

// Model step function
void flightControllerModelClass::step()
{
  real32_T rtb_SaturationThrust;
  real32_T rtb_TrigonometricFunction_o2;
  int32_T i;
  real32_T rtb_Switch_refAtt_idx_0;
  real32_T rtb_Switch_refAtt_idx_1;
  real32_T rtb_pitchrollerror_idx_1;
  real32_T rtb_DiscreteTimeIntegrator_idx_0;
  real32_T rtb_SaturationThrust_idx_2;
  real32_T rtb_DiscreteTimeIntegrator_idx_1;
  real32_T rtb_SaturationThrust_idx_3;
  real32_T tmp;
  real32_T u0;

  // Switch: '<S6>/TakeoffOrControl_Switch' incorporates:
  //   Constant: '<S6>/w0'
  //   Gain: '<S6>/D_z'
  //   Gain: '<S6>/P_z'
  //   Gain: '<S6>/takeoff_gain'
  //   Inport: '<Root>/ReferenceValueServerBus'
  //   Inport: '<Root>/states_estim'
  //   Sum: '<S6>/Sum15'
  //   Sum: '<S6>/Sum3'

  if (rtU.ReferenceValueServerBus.takeoff_flag) {
    u0 = -0.278113484F;
  } else {
    u0 = (rtU.ReferenceValueServerBus.pos_ref[2] - rtU.states_estim.Z) * 0.8F -
      0.3F * rtU.states_estim.dz;
  }

  // End of Switch: '<S6>/TakeoffOrControl_Switch'

  // Sum: '<S6>/Sum4' incorporates:
  //   Constant: '<S6>/w0'

  rtb_SaturationThrust = -0.61803F + u0;

  // Saturate: '<S6>/SaturationThrust' incorporates:
  //   Constant: '<S6>/w0'
  //   Sum: '<S6>/Sum4'

  if (-0.61803F + u0 > 1.20204329F) {
    rtb_SaturationThrust = 1.20204329F;
  } else {
    if (-0.61803F + u0 < -1.20204329F) {
      rtb_SaturationThrust = -1.20204329F;
    }
  }

  // End of Saturate: '<S6>/SaturationThrust'

  // Switch: '<S1>/Switch_refAtt' incorporates:
  //   Gain: '<S4>/D_xy'
  //   Gain: '<S4>/P_xy'
  //   Inport: '<Root>/ReferenceValueServerBus'
  //   Inport: '<Root>/states_estim'
  //   Sum: '<S4>/Sum18'

  if (rtU.ReferenceValueServerBus.controlModePosVSOrient) {
    // Trigonometry: '<S4>/Trigonometric Function' incorporates:
    //   Inport: '<Root>/states_estim'

    rtb_Switch_refAtt_idx_1 = std::sin(rtU.states_estim.yaw);
    rtb_TrigonometricFunction_o2 = std::cos(rtU.states_estim.yaw);

    // Sum: '<S4>/Sum17' incorporates:
    //   Inport: '<Root>/states_estim'
    //   Product: '<S4>/Product'

    rtb_pitchrollerror_idx_1 = rtU.ReferenceValueServerBus.pos_ref[0] -
      rtU.states_estim.X;
    tmp = rtU.ReferenceValueServerBus.pos_ref[1] - rtU.states_estim.Y;

    // Product: '<S4>/Product' incorporates:
    //   SignalConversion: '<S4>/ConcatBufferAtVector Concatenate1In1'
    //   SignalConversion: '<S4>/ConcatBufferAtVector ConcatenateIn1'

    rtb_DiscreteTimeIntegrator_idx_0 = rtb_TrigonometricFunction_o2 *
      rtb_pitchrollerror_idx_1 + rtb_Switch_refAtt_idx_1 * tmp;

    // Saturate: '<S4>/Saturation'
    if (rtb_DiscreteTimeIntegrator_idx_0 > 3.0F) {
      rtb_DiscreteTimeIntegrator_idx_0 = 3.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator_idx_0 < -3.0F) {
        rtb_DiscreteTimeIntegrator_idx_0 = -3.0F;
      }
    }

    rtb_Switch_refAtt_idx_0 = -0.24F * rtb_DiscreteTimeIntegrator_idx_0 + 0.1F *
      rtU.states_estim.dx;

    // Product: '<S4>/Product' incorporates:
    //   Gain: '<S4>/D_xy'
    //   Gain: '<S4>/Gain'
    //   Gain: '<S4>/P_xy'
    //   Inport: '<Root>/states_estim'
    //   SignalConversion: '<S4>/ConcatBufferAtVector Concatenate1In2'
    //   Sum: '<S4>/Sum18'

    rtb_DiscreteTimeIntegrator_idx_0 = -rtb_Switch_refAtt_idx_1 *
      rtb_pitchrollerror_idx_1 + rtb_TrigonometricFunction_o2 * tmp;

    // Saturate: '<S4>/Saturation'
    if (rtb_DiscreteTimeIntegrator_idx_0 > 3.0F) {
      rtb_DiscreteTimeIntegrator_idx_0 = 3.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator_idx_0 < -3.0F) {
        rtb_DiscreteTimeIntegrator_idx_0 = -3.0F;
      }
    }

    rtb_Switch_refAtt_idx_1 = 0.24F * rtb_DiscreteTimeIntegrator_idx_0 + -0.1F *
      rtU.states_estim.dy;
  } else {
    rtb_Switch_refAtt_idx_0 = rtU.ReferenceValueServerBus.orient_ref[1];
    rtb_Switch_refAtt_idx_1 = rtU.ReferenceValueServerBus.orient_ref[2];
  }

  // End of Switch: '<S1>/Switch_refAtt'

  // Sum: '<S2>/Sum19' incorporates:
  //   Inport: '<Root>/states_estim'

  rtb_TrigonometricFunction_o2 = rtb_Switch_refAtt_idx_0 -
    rtU.states_estim.pitch;
  rtb_pitchrollerror_idx_1 = rtb_Switch_refAtt_idx_1 - rtU.states_estim.roll;

  // SignalConversion: '<S3>/TmpSignal ConversionAtProductInport2' incorporates:
  //   Gain: '<S5>/D_yaw'
  //   Gain: '<S5>/P_yaw'
  //   Inport: '<Root>/ReferenceValueServerBus'
  //   Inport: '<Root>/states_estim'
  //   Sum: '<S5>/Sum1'
  //   Sum: '<S5>/Sum2'

  tmp = (rtU.ReferenceValueServerBus.orient_ref[0] - rtU.states_estim.yaw) *
    0.004F - 0.0012F * rtU.states_estim.r;

  // DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  rtb_DiscreteTimeIntegrator_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // SignalConversion: '<S3>/TmpSignal ConversionAtProductInport2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/D_pr'
  //   Gain: '<S2>/I_pr'
  //   Gain: '<S2>/P_pr'
  //   Inport: '<Root>/states_estim'
  //   Sum: '<S2>/Sum16'

  rtb_SaturationThrust_idx_2 = (0.013F * rtb_TrigonometricFunction_o2 + 0.01F *
    rtDW.DiscreteTimeIntegrator_DSTATE[0]) - 0.002F * rtU.states_estim.q;

  // DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  rtb_DiscreteTimeIntegrator_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE[1];

  // SignalConversion: '<S3>/TmpSignal ConversionAtProductInport2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/D_pr'
  //   Gain: '<S2>/I_pr'
  //   Gain: '<S2>/P_pr'
  //   Inport: '<Root>/states_estim'
  //   Sum: '<S2>/Sum16'

  rtb_SaturationThrust_idx_3 = (0.02F * rtb_pitchrollerror_idx_1 + 0.01F *
    rtDW.DiscreteTimeIntegrator_DSTATE[1]) - 0.003F * rtU.states_estim.p;
  for (i = 0; i < 4; i++) {
    // Product: '<S3>/Product' incorporates:
    //   Constant: '<S3>/TorqueTotalThrustToThrustPerMotor'
    //   Gain: '<S7>/ThrustToMotorCommand'
    //   SignalConversion: '<S3>/TmpSignal ConversionAtProductInport2'

    u0 = rtConstP.TorqueTotalThrustToThrustPerMotor_Value[i + 12] *
      rtb_SaturationThrust_idx_3 +
      (rtConstP.TorqueTotalThrustToThrustPerMotor_Value[i + 8] *
       rtb_SaturationThrust_idx_2 +
       (rtConstP.TorqueTotalThrustToThrustPerMotor_Value[i + 4] * tmp +
        rtConstP.TorqueTotalThrustToThrustPerMotor_Value[i] *
        rtb_SaturationThrust));

    // Saturate: '<S7>/Saturation5' incorporates:
    //   Gain: '<S7>/ThrustToMotorCommand'

    u0 *= -1530.72681F;
    if (u0 > 500.0F) {
      u0 = 500.0F;
    } else {
      if (u0 < 10.0F) {
        u0 = 10.0F;
      }
    }

    // End of Saturate: '<S7>/Saturation5'

    // Outport: '<Root>/motors_refout' incorporates:
    //   Gain: '<S7>/MotorDirections'

    rtY.motors_refout[i] = rtConstP.MotorDirections_Gain[i] * u0;
  }

  // Outport: '<Root>/pose_refout' incorporates:
  //   Inport: '<Root>/ReferenceValueServerBus'

  rtY.pose_refout[0] = rtU.ReferenceValueServerBus.pos_ref[0];
  rtY.pose_refout[3] = rtU.ReferenceValueServerBus.orient_ref[0];
  rtY.pose_refout[1] = rtU.ReferenceValueServerBus.pos_ref[1];
  rtY.pose_refout[4] = rtU.ReferenceValueServerBus.orient_ref[1];
  rtY.pose_refout[2] = rtU.ReferenceValueServerBus.pos_ref[2];
  rtY.pose_refout[5] = rtU.ReferenceValueServerBus.orient_ref[2];
  rtY.pose_refout[6] = rtb_Switch_refAtt_idx_0;

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
  //   Delay: '<S2>/Delay'
  //   Gain: '<S2>/antiWU_Gain'
  //   Sum: '<S2>/Add'

  rtDW.DiscreteTimeIntegrator_DSTATE[0] += (rtb_TrigonometricFunction_o2 -
    0.001F * rtDW.Delay_DSTATE[0]) * 0.005F;
  if (rtDW.DiscreteTimeIntegrator_DSTATE[0] >= 2.0F) {
    rtDW.DiscreteTimeIntegrator_DSTATE[0] = 2.0F;
  } else {
    if (rtDW.DiscreteTimeIntegrator_DSTATE[0] <= -2.0F) {
      rtDW.DiscreteTimeIntegrator_DSTATE[0] = -2.0F;
    }
  }

  // Update for Delay: '<S2>/Delay'
  rtDW.Delay_DSTATE[0] = rtb_DiscreteTimeIntegrator_idx_0;

  // Outport: '<Root>/pose_refout'
  rtY.pose_refout[7] = rtb_Switch_refAtt_idx_1;

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
  //   Delay: '<S2>/Delay'
  //   Gain: '<S2>/antiWU_Gain'
  //   Sum: '<S2>/Add'

  rtDW.DiscreteTimeIntegrator_DSTATE[1] += (rtb_pitchrollerror_idx_1 - 0.001F *
    rtDW.Delay_DSTATE[1]) * 0.005F;
  if (rtDW.DiscreteTimeIntegrator_DSTATE[1] >= 2.0F) {
    rtDW.DiscreteTimeIntegrator_DSTATE[1] = 2.0F;
  } else {
    if (rtDW.DiscreteTimeIntegrator_DSTATE[1] <= -2.0F) {
      rtDW.DiscreteTimeIntegrator_DSTATE[1] = -2.0F;
    }
  }

  // Update for Delay: '<S2>/Delay'
  rtDW.Delay_DSTATE[1] = rtb_DiscreteTimeIntegrator_idx_1;
}

// Model initialize function
void flightControllerModelClass::initialize()
{
  // (no initialization code required)
}

// Constructor
flightControllerModelClass::flightControllerModelClass()
{
}

// Destructor
flightControllerModelClass::~flightControllerModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
