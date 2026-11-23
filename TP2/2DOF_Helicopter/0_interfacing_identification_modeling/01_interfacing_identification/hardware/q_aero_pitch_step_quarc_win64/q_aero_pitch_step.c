/*
 * q_aero_pitch_step.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "q_aero_pitch_step".
 *
 * Model version              : 11.2
 * Simulink Coder version : 25.1 (R2025a) 21-Nov-2024
 * C source code generated on : Thu Oct 16 19:52:48 2025
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "q_aero_pitch_step.h"
#include "rtwtypes.h"
#include "q_aero_pitch_step_private.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "q_aero_pitch_step_dt.h"

/* Block signals (default storage) */
B_q_aero_pitch_step_T q_aero_pitch_step_B;

/* Block states (default storage) */
DW_q_aero_pitch_step_T q_aero_pitch_step_DW;

/* Real-time model */
static RT_MODEL_q_aero_pitch_step_T q_aero_pitch_step_M_;
RT_MODEL_q_aero_pitch_step_T *const q_aero_pitch_step_M = &q_aero_pitch_step_M_;

/* Model output function */
void q_aero_pitch_step_output(void)
{
  real_T rtb_HILReadTimebase_o1;
  real_T rtb_HILReadTimebase_o2;
  real_T tmp;

  /* S-Function (hil_read_timebase_block): '<Root>/HIL Read Timebase' */

  /* S-Function Block: q_aero_pitch_step/HIL Read Timebase (hil_read_timebase_block) */
  {
    t_error result;
    result = hil_task_read(q_aero_pitch_step_DW.HILReadTimebase_Task, 1,
      NULL,
      &q_aero_pitch_step_DW.HILReadTimebase_EncoderBuffer,
      NULL,
      &q_aero_pitch_step_DW.HILReadTimebase_OtherBuffer
      );
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
    } else {
      rtb_HILReadTimebase_o1 =
        q_aero_pitch_step_DW.HILReadTimebase_EncoderBuffer;
      rtb_HILReadTimebase_o2 = q_aero_pitch_step_DW.HILReadTimebase_OtherBuffer;
    }
  }

  /* Step: '<Root>/Step' */
  if (q_aero_pitch_step_M->Timing.t[0] < q_aero_pitch_step_P.Step_Time) {
    tmp = q_aero_pitch_step_P.Step_Y0;
  } else {
    tmp = q_aero_pitch_step_P.Step_YFinal;
  }

  /* Gain: '<Root>/Gain' incorporates:
   *  Constant: '<Root>/Constant'
   *  Step: '<Root>/Step'
   *  Sum: '<Root>/Sum'
   */
  q_aero_pitch_step_B.Gain = (tmp + q_aero_pitch_step_P.Constant_Value) *
    q_aero_pitch_step_P.Gain_Gain;

  /* S-Function (hil_write_analog_block): '<Root>/HIL Write Analog' */

  /* S-Function Block: q_aero_pitch_step/HIL Write Analog (hil_write_analog_block) */
  {
    t_error result;
    result = hil_write_analog(q_aero_pitch_step_DW.HILInitialize_Card,
      &q_aero_pitch_step_P.HILWriteAnalog_channels, 1, &q_aero_pitch_step_B.Gain);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
    }
  }

  /* Gain: '<Root>/Counts to rad' */
  q_aero_pitch_step_B.Countstorad = q_aero_pitch_step_P.Countstorad_Gain *
    rtb_HILReadTimebase_o1;

  /* Gain: '<Root>/Counts//s to rad//s' */
  q_aero_pitch_step_B.Countsstorads = q_aero_pitch_step_P.Countsstorads_Gain *
    rtb_HILReadTimebase_o2;
}

/* Model update function */
void q_aero_pitch_step_update(void)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++q_aero_pitch_step_M->Timing.clockTick0)) {
    ++q_aero_pitch_step_M->Timing.clockTickH0;
  }

  q_aero_pitch_step_M->Timing.t[0] = q_aero_pitch_step_M->Timing.clockTick0 *
    q_aero_pitch_step_M->Timing.stepSize0 +
    q_aero_pitch_step_M->Timing.clockTickH0 *
    q_aero_pitch_step_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.002s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++q_aero_pitch_step_M->Timing.clockTick1)) {
      ++q_aero_pitch_step_M->Timing.clockTickH1;
    }

    q_aero_pitch_step_M->Timing.t[1] = q_aero_pitch_step_M->Timing.clockTick1 *
      q_aero_pitch_step_M->Timing.stepSize1 +
      q_aero_pitch_step_M->Timing.clockTickH1 *
      q_aero_pitch_step_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
void q_aero_pitch_step_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: q_aero_pitch_step/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("quanser_aero_usb", "0",
                      &q_aero_pitch_step_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options
      (q_aero_pitch_step_DW.HILInitialize_Card, " ", 2);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(q_aero_pitch_step_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
      return;
    }

    if ((q_aero_pitch_step_P.HILInitialize_AIPStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_AIPEnter && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_AIMinimums[0] =
        (q_aero_pitch_step_P.HILInitialize_AILow);
      q_aero_pitch_step_DW.HILInitialize_AIMinimums[1] =
        (q_aero_pitch_step_P.HILInitialize_AILow);
      q_aero_pitch_step_DW.HILInitialize_AIMaximums[0] =
        q_aero_pitch_step_P.HILInitialize_AIHigh;
      q_aero_pitch_step_DW.HILInitialize_AIMaximums[1] =
        q_aero_pitch_step_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges
        (q_aero_pitch_step_DW.HILInitialize_Card,
         q_aero_pitch_step_P.HILInitialize_AIChannels, 2U,
         &q_aero_pitch_step_DW.HILInitialize_AIMinimums[0],
         &q_aero_pitch_step_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if ((q_aero_pitch_step_P.HILInitialize_AOPStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_AOPEnter && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_AOMinimums[0] =
        (q_aero_pitch_step_P.HILInitialize_AOLow);
      q_aero_pitch_step_DW.HILInitialize_AOMinimums[1] =
        (q_aero_pitch_step_P.HILInitialize_AOLow);
      q_aero_pitch_step_DW.HILInitialize_AOMaximums[0] =
        q_aero_pitch_step_P.HILInitialize_AOHigh;
      q_aero_pitch_step_DW.HILInitialize_AOMaximums[1] =
        q_aero_pitch_step_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges
        (q_aero_pitch_step_DW.HILInitialize_Card,
         q_aero_pitch_step_P.HILInitialize_AOChannels, 2U,
         &q_aero_pitch_step_DW.HILInitialize_AOMinimums[0],
         &q_aero_pitch_step_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if ((q_aero_pitch_step_P.HILInitialize_AOStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_AOEnter && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_AOVoltages[0] =
        q_aero_pitch_step_P.HILInitialize_AOInitial;
      q_aero_pitch_step_DW.HILInitialize_AOVoltages[1] =
        q_aero_pitch_step_P.HILInitialize_AOInitial;
      result = hil_write_analog(q_aero_pitch_step_DW.HILInitialize_Card,
        q_aero_pitch_step_P.HILInitialize_AOChannels, 2U,
        &q_aero_pitch_step_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if (q_aero_pitch_step_P.HILInitialize_AOReset) {
      q_aero_pitch_step_DW.HILInitialize_AOVoltages[0] =
        q_aero_pitch_step_P.HILInitialize_AOWatchdog;
      q_aero_pitch_step_DW.HILInitialize_AOVoltages[1] =
        q_aero_pitch_step_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (q_aero_pitch_step_DW.HILInitialize_Card,
         q_aero_pitch_step_P.HILInitialize_AOChannels, 2U,
         &q_aero_pitch_step_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    result = hil_set_digital_directions(q_aero_pitch_step_DW.HILInitialize_Card,
      NULL, 0U, q_aero_pitch_step_P.HILInitialize_DOChannels, 2U);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
      return;
    }

    if ((q_aero_pitch_step_P.HILInitialize_DOStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_DOEnter && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_DOBits[0] =
        q_aero_pitch_step_P.HILInitialize_DOInitial;
      q_aero_pitch_step_DW.HILInitialize_DOBits[1] =
        q_aero_pitch_step_P.HILInitialize_DOInitial;
      result = hil_write_digital(q_aero_pitch_step_DW.HILInitialize_Card,
        q_aero_pitch_step_P.HILInitialize_DOChannels, 2U, (t_boolean *)
        &q_aero_pitch_step_DW.HILInitialize_DOBits[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if (q_aero_pitch_step_P.HILInitialize_DOReset) {
      q_aero_pitch_step_DW.HILInitialize_DOStates[0] =
        q_aero_pitch_step_P.HILInitialize_DOWatchdog;
      q_aero_pitch_step_DW.HILInitialize_DOStates[1] =
        q_aero_pitch_step_P.HILInitialize_DOWatchdog;
      result = hil_watchdog_set_digital_expiration_state
        (q_aero_pitch_step_DW.HILInitialize_Card,
         q_aero_pitch_step_P.HILInitialize_DOChannels, 2U, (const
          t_digital_state *) &q_aero_pitch_step_DW.HILInitialize_DOStates[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if ((q_aero_pitch_step_P.HILInitialize_EIPStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_EIPEnter && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_QuadratureModes[0] =
        q_aero_pitch_step_P.HILInitialize_EIQuadrature;
      q_aero_pitch_step_DW.HILInitialize_QuadratureModes[1] =
        q_aero_pitch_step_P.HILInitialize_EIQuadrature;
      q_aero_pitch_step_DW.HILInitialize_QuadratureModes[2] =
        q_aero_pitch_step_P.HILInitialize_EIQuadrature;
      q_aero_pitch_step_DW.HILInitialize_QuadratureModes[3] =
        q_aero_pitch_step_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (q_aero_pitch_step_DW.HILInitialize_Card,
         q_aero_pitch_step_P.HILInitialize_EIChannels, 4U,
         (t_encoder_quadrature_mode *)
         &q_aero_pitch_step_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if ((q_aero_pitch_step_P.HILInitialize_EIStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_EIEnter && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_InitialEICounts[0] =
        q_aero_pitch_step_P.HILInitialize_EIInitial;
      q_aero_pitch_step_DW.HILInitialize_InitialEICounts[1] =
        q_aero_pitch_step_P.HILInitialize_EIInitial;
      q_aero_pitch_step_DW.HILInitialize_InitialEICounts[2] =
        q_aero_pitch_step_P.HILInitialize_EIInitial;
      q_aero_pitch_step_DW.HILInitialize_InitialEICounts[3] =
        q_aero_pitch_step_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(q_aero_pitch_step_DW.HILInitialize_Card,
        q_aero_pitch_step_P.HILInitialize_EIChannels, 4U,
        &q_aero_pitch_step_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if ((q_aero_pitch_step_P.HILInitialize_OOStart && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_OOEnter && is_switching)) {
      result = hil_write_other(q_aero_pitch_step_DW.HILInitialize_Card,
        q_aero_pitch_step_P.HILInitialize_OOChannels, 3U,
        q_aero_pitch_step_P.HILInitialize_OOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }

    if (q_aero_pitch_step_P.HILInitialize_OOReset) {
      result = hil_watchdog_set_other_expiration_state
        (q_aero_pitch_step_DW.HILInitialize_Card,
         q_aero_pitch_step_P.HILInitialize_OOChannels, 3U,
         q_aero_pitch_step_P.HILInitialize_OOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_timebase_block): '<Root>/HIL Read Timebase' */

  /* S-Function Block: q_aero_pitch_step/HIL Read Timebase (hil_read_timebase_block) */
  {
    t_error result;
    result = hil_task_create_reader(q_aero_pitch_step_DW.HILInitialize_Card,
      q_aero_pitch_step_P.HILReadTimebase_SamplesInBuffer,
      NULL, 0U,
      &q_aero_pitch_step_P.HILReadTimebase_EncoderChannels, 1U,
      NULL, 0U,
      &q_aero_pitch_step_P.HILReadTimebase_OtherChannels, 1U,
      &q_aero_pitch_step_DW.HILReadTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (q_aero_pitch_step_DW.HILReadTimebase_Task, (t_buffer_overflow_mode)
         (q_aero_pitch_step_P.HILReadTimebase_OverflowMode - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
    }
  }
}

/* Model terminate function */
void q_aero_pitch_step_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: q_aero_pitch_step/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_digital_outputs = 0;
    t_uint32 num_final_other_outputs = 0;
    hil_task_stop_all(q_aero_pitch_step_DW.HILInitialize_Card);
    hil_monitor_stop_all(q_aero_pitch_step_DW.HILInitialize_Card);
    is_switching = false;
    if ((q_aero_pitch_step_P.HILInitialize_AOTerminate && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_AOExit && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_AOVoltages[0] =
        q_aero_pitch_step_P.HILInitialize_AOFinal;
      q_aero_pitch_step_DW.HILInitialize_AOVoltages[1] =
        q_aero_pitch_step_P.HILInitialize_AOFinal;
      num_final_analog_outputs = 2U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((q_aero_pitch_step_P.HILInitialize_DOTerminate && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_DOExit && is_switching)) {
      q_aero_pitch_step_DW.HILInitialize_DOBits[0] =
        q_aero_pitch_step_P.HILInitialize_DOFinal;
      q_aero_pitch_step_DW.HILInitialize_DOBits[1] =
        q_aero_pitch_step_P.HILInitialize_DOFinal;
      num_final_digital_outputs = 2U;
    } else {
      num_final_digital_outputs = 0;
    }

    if ((q_aero_pitch_step_P.HILInitialize_OOTerminate && !is_switching) ||
        (q_aero_pitch_step_P.HILInitialize_OOExit && is_switching)) {
      num_final_other_outputs = 3U;
    } else {
      num_final_other_outputs = 0;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_digital_outputs > 0
        || num_final_other_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(q_aero_pitch_step_DW.HILInitialize_Card
                         , q_aero_pitch_step_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , NULL, 0
                         , q_aero_pitch_step_P.HILInitialize_DOChannels,
                         num_final_digital_outputs
                         , q_aero_pitch_step_P.HILInitialize_OOChannels,
                         num_final_other_outputs
                         , &q_aero_pitch_step_DW.HILInitialize_AOVoltages[0]
                         , NULL
                         , (t_boolean *)
                         &q_aero_pitch_step_DW.HILInitialize_DOBits[0]
                         , q_aero_pitch_step_P.HILInitialize_OOFinal
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog
            (q_aero_pitch_step_DW.HILInitialize_Card,
             q_aero_pitch_step_P.HILInitialize_AOChannels,
             num_final_analog_outputs,
             &q_aero_pitch_step_DW.HILInitialize_AOVoltages[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_digital_outputs > 0) {
          local_result = hil_write_digital
            (q_aero_pitch_step_DW.HILInitialize_Card,
             q_aero_pitch_step_P.HILInitialize_DOChannels,
             num_final_digital_outputs, (t_boolean *)
             &q_aero_pitch_step_DW.HILInitialize_DOBits[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_other_outputs > 0) {
          local_result = hil_write_other(q_aero_pitch_step_DW.HILInitialize_Card,
            q_aero_pitch_step_P.HILInitialize_OOChannels,
            num_final_other_outputs, q_aero_pitch_step_P.HILInitialize_OOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(q_aero_pitch_step_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(q_aero_pitch_step_DW.HILInitialize_Card);
    hil_monitor_delete_all(q_aero_pitch_step_DW.HILInitialize_Card);
    hil_close(q_aero_pitch_step_DW.HILInitialize_Card);
    q_aero_pitch_step_DW.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  q_aero_pitch_step_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  q_aero_pitch_step_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  q_aero_pitch_step_initialize();
}

void MdlTerminate(void)
{
  q_aero_pitch_step_terminate();
}

/* Registration function */
RT_MODEL_q_aero_pitch_step_T *q_aero_pitch_step(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)q_aero_pitch_step_M, 0,
                sizeof(RT_MODEL_q_aero_pitch_step_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&q_aero_pitch_step_M->solverInfo,
                          &q_aero_pitch_step_M->Timing.simTimeStep);
    rtsiSetTPtr(&q_aero_pitch_step_M->solverInfo, &rtmGetTPtr
                (q_aero_pitch_step_M));
    rtsiSetStepSizePtr(&q_aero_pitch_step_M->solverInfo,
                       &q_aero_pitch_step_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&q_aero_pitch_step_M->solverInfo, (&rtmGetErrorStatus
      (q_aero_pitch_step_M)));
    rtsiSetRTModelPtr(&q_aero_pitch_step_M->solverInfo, q_aero_pitch_step_M);
  }

  rtsiSetSimTimeStep(&q_aero_pitch_step_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&q_aero_pitch_step_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&q_aero_pitch_step_M->solverInfo, false);
  rtsiSetSolverName(&q_aero_pitch_step_M->solverInfo,"FixedStepDiscrete");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = q_aero_pitch_step_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    q_aero_pitch_step_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    q_aero_pitch_step_M->Timing.sampleTimes =
      (&q_aero_pitch_step_M->Timing.sampleTimesArray[0]);
    q_aero_pitch_step_M->Timing.offsetTimes =
      (&q_aero_pitch_step_M->Timing.offsetTimesArray[0]);

    /* task periods */
    q_aero_pitch_step_M->Timing.sampleTimes[0] = (0.0);
    q_aero_pitch_step_M->Timing.sampleTimes[1] = (0.002);

    /* task offsets */
    q_aero_pitch_step_M->Timing.offsetTimes[0] = (0.0);
    q_aero_pitch_step_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(q_aero_pitch_step_M, &q_aero_pitch_step_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = q_aero_pitch_step_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    q_aero_pitch_step_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(q_aero_pitch_step_M, 160.0);
  q_aero_pitch_step_M->Timing.stepSize0 = 0.002;
  q_aero_pitch_step_M->Timing.stepSize1 = 0.002;

  /* External mode info */
  q_aero_pitch_step_M->Sizes.checksums[0] = (499111705U);
  q_aero_pitch_step_M->Sizes.checksums[1] = (2856182758U);
  q_aero_pitch_step_M->Sizes.checksums[2] = (1211396456U);
  q_aero_pitch_step_M->Sizes.checksums[3] = (1635994954U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    q_aero_pitch_step_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(q_aero_pitch_step_M->extModeInfo,
      &q_aero_pitch_step_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(q_aero_pitch_step_M->extModeInfo,
                        q_aero_pitch_step_M->Sizes.checksums);
    rteiSetTPtr(q_aero_pitch_step_M->extModeInfo, rtmGetTPtr(q_aero_pitch_step_M));
  }

  q_aero_pitch_step_M->solverInfoPtr = (&q_aero_pitch_step_M->solverInfo);
  q_aero_pitch_step_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&q_aero_pitch_step_M->solverInfo, 0.002);
  rtsiSetSolverMode(&q_aero_pitch_step_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  q_aero_pitch_step_M->blockIO = ((void *) &q_aero_pitch_step_B);
  (void) memset(((void *) &q_aero_pitch_step_B), 0,
                sizeof(B_q_aero_pitch_step_T));

  /* parameters */
  q_aero_pitch_step_M->defaultParam = ((real_T *)&q_aero_pitch_step_P);

  /* states (dwork) */
  q_aero_pitch_step_M->dwork = ((void *) &q_aero_pitch_step_DW);
  (void) memset((void *)&q_aero_pitch_step_DW, 0,
                sizeof(DW_q_aero_pitch_step_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    q_aero_pitch_step_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 22;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  q_aero_pitch_step_M->Sizes.numContStates = (0);/* Number of continuous states */
  q_aero_pitch_step_M->Sizes.numY = (0);/* Number of model outputs */
  q_aero_pitch_step_M->Sizes.numU = (0);/* Number of model inputs */
  q_aero_pitch_step_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  q_aero_pitch_step_M->Sizes.numSampTimes = (2);/* Number of sample times */
  q_aero_pitch_step_M->Sizes.numBlocks = (13);/* Number of blocks */
  q_aero_pitch_step_M->Sizes.numBlockIO = (3);/* Number of block outputs */
  q_aero_pitch_step_M->Sizes.numBlockPrms = (123);/* Sum of parameter "widths" */
  return q_aero_pitch_step_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
