#include "Input.h"

#include <MY17_Can_Library.h>

#include "Adc.h"
#include "Serial.h"

void update_adc(Input_T *input);
void update_can(Input_T *input);

void can_process_error(void);
void can_process_unknown(Input_T *input);
void can_process_voltage(Input_T *input);
void can_process_current(Input_T *input);
void can_process_power(Input_T *input);
void can_process_energy(Input_T *input);
void can_process_mc_data(Input_T *input);
void can_process_mc_state(Input_T *input);
void can_process_vcu_dash(Input_T *input);

#define ADC_UPDATE_PERIOD_MS 10

void Input_initialize(Input_T *input) {
  input->adc->accel_1_raw = 0;
  input->adc->accel_2_raw = 0;
  input->adc->brake_1_raw = 0;
  input->adc->brake_2_raw = 0;
  input->adc->last_updated = 0;

  uint8_t wheel;
  for (wheel = 0; wheel < NUM_WHEELS; wheel++) {
    input->speed->tick_count[wheel] = 0;
    input->speed->tick_us[wheel] = 0;
    input->speed->moving_avg_us[wheel] = 0;
    input->speed->wheel_stopped[wheel] = false;
  }

  input->mc->motor_speed = 0;
  input->mc->last_updated = 0;

  uint8_t i;
  for(i = 0; i < CS_VALUES_LENGTH; i++) {
    input->current_sensor->data[i] = 0;
    input->current_sensor->last_updated[i] = 0;
  }

  input->misc->lv_voltage = 0;
  input->misc->hv_enabled = false;
  input->misc->limp_state = CAN_LIMP_NORMAL;
}

void Input_fill_input(Input_T *input) {
  update_adc(input);
  update_can(input);
}

void update_adc(Input_T *input) {
  Adc_Input_T *adc = input->adc;
  uint32_t next_updated = adc->last_updated + ADC_UPDATE_PERIOD_MS;

  if (next_updated < input->msTicks) {
    adc->accel_1_raw = ADC_Read(ACCEL_1_CHANNEL);
    adc->accel_2_raw = ADC_Read(ACCEL_2_CHANNEL);
    adc->brake_1_raw = ADC_Read(BRAKE_1_CHANNEL);
    adc->brake_2_raw = ADC_Read(BRAKE_2_CHANNEL);
    adc->last_updated = input->msTicks;
  }
}

void update_can(Input_T *input) {
  Can_MsgID_T msgID = Can_MsgType();
  switch(msgID) {
    case Can_Error_Msg:
      can_process_error();
      break;

    case Can_Unknown_Msg:
      can_process_unknown(input);
      break;

    case Can_Vcu_DashHeartbeat_Msg:
      can_process_vcu_dash(input);
      break;

    case Can_MC_DataReading_Msg:
      can_process_mc_data(input);

    case Can_No_Msg:
    default:
      break;
  }
}

void can_process_error(void) {
  Can_ErrorID_T err = Can_Error_Read();
  UNUSED(err);
  /* Serial_Print("can_read_err: "); */
  /* Serial_PrintlnNumber(err, 16); */
}

void can_process_unknown(Input_T *input) {
  UNUSED(input);
  Frame f;
  Can_Unknown_Read(&f);
}

void can_process_vcu_dash(Input_T *input) {
  Can_Vcu_DashHeartbeat_T msg;
  Can_Vcu_DashHeartbeat_Read(&msg);

  input->misc->hv_enabled = msg.hv_light;
  input->misc->lv_voltage = msg.lv_battery_voltage;
  input->misc->limp_state = msg.limp_state;
}

void can_process_mc_data(Input_T *input) {
  Can_MC_DataReading_T msg;
  Can_MC_DataReading_Read(&msg);
  if (msg.type == CAN_MC_REG_SPEED_ACTUAL_RPM) {
    input->mc->motor_speed = msg.value;
    input->mc->last_updated = input->msTicks;
  }
}
