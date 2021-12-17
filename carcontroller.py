from cereal import car
from common.numpy_fast import clip
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.ocelot.ocelotcan import create_steer_command, create_ibst_command, \
                                           create_pedal_command, create_msg_command
from selfdrive.car.ocelot.values import SteerLimitParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 1.5  # 1.5 m/s2
ACCEL_MIN = -3.0  # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

def accel_hysteresis(accel, accel_steady, enabled):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.

    self.last_steer = 0
    self.accel_steady = 0.
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False

    self.last_fault_frame = -200
    self.steer_rate_limited = False

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    # *** compute control surfaces ***

    # gas and brake
    apply_gas = clip(actuators.gas, 0., 1.)

    if CS.CP.enableGasInterceptor:
      # send only negative accel if interceptor is detected. otherwise, send the regular value
      # +0.06 offset to reduce ABS pump usage when OP is engaged
      apply_accel = 0.06 - actuators.brake
    else:
      apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    if CS.CP.enableGasInterceptor:
      if CS.out.gasPressed:
        apply_accel = max(apply_accel, 0.06)
      if CS.out.brakePressed:
        apply_gas = 0.0
        apply_accel = min(apply_accel, 0.00)
    else:
      if CS.out.gasPressed:
        apply_accel = max(apply_accel, 0.0)
      if CS.out.brakePressed and CS.out.vEgo > 1:
        apply_accel = min(apply_accel, 0.0)

    # steer torque
    new_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # only cut torque when steer state is a known fault
    if CS.brakeUnavailable:
      self.last_fault_frame = frame

    # Cut steering for 2s after fault
    if not enabled or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    self.last_steer = apply_steer
    self.last_accel = apply_accel
    self.last_standstill = CS.out.standstill

    can_sends = []

    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    if CS.out.vEgo < 0.2:
      apply_brakes = 0.25
    else:
      apply_brakes = actuators.brake

    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
    can_sends.append(create_ibst_command(self.packer, enabled, apply_brakes, frame))
    can_sends.append(create_pedal_command(self.packer, apply_gas, frame))

    #UI mesg is at 100Hz but we send asap if:
    if (frame % 100 == 0):
      can_sends.append(create_msg_command(self.packer, enabled, CS.out.cruiseState.speed * CV.MS_TO_MPH, CS.out.vEgo * CV.MS_TO_MPH))


    return can_sends
