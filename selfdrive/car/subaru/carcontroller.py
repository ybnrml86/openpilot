from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import DBC, CAR, PREGLOBAL_CARS, CarControllerParams
from opendbc.can.packer import CANPacker


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_accel_cnt = -1
    self.es_lkas_cnt = -1
    self.cruise_button_prev = 0
    self.steer_rate_limited = False
    self.sng_acc_resume = False
    self.sng_acc_resume_cnt = -1


    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart):

    can_sends = []

    # *** steering ***
    if (frame % CarControllerParams.STEER_STEP) == 0:

      apply_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))

      # limits due to driver torque

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, CarControllerParams)
      self.steer_rate_limited = new_steer != apply_steer

      if not enabled:
        apply_steer = 0

      if CS.CP.carFingerprint in PREGLOBAL_CARS:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, apply_steer, frame, CarControllerParams.STEER_STEP))
      else:
        can_sends.append(subarucan.create_steering_control(self.packer, apply_steer, frame, CarControllerParams.STEER_STEP))

      self.apply_steer_last = apply_steer

    # *** stop and go ***

    throttle_cmd = False
    speed_cmd = False

    if CS.CP.carFingerprint in PREGLOBAL_CARS and CS.CP.carFingerprint != CAR.FORESTER_PREGLOBAL:
      if (enabled                                             # ACC active
           and CS.car_follow == 1                             # lead car
           and CS.close_distance > 3                          # acc resume trigger threshold
           and CS.close_distance < 4.5                        # max operating distance to filter false positives
           and CS.close_distance > self.prev_close_distance   # distance with lead car is increasing
           and CS.out.standstill):                            # must be standing still
         self.sng_resume_acc = True
    elif CS.CP.carFingerprint not in PREGLOBAL_CARS:
      if (enabled                                             # ACC active
           and CS.car_follow == 1                             # lead car
           and CS.cruise_state == 3                           # ACC HOLD
           and CS.close_distance > 150                        # acc resume trigger threshold
           and CS.close_distance < 255                        # ignore max value
           and CS.close_distance > self.prev_close_distance): # distance with lead car is increasing
         self.sng_acc_resume = True

      # Detect NON-EPB
      if (enabled and (CS.cruise_state != 3) and CS.out.standstill and frame > self.standstill_start + 100):
        speed_cmd = True

      if CS.out.standstill and not self.prev_standstill:
        self.standstill_start = frame
        self.prev_standstill = CS.out.standstill

    if self.sng_acc_resume:
      if self.sng_acc_resume_cnt < 5:
        throttle_cmd = True
        self.sng_acc_resume_cnt += 1
      else:
        self.sng_acc_resume = False
        self.sng_acc_resume_cnt = -1

    # Cancel ACC if stopped, brake pressed and not stopped behind another car
    if enabled and CS.out.brakePressed and CS.car_follow == 0 and CS.out.standstill:
      pcm_cancel_cmd = True

    self.prev_close_distance = CS.close_distance

    # *** alerts and pcm cancel ***

    if CS.CP.carFingerprint in PREGLOBAL_CARS:
      if self.es_accel_cnt != CS.es_accel_msg["Counter"]:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_es_throttle_control(self.packer, cruise_button, CS.es_accel_msg))
        self.es_accel_cnt = CS.es_accel_msg["Counter"]

      if self.throttle_cnt != CS.throttle_msg["Counter"]:
         can_sends.append(subarucan.create_preglobal_throttle(self.packer, CS.throttle_msg, throttle_cmd))
         self.throttle_cnt = CS.throttle_msg["Counter"]

    else:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        can_sends.append(subarucan.create_es_lkas(self.packer, CS.es_lkas_msg, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]

      if self.throttle_cnt != CS.throttle_msg["Counter"]:
         can_sends.append(subarucan.create_throttle(self.packer, CS.throttle_msg, throttle_cmd))
         self.throttle_cnt = CS.throttle_msg["Counter"]

      if self.brake_pedal_cnt != CS.brake_pedal_msg["Counter"]:
         can_sends.append(subarucan.create_brake_pedal(self.packer, CS.brake_pedal_msg, speed_cmd))
         self.brake_pedal_cnt = CS.brake_pedal_msg["Counter"]

    return can_sends
