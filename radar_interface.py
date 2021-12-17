#!/usr/bin/env python3
import time
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.ocelot.values import DBC
from selfdrive.car.interfaces import RadarInterfaceBase
#from common.numpy_fast import sin
from math import sin

RADAR_MSGS = list(range(0x120, 0x12F))

def _create_radar_can_parser(car_fingerprint):
  msg_n = len(RADAR_MSGS)
  signals = list(zip(['CAN_DET_RANGE'] * msg_n + ['CAN_DET_AZIMUTH'] * msg_n + ['CAN_DET_RANGE_RATE'] * msg_n + ['CAN_DET_VALID_LEVEL'] * msg_n,
                     RADAR_MSGS * 4,
                     [0] * msg_n + [0] * msg_n + [0] * msg_n + [0] * msg_n))
  checks = list(zip(RADAR_MSGS, [20]*msg_n))

  return CANParser(DBC[car_fingerprint]['radar'], signals, checks, 2)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.validCnt = {key: 0 for key in RADAR_MSGS}
    self.track_id = 0
    self.radar_ts = CP.radarTimeStep
    #self.no_radar_sleep = 'NO_RADAR_SLEEP' in os.environ

    self.rcp = _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = 0x12E
    self.updated_messages = set()

    #Disable radar for vision only testing
    self.no_radar = True

  def update(self, can_strings):
    if self.no_radar:
      #time.sleep(self.radar_ts)
      time.sleep(0.02)
      return car.RadarData.new_message()
    
    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    errors = []
    #if not self.rcp.can_valid:
      #errors.append("canError")    #Test ignore errors for now
    ret.errors = errors

    for ii in sorted(self.updated_messages):
      cpt = self.rcp.vl[ii]

      # radar point only valid if valid signal asserted
      if cpt['CAN_DET_VALID_LEVEL'] > 0:
        if ii not in self.pts:
          self.pts[ii] = car.RadarData.RadarPoint.new_message()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1
        self.pts[ii].dRel = cpt['CAN_DET_RANGE']  # from front of car
        #self.pts[ii].yRel = cpt['CAN_DET_RANGE'] * -cpt['CAN_DET_AZIMUTH']   # in car frame's y axis, left is positive
        self.pts[ii].yRel = sin(-cpt['CAN_DET_AZIMUTH'])*cpt['CAN_DET_RANGE']
        self.pts[ii].vRel = cpt['CAN_DET_RANGE_RATE']
        self.pts[ii].aRel = float('nan')
        self.pts[ii].yvRel = float('nan')
        self.pts[ii].measured = True
      else:
        if ii in self.pts:
          del self.pts[ii]

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
