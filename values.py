# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

# Steer torque limits
class SteerLimitParams:
  STEER_MAX = 255               # max_steer 255 (100% duty cycle)
  STEER_STEP = 1                # how often we update the steer cmd
  STEER_DELTA_UP = 8            # torque increase per refresh, to max
  STEER_DELTA_DOWN = 10         # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 150  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 10  # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1

class CAR:
  SMART_ROADSTER_COUPE = "SMART ROADSTER COUPE 2003-2006"

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

STATIC_MSGS = [
]

ECU_FINGERPRINT = {
}

FINGERPRINTS = {
  CAR.SMART_ROADSTER_COUPE: [{
     33: 8, 34: 8, 85: 8, 128: 8, 144: 8, 194: 8, 256: 8, 257: 8, 261: 8, 264: 8, 265: 8, 272: 8, 288: 8, 289: 8, 290: 8, 291: 8, 292: 8, 293: 8, 294: 8, 295: 8, 296: 8, 297: 8, 298: 8, 299: 8, 300: 8, 301: 8, 302: 8, 303: 8, 304: 8, 305: 8, 306: 8, 307: 8, 308: 8, 309: 8, 310: 8, 311: 8, 312: 8, 313: 8, 314: 8, 315: 8, 316: 8, 317: 8, 318: 8, 319: 8, 320: 8, 321: 8, 322: 8, 323: 8, 324: 8, 325: 8, 326: 8, 327: 8, 328: 8, 329: 8, 330: 8, 331: 8, 332: 8, 333: 8, 334: 8, 335: 8, 336: 8, 337: 8, 338: 8, 339: 8, 340: 8, 341: 8, 342: 8, 343: 8, 344: 8, 345: 8, 346: 8, 347: 8, 348: 8, 349: 8, 350: 8, 351: 8, 368: 8, 369: 8, 371: 8, 372: 8, 373: 8, 400: 8, 520: 8, 528: 8, 544: 8, 568: 8, 570: 6, 768: 8, 784: 8, 1296: 1, 1301: 7, 1376: 8, 1585: 8
   }]
}

STEER_THRESHOLD = 70

DBC = {
    CAR.SMART_ROADSTER_COUPE: dbc_dict('ocelot_can', 'ford_focus_adas', chassis_dbc = 'ocelot_smart_roadster_pt'),
}
