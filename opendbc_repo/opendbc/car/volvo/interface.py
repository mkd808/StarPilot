from opendbc.car import Bus, structs, get_safety_config
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.values import CAR, DBC, VolvoFlags, C1_CAR, EUCD_CAR

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, dp_params, docs) -> structs.CarParams:
    """Get vehicle parameters for the specified candidate."""
    ret.brand = "volvo"

    # C1 platform (V40) - Community supported
    if ret.flags & VolvoFlags.C1:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
      
      # V40 specific parameters
      if candidate == str(CAR.VOLVO_V40):
        ret.mass = 1610.
        ret.wheelbase = 2.647
        ret.centerToFront = ret.wheelbase * 0.44
        ret.steerRatio = 14.7

    # EUCD platform (V60) - dashcam only
    elif ret.flags & VolvoFlags.EUCD:
      ret.dashcamOnly = True
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
      
      if candidate == str(CAR.VOLVO_V60):
        ret.mass = 1750.
        ret.wheelbase = 2.776
        ret.centerToFront = ret.wheelbase * 0.44
        ret.steerRatio = 15.0

    # Common parameters
    ret.radarUnavailable = True  # No radar objects on CAN
    
    # Steering control - Volvo uses angle control
    ret.steerControlType = SteerControlType.angle
    ret.minSteerSpeed = 1. * CV.KPH_TO_MS
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 1.0

    # Lateral tuning (PID for angle control)
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.0
    ret.lateralTuning.pid.kpV = [0.0]
    ret.lateralTuning.pid.kiV = [0.0]

    # Transmission
    ret.transmissionType = structs.CarParams.TransmissionType.automatic

    # Cruise control
    ret.pcmCruise = True
    ret.minEnableSpeed = -1.

    return ret
