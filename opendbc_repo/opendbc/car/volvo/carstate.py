from collections import deque

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import CAR, DBC, VolvoFlags, C1_CAR, EUCD_CAR, CarControllerParams as CCP

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class diagInfo:
  """Diagnostic information container"""
  def __init__(self):
    self.diagFSMResp = 0
    self.diagCEMResp = 0
    self.diagPSCMResp = 0
    self.diagCVMResp = 0


class PSCMInfo:
  """Power Steering Control Module information"""
  def __init__(self):
    self.byte0 = 0
    self.byte4 = 0
    self.byte7 = 0
    self.LKAActive = 0
    self.LKATorque = 0
    self.SteeringAngleServo = 0
    self.byte3 = 0  # C1 platform
    self.SteeringWheelRateOfChange = 0  # EUCD platform


class FSMInfo:
  """Forward Sensing Module information"""
  def __init__(self):
    self.TrqLim = 0
    self.LKAAngleReq = 0
    self.Checksum = 0
    self.LKASteerDirection = 0
    # C1 platform
    self.SET_X_E3 = 0
    self.SET_X_B4 = 0
    self.SET_X_08 = 0
    self.SET_X_02 = 0
    self.SET_X_25 = 0
    # EUCD platform
    self.SET_X_22 = 0
    self.SET_X_A4 = 0
    self.SET_X_10 = 0


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.diag = diagInfo()
    self.PSCMInfo = PSCMInfo()
    self.FSMInfo = FSMInfo()
    
    self.trq_fifo = deque([])
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    if CP.flags & VolvoFlags.C1:
      self.shifter_values = self.can_define.dv["TCM0"]["GearShifter"]

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    # Speeds
    ret.vEgoRaw = cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # Steering
    ret.steeringAngleDeg = cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringTorque = cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = bool(
      cp.vl["CCButtons"]["ACCSetBtn"] or
      cp.vl["CCButtons"]["ACCMinusBtn"] or
      cp.vl["CCButtons"]["ACCResumeBtn"]
    )

    # Update gas and brake
    if self.CP.flags & VolvoFlags.C1:
      ret.gas = cp.vl["PedalandBrake"]["AccPedal"] / 102.3
      ret.gasPressed = ret.gas > 0.05
    elif self.CP.flags & VolvoFlags.EUCD:
      ret.gas = cp.vl["AccPedal"]["AccPedal"] / 102.3
      ret.gasPressed = ret.gas > 0.1
    ret.brakePressed = False

    # Update gear position
    if self.CP.flags & VolvoFlags.C1:
      can_gear = int(cp.vl["TCM0"]["GearShifter"])
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    elif self.CP.flags & VolvoFlags.EUCD:
      ret.gearShifter = self.parse_gear_shifter('D')  # TODO: Gear EUCD

    # Belt and doors
    ret.doorOpen = False
    ret.seatbeltUnlatched = False  # No signal yet

    # ACC status from camera
    if self.CP.flags & VolvoFlags.C1:
      ret.cruiseState.available = bool(cp_cam.vl["FSM0"]["ACCStatusOnOff"])
      ret.cruiseState.enabled = bool(cp_cam.vl["FSM0"]["ACCStatusActive"])
      ret.cruiseState.speed = cp.vl["ACC"]["SpeedTargetACC"] * CV.KPH_TO_MS

    elif self.CP.flags & VolvoFlags.EUCD:
      accStatus = cp_cam.vl["FSM0"]["ACCStatus"]
      if accStatus == 2:
        ret.cruiseState.available = True
        ret.cruiseState.enabled = False
      elif accStatus >= 6:
        ret.cruiseState.available = True
        ret.cruiseState.enabled = True
      else:
        ret.cruiseState.available = False
        ret.cruiseState.enabled = False

    # Blinkers
    ret.leftBlinker = cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # Diagnostics
    self.diag.diagFSMResp = int(cp_cam.vl["diagFSMResp"]["byte03"])
    self.diag.diagCEMResp = int(cp.vl["diagCEMResp"]["byte03"])
    self.diag.diagCVMResp = int(cp.vl["diagCVMResp"]["byte03"])
    self.diag.diagPSCMResp = int(cp.vl["diagPSCMResp"]["byte03"])

    # PSCMInfo
    self.PSCMInfo.byte0 = int(cp.vl["PSCM1"]["byte0"])
    self.PSCMInfo.byte4 = int(cp.vl["PSCM1"]["byte4"])
    self.PSCMInfo.byte7 = int(cp.vl["PSCM1"]["byte7"])
    self.PSCMInfo.LKATorque = int(cp.vl["PSCM1"]["LKATorque"])
    self.PSCMInfo.LKAActive = int(cp.vl["PSCM1"]["LKAActive"])
    self.PSCMInfo.SteeringAngleServo = float(cp.vl["PSCM1"]["SteeringAngleServo"])

    if self.CP.flags & VolvoFlags.C1:
      self.PSCMInfo.byte3 = int(cp.vl["PSCM1"]["byte3"])
    elif self.CP.flags & VolvoFlags.EUCD:
      self.PSCMInfo.SteeringWheelRateOfChange = float(cp.vl["PSCM1"]["SteeringWheelRateOfChange"])

    # FSMInfo
    if self.CP.flags & VolvoFlags.C1:
      self.FSMInfo.TrqLim = int(cp_cam.vl["FSM1"]["TrqLim"])
      self.FSMInfo.LKAAngleReq = float(cp_cam.vl["FSM1"]["LKAAngleReq"])
      self.FSMInfo.Checksum = int(cp_cam.vl["FSM1"]["Checksum"])
      self.FSMInfo.LKASteerDirection = int(cp_cam.vl["FSM1"]["LKASteerDirection"])
      self.FSMInfo.SET_X_E3 = int(cp_cam.vl["FSM1"]["SET_X_E3"])
      self.FSMInfo.SET_X_B4 = int(cp_cam.vl["FSM1"]["SET_X_B4"])
      self.FSMInfo.SET_X_08 = int(cp_cam.vl["FSM1"]["SET_X_08"])
      self.FSMInfo.SET_X_02 = int(cp_cam.vl["FSM1"]["SET_X_02"])
      self.FSMInfo.SET_X_25 = int(cp_cam.vl["FSM1"]["SET_X_25"])

    elif self.CP.flags & VolvoFlags.EUCD:
      self.FSMInfo.TrqLim = int(cp_cam.vl["FSM2"]["TrqLim"])
      self.FSMInfo.LKAAngleReq = float(cp_cam.vl["FSM2"]["LKAAngleReq"])
      self.FSMInfo.Checksum = int(cp_cam.vl["FSM2"]["Checksum"])
      self.FSMInfo.LKASteerDirection = int(cp_cam.vl["FSM2"]["LKASteerDirection"])
      self.FSMInfo.SET_X_22 = int(cp_cam.vl["FSM2"]["SET_X_22"])
      self.FSMInfo.SET_X_02 = int(cp_cam.vl["FSM2"]["SET_X_02"])
      self.FSMInfo.SET_X_A4 = int(cp_cam.vl["FSM2"]["SET_X_A4"])
      self.FSMInfo.SET_X_10 = int(cp_cam.vl["FSM2"]["SET_X_10"])

    # Check if servo stops responding when ACC is active
    if self.CP.flags & VolvoFlags.C1:
      if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
        self.trq_fifo.append(self.PSCMInfo.LKATorque)
        ret.steerWarning = bool(self.trq_fifo.count(0) >= CCP.N_ZERO_TRQ * 2)
        if len(self.trq_fifo) > CCP.N_ZERO_TRQ * 2:
          self.trq_fifo.popleft()
      else:
        self.trq_fifo.clear()
        ret.steerWarning = False

    # Brake lights
    ret.brakeLights = ret.brakePressed

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Common signals for both platforms
    signals = [
      ("VehicleSpeed", "VehicleSpeed1", 0),
      ("TurnSignal", "MiscCarInfo", 0),
      ("ACCOnOffBtn", "CCButtons", 0),
      ("ACCResumeBtn", "CCButtons", 0),
      ("ACCSetBtn", "CCButtons", 0),
      ("ACCMinusBtn", "CCButtons", 0),
      ("TimeGapIncreaseBtn", "CCButtons", 0),
      ("TimeGapDecreaseBtn", "CCButtons", 0),
      # PSCM signals
      ("SteeringAngleServo", "PSCM1", 0),
      ("LKATorque", "PSCM1", 0),
      ("LKAActive", "PSCM1", 0),
      ("byte0", "PSCM1", 0),
      ("byte4", "PSCM1", 0),
      ("byte7", "PSCM1", 0),
      # Diagnostic
      ("byte03", "diagCEMResp", 0),
      ("byte47", "diagCEMResp", 0),
      ("byte03", "diagPSCMResp", 0),
      ("byte47", "diagPSCMResp", 0),
      ("byte03", "diagCVMResp", 0),
      ("byte47", "diagCVMResp", 0),
    ]

    checks = [
      ("CCButtons", 100),
      ("PSCM1", 50),
      ("VehicleSpeed1", 50),
      ("MiscCarInfo", 25),
      ("diagCEMResp", 0),
      ("diagPSCMResp", 0),
      ("diagCVMResp", 0),
    ]

    # Platform specific signals
    if CP.flags & VolvoFlags.C1:
      signals.extend([
        ("SpeedTargetACC", "ACC", 0),
        ("BrakePedalActive2", "PedalandBrake", 0),
        ("AccPedal", "PedalandBrake", 0),
        ("BrakePress0", "BrakeMessages", 0),
        ("BrakePress1", "BrakeMessages", 0),
        ("BrakeStatus", "BrakeMessages", 0),
        ("GearShifter", "TCM0", 0),
        ("byte3", "PSCM1", 0),
        ("ACCStopBtn", "CCButtons", 0),
      ])
      checks.extend([
        ("BrakeMessages", 50),
        ("ACC", 17),
        ("PedalandBrake", 100),
        ("TCM0", 10),
      ])

    if CP.flags & VolvoFlags.EUCD:
      signals.extend([
        ("AccPedal", "AccPedal", 0),
        ("BrakePedal", "BrakePedal", 0),
        ("SteeringWheelRateOfChange", "PSCM1", 0),
        # Inverted button states
        ("ACCOnOffBtnInv", "CCButtons", 1),
        ("ACCResumeBtnInv", "CCButtons", 1),
        ("ACCSetBtnInv", "CCButtons", 1),
        ("ACCMinusBtnInv", "CCButtons", 1),
        ("TimeGapDecreaseBtnInv", "CCButtons", 1),
        ("TimeGapIncreaseBtnInv", "CCButtons", 1),
      ])
      checks.extend([
        ("AccPedal", 100),
        ("BrakePedal", 50),
      ])

    pt_parser = CANParser(DBC[CP.carFingerprint][Bus.pt], signals, checks, 0)

    # Camera CAN parser
    cam_signals = [
      ("byte03", "diagFSMResp", 0),
      ("byte47", "diagFSMResp", 0),
    ]
    cam_checks = [
      ("diagFSMResp", 0),
    ]

    if CP.flags & VolvoFlags.C1:
      cam_signals.extend([
        ("TrqLim", "FSM1", 0x80),
        ("LKAAngleReq", "FSM1", 0x2000),
        ("Checksum", "FSM1", 0x5f),
        ("LKASteerDirection", "FSM1", 0x00),
        ("SET_X_E3", "FSM1", 0xE3),
        ("SET_X_B4", "FSM1", 0xB4),
        ("SET_X_08", "FSM1", 0x08),
        ("SET_X_02", "FSM1", 0x02),
        ("SET_X_25", "FSM1", 0x25),
        ("ACCStatusOnOff", "FSM0", 0x00),
        ("ACCStatusActive", "FSM0", 0x00),
      ])
      cam_checks.extend([
        ("FSM0", 100),
        ("FSM1", 50),
      ])

    elif CP.flags & VolvoFlags.EUCD:
      cam_signals.extend([
        ("ACCStatus", "FSM0", 0),
        ("TrqLim", "FSM2", 0x80),
        ("LKAAngleReq", "FSM2", 0x2000),
        ("Checksum", "FSM2", 0x5f),
        ("LKASteerDirection", "FSM2", 0x00),
        ("SET_X_22", "FSM2", 0x00),
        ("SET_X_02", "FSM2", 0x00),
        ("SET_X_10", "FSM2", 0x00),
        ("SET_X_A4", "FSM2", 0x00),
      ])
      cam_checks.extend([
        ("FSM0", 100),
        ("FSM2", 50),
      ])

    cam_parser = CANParser(DBC[CP.carFingerprint][Bus.pt], cam_signals, cam_checks, 2)

    return {Bus.pt: pt_parser, Bus.cam: cam_parser}
