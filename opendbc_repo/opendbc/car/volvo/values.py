from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, Column, SupportType, CarParts

Ecu = CarParams.Ecu


class CarControllerParams:
  """Controller parameters for Volvo vehicles."""
  # Steering direction constants
  STEER_NO = 0
  STEER_RIGHT = 1
  STEER_LEFT = 2
  STEER = 3

  # Rate limits for angle control (wind up / unwind)
  ANGLE_DELTA_BP = [0., 8.33, 13.89, 19.44, 25., 30.55, 36.1]   # 0, 30, 50, 70, 90, 110, 130 km/h
  ANGLE_DELTA_V = [2., 1.2, .25, .20, .15, .10, .10]           # windup limit
  ANGLE_DELTA_VU = [2., 1.2, .25, .20, .15, .10, .10]          # unwind limit

  # Number of zero torque samples before trying to restore steering
  N_ZERO_TRQ = 12

  # EUCD platform specific
  BLOCK_LEN = 8      # Number of samples to block steering direction change
  DEADZONE = 0.1     # Deadzone for steer direction change


class VolvoFlags(IntFlag):
  C1 = 1
  EUCD = 2


def dbc_dict(pt: str) -> dict:
  return {Bus.pt: pt}


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  support_type: SupportType = SupportType.COMMUNITY
  support_link: str = "#community"


@dataclass
class VolvoDashcamCarDocs(VolvoCarDocs):
  """Dashcam only - not validated by comma"""
  support_type: SupportType = SupportType.DASHCAM
  support_link: str = "#dashcam"


@dataclass
class VolvoC1PlatformConfig(PlatformConfig):
  """Platform config for C1 platform (V40) - Community supported"""
  def init(self):
    self.flags |= VolvoFlags.C1


@dataclass  
class VolvoEUCDPlatformConfig(PlatformConfig):
  """Platform config for EUCD platform (V60) - Dashcam only"""
  def init(self):
    self.flags |= VolvoFlags.EUCD


class CAR(Platforms):
  # C1 Platform - V40 (2014-2017) - Community supported
  VOLVO_V40 = VolvoC1PlatformConfig(
    [
      VolvoCarDocs("Volvo V40 2014"),
      VolvoCarDocs("Volvo V40 2015"),
      VolvoCarDocs("Volvo V40 2016"),
      VolvoCarDocs("Volvo V40 2017"),
    ],
    CarSpecs(mass=1610., wheelbase=2.647, steerRatio=14.7, centerToFrontRatio=0.44),
    dbc_dict('volvo_v40_2017_pt'),
  )

  # EUCD Platform - V60 (2015) - Dashcam only (safety model not complete)
  VOLVO_V60 = VolvoEUCDPlatformConfig(
    [
      VolvoDashcamCarDocs("Volvo V60 2015"),
    ],
    CarSpecs(mass=1750., wheelbase=2.776, steerRatio=15.0, centerToFrontRatio=0.44),
    dbc_dict('volvo_v60_2015_pt'),
  )


# Platform groupings
C1_CAR = CAR.with_flags(VolvoFlags.C1)
EUCD_CAR = CAR.with_flags(VolvoFlags.EUCD)

# ECU addresses for firmware queries
ECU_ADDRESS = {
  CAR.VOLVO_V40: {
    "BCM": 0x760,
    "ECM": 0x7E0,
    "DIM": 0x720,
    "CEM": 0x726,
    "FSM": 0x764,
    "PSCM": 0x730,
    "TCM": 0x7E1,
    "CVM": 0x793,
  },
}

# Firmware versions for fingerprinting
FW_VERSIONS = {
  CAR.VOLVO_V40: {
    (Ecu.unknown, ECU_ADDRESS[CAR.VOLVO_V40]["CEM"], None): [
      b'31453061 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.eps, ECU_ADDRESS[CAR.VOLVO_V40]["PSCM"], None): [
      b'31288595 AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, ECU_ADDRESS[CAR.VOLVO_V40]["FSM"], None): [
      b'31400454 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}

# DBC mapping
DBC = CAR.create_dbc_map()
