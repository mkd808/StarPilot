from opendbc.car.volvo.values import VolvoFlags


def cancelACC(packer, CP, CS):
  # Send cancel button to disengage ACC
  # TODO add support for EUCD
  msg = {}

  if CP.flags & VolvoFlags.C1:
    msg["ACCStopBtn"] = 1
  
  elif CP.flags & VolvoFlags.EUCD:
    msg["ACCOnOffBtn"] = 1
    msg["ACCOnOffBtnInv"] = 0

  return packer.make_can_msg("CCButtons", 0, msg)


def manipulateServo(packer, CP, CS):
  # Manipulate data from servo to FSM
  # Set LKATorque and LKAActive to zero otherwise LKA will be disabled. (Check dbc)
  msg = {
      "LKATorque" : 0,
      "SteeringAngleServo" : CS.PSCMInfo.SteeringAngleServo,
      "byte0" : CS.PSCMInfo.byte0,
      "byte4" : CS.PSCMInfo.byte4,
      "byte7" : CS.PSCMInfo.byte7,
  }

  if CP.flags & VolvoFlags.C1: 
    msg["LKAActive"] = CS.PSCMInfo.LKAActive & 0xFD
    msg["byte3"] = CS.PSCMInfo.byte3
  elif CP.flags & VolvoFlags.EUCD:
    msg["LKAActive"] = CS.PSCMInfo.LKAActive & 0xF5   # Filter out bit 1 and 3
    msg["SteeringWheelRateOfChange"] = CS.PSCMInfo.SteeringWheelRateOfChange

  return packer.make_can_msg("PSCM1", 2, msg)


def create_chksum(dat, CP):
  # Input: dat byte array, and fingerprint
  # Steering direction = 0 -> 3
  # TrqLim = 0 -> 255
  # Steering angle request = -360 -> 360
    
  # Extract LKAAngleRequest, LKADirection and Unknown
  if CP.flags & VolvoFlags.C1: 
    steer_angle_request = ((dat[4] & 0x3F) << 8) + dat[5]
    steering_direction_request = dat[7] & 0x03  
    trqlim = dat[3]
  elif CP.flags & VolvoFlags.EUCD:
    steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
    steering_direction_request = dat[5] & 0x03  
    trqlim = dat[2]
  
  # Sum of all bytes, carry ignored.
  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  # Checksum is inverted sum of all bytes
  return s ^ 0xFF


def create_steering_control(packer, frame, CP, SteerCommand, FSMInfo):  
 
  # Set common parameters
  values = {
    "LKAAngleReq": SteerCommand.angle_request,
    "LKASteerDirection": SteerCommand.steer_direction,
    "TrqLim": SteerCommand.trqlim,
  }
  
  # Set car specific parameters
  if CP.flags & VolvoFlags.C1:
    values_static = {
      "SET_X_E3": 0xE3,
      "SET_X_B4": 0xB4,
      "SET_X_08": 0x08,
      "SET_X_02": 0x02,
      "SET_X_25": 0x25,
    }
  elif CP.flags & VolvoFlags.EUCD:
    values_static = {
      "SET_X_22": 0x25, # Test these values: 0x24, 0x22
      "SET_X_02": 0,    # Test 0x00, 0x02
      "SET_X_10": 0x10, # Test 0x10, 0x1c, 0x18, 0x00
      "SET_X_A4": 0xa7, # Test 0xa4, 0xa6, 0xa5, 0xe5, 0xe7
      #"SET_X_22": FSMInfo.SET_X_22,
      #"SET_X_02": FSMInfo.SET_X_02,
      #"SET_X_10": FSMInfo.SET_X_10,
      #"SET_X_A4": FSMInfo.SET_X_A4,
    }
    # Which numbers stops lka? X_22? X_02? X_10? X_A4?
    # From working test to change one field at a time. When does it stop to work?
    # Do any of the changes make the CCP.STEER command work?

  # Combine common and static parameters
  values.update(values_static)

  # Create can message with "translated" can bytes.
  if CP.flags & VolvoFlags.C1:
    dat = packer.make_can_msg("FSM1", 0, values)[2]
  elif CP.flags & VolvoFlags.EUCD:
    dat = packer.make_can_msg("FSM2", 0, values)[2]

  values["Checksum"] = create_chksum(dat, CP)

  if CP.flags & VolvoFlags.C1:
    return packer.make_can_msg("FSM1", 0, values)
  elif CP.flags & VolvoFlags.EUCD:
    return packer.make_can_msg("FSM2", 0, values)
