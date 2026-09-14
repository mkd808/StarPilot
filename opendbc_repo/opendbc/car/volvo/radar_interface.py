from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import CarParams


class RadarInterface(RadarInterfaceBase):
  """Radar interface for Volvo vehicles.
  
  Volvo vehicles do not have radar objects available on CAN,
  so this is a no-op implementation.
  """
  
  def __init__(self, CP: CarParams):
    super().__init__(CP)
    # Volvo has no radar objects on CAN
    self.no_radar = True
