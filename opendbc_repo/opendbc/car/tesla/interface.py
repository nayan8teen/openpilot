#!/usr/bin/env python3
from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "tesla"

    # Needs safety validation and final testing before pulling out of dashcam
    ret.dashcamOnly = True

    # Not merged yet
    #ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]
    #ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TESLA_LONG_CONTROL

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.openpilotLongitudinalControl = True

    return ret

  @staticmethod
  def _get_params_sp(ret_stock: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], experimental_long: bool, docs: bool) -> tuple[structs.CarParams, structs.CarParamsSP]:
    return ret_stock, ret
