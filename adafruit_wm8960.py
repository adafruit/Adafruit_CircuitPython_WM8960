# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT
"""
`adafruit_wm8960`
================================================================================

CircuitPython driver for WM8960 Stereo CODEC

* Author(s): Scott Shawcroft, Cooper Dalrymple

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_Arduino_Library

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_WM8960.git"

from busio import I2C
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_simplemath import constrain, map_range
import math
from micropython import const

try:
    from typing import Optional, Type, Protocol
    from circuitpython_typing.device_drivers import I2CDeviceDriver
except ImportError:
    pass

# I2C address
_DEFAULT_I2C_ADDR = const(0x1A)

# WM8960 register addresses
_REG_LEFT_INPUT_VOLUME = const(0x00)
_REG_RIGHT_INPUT_VOLUME = const(0x01)
_REG_LOUT1_VOLUME = const(0x02)
_REG_ROUT1_VOLUME = const(0x03)
_REG_CLOCKING_1 = const(0x04)
_REG_ADC_DAC_CTRL_1 = const(0x05)
_REG_ADC_DAC_CTRL_2 = const(0x06)
_REG_AUDIO_INTERFACE_1 = const(0x07)
_REG_CLOCKING_2 = const(0x08)
_REG_AUDIO_INTERFACE_2 = const(0x09)
_REG_LEFT_DAC_VOLUME = const(0x0A)
_REG_RIGHT_DAC_VOLUME = const(0x0B)
_REG_RESET = const(0x0F)
_REG_3D_CONTROL = const(0x10)
_REG_ALC1 = const(0x11)
_REG_ALC2 = const(0x12)
_REG_ALC3 = const(0x13)
_REG_NOISE_GATE = const(0x14)
_REG_LEFT_ADC_VOLUME = const(0x15)
_REG_RIGHT_ADC_VOLUME = const(0x16)
_REG_ADDITIONAL_CONTROL_1 = const(0x17)
_REG_ADDITIONAL_CONTROL_2 = const(0x18)
_REG_PWR_MGMT_1 = const(0x19)
_REG_PWR_MGMT_2 = const(0x1A)
_REG_ADDITIONAL_CONTROL_3 = const(0x1B)
_REG_ANTI_POP_1 = const(0x1C)
_REG_ANTI_POP_2 = const(0x1D)
_REG_ADCL_SIGNAL_PATH = const(0x20)
_REG_ADCR_SIGNAL_PATH = const(0x21)
_REG_LEFT_OUT_MIX = const(0x22)
_REG_RIGHT_OUT_MIX = const(0x25)
_REG_MONO_OUT_MIX_1 = const(0x26)
_REG_MONO_OUT_MIX_2 = const(0x27)
_REG_LOUT2_VOLUME = const(0x28)
_REG_ROUT2_VOLUME = const(0x29)
_REG_MONO_OUT_VOLUME = const(0x2A)
_REG_INPUT_BOOST_MIXER_1 = const(0x2B)
_REG_INPUT_BOOST_MIXER_2 = const(0x2C)
_REG_BYPASS_1 = const(0x2D)
_REG_BYPASS_2 = const(0x2E)
_REG_PWR_MGMT_3 = const(0x2F)
_REG_ADDITIONAL_CONTROL_4 = const(0x30)
_REG_CLASS_D_CONTROL_1 = const(0x31)
_REG_CLASS_D_CONTROL_3 = const(0x33)
_REG_PLL_N = const(0x34)
_REG_PLL_K_1 = const(0x35)
_REG_PLL_K_2 = const(0x36)
_REG_PLL_K_3 = const(0x37)

# PGA input selections
PGA_INPUT2 = 0
PGA_INPUT3 = 1
PGA_VMID = 2

# Mic (aka PGA) BOOST gain options
MIC_BOOST_GAIN_0DB = 0
MIC_BOOST_GAIN_13DB = 1
MIC_BOOST_GAIN_20DB = 2
MIC_BOOST_GAIN_29DB = 3

'''
Boost Mixer gain options
These are used to control the gain (aka volume) at the following settings:
LIN2BOOST
LIN3BOOST
RIN2BOOST
RIN3BOOST
'''
BOOST_MIXER_GAIN_MUTE = 0
BOOST_MIXER_GAIN_NEG_12DB = 1
BOOST_MIXER_GAIN_NEG_9DB = 2
BOOST_MIXER_GAIN_NEG_6DB = 3
BOOST_MIXER_GAIN_NEG_3DB = 4
BOOST_MIXER_GAIN_0DB = 5
BOOST_MIXER_GAIN_3DB = 6
BOOST_MIXER_GAIN_6DB = 7

# Mic Bias voltage options
MIC_BIAS_VOLTAGE_0_9_AVDD = 0
MIC_BIAS_VOLTAGE_0_65_AVDD = 1

# SYSCLK divide
_SYSCLK_DIV_BY_1 = const(0)
_SYSCLK_DIV_BY_2 = const(2)

# Gain/Level mins, maxes, offsets and step-sizes
_PGA_GAIN_MIN = -17.25
_PGA_GAIN_MAX = 30.00

_ADC_VOLUME_MIN = -97.00
_ADC_VOLUME_MAX = 30.00

_DAC_VOLUME_MIN = -127.00
_DAC_VOLUME_MAX = 0.00

_ALC_TARGET_MIN = -22.50
_ALC_TARGET_MAX = -1.50

_ALC_MAX_GAIN_MIN = -12.00
_ALC_MAX_GAIN_MAX = 30.00

_ALC_MIN_GAIN_MIN = -17.25
_ALC_MIN_GAIN_MAX = 24.75

_GATE_THRESHOLD_MIN = -76.50
_GATE_THRESHOLD_MAX = -30.00

_OUTPUT_VOLUME_MIN = -21.00
_OUTPUT_VOLUME_MAX = 0.00

_AMP_VOLUME_MIN = -73.00
_AMP_VOLUME_MAX = 6.00

# ALC Time mapping
_ALC_ATTACK_MAX = const(10)
_ALC_ATTACK_TIME_MIN = 0.006
_ALC_ATTACK_TIME_MAX = 6.140

_ALC_DECAY_MAX = const(10)
_ALC_DECAY_TIME_MIN = 0.024
_ALC_DECAY_TIME_MAX = 24.580

_ALC_HOLD_TIME_MIN = 0.00267
_ALC_HOLD_TIME_MAX = 43.691

# Speaker Boost Gains (DC and AC)
SPEAKER_BOOST_GAIN_0DB = 0
SPEAKER_BOOST_GAIN_2_1DB = 1
SPEAKER_BOOST_GAIN_2_9DB = 2
SPEAKER_BOOST_GAIN_3_6DB = 3
SPEAKER_BOOST_GAIN_4_5DB = 4
SPEAKER_BOOST_GAIN_5_1DB = 5

# VMIDSEL settings
VMIDSEL_DISABLED = 0
VMIDSEL_PLAYBACK = 1 # 2X50KOHM
VMIDSEL_LOWPOWER = 2 # 2X250KOHM
VMIDSEL_FASTSTART = 3 # 2X5KOHM

# Clock Divider Tables
_BCLKDIV = [
    1.0,
    1.5,
    2.0,
    3.0,
    4.0,
    5.5,
    6.0,
    8.0,
    11.0,
    12.0,
    16.0,
    22.0,
    24.0,
    32.0
]

_DCLKDIV = [
    1.5,
    2.0,
    3.0,
    4.0,
    6.0,
    8.0,
    12.0,
    16.0
]

_ADCDACDIV = [
    1.0,
    1.5,
    2.0,
    3.0,
    4.0,
    5.5,
    6.0
]

class WMBit:

    def __init__(
        self,
        register_address: int,
        bit: int,
    ) -> None:
        self.register_address = register_address
        self.bit = bit
        self.bit_mask = 1 << (bit % 8) # the bitmask *within* the byte!
        self.byte = 1 - (bit // 8) # the byte number within the buffer

    def _set(self, obj:Optional[I2CDeviceDriver], value:bool) -> None:
        if value:
            obj._registers[self.register_address][self.byte] |= self.bit_mask
        else:
            obj._registers[self.register_address][self.byte] &= ~self.bit_mask
    
    def reset(self, obj: Optional[I2CDeviceDriver]) -> None:
        if not hasattr(self, 'default'):
            self.default = self.__get__(obj)
        self._set(obj, self.default)

    def __get__(
        self,
        obj: Optional[I2CDeviceDriver],
        objtype: Optional[Type[I2CDeviceDriver]] = None,
    ) -> bool:
        return bool(obj._registers[self.register_address][self.byte] & self.bit_mask)

    def __set__(self, obj: Optional[I2CDeviceDriver], value: bool) -> None:
        self._set(obj, value)
        with obj.i2c_device as i2c:
            i2c.write(obj._registers[self.register_address])

class WMBits:

    def __init__(  # pylint: disable=too-many-arguments
        self,
        num_bits: int,
        register_address: int,
        lowest_bit: int,
    ) -> None:
        self.register_width = 1
        self.register_address = register_address
        self.bit_mask = ((1 << num_bits) - 1) << lowest_bit
        self.lowest_bit = lowest_bit

    def _set(self, obj:Optional[I2CDeviceDriver], value:int) -> None:
        value <<= self.lowest_bit  # shift the value over to the right spot
        reg = 0
        for i in range(2):
            reg = (reg << 8) | obj._registers[self.register_address][i]
        reg &= ~self.bit_mask  # mask off the bits we're about to change
        reg |= value  # then or in our new value
        for i in range(1, -1, -1):
            obj._registers[self.register_address][i] = reg & 0xFF
            reg >>= 8
    
    # Must call first before using object
    def reset(self, obj:Optional[I2CDeviceDriver]) -> None:
        if not hasattr(self, 'default'):
            self.default = self.__get__(obj)
        self._set(obj, self.default)

    def __get__(
        self,
        obj: Optional[I2CDeviceDriver],
        objtype: Optional[Type[I2CDeviceDriver]] = None,
    ) -> int:
        # read the number of bytes into a single variable
        reg = 0
        for i in range(2):
            reg = (reg << 8) | obj._registers[self.register_address][i]
        reg = (reg & self.bit_mask) >> self.lowest_bit
        return reg

    def __set__(self, obj: Optional[I2CDeviceDriver], value: int) -> None:
        self._set(obj, value)
        with obj.i2c_device as i2c:
            i2c.write(obj._registers[self.register_address])

class WM8960:

    # Power

    vref = WMBit(_REG_PWR_MGMT_1, 6)

    analogInputLeftEnabled = WMBit(_REG_PWR_MGMT_1, 5)
    analogInputRightEnabled = WMBit(_REG_PWR_MGMT_1, 4)

    @property
    def analogInputEnabled(self) -> bool:
        return self.analogInputLeftEnabled and self.analogInputRightEnabled
    @analogInputEnabled.setter
    def analogInputEnabled(self, value:bool) -> None:
        self.analogInputLeftEnabled = self.analogInputRightEnabled = value

    # PGA
    
    ## PWR_MGMT

    pgaLeftEnabled = WMBit(_REG_PWR_MGMT_3, 5)
    pgaRightEnabled = WMBit(_REG_PWR_MGMT_3, 4)

    @property
    def pgaEnabled(self) -> bool:
        return self.pgaLeftEnabled and self.pgaRightEnabled
    @pgaEnabled.setter
    def pgaEnabled(self, value:bool) -> None:
        self.pgaLeftEnabled = self.pgaRightEnabled = value

    ## SIGNAL_PATH

    _pgaInput1Left = WMBit(_REG_ADCL_SIGNAL_PATH, 8)
    _pgaInput2Left = WMBit(_REG_ADCL_SIGNAL_PATH, 6)
    _pgaInput3Left = WMBit(_REG_ADCL_SIGNAL_PATH, 7)

    _pgaInput1Right = WMBit(_REG_ADCR_SIGNAL_PATH, 8)
    _pgaInput2Right = WMBit(_REG_ADCR_SIGNAL_PATH, 6)
    _pgaInput3Right = WMBit(_REG_ADCR_SIGNAL_PATH, 7)

    @property
    def pgaNonInvSignalLeft(self) -> int:
        if self._pgaInput2Left:
            return PGA_INPUT2
        elif self._pgaInput3Left:
            return PGA_INPUT3
        else:
            return PGA_VMID
    @pgaNonInvSignalLeft.setter
    def pgaNonInvSignalLeft(self, signal:int) -> None:
        self._pgaInput2Left = signal == PGA_INPUT2
        self._pgaInput3Left = signal == PGA_INPUT3
    
    @property
    def pgaNonInvSignalRight(self) -> int:
        if self._pgaInput2Right:
            return PGA_INPUT2
        elif self._pgaInput3Right:
            return PGA_INPUT3
        else:
            return PGA_VMID
    @pgaNonInvSignalRight.setter
    def pgaNonInvSignalRight(self, signal:int) -> None:
        self._pgaInput2Right = signal == PGA_INPUT2
        self._pgaInput3Right = signal == PGA_INPUT3

    @property
    def pgaNonInvSignal(self) -> int:
        # NOTE: Not checking right signal
        return self.pgaNonInvSignalLeft
    @pgaNonInvSignal.setter
    def pgaNonInvSignal(self, signal:int) -> None:
        self.pgaNonInvSignalLeft = self.pgaNonInvSignalRight = signal

    ## Boost

    pgaLeftBoostEnabled = WMBit(_REG_ADCL_SIGNAL_PATH, 3)
    pgaRightBoostEnabled = WMBit(_REG_ADCR_SIGNAL_PATH, 3)

    @property
    def pgaBoostEnabled(self) -> None:
        return self.pgaLeftBoostEnabled and self.pgaRightBoostEnabled
    @pgaBoostEnabled.setter
    def pgaBoostEnabled(self, value:int) -> None:
        self.pgaLeftBoostEnabled = self.pgaRightBoostEnabled = value

    pgaBoostGainLeft = WMBits(2, _REG_ADCL_SIGNAL_PATH, 4)
    pgaBoostGainRight = WMBits(2, _REG_ADCR_SIGNAL_PATH, 4)

    @property
    def pgaBoostGain(self) -> None:
        return max(self.pgaBoostGainLeft, self.pgaBoostGainRight)
    @pgaBoostGain.setter
    def pgaBoostGain(self, value:int) -> None:
        self.pgaGainLeft = self.pgaGainRight = value

    ## Volume

    _pgaLeftVolume = WMBits(6, _REG_LEFT_INPUT_VOLUME, 0)
    _pgaLeftVolumeSet = WMBit(_REG_LEFT_INPUT_VOLUME, 8)

    _pgaRightVolume = WMBits(6, _REG_RIGHT_INPUT_VOLUME, 0)
    _pgaRightVolumeSet = WMBit(_REG_RIGHT_INPUT_VOLUME, 8)

    @property
    def pgaLeftVolume(self) -> int:
        return self._pgaLeftVolume
    @pgaLeftVolume.setter
    def pgaLeftVolume(self, value:int) -> None:
        self._pgaLeftVolume = value
        self._pgaLeftVolumeSet = True

    @property
    def pgaLeftVolumeDb(self) -> float:
        return map_range(self.pgaLeftVolume, 0, 63, _PGA_GAIN_MIN, _PGA_GAIN_MAX)
    @pgaLeftVolumeDb.setter
    def pgaLeftVolumeDb(self, value:float) -> None:
        self.pgaLeftVolume = round(map_range(value, _PGA_GAIN_MIN, _PGA_GAIN_MAX, 0, 63))
    
    @property
    def pgaRightVolume(self) -> int:
        return self._pgaRightVolume
    @pgaRightVolume.setter
    def pgaRightVolume(self, value:int) -> None:
        self._pgaRightVolume = value
        self._pgaRightVolumeSet = True

    @property
    def pgaRightVolumeDb(self) -> float:
        return map_range(self.pgaRightVolume, 0, 63, _PGA_GAIN_MIN, _PGA_GAIN_MAX)
    @pgaRightVolumeDb.setter
    def pgaRightVolumeDb(self, value:float) -> None:
        self.pgaRightVolume = round(map_range(value, _PGA_GAIN_MIN, _PGA_GAIN_MAX, 0, 63))
    
    @property
    def pgaVolume(self) -> int:
        return self.pgaLeftVolume
    @pgaVolume.setter
    def pgaVolume(self, value:int) -> None:
        self.pgaLeftVolume = self.pgaRightVolume = value

    @property
    def pgaVolumeDb(self) -> float:
        return self.pgaLeftVolumeDb
    @pgaVolumeDb.setter
    def pgaVolumeDb(self, value:float) -> None:
        self.pgaVolume = round(map_range(value, _PGA_GAIN_MIN, _PGA_GAIN_MAX, 0, 63))

    ## Zero Cross

    pgaLeftZeroCross = WMBit(_REG_LEFT_INPUT_VOLUME, 6)
    pgaRightZeroCross = WMBit(_REG_RIGHT_INPUT_VOLUME, 6)

    @property
    def pgaZeroCross(self) -> bool:
        return self.pgaLeftZeroCross and self.pgaRightZeroCross
    @pgaZeroCross.setter
    def pgaZeroCross(self, value:bool) -> None:
        self.pgaLeftZeroCross = self.pgaRightZeroCross = value

    ## Mute

    _pgaLeftMute = WMBit(_REG_LEFT_INPUT_VOLUME, 7)
    _pgaRightMute = WMBit(_REG_RIGHT_INPUT_VOLUME, 7)

    @property
    def pgaLeftMute(self) -> bool:
        return self._pgaLeftMute
    @pgaLeftMute.setter
    def pgaLeftMute(self, value:bool) -> None:
        self._pgaLeftMute = value
        self._pgaLeftVolumeSet = True

    @property
    def pgaRightMute(self) -> bool:
        return self._pgaRightMute
    @pgaRightMute.setter
    def pgaRightMute(self, value:bool) -> None:
        self._pgaRightMute = value
        self._pgaRightVolumeSet = True

    @property
    def pgaMute(self) -> bool:
        return self._pgaLeftMute and self._pgaRightMute
    @pgaMute.setter
    def pgaMute(self, value:bool) -> None:
        self._pgaLeftMute = self._pgaRightMute = value

    # Boost Mixer

    input2LeftBoost = WMBits(3, _REG_INPUT_BOOST_MIXER_1, 1)
    input2RightBoost = WMBits(3, _REG_INPUT_BOOST_MIXER_2, 1)

    @property
    def input2Boost(self) -> int:
        return self.input2LeftBoost
    @input2Boost.setter
    def input2Boost(self, value:int) -> None:
        self.input2LeftBoost = self.input2RightBoost = value

    input3LeftBoost = WMBits(3, _REG_INPUT_BOOST_MIXER_1, 4)
    input3RightBoost = WMBits(3, _REG_INPUT_BOOST_MIXER_2, 4)

    # Mic Bias

    micBias = WMBit(_REG_PWR_MGMT_1, 1)
    micBiasVoltage = WMBit(_REG_ADDITIONAL_CONTROL_4, 0)

    @property
    def input3Boost(self) -> int:
        return self.input3LeftBoost
    @input3Boost.setter
    def input3Boost(self, value:int) -> None:
        self.input3LeftBoost = self.input3RightBoost = value

    # ADC

    adcLeftEnabled = WMBit(_REG_PWR_MGMT_1, 3)
    adcRightEnabled = WMBit(_REG_PWR_MGMT_1, 2)

    @property
    def adcEnabled(self) -> bool:
        return self.adcLeftEnabled and self.adcRightEnabled
    @adcEnabled.setter
    def adcEnabled(self, value:bool) -> None:
        self.adcLeftEnabled = self.adcRightEnabled = value
    
    ## Volume

    _adcLeftVolume = WMBits(8, _REG_LEFT_ADC_VOLUME, 0)
    _adcLeftVolumeSet = WMBit(_REG_LEFT_ADC_VOLUME, 8)

    @property
    def adcLeftVolume(self) -> int:
        return self._adcLeftVolume
    @adcLeftVolume.setter
    def adcLeftVolume(self, value:int) -> None:
        self._adcLeftVolume = value
        self._adcLeftVolumeSet = True

    @property
    def adcLeftVolumeDb(self) -> float:
        return map_range(max(self.adcLeftVolume, 1), 1, 255, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX)
    @adcLeftVolumeDb.setter
    def adcLeftVolumeDb(self, value:float) -> None:
        self.adcLeftVolume = round(map_range(value, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX, 0, 254) + 1.0)

    _adcRightVolume = WMBits(8, _REG_RIGHT_ADC_VOLUME, 0)
    _adcRightVolumeSet = WMBit(_REG_RIGHT_ADC_VOLUME, 8)

    @property
    def adcRightVolume(self) -> int:
        return self._adcRightVolume
    @adcRightVolume.setter
    def adcRightVolume(self, value:int) -> None:
        self._adcRightVolume = value
        self._adcRightVolumeSet = True

    @property
    def adcRightVolumeDb(self) -> float:
        return map_range(max(self.adcRightVolume, 1), 1, 255, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX)
    @adcRightVolumeDb.setter
    def adcRightVolumeDb(self, value:float) -> None:
        self.adcRightVolume = round(map_range(value, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX, 0, 254) + 1.0)

    @property
    def adcVolume(self) -> int:
        return self.adcLeftVolume
    @adcVolume.setter
    def adcVolume(self, value:float) -> None:
        self.adcLeftVolume = self.adcRightVolume = value
    
    @property
    def adcVolumeDb(self) -> int:
        return self.adcLeftVolumeDb
    @adcVolume.setter
    def adcVolume(self, value:float) -> None:
        self.adcLeftVolume = self.adcRightVolume = round(map_range(value, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX, 0, 254) + 1.0)

    # ALC

    alcLeftEnabled = WMBit(_REG_ALC1, 7)
    alcRightEnabled = WMBit(_REG_ALC1, 8)

    @property
    def alcEnabled(self) -> bool:
        return self.alcLeftEnabled and self.alcRightEnabled
    @alcEnabled.setter
    def alcEnabled(self, value:bool) -> None:
        self.alcLeftEnabled = self.alcRightEnabled = value

    alcTarget = WMBits(4, _REG_ALC1, 0)

    @property
    def alcTargetDb(self) -> float:
        return map_range(self.alcTarget, 0, 15, _ALC_TARGET_MIN, _ALC_TARGET_MAX)
    @alcTargetDb.setter
    def alcTargetDb(self, value:float) -> None:
        self.alcTarget = round(map_range(value, _ALC_TARGET_MIN, _ALC_TARGET_MAX, 0, 15))

    alcMaxGain = WMBits(3, _REG_ALC1, 4)

    @property
    def alcMaxGainDb(self) -> float:
        return map_range(self.alcMaxGain, 0, 7, _ALC_MAX_GAIN_MIN, _ALC_MAX_GAIN_MAX)
    @alcMaxGainDb.setter
    def alcMaxGainDb(self, value:float) -> None:
        self.alcMaxGain = round(map_range(value, _ALC_MAX_GAIN_MIN, _ALC_MAX_GAIN_MAX, 0, 7))

    alcMinGain = WMBits(3, _REG_ALC2, 4)

    @property
    def alcMinGainDb(self) -> float:
        return map_range(self.alcMinGain, 0, 7, _ALC_MIN_GAIN_MIN, _ALC_MIN_GAIN_MAX)
    @alcMinGainDb.setter
    def alcMinGainDb(self, value:float) -> None:
        self.alcMinGain = round(map_range(value, _ALC_MIN_GAIN_MIN, _ALC_MIN_GAIN_MAX, 0, 7))

    _alcAttack = WMBits(4, _REG_ALC3, 0)

    @property
    def alcAttack(self) -> int:
        return self._alcAttack
    @alcAttack.setter
    def alcAttack(self, value:int) -> None:
        self._alcAttack = min(value, _ALC_ATTACK_MAX)
    
    @property
    def alcAttackTime(self) -> float:
        return _ALC_ATTACK_TIME_MIN * pow(2, self.alcAttack)
    @alcAttackTime.setter
    def alcAttackTime(self, value:float) -> None:
        self.alcAttack = round(math.log2((constrain(value, _ALC_ATTACK_TIME_MIN, _ALC_ATTACK_TIME_MAX) - _ALC_ATTACK_TIME_MIN) / _ALC_ATTACK_TIME_MIN))

    _alcDecay = WMBits(4, _REG_ALC3, 4)

    @property
    def alcDecay(self) -> int:
        return self._alcDecay
    @alcDecay.setter
    def alcDecay(self, value:int) -> None:
        self._alcDecay = min(value, _ALC_DECAY_MAX)

    @property
    def alcDecayTime(self) -> float:
        return _ALC_DECAY_TIME_MIN * pow(2, self.alcDecay)
    @alcDecayTime.setter
    def alcDecayTime(self, value:float) -> None:
        self.alcDecay = round(math.log2((constrain(value, _ALC_DECAY_TIME_MIN, _ALC_DECAY_TIME_MAX) - _ALC_DECAY_TIME_MIN) / _ALC_DECAY_TIME_MIN))

    alcHold = WMBits(4, _REG_ALC2, 0)

    @property
    def alcHoldTime(self) -> float:
        value = self.alcHold
        if value == 0: return 0.0
        return _ALC_HOLD_TIME_MIN * pow(2, self.alcHold - 1)
    @alcHoldTime.setter
    def alcHoldTime(self, value:float) -> None:
        if value <= 0.0:
            self.alcHold = 0
        else:
            self.alcHold = round(math.log2((constrain(value, _ALC_HOLD_TIME_MIN, _ALC_HOLD_TIME_MAX) - _ALC_HOLD_TIME_MIN) / _ALC_HOLD_TIME_MIN) + 1.0)

    alcLimiter = WMBit(_REG_ALC3, 8)

    # Noise Gate

    noiseGateEnabled = WMBit(_REG_NOISE_GATE, 0)
    noiseGateThreshold = WMBits(5, _REG_NOISE_GATE, 3)

    @property
    def noiseGateThresholdDb(self) -> float:
        return map_range(self.noiseGateThreshold, 0, 31, _GATE_THRESHOLD_MIN, _GATE_THRESHOLD_MAX)
    @noiseGateThresholdDb.setter
    def noiseGateThresholdDb(self, value:float) -> None:
        self.noiseGateThreshold = round(map_range(value, _GATE_THRESHOLD_MIN, _GATE_THRESHOLD_MAX, 0, 31))

    # DAC

    dacLeftEnabled = WMBit(_REG_PWR_MGMT_2, 8)
    dacRightEnabled = WMBit(_REG_PWR_MGMT_2, 7)

    @property
    def dacEnabled(self) -> bool:
        return self.dacLeftEnabled and self.dacRightEnabled
    @dacEnabled.setter
    def dacEnabled(self, value:bool) -> None:
        self.dacLeftEnabled = self.dacRightEnabled = value
    
    _dacLeftVolume = WMBits(8, _REG_LEFT_DAC_VOLUME, 0)
    _dacLeftVolumeSet = WMBit(_REG_LEFT_DAC_VOLUME, 8)

    @property
    def dacLeftVolume(self) -> int:
        return self._dacLeftVolume
    @dacLeftVolume.setter
    def dacLeftVolume(self, value:int) -> None:
        self._dacLeftVolume = value
        self._dacLeftVolumeSet = True

    @property
    def dacLeftVolumeDb(self) -> float:
        return map_range(max(self.dacLeftVolume, 1), 1, 255, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX)
    @dacLeftVolumeDb.setter
    def dacLeftVolumeDb(self, value:float) -> None:
        self.dacLeftVolume = round(map_range(value, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX, 0, 254) + 1.0)

    _dacRightVolume = WMBits(8, _REG_RIGHT_DAC_VOLUME, 0)
    _dacRightVolumeSet = WMBit(_REG_RIGHT_DAC_VOLUME, 8)

    @property
    def dacRightVolume(self) -> int:
        return self._dacRightVolume
    @dacRightVolume.setter
    def dacRightVolume(self, value:int) -> None:
        self._dacRightVolume = value
        self._dacRightVolumeSet = True

    @property
    def dacRightVolumeDb(self) -> float:
        return map_range(max(self.dacRightVolume, 1), 1, 255, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX)
    @dacRightVolumeDb.setter
    def dacRightVolumeDb(self, value:float) -> None:
        self.dacRightVolume = round(map_range(value, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX, 0, 254) + 1.0)

    @property
    def dacVolume(self) -> int:
        return self.dacLeftVolume
    @dacVolume.setter
    def dacVolume(self, value:float) -> None:
        self.dacLeftVolume = self.dacRightVolume = value
    
    @property
    def dacVolumeDb(self) -> int:
        return self.dacLeftVolumeDb
    @dacVolume.setter
    def dacVolume(self, value:float) -> None:
        self.dacLeftVolume = self.dacRightVolume = round(map_range(value, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX, 0, 254) + 1.0)

    dacMute = WMBit(_REG_ADC_DAC_CTRL_1, 3)
    dacSoftMute = WMBit(_REG_ADC_DAC_CTRL_2, 3)
    dacSlowSoftMute = WMBit(_REG_ADC_DAC_CTRL_2, 2)
    dacAttenuation = WMBit(_REG_ADC_DAC_CTRL_1, 7)

    # 3D Enhance

    enhanceEnabled = WMBit(_REG_3D_CONTROL, 0)
    enhanceDepth = WMBits(4, _REG_3D_CONTROL, 1)
    enhanceFilterLPF = WMBit(_REG_3D_CONTROL, 6)
    enhanceFilterHPF = WMBit(_REG_3D_CONTROL, 5)

    # Output Mixer

    leftOutputEnabled = WMBit(_REG_PWR_MGMT_3, 3)
    rightOutputEnabled = WMBit(_REG_PWR_MGMT_3, 2)

    @property
    def stereoOutputEnabled(self) -> bool:
        return self.leftOutputEnabled and self.rightOutputEnabled
    @stereoOutputEnabled.setter
    def stereoOutputEnabled(self, value:bool):
        self.leftOutputEnabled = self.rightOutputEnabled = value

    ## DAC Output

    dacLeftOutputEnabled = WMBit(_REG_LEFT_OUT_MIX, 8)
    dacRightOutputEnabled = WMBit(_REG_RIGHT_OUT_MIX, 8)

    @property
    def dacOutputEnabled(self) -> bool:
        return self.dacLeftOutputEnabled and self.dacRightOutputEnabled
    @dacOutputEnabled.setter
    def dacOutputEnabled(self, value:bool) -> None:
        self.dacLeftOutputEnabled = self.dacRightOutputEnabled = value

    ## Input 3 Output

    input3LeftOutputEnabled = WMBit(_REG_LEFT_OUT_MIX, 7)
    input3RightOutputEnabled = WMBit(_REG_RIGHT_OUT_MIX, 7)
    
    @property
    def input3OutputEnabled(self) -> bool:
        return self.input3LeftOutputEnabled and self.input3RightOutputEnabled
    @input3OutputEnabled.setter
    def input3OutputEnabled(self, value:bool) -> None:
        self.input3LeftOutputEnabled = self.input3RightOutputEnabled = value

    input3LeftOutputVolume = WMBits(3, _REG_LEFT_OUT_MIX, 4)

    @property
    def input3LeftOutputVolumeDb(self) -> float:
        return map_range(self.input3LeftOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @input3LeftOutputVolumeDb.setter
    def input3LeftOutputVolumeDb(self, value:float) -> None:
        self.input3LeftOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    input3RightOutputVolume = WMBits(3, _REG_RIGHT_OUT_MIX, 4)

    @property
    def input3RightOutputVolumeDb(self) -> float:
        return map_range(self.input3RightOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @input3RightOutputVolumeDb.setter
    def input3RightOutputVolumeDb(self, value:float) -> None:
        self.input3RightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    @property
    def input3OutputVolume(self) -> int:
        return self.input3LeftOutputVolume
    @input3OutputVolume.setter
    def input3OutputVolume(self, value:int) -> None:
        self.input3LeftOutputVolume = self.input3RightOutputVolume = value

    @property
    def input3OutputVolumeDb(self) -> float:
        return self.input3LeftOutputVolumeDb
    @input3OutputVolumeDb.setter
    def input3OutputVolumeDb(self, value:float) -> None:
        self.input3LeftOutputVolume = self.input3RightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    ## PGA Boost Mixer Output

    pgaBoostLeftOutputEnabled = WMBit(_REG_BYPASS_1, 7)
    pgaBoostRightOutputEnabled = WMBit(_REG_BYPASS_2, 7)

    @property
    def pgaBoostOutputEnabled(self) -> bool:
        return self.pgaBoostLeftOutputEnabled and self.pgaBoostRightOutputEnabled

    pgaBoostLeftOutputVolume = WMBits(3, _REG_BYPASS_1, 4)

    @property
    def pgaBoostLeftOutputVolumeDb(self) -> float:
        return map_range(self.pgaBoostLeftOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @pgaBoostLeftOutputVolumeDb.setter
    def pgaBoostLeftOutputVolumeDb(self, value:float) -> None:
        self.pgaBoostLeftOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    pgaBoostRightOutputVolume = WMBits(3, _REG_BYPASS_2, 4)

    @property
    def pgaBoostRightOutputVolumeDb(self) -> float:
        return map_range(self.pgaBoostRightOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @pgaBoostRightOutputVolumeDb.setter
    def pgaBoostRightOutputVolumeDb(self, value:float) -> None:
        self.pgaBoostRightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    @property
    def pgaBoostOutputVolume(self) -> int:
        return self.pgaBoostLeftOutputVolume
    @pgaBoostOutputVolume.setter
    def pgaBoostOutputVolume(self, value:int) -> None:
        self.pgaBoostLeftOutputVolume = self.pgaBoostRightOutputVolume = value

    @property
    def pgaBoostOutputVolumeDb(self) -> float:
        return self.pgaBoostLeftOutputVolumeDb
    @pgaBoostOutputVolumeDb.setter
    def pgaBoostOutputVolumeDb(self, value:float) -> None:
        self.pgaBoostLeftOutputVolume = self.pgaBoostRightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    ## Mono Output

    monoOutputEnabled = WMBit(_REG_PWR_MGMT_2, 1)

    monoLeftMixEnabled = WMBit(_REG_MONO_OUT_MIX_1, 7)
    monoRightMixEnabled = WMBit(_REG_MONO_OUT_MIX_2, 7)

    @property
    def monoMixEnabled(self) -> bool:
        return self.monoLeftMixEnabled and self.monoRightMixEnabled
    @monoMixEnabled.setter
    def monoMixEnabled(self, value:bool) -> None:
        self.monoLeftMixEnabled = self.monoRightMixEnabled = value

    monoOutputAttenuation = WMBit(_REG_MONO_OUT_VOLUME, 6)

    vmid = WMBits(2, _REG_PWR_MGMT_1, 7)

    # Amplifier

    ## Headphones

    leftHeadphoneEnabled = WMBit(_REG_PWR_MGMT_2, 5)
    rightHeadphoneEnabled = WMBit(_REG_PWR_MGMT_2, 6)

    @property
    def headphoneEnabled(self) -> bool:
        return self.leftHeadphoneEnabled and self.rightHeadphoneEnabled
    @headphoneEnabled.setter
    def headphoneEnabled(self, value:bool) -> None:
        self.leftHeadphoneEnabled = self.rightHeadphoneEnabled = value

    headphoneStandby = WMBit(_REG_ANTI_POP_1, 0)

    _leftHeadphoneVolume = WMBits(7, _REG_LOUT1_VOLUME, 0)
    _leftHeadphoneVolumeSet = WMBit(_REG_LOUT1_VOLUME, 8)

    @property
    def leftHeadphoneVolume(self) -> int:
        return self._leftHeadphoneVolume
    @leftHeadphoneVolume.setter
    def leftHeadphoneVolume(self, value:int) -> None:
        self._leftHeadphoneVolume = value
        self._leftHeadphoneVolumeSet = True

    @property
    def leftHeadphoneVolumeDb(self) -> float:
        return map_range(max(self.leftHeadphoneVolume, 48), 48, 127, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @leftHeadphoneVolumeDb.setter
    def leftHeadphoneVolumeDb(self, value:float) -> None:
        self.leftHeadphoneVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    _rightHeadphoneVolume = WMBits(7, _REG_ROUT1_VOLUME, 0)
    _rightHeadphoneVolumeSet = WMBit(_REG_ROUT1_VOLUME, 8)

    @property
    def rightHeadphoneVolume(self) -> int:
        return self._rightHeadphoneVolume
    @rightHeadphoneVolume.setter
    def rightHeadphoneVolume(self, value:int) -> None:
        self._rightHeadphoneVolume = value
        self._rightHeadphoneVolumeSet = True

    @property
    def rightHeadphoneVolumeDb(self) -> float:
        return map_range(max(self.rightHeadphoneVolume, 48), 48, 255, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @rightHeadphoneVolumeDb.setter
    def rightHeadphoneVolumeDb(self, value:float) -> None:
        self.rightHeadphoneVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    @property
    def headphoneVolume(self) -> int:
        return self.leftHeadphoneVolume
    @headphoneVolume.setter
    def headphoneVolume(self, value:float) -> None:
        self.leftHeadphoneVolume = self.rightHeadphoneVolume = value
    
    @property
    def headphoneVolumeDb(self) -> int:
        return self.leftHeadphoneVolumeDb
    @headphoneVolumeDb.setter
    def headphoneVolumeDb(self, value:float) -> None:
        self.leftHeadphoneVolume = self.rightHeadphoneVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127) + 1.0)

    leftHeadphoneZeroCross = WMBit(_REG_LOUT1_VOLUME, 7)
    rightHeadphoneZeroCross = WMBit(_REG_LOUT1_VOLUME, 7)

    @property
    def headphoneZeroCross(self) -> bool:
        return self.leftHeadphoneZeroCross and self.rightHeadphoneZeroCross
    @headphoneZeroCross.setter
    def headphoneZeroCross(self, value:bool) -> None:
        self.leftHeadphoneZeroCross = self.rightHeadphoneZeroCross = value

    ## Speakers

    _leftSpeakerEnabled = WMBit(_REG_PWR_MGMT_2, 4)
    _leftSpeakerAmpEnabled = WMBit(_REG_CLASS_D_CONTROL_1, 6)

    @property
    def leftSpeakerEnabled(self) -> bool:
        return self._leftSpeakerEnabled and self._leftSpeakerAmpEnabled
    @leftSpeakerEnabled.setter
    def leftSpeakerEnabled(self, value:bool) -> None:
        self._leftSpeakerEnabled = self._leftSpeakerAmpEnabled = value
    
    _rightSpeakerEnabled = WMBit(_REG_PWR_MGMT_2, 3)
    _rightSpeakerAmpEnabled = WMBit(_REG_CLASS_D_CONTROL_1, 7)

    @property
    def rightSpeakerEnabled(self) -> bool:
        return self._rightSpeakerEnabled and self._rightSpeakerAmpEnabled
    @rightSpeakerEnabled.setter
    def rightSpeakerEnabled(self, value:bool) -> None:
        self._rightSpeakerEnabled = self._rightSpeakerAmpEnabled = value

    @property
    def speakerEnabled(self) -> bool:
        return self.leftSpeakerEnabled and self.rightSpeakerEnabled
    @speakerEnabled.setter
    def speakerEnabled(self, value:bool) -> None:
        self.leftSpeakerEnabled = self.rightSpeakerEnabled = value

    _leftSpeakerVolume = WMBits(7, _REG_LOUT2_VOLUME, 0)
    _leftSpeakerVolumeSet = WMBit(_REG_LOUT2_VOLUME, 8)

    @property
    def leftSpeakerVolume(self) -> int:
        return self._leftSpeakerVolume
    @leftSpeakerVolume.setter
    def leftSpeakerVolume(self, value:int) -> None:
        self._leftSpeakerVolume = value
        self._leftSpeakerVolumeSet = True

    @property
    def leftSpeakerVolumeDb(self) -> float:
        return map_range(max(self.leftSpeakerVolume, 48), 48, 127, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @leftSpeakerVolumeDb.setter
    def leftSpeakerVolumeDb(self, value:float) -> None:
        self.leftSpeakerVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    _rightSpeakerVolume = WMBits(7, _REG_ROUT2_VOLUME, 0)
    _rightSpeakerVolumeSet = WMBit(_REG_ROUT2_VOLUME, 8)

    @property
    def rightSpeakerVolume(self) -> int:
        return self._rightSpeakerVolume
    @rightSpeakerVolume.setter
    def rightSpeakerVolume(self, value:int) -> None:
        self._rightSpeakerVolume = value
        self._rightSpeakerVolumeSet = True

    @property
    def rightSpeakerVolumeDb(self) -> float:
        return map_range(max(self.rightSpeakerVolume, 48), 48, 255, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @rightSpeakerVolumeDb.setter
    def rightSpeakerVolumeDb(self, value:float) -> None:
        self.rightSpeakerVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    @property
    def speakerVolume(self) -> int:
        return self.leftSpeakerVolume
    @speakerVolume.setter
    def speakerVolume(self, value:float) -> None:
        self.leftSpeakerVolume = self.rightSpeakerVolume = value
    
    @property
    def speakerVolumeDb(self) -> int:
        return self.leftSpeakerVolumeDb
    @speakerVolume.setter
    def speakerVolume(self, value:float) -> None:
        self.leftSpeakerVolume = self.rightSpeakerVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127) + 1.0)

    leftSpeakerZeroCross = WMBit(_REG_LOUT2_VOLUME, 7)
    rightSpeakerZeroCross = WMBit(_REG_LOUT2_VOLUME, 7)

    @property
    def speakerZeroCross(self) -> bool:
        return self.leftSpeakerZeroCross and self.rightSpeakerZeroCross
    @speakerZeroCross.setter
    def speakerZeroCross(self, value:bool) -> None:
        self.leftSpeakerZeroCross = self.rightSpeakerZeroCross = value

    _speakerDcGain = WMBits(3, _REG_CLASS_D_CONTROL_3, 3)

    @property
    def speakerDcGain(self) -> int:
        return self._speakerDcGain
    @speakerDcGain.setter
    def speakerDcGain(self, value:int) -> None:
        self._speakerDcGain = min(value, 5)

    _speakerAcGain = WMBits(3, _REG_CLASS_D_CONTROL_3, 0)

    @property
    def speakerAcGain(self) -> int:
        return self._speakerAcGain
    @speakerAcGain.setter
    def speakerAcGain(self, value:int) -> None:
        self._speakerAcGain = min(value, 5)

    # Digital Audio Interface Control

    loopback = WMBit(_REG_AUDIO_INTERFACE_2, 0)
    
    pll = WMBit(_REG_PWR_MGMT_2, 0)
    pllPrescaleDiv2 = WMBit(_REG_PWR_MGMT_2, 4)
    pllN = WMBits(4, _REG_PLL_N, 0)

    _pllK1 = WMBits(6, _REG_PLL_K_1, 0)
    _pllK2 = WMBits(9, _REG_PLL_K_2, 0)
    _pllK3 = WMBits(9, _REG_PLL_K_3, 0)

    @property
    def pllK(self) -> int:
        return self._pllK1 << 18 + self._pllK2 << 9 + self._pllK3
    @pllK.setter
    def pllK(self, value:int) -> None:
        self._pllK1 = (value >> 18) & 0b111111
        self._pllK2 = (value >> 9) & 0b111111111
        self._pllK3 = value & 0b111111111

    clockFractionalMode = WMBit(_REG_PLL_N, 5)

    clockFromPLL = WMBit(_REG_CLOCKING_1, 0)

    _systemClockDivider = WMBits(2, _REG_CLOCKING_1, 1)
    
    @property
    def systemClockDiv2(self) -> bool:
        return self._systemClockDivider == _SYSCLK_DIV_BY_2
    @systemClockDiv2.setter
    def systemClockDiv2(self, value:bool) -> None:
        self._systemClockDivider = _SYSCLK_DIV_BY_2 if value else _SYSCLK_DIV_BY_1
    
    _adcClockDivider = WMBits(3, _REG_CLOCKING_1, 6)

    @property
    def adcClockDivider(self) -> int:
        return _ADCDACDIV[min(self._adcClockDivider, len(_ADCDACDIV))]
    @adcClockDivider.setter
    def adcClockDivider(self, value:int) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._adcClockDivider = _ADCDACDIV.index(value)

    _dacClockDivider = WMBits(3, _REG_CLOCKING_1, 3)

    @property
    def dacClockDivider(self) -> int:
        return _ADCDACDIV[min(self._dacClockDivider, len(_ADCDACDIV))]
    @dacClockDivider.setter
    def dacClockDivider(self, value:int) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._dacClockDivider = _ADCDACDIV.index(value)

    _baseClockDivider = WMBits(4, _REG_CLOCKING_2, 0)

    @property
    def baseClockDivider(self) -> float:
        return _BCLKDIV[min(self._baseClockDivider, len(_BCLKDIV))]
    @baseClockDivider.setter
    def baseClockDivider(self, value:float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _BCLKDIV:
            self._baseClockDivider = _BCLKDIV.index(value)
        

    _ampClockDivider = WMBits(3, _REG_CLOCKING_2, 6)

    @property
    def ampClockDivider(self) -> float:
        return _DCLKDIV[min(self._ampClockDivider, len(_DCLKDIV))]
    @ampClockDivider.setter
    def ampClockDivider(self, value:float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _DCLKDIV:
            self._ampClockDivider = _DCLKDIV.index(value)

    ## Mode

    masterMode = WMBit(_REG_AUDIO_INTERFACE_1, 6)

    _wordLength = WMBits(2, _REG_AUDIO_INTERFACE_1, 2)

    @property
    def wordLength(self) -> int:
        value = self._wordLength
        if value == 3:
            return 32
        else:
            return 16 + 4 * value
    @wordLength.setter
    def wordLength(self, value:int) -> None:
        self._wordLength = (min(value, 28) - 16) // 4

    wordSelectInverted = WMBit(_REG_AUDIO_INTERFACE_1, 4)

    adcChannelSwap = WMBit(_REG_AUDIO_INTERFACE_1, 8)

    vrefOutputDisabled = WMBit(_REG_ADDITIONAL_CONTROL_3, 6)

    _vsel = WMBits(2, _REG_ADDITIONAL_CONTROL_1, 6)

    @property
    def powerSupply(self) -> float:
        return (constrain(self._vsel, 1, 2) - 1) * 0.6 + 2.7
    @powerSupply.setter
    def powerSupply(self, value:float) -> None:
        self._vsel = (constrain(value, 2.7, 3.3) - 2.7) // 0.6 * 2 + 1

    ## GPIO

    gpioOutput = WMBit(_REG_AUDIO_INTERFACE_2, 6)
    gpioOutputMode = WMBits(3, _REG_ADDITIONAL_CONTROL_4, 4)
    gpioOutputInverted = WMBit(_REG_ADDITIONAL_CONTROL_4, 7)
    
    _gpioClockDivider = WMBits(3, _REG_CLOCKING_2, 6)

    @property
    def gpioClockDivider(self) -> int:
        return self._gpioClockDivider
    @gpioClockDivider.setter
    def gpioClockDivider(self, value:int) -> None:
        self._gpioClockDivider = min(value, 5)
    
    @property
    def sampleRate(self) -> int:
        return self._sampleRate
    @sampleRate.setter
    def sampleRate(self, value:int) -> None:
        # MCLK = 24 MHz
        self.pll = True # Needed for class-d amp clock
        self.clockFractionalMode = True
        self.clockFromPLL = True

        self.pllPrescaleDiv2 = True
        self.systemClockDiv2 = True
        self.baseClockDivider = 4.0
        self.ampClockDivider = 16.0

        if value in [8000, 12000, 16000, 24000, 32000, 48000]:
            # SYSCLK = 12.288 MHz
            # DCLK = 768.0kHz
            self.pllN = 8
            self.pllK = 0x3126E8
            self.adcClockDivider = self.dacClockDivider = 48000 / value
            
        elif value in [11025, 22050, 44100]:
            # SYSCLK = 11.2896 MHz
            # DCLK = 705.6kHz
            self.pllN = 7
            self.pllK = 0x86C226
            self.adcClockDivider = self.dacClockDivider = 44100 / value
            
        else:
            raise Exception("Invalid sample rate")

        self._sampleRate = value
    
    def __init__(self, i2c_bus:I2C, address:int = _DEFAULT_I2C_ADDR) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._sampleRate = None

        self._registers = [
            0x0097, # R0 (0x00)
            0x0097, # R1 (0x01)
            0x0000, # R2 (0x02)
            0x0000, # R3 (0x03)
            0x0000, # R4 (0x04)
            0x0008, # F5 (0x05)
            0x0000, # R6 (0x06)
            0x000A, # R7 (0x07)
            0x01C0, # R8 (0x08)
            0x0000, # R9 (0x09)
            0x00FF, # R10 (0x0a)
            0x00FF, # R11 (0x0b)
            0x0000, # R12 (0x0C) RESERVED
            0x0000, # R13 (0x0D) RESERVED
            0x0000, # R14 (0x0E) RESERVED
            0x0000, # R15 (0x0F) RESERVED
            0x0000, # R16 (0x10)
            0x007B, # R17 (0x11)
            0x0100, # R18 (0x12)
            0x0032, # R19 (0x13)
            0x0000, # R20 (0x14)
            0x00C3, # R21 (0x15)
            0x00C3, # R22 (0x16)
            0x01C0, # R23 (0x17)
            0x0000, # R24 (0x18)
            0x0000, # R25 (0x19)
            0x0000, # R26 (0x1A)
            0x0000, # R27 (0x1B)
            0x0000, # R28 (0x1C)
            0x0000, # R29 (0x1D)
            0x0000, # R30 (0x1E) RESERVED
            0x0000, # R31 (0x1F) RESERVED
            0x0100, # R32 (0x20)
            0x0100, # R33 (0x21)
            0x0050, # R34 (0x22)
            0x0000, # R35 (0x23) RESERVED
            0x0000, # R36 (0x24) RESERVED
            0x0050, # R37 (0x25)
            0x0000, # R38 (0x26)
            0x0000, # R39 (0x27)
            0x0000, # R40 (0x28)
            0x0000, # R41 (0x29)
            0x0040, # R42 (0x2A)
            0x0000, # R43 (0x2B)
            0x0000, # R44 (0x2C)
            0x0050, # R45 (0x2D)
            0x0050, # R46 (0x2E)
            0x0000, # R47 (0x2F)
            0x0002, # R48 (0x30)
            0x0037, # R49 (0x31)
            0x0000, # R50 (0x32) RESERVED
            0x0080, # R51 (0x33)
            0x0008, # R52 (0x34)
            0x0031, # R53 (0x35)
            0x0026, # R54 (0x36)
            0x00e9, # R55 (0x37)
        ]
        for i in range(len(self._registers)):
            self._registers[i] = bytearray(self._registers[i].to_bytes(2, 'big'))
            self._registers[i][0] |= i << 1

        self.reset()

        # General setup
        self.vref = True
        self.vmid = VMIDSEL_PLAYBACK

    # Resets all registers to their default state
    _reset = WMBit(_REG_RESET, 7)
    def reset(self) -> None:
        self._reset = True
        for name in dir(self):
            if not name.startswith('_') and isinstance(getattr(self, name), (WMBit, WMBits)):
                getattr(self, name).reset(self)
