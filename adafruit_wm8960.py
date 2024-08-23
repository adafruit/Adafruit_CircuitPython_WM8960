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

# Microphone input selections
MIC_INPUT2 = 0
MIC_INPUT3 = 1
MIC_VMID = 2

# Microphone Boost gain options
_MIC_BOOST_GAIN = [0.0, 13.0, 20.0, 29.0]  # in dB

# Mic Bias voltage options
MIC_BIAS_VOLTAGE_0_9_AVDD = 0
MIC_BIAS_VOLTAGE_0_65_AVDD = 1

# SYSCLK divide
_SYSCLK_DIV_BY_1 = const(0)
_SYSCLK_DIV_BY_2 = const(2)

# Gain/Level min/max
BOOST_GAIN_MIN = -12.00
BOOST_GAIN_MAX = 6.0

MIC_GAIN_MIN = -17.25
MIC_GAIN_MAX = 30.00

ADC_VOLUME_MIN = -97.00
ADC_VOLUME_MAX = 30.00

DAC_VOLUME_MIN = -127.00
DAC_VOLUME_MAX = 0.00

ALC_TARGET_MIN = -22.50
ALC_TARGET_MAX = -1.50

ALC_MAX_GAIN_MIN = -12.00
ALC_MAX_GAIN_MAX = 30.00

ALC_MIN_GAIN_MIN = -17.25
ALC_MIN_GAIN_MAX = 24.75

GATE_THRESHOLD_MIN = -76.50
GATE_THRESHOLD_MAX = -30.00

OUTPUT_VOLUME_MIN = -21.00
OUTPUT_VOLUME_MAX = 0.00

AMP_VOLUME_MIN = -73.00
AMP_VOLUME_MAX = 6.00

# ALC Time mapping
_ALC_ATTACK_MAX = const(10)
ALC_ATTACK_TIME_MIN = 0.006
ALC_ATTACK_TIME_MAX = 6.140

_ALC_DECAY_MAX = const(10)
ALC_DECAY_TIME_MIN = 0.024
ALC_DECAY_TIME_MAX = 24.580

ALC_HOLD_TIME_MIN = 0.00267
ALC_HOLD_TIME_MAX = 43.691

# Speaker Boost Gains (DC and AC)
_SPEAKER_BOOST_GAIN = [0.0, 2.1, 2.9, 3.6, 4.5, 5.1]  # in dB

# VMIDSEL settings
VMIDSEL_DISABLED = 0
VMIDSEL_PLAYBACK = 1  # 2X50KOHM
VMIDSEL_LOWPOWER = 2  # 2X250KOHM
VMIDSEL_FASTSTART = 3  # 2X5KOHM

# Clock Divider Tables
_BCLKDIV = [1.0, 1.5, 2.0, 3.0, 4.0, 5.5, 6.0, 8.0, 11.0, 12.0, 16.0, 22.0, 24.0, 32.0]
_DCLKDIV = [1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0]
_ADCDACDIV = [1.0, 1.5, 2.0, 3.0, 4.0, 5.5, 6.0]


class WOBit:

    def __init__(
        self,
        register_address: int,
        bit: int,
    ) -> None:
        self.register_address = register_address
        self.bit = bit
        self.bit_mask = 1 << (bit % 8)  # the bitmask *within* the byte!
        self.byte = 1 - (bit // 8)  # the byte number within the buffer

    def _set(self, obj: Optional[I2CDeviceDriver], value: bool) -> None:
        if value:
            obj._registers[self.register_address][self.byte] |= self.bit_mask
        else:
            obj._registers[self.register_address][self.byte] &= ~self.bit_mask

    def reset(self, obj: Optional[I2CDeviceDriver]) -> None:
        if not hasattr(self, "default"):
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


class WOBits:

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

    def _set(self, obj: Optional[I2CDeviceDriver], value: int) -> None:
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
    def reset(self, obj: Optional[I2CDeviceDriver]) -> None:
        if not hasattr(self, "default"):
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

    vref = WOBit(_REG_PWR_MGMT_1, 6)

    left_input = WOBit(_REG_PWR_MGMT_1, 5)
    right_input = WOBit(_REG_PWR_MGMT_1, 4)

    @property
    def input(self) -> bool:
        return self.left_input and self.right_input

    @input.setter
    def input(self, value: bool) -> None:
        self.left_input = self.right_input = value

    # MIC

    ## PWR_MGMT

    left_mic = WOBit(_REG_PWR_MGMT_3, 5)
    right_mic = WOBit(_REG_PWR_MGMT_3, 4)

    @property
    def mic(self) -> bool:
        return self.left_mic and self.right_mic

    @mic.setter
    def mic(self, value: bool) -> None:
        self.left_mic = self.right_mic = value

    ## SIGNAL_PATH

    left_mic_inverting_input = WOBit(_REG_ADCL_SIGNAL_PATH, 8)
    right_mic_inverting_input = WOBit(_REG_ADCR_SIGNAL_PATH, 8)

    @property
    def mic_inverting_input(self) -> bool:
        return self.left_mic_inverting_input and self.right_mic_inverting_input

    @mic_inverting_input.setter
    def mic_inverting_input(self, value: bool) -> None:
        self.left_mic_inverting_input = self.right_mic_inverting_input = value

    _left_mic_input2 = WOBit(_REG_ADCL_SIGNAL_PATH, 6)
    _left_mic_input3 = WOBit(_REG_ADCL_SIGNAL_PATH, 7)

    @property
    def left_mic_input(self) -> int:
        if self._left_mic_input2:
            return MIC_INPUT2
        elif self._left_mic_input3:
            return MIC_INPUT3
        else:
            return MIC_VMID

    @left_mic_input.setter
    def left_mic_input(self, input: int) -> None:
        self._left_mic_input2 = input == MIC_INPUT2
        self._left_mic_input3 = input == MIC_INPUT3

    _right_mic_input2 = WOBit(_REG_ADCR_SIGNAL_PATH, 6)
    _right_mic_input3 = WOBit(_REG_ADCR_SIGNAL_PATH, 7)

    @property
    def right_mic_input(self) -> int:
        if self._right_mic_input2:
            return MIC_INPUT2
        elif self._right_mic_input3:
            return MIC_INPUT3
        else:
            return MIC_VMID

    @right_mic_input.setter
    def right_mic_input(self, input: int) -> None:
        self._right_mic_input2 = input == MIC_INPUT2
        self._right_mic_input3 = input == MIC_INPUT3

    @property
    def mic_input(self) -> int:
        # NOTE: Not checking right signal
        return self.left_mic_input

    @mic_input.setter
    def mic_input(self, input: int) -> None:
        self.left_mic_input = self.right_mic_input = input

    ## Boost

    left_mic_boost = WOBit(_REG_ADCL_SIGNAL_PATH, 3)
    right_mic_boost = WOBit(_REG_ADCR_SIGNAL_PATH, 3)

    @property
    def mic_boost(self) -> bool:
        return self.left_mic_boost and self.right_mic_boost

    @mic_boost.setter
    def mic_boost(self, value: bool) -> None:
        self.left_mic_boost = self.right_mic_boost = value

    def _get_mic_boost_gain(self, value: float) -> int:
        for i in reversed(range(len(_MIC_BOOST_GAIN))):
            if value >= _MIC_BOOST_GAIN[i]:
                return i
        return 0

    _left_mic_boost_gain = WOBits(2, _REG_ADCL_SIGNAL_PATH, 4)

    @property
    def left_mic_boost_gain(self) -> float:
        return _MIC_BOOST_GAIN[self._left_mic_boost_gain]

    @left_mic_boost_gain.setter
    def left_mic_boost_gain(self, value: float) -> None:
        self._left_mic_boost_gain = self._get_mic_boost_gain(value)

    _right_mic_boost_gain = WOBits(2, _REG_ADCR_SIGNAL_PATH, 4)

    @property
    def right_mic_boost_gain(self) -> float:
        return _MIC_BOOST_GAIN[self._right_mic_boost_gain]

    @right_mic_boost_gain.setter
    def right_mic_boost_gain(self, value: float) -> None:
        self._right_mic_boost_gain = self._get_mic_boost_gain(value)

    @property
    def mic_boost_gain(self) -> float:
        return max(self.left_mic_boost_gain, self.right_mic_boost_gain)

    @mic_boost_gain.setter
    def mic_boost_gain(self, value: float) -> None:
        self._left_mic_boost_gain = self._right_mic_boost_gain = (
            self._get_mic_boost_gain(value)
        )

    ## Volume

    _left_mic_volume = WOBits(6, _REG_LEFT_INPUT_VOLUME, 0)
    _left_mic_volume_set = WOBit(_REG_LEFT_INPUT_VOLUME, 8)

    _right_mic_volume = WOBits(6, _REG_RIGHT_INPUT_VOLUME, 0)
    _right_mic_volume_set = WOBit(_REG_RIGHT_INPUT_VOLUME, 8)

    @property
    def left_mic_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_mic_volume, 0, 63,
            MIC_GAIN_MIN, MIC_GAIN_MAX
        )
        # fmt: on

    @left_mic_volume.setter
    def left_mic_volume(self, value: float) -> None:
        # fmt: off
        self._left_mic_volume = round(map_range(
            value, MIC_GAIN_MIN, MIC_GAIN_MAX,
            0, 63
        ))
        # fmt: on
        self._left_mic_volume_set = True

    @property
    def right_mic_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_mic_volume, 0, 63,
            MIC_GAIN_MIN, MIC_GAIN_MAX
        )
        # fmt: on

    @right_mic_volume.setter
    def right_mic_volume(self, value: float) -> None:
        # fmt: off
        self._right_mic_volume = round(map_range(
            value, MIC_GAIN_MIN, MIC_GAIN_MAX,
            0, 63
        ))
        # fmt: on
        self._right_mic_volume_set = True

    @property
    def mic_volume(self) -> float:
        return max(self.left_mic_volume, self.right_mic_volume)

    @mic_volume.setter
    def mic_volume(self, value: float) -> None:
        # fmt: off
        self._left_mic_volume = self._right_mic_volume = round(map_range(
            value, MIC_GAIN_MIN, MIC_GAIN_MAX,
            0, 63
        ))
        # fmt: on
        self._left_mic_volume_set = self._right_mic_volume_set = True

    ## Zero Cross

    left_mic_zero_cross = WOBit(_REG_LEFT_INPUT_VOLUME, 6)
    right_mic_zero_cross = WOBit(_REG_RIGHT_INPUT_VOLUME, 6)

    @property
    def mic_zero_cross(self) -> bool:
        return self.left_mic_zero_cross and self.right_mic_zero_cross

    @mic_zero_cross.setter
    def mic_zero_cross(self, value: bool) -> None:
        self.left_mic_zero_cross = self.right_mic_zero_cross = value

    ## Mute

    _left_mic_mute = WOBit(_REG_LEFT_INPUT_VOLUME, 7)
    _right_mic_mute = WOBit(_REG_RIGHT_INPUT_VOLUME, 7)

    @property
    def left_mic_mute(self) -> bool:
        return self._left_mic_mute

    @left_mic_mute.setter
    def left_mic_mute(self, value: bool) -> None:
        self._left_mic_mute = value
        self._left_mic_volume_set = True

    @property
    def right_mic_mute(self) -> bool:
        return self._right_mic_mute

    @right_mic_mute.setter
    def right_mic_mute(self, value: bool) -> None:
        self._right_mic_mute = value
        self._right_mic_volume_set = True

    @property
    def mic_mute(self) -> bool:
        return self._left_mic_mute and self._right_mic_mute

    @mic_mute.setter
    def mic_mute(self, value: bool) -> None:
        self._left_mic_mute = self._right_mic_mute = value

    # Boost Mixer

    _left_input2_boost = WOBits(3, _REG_INPUT_BOOST_MIXER_1, 1)

    @property
    def left_input2_boost(self) -> float:
        value = self._left_input2_boost
        if value == 0:
            return None
        # fmt: off
        return map_range(
            value, 1, 7,
            BOOST_GAIN_MIN, BOOST_GAIN_MAX
        )
        # fmt: on

    @left_input2_boost.setter
    def left_input2_boost(self, value: float) -> None:
        # fmt: off
        self._left_input2_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(
                value, BOOST_GAIN_MIN, BOOST_GAIN_MAX,
                1, 7
            ))
        )
        # fmt: on

    _right_input2_boost = WOBits(3, _REG_INPUT_BOOST_MIXER_2, 1)

    @property
    def right_input2_boost(self) -> float:
        value = self._right_input2_boost
        if value == 0:
            return None
        # fmt: off
        return map_range(
            value, 1, 7,
            BOOST_GAIN_MIN, BOOST_GAIN_MAX
        )
        # fmt: on

    @right_input2_boost.setter
    def right_input2_boost(self, value: float) -> None:
        # fmt: off
        self._right_input2_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(
                value, BOOST_GAIN_MIN, BOOST_GAIN_MAX,
                1, 7
            ))
        )
        # fmt: on

    @property
    def input2_boost(self) -> float:
        value = max(self._left_input2_boost, self._right_input2_boost)
        # fmt: off
        return (
            None
            if value is 0
            else map_range(
                value, 1, 7,
                BOOST_GAIN_MIN, BOOST_GAIN_MAX
            )
        )
        # fmt: on

    @input2_boost.setter
    def input2_boost(self, value: float) -> None:
        # fmt: off
        self._left_input2_boost = self._right_input2_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(
                value, BOOST_GAIN_MIN, BOOST_GAIN_MAX,
                1, 7
            ))
        )
        # fmt: on

    _left_input3_boost = WOBits(3, _REG_INPUT_BOOST_MIXER_1, 4)

    @property
    def left_input3_boost(self) -> float:
        value = self._left_input3_boost
        if value == 0:
            return None
        # fmt: off
        return map_range(
            value, 1, 7,
            BOOST_GAIN_MIN, BOOST_GAIN_MAX
        )
        # fmt: on

    @left_input3_boost.setter
    def left_input3_boost(self, value: float) -> None:
        # fmt: off
        self._left_input3_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(
                value, BOOST_GAIN_MIN, BOOST_GAIN_MAX,
                1, 7
            ))
        )
        # fmt: on

    _right_input3_boost = WOBits(3, _REG_INPUT_BOOST_MIXER_2, 4)

    @property
    def right_input3_boost(self) -> float:
        value = self._right_input3_boost
        if value == 0:
            return None
        # fmt: off
        return map_range(
            value, 1, 7,
            BOOST_GAIN_MIN, BOOST_GAIN_MAX
        )
        # fmt: on

    @right_input3_boost.setter
    def right_input3_boost(self, value: float) -> None:
        # fmt: off
        self._right_input3_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(
                value, BOOST_GAIN_MIN, BOOST_GAIN_MAX,
                1, 7
            ))
        )
        # fmt: on

    @property
    def input3_boost(self) -> float:
        value = max(self._left_input3_boost, self._right_input3_boost)
        # fmt: off
        return (
            None
            if value is 0
            else map_range(
                value, 1, 7,
                BOOST_GAIN_MIN, BOOST_GAIN_MAX
            )
        )
        # fmt: on

    @input3_boost.setter
    def input3_boost(self, value: float) -> None:
        # fmt: off
        self._left_input3_boost = self._right_input3_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(
                value, BOOST_GAIN_MIN, BOOST_GAIN_MAX,
                1, 7
            ))
        )
        # fmt: on

    # Mic Bias

    mic_bias = WOBit(_REG_PWR_MGMT_1, 1)
    mic_bias_voltage = WOBit(_REG_ADDITIONAL_CONTROL_4, 0)

    # ADC

    left_adc = WOBit(_REG_PWR_MGMT_1, 3)
    right_adc = WOBit(_REG_PWR_MGMT_1, 2)

    @property
    def adc(self) -> bool:
        return self.left_adc and self.right_adc

    @adc.setter
    def adc(self, value: bool) -> None:
        self.left_adc = self.right_adc = value

    ## Volume

    _left_adc_volume = WOBits(8, _REG_LEFT_ADC_VOLUME, 0)
    _left_adc_volume_set = WOBit(_REG_LEFT_ADC_VOLUME, 8)

    @property
    def left_adc_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_adc_volume, 1, 255,
            ADC_VOLUME_MIN, ADC_VOLUME_MAX
        )
        # fmt: on

    @left_adc_volume.setter
    def left_adc_volume(self, value: float) -> None:
        # fmt: off
        self._left_adc_volume = round(map_range(
            value, ADC_VOLUME_MIN, ADC_VOLUME_MAX,
            1, 255
        ))
        # fmt: on
        self._left_adc_volume_set = True

    _right_adc_volume = WOBits(8, _REG_RIGHT_ADC_VOLUME, 0)
    _right_adc_volume_set = WOBit(_REG_RIGHT_ADC_VOLUME, 8)

    @property
    def right_adc_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_adc_volume, 1, 255,
            ADC_VOLUME_MIN, ADC_VOLUME_MAX
        )
        # fmt: on

    @right_adc_volume.setter
    def right_adc_volume(self, value: float) -> None:
        # fmt: off
        self._right_adc_volume = round(map_range(
            value, ADC_VOLUME_MIN, ADC_VOLUME_MAX,
            1, 255
        ))
        # fmt: on
        self._right_adc_volume_set = True

    @property
    def adc_volume(self) -> int:
        return max(self.left_adc_volume, self.right_adc_volume)

    @adc_volume.setter
    def adc_volume(self, value: float) -> None:
        # fmt: off
        self._left_adc_volume = self._right_adc_volume = round(map_range(
            value, ADC_VOLUME_MIN, ADC_VOLUME_MAX,
            1, 255
        ))
        # fmt: on
        self._left_adc_volume_set = self._right_adc_volume_set = True

    # ALC

    left_alc = WOBit(_REG_ALC1, 7)
    right_alc = WOBit(_REG_ALC1, 8)

    @property
    def alc(self) -> bool:
        return self.left_alc and self.right_alc

    @alc.setter
    def alc(self, value: bool) -> None:
        self.left_alc = self.right_alc = value

    _alc_target = WOBits(4, _REG_ALC1, 0)

    @property
    def alc_target(self) -> float:
        # fmt: off
        return map_range(
            self._alc_target, 0, 15,
            ALC_TARGET_MIN, ALC_TARGET_MAX
        )
        # fmt: on

    @alc_target.setter
    def alc_target(self, value: float) -> None:
        # fmt: off
        self._alc_target = round(map_range(
            value, ALC_TARGET_MIN, ALC_TARGET_MAX,
            0, 15
        ))
        # fmt: on

    _alc_max_gain = WOBits(3, _REG_ALC1, 4)

    @property
    def alc_max_gain(self) -> float:
        # fmt: off
        return map_range(
            self._alc_max_gain, 0, 7,
            ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX
        )
        # fmt: on

    @alc_max_gain.setter
    def alc_max_gain(self, value: float) -> None:
        # fmt: off
        self._alc_max_gain = round(map_range(
            value, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX,
            0, 7
        ))
        # fmt: on

    _alc_min_gain = WOBits(3, _REG_ALC2, 4)

    @property
    def alc_min_gain(self) -> float:
        # fmt: off
        return map_range(
            self._alc_min_gain, 0, 7,
            ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX
        )
        # fmt: on

    @alc_min_gain.setter
    def alc_min_gain(self, value: float) -> None:
        # fmt: off
        self._alc_min_gain = round(map_range(
            value, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX,
            0, 7
        ))
        # fmt: on

    _alc_attack = WOBits(4, _REG_ALC3, 0)

    @property
    def alc_attack_time(self) -> float:
        return ALC_ATTACK_TIME_MIN * pow(2, self._alc_attack)

    @alc_attack_time.setter
    def alc_attack_time(self, value: float) -> None:
        # fmt: off
        self._alc_attack = min(
            round(math.log2(
                (constrain(value, ALC_ATTACK_TIME_MIN, ALC_ATTACK_TIME_MAX) - ALC_ATTACK_TIME_MIN)
                / ALC_ATTACK_TIME_MIN
            )),
            _ALC_ATTACK_MAX
        )
        # fmt: on

    _alc_decay = WOBits(4, _REG_ALC3, 4)

    @property
    def alc_decay_time(self) -> float:
        return ALC_DECAY_TIME_MIN * pow(2, self._alc_decay)

    @alc_decay_time.setter
    def alc_decay_time(self, value: float) -> None:
        # fmt: off
        self._alc_decay = min(
            round(math.log2(
                (constrain(value, ALC_DECAY_TIME_MIN, ALC_DECAY_TIME_MAX) - ALC_DECAY_TIME_MIN)
                / ALC_DECAY_TIME_MIN
            )),
            _ALC_DECAY_MAX
        )
        # fmt: on

    _alc_hold = WOBits(4, _REG_ALC2, 0)

    @property
    def alc_hold_time(self) -> float:
        value = self._alc_hold
        if value == 0:
            return 0.0
        return ALC_HOLD_TIME_MIN * pow(2, self._alc_hold - 1)

    @alc_hold_time.setter
    def alc_hold_time(self, value: float) -> None:
        if value <= 0.0:
            self._alc_hold = 0
        else:
            # fmt: off
            self._alc_hold = round(
                math.log2(
                    (constrain(value, ALC_HOLD_TIME_MIN, ALC_HOLD_TIME_MAX) - ALC_HOLD_TIME_MIN)
                    / ALC_HOLD_TIME_MIN
                )
                + 1.0
            )
            # fmt: on

    alc_limiter = WOBit(_REG_ALC3, 8)

    # Noise Gate

    noise_gate = WOBit(_REG_NOISE_GATE, 0)

    _noise_gate_threshold = WOBits(5, _REG_NOISE_GATE, 3)

    @property
    def noise_gate_threshold(self) -> float:
        # fmt: off
        return map_range(
            self._noise_gate_threshold, 0, 31,
            GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX
        )
        # fmt: on

    @noise_gate_threshold.setter
    def noise_gate_threshold(self, value: float) -> None:
        # fmt: off
        self._noise_gate_threshold = round(map_range(
            value, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX,
            0, 31
        ))
        # fmt: on

    # DAC

    left_dac = WOBit(_REG_PWR_MGMT_2, 8)
    right_dac = WOBit(_REG_PWR_MGMT_2, 7)

    @property
    def dac(self) -> bool:
        return self.left_dac and self.right_dac

    @dac.setter
    def dac(self, value: bool) -> None:
        self.left_dac = self.right_dac = value

    _left_dac_volume = WOBits(8, _REG_LEFT_DAC_VOLUME, 0)
    _left_dac_volume_set = WOBit(_REG_LEFT_DAC_VOLUME, 8)

    @property
    def left_dac_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_dac_volume, 1, 255,
            DAC_VOLUME_MIN, DAC_VOLUME_MAX
        )
        # fmt: on

    @left_dac_volume.setter
    def left_dac_volume(self, value: float) -> None:
        # fmt: off
        self._left_dac_volume = round(map_range(
            value, DAC_VOLUME_MIN, DAC_VOLUME_MAX,
            1, 255
        ))
        # fmt: on
        self._left_dac_volume_set = True

    _right_dac_volume = WOBits(8, _REG_RIGHT_DAC_VOLUME, 0)
    _right_dac_volume_set = WOBit(_REG_RIGHT_DAC_VOLUME, 8)

    @property
    def right_dac_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_dac_volume, 1, 255,
            DAC_VOLUME_MIN, DAC_VOLUME_MAX
        )
        # fmt: on

    @right_dac_volume.setter
    def right_dac_volume(self, value: float) -> None:
        # fmt: off
        self._right_dac_volume = round(map_range(
            value, DAC_VOLUME_MIN, DAC_VOLUME_MAX,
            1, 255
        ))
        # fmt: on
        self._right_dac_volume_set = True

    @property
    def dac_volume(self) -> int:
        return max(self.left_dac_volume, self.right_dac_volume)

    @dac_volume.setter
    def dac_volume(self, value: float) -> None:
        # fmt: off
        self._left_dac_volume = self._right_dac_volume = round(map_range(
            value, DAC_VOLUME_MIN, DAC_VOLUME_MAX,
            1, 255
        ))
        # fmt: on
        self._left_dac_volume_set = self._right_dac_volume_set = True

    dac_mute = WOBit(_REG_ADC_DAC_CTRL_1, 3)
    dac_soft_mute = WOBit(_REG_ADC_DAC_CTRL_2, 3)
    dac_slow_soft_mute = WOBit(_REG_ADC_DAC_CTRL_2, 2)
    dac_attenuation = WOBit(_REG_ADC_DAC_CTRL_1, 7)

    # 3D Enhance

    enhance = WOBit(_REG_3D_CONTROL, 0)
    enhance_filter_lPF = WOBit(_REG_3D_CONTROL, 6)
    enhance_filter_hPF = WOBit(_REG_3D_CONTROL, 5)

    _enhance_depth = WOBits(4, _REG_3D_CONTROL, 1)

    @property
    def enhance_depth(self) -> float:
        return self._enhance_depth / 15.0

    @enhance_depth.setter
    def enhance_depth(self, value: float) -> None:
        # fmt: off
        self._enhance_depth = round(map_range(
            value, 0.0, 1.0,
            0, 15
        ))
        # fmt: on

    # Output Mixer

    left_output = WOBit(_REG_PWR_MGMT_3, 3)
    right_output = WOBit(_REG_PWR_MGMT_3, 2)

    @property
    def output(self) -> bool:
        return self.left_output and self.right_output

    @output.setter
    def output(self, value: bool) -> None:
        self.left_output = self.right_output = value

    ## DAC Output

    left_dac_output = WOBit(_REG_LEFT_OUT_MIX, 8)
    right_dac_output = WOBit(_REG_RIGHT_OUT_MIX, 8)

    @property
    def dac_output(self) -> bool:
        return self.left_dac_output and self.right_dac_output

    @dac_output.setter
    def dac_output(self, value: bool) -> None:
        self.left_dac_output = self.right_dac_output = value

    ## Input 3 Output

    left_input3_output = WOBit(_REG_LEFT_OUT_MIX, 7)
    right_input3_output = WOBit(_REG_RIGHT_OUT_MIX, 7)

    @property
    def input3_output(self) -> bool:
        return self.left_input3_output and self.right_input3_output

    @input3_output.setter
    def input3_output(self, value: bool) -> None:
        self.left_input3_output = self.right_input3_output = value

    _left_input3_output_volume = WOBits(3, _REG_LEFT_OUT_MIX, 4)

    @property
    def left_input3_output_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_input3_output_volume, 0, 7,
            OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN
        )
        # fmt: on

    @left_input3_output_volume.setter
    def left_input3_output_volume(self, value: float) -> None:
        # fmt: off
        self._left_input3_output_volume = round(map_range(
            value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX,
            7, 0
        ))
        # fmt: on

    _right_input3_output_volume = WOBits(3, _REG_RIGHT_OUT_MIX, 4)

    @property
    def right_input3_output_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_input3_output_volume, 0, 7,
            OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN
        )
        # fmt: on

    @right_input3_output_volume.setter
    def right_input3_output_volume(self, value: float) -> None:
        # fmt: off
        self._right_input3_output_volume = round(map_range(
            value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX,
            7, 0
        ))
        # fmt: on

    @property
    def input3_output_volume(self) -> float:
        return max(self.left_input3_output_volume, self.right_input3_output_volume)

    @input3_output_volume.setter
    def input3_output_volume(self, value: float) -> None:
        # fmt: off
        self._left_input3_output_volume = self._right_input3_output_volume = round(map_range(
            value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX,
            7, 0
        ))
        # fmt: on

    ## MIC Boost Mixer Output

    left_mic_output = WOBit(_REG_BYPASS_1, 7)
    right_mic_output = WOBit(_REG_BYPASS_2, 7)

    @property
    def mic_output(self) -> bool:
        return self.left_mic_output and self.right_mic_output

    @mic_output.setter
    def mic_output(self, value: bool) -> None:
        self.left_mic_output = self.right_mic_output = value

    _left_mic_output_volume = WOBits(3, _REG_BYPASS_1, 4)

    @property
    def left_mic_output_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_mic_output_volume, 0, 7,
            OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN
        )
        # fmt: on

    @left_mic_output_volume.setter
    def left_mic_output_volume(self, value: float) -> None:
        # fmt: off
        self._left_mic_output_volume = round(map_range(
            value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX,
            7, 0
        ))
        # fmt: on

    _right_mic_output_volume = WOBits(3, _REG_BYPASS_2, 4)

    @property
    def right_mic_output_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_mic_output_volume, 0, 7,
            OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN
        )
        # fmt: on

    @right_mic_output_volume.setter
    def right_mic_output_volume(self, value: float) -> None:
        # fmt: off
        self._right_mic_output_volume = round(map_range(
            value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX,
            7, 0
        ))
        # fmt: on

    @property
    def mic_output_volume(self) -> float:
        return max(self.left_mic_output_volume, self.right_mic_output_volume)

    @mic_output_volume.setter
    def mic_output_volume(self, value: float) -> None:
        # fmt: off
        self._left_mic_output_volume = self._right_mic_output_volume = round(map_range(
            value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX,
            7, 0
        ))
        # fmt: on

    ## Mono Output

    mono_output = WOBit(_REG_PWR_MGMT_2, 1)

    mono_left_mix = WOBit(_REG_MONO_OUT_MIX_1, 7)
    mono_right_mix = WOBit(_REG_MONO_OUT_MIX_2, 7)

    @property
    def mono_mix(self) -> bool:
        return self.mono_left_mix and self.mono_right_mix

    @mono_mix.setter
    def mono_mix(self, value: bool) -> None:
        self.mono_left_mix = self.mono_right_mix = value

    mono_output_attenuation = WOBit(_REG_MONO_OUT_VOLUME, 6)

    vmid = WOBits(2, _REG_PWR_MGMT_1, 7)

    # Amplifier

    ## Headphones

    left_headphone = WOBit(_REG_PWR_MGMT_2, 5)
    right_headphone = WOBit(_REG_PWR_MGMT_2, 6)

    @property
    def headphone(self) -> bool:
        return self.left_headphone and self.right_headphone

    @headphone.setter
    def headphone(self, value: bool) -> None:
        self.left_headphone = self.right_headphone = value

    headphone_standby = WOBit(_REG_ANTI_POP_1, 0)

    _left_headphone_volume = WOBits(7, _REG_LOUT1_VOLUME, 0)
    _left_headphone_volume_set = WOBit(_REG_LOUT1_VOLUME, 8)

    @property
    def left_headphone_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_headphone_volume, 48, 127,
            AMP_VOLUME_MIN, AMP_VOLUME_MAX
        )
        # fmt: on

    @left_headphone_volume.setter
    def left_headphone_volume(self, value: float) -> None:
        # fmt: off
        self._left_headphone_volume = round(map_range(
            value, AMP_VOLUME_MIN, AMP_VOLUME_MAX,
            48, 127
        ))
        # fmt: on
        self._left_headphone_volume_set = True

    _right_headphone_volume = WOBits(7, _REG_ROUT1_VOLUME, 0)
    _right_headphone_volume_set = WOBit(_REG_ROUT1_VOLUME, 8)

    @property
    def right_headphone_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_headphone_volume, 48, 127,
            AMP_VOLUME_MIN, AMP_VOLUME_MAX
        )
        # fmt: on

    @right_headphone_volume.setter
    def right_headphone_volume(self, value: float) -> None:
        # fmt: off
        self._right_headphone_volume = round(map_range(
            value, AMP_VOLUME_MIN, AMP_VOLUME_MAX,
            48, 127
        ))
        # fmt: on
        self._right_headphone_volume_set = True

    @property
    def headphone_volume(self) -> int:
        return max(self.left_headphone_volume, self.right_headphone_volume)

    @headphone_volume.setter
    def headphone_volume(self, value: float) -> None:
        # fmt: off
        self._left_headphone_volume = self._right_headphone_volume = round(map_range(
            value, AMP_VOLUME_MIN, AMP_VOLUME_MAX,
            48, 127
        ))
        # fmt: on
        self._left_headphone_volume_set = self._right_headphone_volume_set = True

    left_headphone_zero_cross = WOBit(_REG_LOUT1_VOLUME, 7)
    right_headphone_zero_cross = WOBit(_REG_LOUT1_VOLUME, 7)

    @property
    def headphone_zero_cross(self) -> bool:
        return self.left_headphone_zero_cross and self.right_headphone_zero_cross

    @headphone_zero_cross.setter
    def headphone_zero_cross(self, value: bool) -> None:
        self.left_headphone_zero_cross = self.right_headphone_zero_cross = value

    ## Speakers

    _left_speaker = WOBit(_REG_PWR_MGMT_2, 4)
    _left_speaker_amp = WOBit(_REG_CLASS_D_CONTROL_1, 6)

    @property
    def left_speaker(self) -> bool:
        return self._left_speaker and self._left_speaker_amp

    @left_speaker.setter
    def left_speaker(self, value: bool) -> None:
        self._left_speaker = self._left_speaker_amp = value

    _right_speaker = WOBit(_REG_PWR_MGMT_2, 3)
    _right_speaker_amp = WOBit(_REG_CLASS_D_CONTROL_1, 7)

    @property
    def right_speaker(self) -> bool:
        return self._right_speaker and self._right_speaker_amp

    @right_speaker.setter
    def right_speaker(self, value: bool) -> None:
        self._right_speaker = self._right_speaker_amp = value

    @property
    def speaker(self) -> bool:
        return self.left_speaker and self.right_speaker

    @speaker.setter
    def speaker(self, value: bool) -> None:
        self.left_speaker = self.right_speaker = value

    _left_speaker_volume = WOBits(7, _REG_LOUT2_VOLUME, 0)
    _left_speaker_volume_set = WOBit(_REG_LOUT2_VOLUME, 8)

    @property
    def left_speaker_volume(self) -> float:
        # fmt: off
        return map_range(
            self._left_speaker_volume, 48, 127,
            AMP_VOLUME_MIN, AMP_VOLUME_MAX
        )
        # fmt: on

    @left_speaker_volume.setter
    def left_speaker_volume(self, value: float) -> None:
        # fmt: off
        self._left_speaker_volume = round(map_range(
            value, AMP_VOLUME_MIN, AMP_VOLUME_MAX,
            48, 127
        ))
        # fmt: on
        self._left_speaker_set = True

    _right_speaker_volume = WOBits(7, _REG_ROUT2_VOLUME, 0)
    _right_speaker_volume_set = WOBit(_REG_ROUT2_VOLUME, 8)

    @property
    def right_speaker_volume(self) -> float:
        # fmt: off
        return map_range(
            self._right_speaker_volume, 48, 127,
            AMP_VOLUME_MIN, AMP_VOLUME_MAX
        )
        # fmt: on

    @right_speaker_volume.setter
    def right_speaker_volume(self, value: float) -> None:
        # fmt: off
        self._right_speaker_volume = round(map_range(
            value, AMP_VOLUME_MIN, AMP_VOLUME_MAX,
            48, 127
        ))
        # fmt: on
        self._right_speaker_volume_set = True

    @property
    def speaker_volume(self) -> int:
        return max(self.left_speaker_volume, self.right_speaker_volume)

    @speaker_volume.setter
    def speaker_volume(self, value: float) -> None:
        # fmt: off
        self._left_speaker_volume = self._right_speaker_volume = round(map_range(
            value, AMP_VOLUME_MIN, AMP_VOLUME_MAX,
            48, 127
        ))
        # fmt: on
        self._left_speaker_volume_set = self._right_speaker_volume_set = True

    left_speaker_zero_cross = WOBit(_REG_LOUT2_VOLUME, 7)
    right_speaker_zero_cross = WOBit(_REG_LOUT2_VOLUME, 7)

    @property
    def speaker_zero_cross(self) -> bool:
        return self.left_speaker_zero_cross and self.right_speaker_zero_cross

    @speaker_zero_cross.setter
    def speaker_zero_cross(self, value: bool) -> None:
        self.left_speaker_zero_cross = self.right_speaker_zero_cross = value

    def _get_speaker_boost_gain(self, value: float) -> int:
        for i in reversed(range(len(_SPEAKER_BOOST_GAIN))):
            if value >= _SPEAKER_BOOST_GAIN[i]:
                return i
        return 0

    _speaker_dc_gain = WOBits(3, _REG_CLASS_D_CONTROL_3, 3)

    @property
    def speaker_dc_gain(self) -> float:
        return _SPEAKER_BOOST_GAIN[self._speaker_dc_gain]

    @speaker_dc_gain.setter
    def speaker_dc_gain(self, value: float) -> None:
        self._speaker_dc_gain = self._get_speaker_boost_gain(value)

    _speaker_ac_gain = WOBits(3, _REG_CLASS_D_CONTROL_3, 0)

    @property
    def speaker_ac_gain(self) -> float:
        return _SPEAKER_BOOST_GAIN[self._speaker_ac_gain]

    @speaker_ac_gain.setter
    def speaker_ac_gain(self, value: float) -> None:
        self._speaker_ac_gain = self._get_speaker_boost_gain(value)

    # Digital Audio Interface Control

    loopback = WOBit(_REG_AUDIO_INTERFACE_2, 0)

    pll = WOBit(_REG_PWR_MGMT_2, 0)
    pll_prescale_div2 = WOBit(_REG_PWR_MGMT_2, 4)
    pll_n = WOBits(4, _REG_PLL_N, 0)

    _pll_k1 = WOBits(6, _REG_PLL_K_1, 0)
    _pll_k2 = WOBits(9, _REG_PLL_K_2, 0)
    _pll_k3 = WOBits(9, _REG_PLL_K_3, 0)

    @property
    def pll_k(self) -> int:
        return self._pll_k1 << 18 + self._pll_k2 << 9 + self._pll_k3

    @pll_k.setter
    def pll_k(self, value: int) -> None:
        self._pll_k1 = (value >> 18) & 0b111111
        self._pll_k2 = (value >> 9) & 0b111111111
        self._pll_k3 = value & 0b111111111

    clock_fractional_mode = WOBit(_REG_PLL_N, 5)

    clock_from_pLL = WOBit(_REG_CLOCKING_1, 0)

    _system_clock_divider = WOBits(2, _REG_CLOCKING_1, 1)

    @property
    def system_clock_div2(self) -> bool:
        return self._system_clock_divider == _SYSCLK_DIV_BY_2

    @system_clock_div2.setter
    def system_clock_div2(self, value: bool) -> None:
        self._system_clock_divider = _SYSCLK_DIV_BY_2 if value else _SYSCLK_DIV_BY_1

    _adc_clock_divider = WOBits(3, _REG_CLOCKING_1, 6)

    @property
    def adc_clock_divider(self) -> int:
        return _ADCDACDIV[min(self._adc_clock_divider, len(_ADCDACDIV))]

    @adc_clock_divider.setter
    def adc_clock_divider(self, value: int) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._adc_clock_divider = _ADCDACDIV.index(value)

    _dac_clock_divider = WOBits(3, _REG_CLOCKING_1, 3)

    @property
    def dac_clock_divider(self) -> int:
        return _ADCDACDIV[min(self._dac_clock_divider, len(_ADCDACDIV))]

    @dac_clock_divider.setter
    def dac_clock_divider(self, value: int) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._dac_clock_divider = _ADCDACDIV.index(value)

    _base_clock_divider = WOBits(4, _REG_CLOCKING_2, 0)

    @property
    def base_clock_divider(self) -> float:
        return _BCLKDIV[min(self._base_clock_divider, len(_BCLKDIV))]

    @base_clock_divider.setter
    def base_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _BCLKDIV:
            self._base_clock_divider = _BCLKDIV.index(value)

    _amp_clock_divider = WOBits(3, _REG_CLOCKING_2, 6)

    @property
    def amp_clock_divider(self) -> float:
        return _DCLKDIV[min(self._amp_clock_divider, len(_DCLKDIV))]

    @amp_clock_divider.setter
    def amp_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _DCLKDIV:
            self._amp_clock_divider = _DCLKDIV.index(value)

    ## Mode

    master_mode = WOBit(_REG_AUDIO_INTERFACE_1, 6)

    _bit_depth = WOBits(2, _REG_AUDIO_INTERFACE_1, 2)

    @property
    def bit_depth(self) -> int:
        value = self._bit_depth
        if value == 3:
            return 32
        else:
            return 16 + 4 * value

    @bit_depth.setter
    def bit_depth(self, value: int) -> None:
        self._bit_depth = (min(value, 28) - 16) // 4

    word_select_invert = WOBit(_REG_AUDIO_INTERFACE_1, 4)

    adc_channel_swap = WOBit(_REG_AUDIO_INTERFACE_1, 8)

    _vref_output_disable = WOBit(_REG_ADDITIONAL_CONTROL_3, 6)

    @property
    def vref_output(self) -> bool:
        return not self._vref_output_disable

    @vref_output.setter
    def vref_output(self, value: bool) -> None:
        self._vref_output_disable = not value

    _vsel = WOBits(2, _REG_ADDITIONAL_CONTROL_1, 6)

    @property
    def power_supply(self) -> float:
        return (constrain(self._vsel, 1, 2) - 1) * 0.6 + 2.7

    @power_supply.setter
    def power_supply(self, value: float) -> None:
        self._vsel = (constrain(value, 2.7, 3.3) - 2.7) // 0.6 * 2 + 1

    ## GPIO

    gpio_output = WOBit(_REG_AUDIO_INTERFACE_2, 6)
    gpio_output_mode = WOBits(3, _REG_ADDITIONAL_CONTROL_4, 4)
    gpio_output_invert = WOBit(_REG_ADDITIONAL_CONTROL_4, 7)

    _gpio_clock_divider = WOBits(3, _REG_CLOCKING_2, 6)

    @property
    def gpio_clock_divider(self) -> int:
        return self._gpio_clock_divider

    @gpio_clock_divider.setter
    def gpio_clock_divider(self, value: int) -> None:
        self._gpio_clock_divider = min(value, 5)

    @property
    def sample_rate(self) -> int:
        return self._sample_rate

    @sample_rate.setter
    def sample_rate(self, value: int) -> None:
        # MCLK = 24 MHz
        self.pll = True  # Needed for class-d amp clock
        self.clock_fractional_mode = True
        self.clock_from_pLL = True

        self.pll_prescale_div2 = True
        self.system_clock_div2 = True
        self.base_clock_divider = 4.0
        self.amp_clock_divider = 16.0

        if value in [8000, 12000, 16000, 24000, 32000, 48000]:
            # SYSCLK = 12.288 MHz
            # DCLK = 768.0k_hz
            self.pll_n = 8
            self.pll_k = 0x3126_E8
            self.adc_clock_divider = self.dac_clock_divider = 48000 / value

        elif value in [11025, 22050, 44100]:
            # SYSCLK = 11.2896 MHz
            # DCLK = 705.6k_hz
            self.pll_n = 7
            self.pll_k = 0x86_C226
            self.adc_clock_divider = self.dac_clock_divider = 44100 / value

        else:
            raise Exception("Invalid sample rate")

        self._sample_rate = value

    def __init__(self, i2c_bus: I2C, address: int = _DEFAULT_I2C_ADDR) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._sample_rate = None

        self._registers = [
            0x0097,  # R0 (0x00)
            0x0097,  # R1 (0x01)
            0x0000,  # R2 (0x02)
            0x0000,  # R3 (0x03)
            0x0000,  # R4 (0x04)
            0x0008,  # F5 (0x05)
            0x0000,  # R6 (0x06)
            0x000A,  # R7 (0x07)
            0x01C0,  # R8 (0x08)
            0x0000,  # R9 (0x09)
            0x00FF,  # R10 (0x0a)
            0x00FF,  # R11 (0x0b)
            0x0000,  # R12 (0x0C) RESERVED
            0x0000,  # R13 (0x0D) RESERVED
            0x0000,  # R14 (0x0E) RESERVED
            0x0000,  # R15 (0x0F) RESERVED
            0x0000,  # R16 (0x10)
            0x007B,  # R17 (0x11)
            0x0100,  # R18 (0x12)
            0x0032,  # R19 (0x13)
            0x0000,  # R20 (0x14)
            0x00C3,  # R21 (0x15)
            0x00C3,  # R22 (0x16)
            0x01C0,  # R23 (0x17)
            0x0000,  # R24 (0x18)
            0x0000,  # R25 (0x19)
            0x0000,  # R26 (0x1A)
            0x0000,  # R27 (0x1B)
            0x0000,  # R28 (0x1C)
            0x0000,  # R29 (0x1D)
            0x0000,  # R30 (0x1E) RESERVED
            0x0000,  # R31 (0x1F) RESERVED
            0x0100,  # R32 (0x20)
            0x0100,  # R33 (0x21)
            0x0050,  # R34 (0x22)
            0x0000,  # R35 (0x23) RESERVED
            0x0000,  # R36 (0x24) RESERVED
            0x0050,  # R37 (0x25)
            0x0000,  # R38 (0x26)
            0x0000,  # R39 (0x27)
            0x0000,  # R40 (0x28)
            0x0000,  # R41 (0x29)
            0x0040,  # R42 (0x2A)
            0x0000,  # R43 (0x2B)
            0x0000,  # R44 (0x2C)
            0x0050,  # R45 (0x2D)
            0x0050,  # R46 (0x2E)
            0x0000,  # R47 (0x2F)
            0x0002,  # R48 (0x30)
            0x0037,  # R49 (0x31)
            0x0000,  # R50 (0x32) RESERVED
            0x0080,  # R51 (0x33)
            0x0008,  # R52 (0x34)
            0x0031,  # R53 (0x35)
            0x0026,  # R54 (0x36)
            0x00E9,  # R55 (0x37)
        ]
        for i in range(len(self._registers)):
            self._registers[i] = bytearray(self._registers[i].to_bytes(2, "big"))
            self._registers[i][0] |= i << 1

        self.reset()

        # General setup
        self.vref = True
        self.vmid = VMIDSEL_PLAYBACK

    # Resets all registers to their default state
    _reset = WOBit(_REG_RESET, 7)

    def reset(self) -> None:
        self._reset = True
        for name in dir(self):
            if not name.startswith("_") and isinstance(
                getattr(self, name), (WOBit, WOBits)
            ):
                getattr(self, name).reset(self)
