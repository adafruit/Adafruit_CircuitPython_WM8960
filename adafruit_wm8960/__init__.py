# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT
"""
`adafruit_wm8960`
================================================================================

CircuitPython driver for WM8960 Stereo CODEC

* Author(s): Scott Shawcroft, Cooper Dalrymple

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_WM8960.git"

from adafruit_simplemath import constrain, map_range
from busio import I2C

from .advanced import *


class Input:
    """An enum-like class representing the input options of the WM8960. Used for
    :attr:`WM8960.input`.
    """

    DISABLED = 0b000
    """Disconnect all inputs."""

    MIC1 = 0b001
    """Connect input 1 to the microphone amplifier in single-ended mode."""

    MIC2 = 0b011
    """Connect input 1 and 2 to the microphone amplifier in differential mode."""

    MIC3 = 0b101
    """Connect input 1 and 3 to the microphone amplifier in differential mode."""

    LINE2 = 0b010
    """Connect input 2 as a line input."""

    LINE3 = 0b100
    """Connect input 3 as a line input."""


class WM8960:
    """Driver for controlling the WM8960 audio codec over an I2C connection. Used for receiving line
    and microphone input through the ADC over I2S, monitoring analog input to the output mixer, and
    sending digital audio to the headphone and speaker amplifiers via I2S.

    NOTE: This driver assumes that the master clock of the WM8960 is 24 MHz in order determine
    appropriate clock settings.

    :param i2c: The I2C bus.
    :type i2c: :class:`I2C`
    :param sample_rate: The sample rate of each audio frame used by the digital interface of the
        WM8960. The sample rates 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, and 48000
        are supported. Defaults to 44100.
    :type sample_rate: `int`, optional
    :param bit_depth: The number of bits per sample. The values 16, 20, 24, and 32 are supported.
        Defaults to 16.
    :type bit_depth: `int`, optional
    """

    def __init__(
        self,
        i2c_bus: I2C,
        sample_rate: int = 44100,
        bit_depth: int = 16,
    ) -> None:
        self._input = Input.DISABLED
        self._gain = 0.0

        self._codec = WM8960_Advanced(i2c_bus)

        # Digital Interface
        self._codec.sample_rate = sample_rate
        self._codec.bit_depth = bit_depth

        # ADC
        self._codec.adc = True
        self._codec.input = True
        self._codec.mic_boost_gain = 0.0
        self._codec.mic_zero_cross = True

        # DAC
        self._codec.dac = True
        self._codec.dac_output = True

        # Output
        self._codec.output = True
        self._codec.mono_output = True
        self._codec.headphone_zero_cross = True
        self._codec.speaker_zero_cross = True

    # Digital Interface

    @property
    def sample_rate(self) -> int:
        """The rate of the I2S bus in samples per second. This property is read-only."""
        return self._codec.sample_rate

    @property
    def bit_depth(self) -> int:
        """The number of bits per sample. This property is read-only."""
        return self._codec.bit_depth

    # Input

    @property
    def input(self) -> int:
        """The signal used as the input to the ADC. See :class:`Input` for available selections.

        :default: :const:`Input.DISABLED`
        """
        return self._input

    @input.setter
    def input(self, value: int) -> None:
        mic = bool(value & 0b001)

        # Configure microphone amplifier
        self._codec.mic = mic
        self._codec.mic_inverting_input = mic
        self._codec.mic_input = (value & 0b110) >> 1 if mic else Mic_Input.VMID
        self._codec.mic_mute = not mic
        self._codec.mic_boost = mic

        self._input = value

        # Reset gain values
        self.gain = self._gain

    @property
    def gain(self) -> float:
        """The amount of analog gain on the selected input before the ADC.

        :default: 0.0
        """
        return self._gain

    @gain.setter
    def gain(self, value: float) -> None:
        mic = bool(self._input & 0b001)

        self._codec.mic_volume = (
            map_range(value, 0.0, 1.0, MIC_GAIN_MIN, MIC_GAIN_MAX) if mic else MIC_GAIN_MIN
        )

        self._codec.input2_boost = (
            map_range(value, 0.0, 1.0, BOOST_GAIN_MIN, BOOST_GAIN_MAX)
            if not mic and self._input & 0b010
            else BOOST_GAIN_MIN - 1.0
        )

        self._codec.input3_boost = (
            map_range(value, 0.0, 1.0, BOOST_GAIN_MIN, BOOST_GAIN_MAX)
            if not mic and self._input & 0b100
            else BOOST_GAIN_MIN - 1.0
        )

        self._gain = constrain(value, 0.0, 1.0)

    @property
    def monitor(self) -> float:
        """The volume of the analog input bypassed to the output mixer. If set to 0.0, this bypass
        will be muted.

        :default: 0.0
        """
        if not self._codec.mic_output:
            return 0.0
        return map_range(
            self._codec.mic_output_volume,
            OUTPUT_VOLUME_MIN,
            OUTPUT_VOLUME_MAX,
            0.0,
            1.0,
        )

    @monitor.setter
    def monitor(self, value: float) -> None:
        if value <= 0.0:
            self._codec.mic_output = False
        elif not self._codec.mic_output:
            self._codec.mic_output = True
        self._codec.mic_output_volume = map_range(
            value, 0.0, 1.0, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX
        )

    @property
    def loopback(self) -> bool:
        """Whether or not the data from the ADC is routed directly into the DAC internally. If set
        to `True`, the I2S interface will be disabled. This property is useful if you would like to
        apply :attr:`enhance` to the input signal.

        :default: `False`
        """
        return self._codec.master_mode and self._codec.gpio_output and self._codec.loopback

    @loopback.setter
    def loopback(self, value: bool) -> None:
        self._codec.master_mode = value
        self._codec.gpio_output = value
        self._codec.loopback = value

    # Output

    @property
    def volume(self) -> float:
        """The level of the output mixer before the :attr:`headphone` and :attr:`speaker`
        amplifiers.

        :default: 0.0
        """
        if self._codec.dac_mute:
            return 0.0
        return map_range(self._codec.dac_volume, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 0.0, 1.0)

    @volume.setter
    def volume(self, value: float) -> None:
        if value <= 0.0:
            self._codec.dac_mute = True
        elif self._codec.dac_mute:
            self._codec.dac_mute = False
        self._codec.dac_volume = map_range(value, 0.0, 1.0, DAC_VOLUME_MIN, DAC_VOLUME_MAX)

    @property
    def headphone(self) -> float:
        """The volume of the headphone amplifier output from 0.0 to 1.0. Setting this property to
        0.0 will mute and power off the headphone amplifier.

        :default: 0.0
        """
        if not self._codec.headphone:
            return 0.0
        value = self._codec.headphone_volume
        if value is None:
            return 0.0
        return map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 0.0, 1.0)

    @headphone.setter
    def headphone(self, value: float) -> None:
        if value <= 0.0:
            self._codec.headphone = False
        elif not self._codec.headphone:
            self._codec.headphone = True
        self._codec.headphone_volume = (
            map_range(value, 0.0, 1.0, AMP_VOLUME_MIN, AMP_VOLUME_MAX)
            if value > 0.0
            else AMP_VOLUME_MIN - 1.0  # Mute
        )

    @property
    def speaker(self) -> float:
        """The volume of the speaker amplifier output from 0.0 to 1.0. Setting this property to 0.0
        will power off the speaker amplifier.

        :default: 0.0
        """
        if not self._codec.speaker:
            return 0.0
        return map_range(self._codec.speaker_volume, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 0.0, 1.0)

    @speaker.setter
    def speaker(self, value: float) -> None:
        if value <= 0.0:
            self._codec.speaker = False
        elif not self._codec.speaker:
            self._codec.speaker = True
        self._codec.speaker_volume = map_range(value, 0.0, 1.0, AMP_VOLUME_MIN, AMP_VOLUME_MAX)

    # 3D Enhance

    @property
    def enhance(self) -> float:
        """The amount of digital 3D stereo enhancement that artificially increases the separation
        between the left and right channels within the DAC. Set to 0.0 to disable.

        :default: 0.0
        """
        if not self._codec.enhance:
            return 0.0
        return self._codec.enhance_depth

    @enhance.setter
    def enhance(self, value: float) -> None:
        if value <= 0.0:
            self._codec.enhance = False
        elif not self._codec.enhance:
            self._codec.enhance = True
        self._codec.enhance_depth = value

    # ALC & Noise Gate

    @property
    def alc(self) -> bool:
        """Whether or not the Automatic Level Control (ALC) is enabled. The ALC uses the digital
        signal within the ADC to control the gain of the microphone amplifier. This feature does not
        apply if one of the line-level inputs is used for :attr:`input` (ie: :const:`Input.LINE2` or
        :const:`Input.LINE3`).

        :default: `False`
        """
        return self._codec.alc

    @alc.setter
    def alc(self, value: bool) -> None:
        self._codec.alc = value

    @property
    def alc_gain(self) -> tuple:
        """The gain properties of the Automatic Level Control (ALC). Provided as a tuple in the
        format of target, max gain, min gain, and noise gate threshold all as monotonic `float`
        values.

        :default: (0.733333, 1.0, 0.0, 0.0)
        """
        return (
            map_range(self._codec.alc_target, ALC_TARGET_MIN, ALC_TARGET_MAX, 0.0, 1.0),
            map_range(self._codec.alc_max_gain, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX, 0.0, 1.0),
            map_range(self._codec.alc_min_gain, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX, 0.0, 1.0),
            (
                map_range(
                    self._codec.noise_gate_threshold,
                    GATE_THRESHOLD_MIN,
                    GATE_THRESHOLD_MAX,
                    0.0,
                    1.0,
                )
                if self._codec.noise_gate
                else 0.0
            ),
        )

    @alc_gain.setter
    def alc_gain(self, value: tuple) -> None:
        if len(value) < 4:
            raise ValueError("Invalid tuple object")

        self._codec.alc_target = map_range(value[0], 0.0, 1.0, ALC_TARGET_MIN, ALC_TARGET_MAX)

        self._codec.alc_max_gain = map_range(value[1], 0.0, 1.0, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX)

        self._codec.alc_min_gain = map_range(value[2], 0.0, 1.0, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX)

        if value[3] <= 0.0:
            self._codec.noise_gate = False
        elif not self._codec.noise_gate:
            self._codec.noise_gate = True
        self._codec.noise_gate_threshold = map_range(
            value[3], 0.0, 1.0, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX
        )

    @property
    def alc_time(self) -> tuple:
        """The time properties of the Automatic Level Control (ALC). Provided as a tuple in the
        format of attack time (from 0.006s to 6.14s), decay time (from 0.024s to 24.58s), and hold
        time (from 0.00267s to 43.691s) all as seconds within `float` values.

        :default: (0.024, 0.192, 0.0)
        """
        return (
            self._codec.alc_attack_time,
            self._codec.alc_decay_time,
            self._codec.alc_hold_time,
        )

    @alc_time.setter
    def alc_time(self, value: tuple) -> None:
        if len(value) < 3:
            raise ValueError("Invalid tuple object")

        self._codec.alc_attack_time = value[0]
        self._codec.alc_decay_time = value[1]
        self._codec.alc_hold_time = value[2]
