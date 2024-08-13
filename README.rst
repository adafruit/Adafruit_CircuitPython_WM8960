Introduction
============
.. image:: https://readthedocs.org/projects/adafruit-circuitpython-wm8960/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/wm8960/en/latest/
    :alt: Documentation Status

.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_WM8960/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_WM8960/actions
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython driver for WM8960 Stereo CODEC

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading `the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_ or individual libraries can be installed using `circup <https://github.com/adafruit/circup>`_.

This library is designed to help facilite the I2C connection with a WM8960 audio codec to configure it to be used for DAC, ADC, headphone and speaker functionality.

This library has been tested using an RP2040 on CircuitPython 9.1.1 and the `SparkFun Audio Codec PBreakout - WM8960 <https://www.sparkfun.com/products/21250>`_.

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included as a standard element. Stay tuned for PyPI availability!

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from PyPI <https://pypi.org/project/adafruit-circuitpython-wm8960/>`_.

To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-wm8960

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-wm8960

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install adafruit-circuitpython-wm8960

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install adafruit_wm8960

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    # Monitor Stereo MIC Input:
    # MIC (INPUT1) => PGA => Boost Mixer => Output Mixer => Headphones
    import board, adafruit_wm8960
    codec = adafruit_wm8960.WM8960(board.I2C())
    codec.enableMIC()
    codec.enableMN1()
    codec.enableINMUTE()
    codec.setMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)
    codec.connectMIC2B()
    codec.enableAIN()
    codec.enableB2O()
    codec.setB2OVOL(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_0DB)
    codec.enableOMIX()
    codec.configureHeadphones()

Documentation
=============
API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/adafruit_wm8960/en/latest/>`_.

Contributing
============
Contributions are welcome! Please read our `Code of Conduct <https://github.com/adafruit/Adafruit_CircuitPython_WM8960/blob/HEAD/CODE_OF_CONDUCT.md>`_ before contributing to help this project stay welcoming.
