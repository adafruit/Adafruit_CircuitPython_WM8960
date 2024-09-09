Simple test
-----------

Ensure your device works with this simple test that uses synthio to generate basic tones, outputs this signal to the codec via I2S, and plays back the DAC output with both headphones and speakers.

.. literalinclude:: ../examples/wm8960_simpletest.py
    :caption: examples/wm8960_simpletest.py
    :linenos:

Input
----------

Demonstrates analog stereo audio input bypass on INPUT1, INPUT2, and INPUT3 simultaneously.

.. literalinclude:: ../examples/wm8960_input.py
    :caption: examples/wm8960_input.py
    :linenos:

3D Enhance
--------------

Demonstrates 3D Enhance and ADC/DAC loopback feature of WM8960 Codec.

.. literalinclude:: ../examples/wm8960_3d_enhance.py
    :caption: examples/wm8960_3d_enhance.py
    :linenos:

Automatic Level Control
---------------------------

Demonstrates how to use the automatic level control feature of the WM8960 Codec.

.. literalinclude:: ../examples/wm8960_automatic_level_control.py
    :caption: examples/wm8960_automatic_level_control.py
    :linenos:
