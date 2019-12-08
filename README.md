# J1772 EV Simulator

This is the source code for the EV Sim Remote version of https://hackaday.io/project/9051-j1772-ev-simulator

You connect your computer up to the USB port and you'll see a serial port. Open this port and configure it for 9600
baud, 8N1. You'll start getting JSON frames that look like this:

{ state=A, state_changes=2000, low_count=50000, high_count=100000, low_adc=172, high_adc=844 }

This represents the raw data for 1 second of sampling of the pilot. "state" is the simulated vehicle's state - A through
D. This is changed by sending the single characters A through D to the sim. "state changes" is the number of times the pilot
crossed zero volts - double the frequency. low_count is the number of samples that were low and high_count is the
number of them that were high. To get a duty cycle as a percentage, it's (high_count / (high_count + low_count)). Note
that if the frequency is zero, either high_count or low_count should be non-zero and the other zero, so the duty cycle
will be either 0% or 100%.

The low_adc and high_adc values are the negative and positive peak readings of the ADC. To turn these into a voltage,
scale them by first adding the PILOT_READ_OFFSET value and then multiplying the sum by the PILOT_READ_SCALE factor.

The PILOT_READ_OFFSET is -556 and the PILOT_READ_SCALE value is 32. The result in in mV.

What you should see is that the negative peak remains close to -12 volts and the positive peak will depend on what
state the EV SIM is in.

To change states, simply send the single character "A", "B", "C" or "D". State A's impedance is infinite, state B is
2.7 kΩ, state C is 882 Ω and state D is 240 Ω. These are supposed to correspond to positive peak voltages of +12, +9,
+6 and +3 volts, respectively.

## Arduino LCD variants

The EV_Sim and EV_Sim_84 directories have Arduino sketches for the LCD-bearing hardware variants. The EV_Sim_84 directory
is for the ATTiny84 backpack variant that has the LCD directly connected and uses LiquidCrystal. The EV_Sim directory
is for the older ATTiny85 variant that uses either serial output or an i2c connected LCD backpack (either the Adafruit
or OpenEVSE versions work).

Either requires that support for "bare" ATTiny chips be added to your Arduino IDE. Be sure to configure Arduino and fuse
your chip for the 16 MHz crystal.
