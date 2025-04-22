# ARDUINO_PICO_TMC2130
Control 3 TMC2130 stepper driver via USB comands (TMX2130 + RP2350 / RP2040)

This is configured to control stepper motors of a bambulab A1 mini.

## Hardware:

* 3x SilentStepStick - TMC2130 Stepper Motor Driver
* 1x Raspberry pi Pico2 dev board

## USB Protocol:

The board will be seen as a virtual COM port.

Send target position, max velocity and max acceleration for all axis (in step, step/s, step/s2)

`X 2000 1000 2000\n`
`Y 500 1000 2000\n` etc..

The driver will send at a chosen period the motor position and velocity:
`X 9030 0 Y 4340 0 Z 0 0`

You can update the target position on the fly. This is usefull for reactive control given other feedback inputs.

## Homing

The code includes a soft homing sequence that as been tuned for Bambulab A1 mini.
This uses stallGuard feature of the tmc2130 drivers.
