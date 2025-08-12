# Project evSim 
## Purpose description
Simulate a ev connected to a Wallbox for testing purposes.

For testing the behaviour of a AC walbox, respecively charging station, for e-mobility a car simulator is needed.
This project aims to provide such a simulator. In addition for test automation this simulator can be controlled via modbus protocoll using a RS485 connection.
You should be aware about the ISO states and the underlying physical protocoll used on the CS and PP line of a type-2 conector.

The project uses a attiny1616 controller to switch a couple of relays to simulate the voltage levels on the signal lines.
The relais are driven by a TPIC6B595 shift register which has the capability to drive the relays.
The RS485 lines are driven by a MAX 485 ECSA transmitter. The PWM is injected via an optocoppler  LTV 817B.

## As user interface there are:
Two buttons to switch states and cable attributes
An OLED display for the states and the measured PWM duty cycle
ws2812 like LED stripe also to indicate states and PWM duty cycle, respectively the current level.

## Schematic and PCB:
The project contains also a schematic and a PCB in KiCad format with the following features:
The evSim contains also an additional FET incl. a flyback diode, to drive an external releay, e.g. to switch the Walbox on and off.
The evSim provides an additional input at the plug connector intended for current on check.
The evSim contains two simulatior entities to be able to test a twin wallbox.
The evSim contains a voltage regulator for 5V to be independant of the power supply.
The evSim contains a coding switch for the modbus-ID.

For connection screw connectors are used.

### Note:
Currently this is done as a prototype and for usage in a real test environment the following improvements could be done.
- Add some more Cs or Ls for hardening against radiation and spikes.
- Add bigger voltage regulator, or use a tree pin DC/DC converter.
- Make PCB a bit larger, avoid SMD on the backside, add proper conector for power, Modbus and additional functions.

## Not tested/implemented yet:
Additional FET
Selection of modbus-ID via coding switch. Idea is to double use some GPIO lines and read the input at start-up.
Some supporting modbus registers.
Read in voltage on phase L1 with FinA/FinB, just with an opto and capacitive devider to be able to check that the walbox has realy turned on power. (But caution here you touch high volate, I would place the opto and devider on a separate small PCB)

