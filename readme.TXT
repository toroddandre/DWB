Test code for DBW-controller..

Using for testing purpose, a ESP32-DEVKITM1-V1.0 piggybacked on a old board with two MC33931EK H-bridge drivers.
this board contains a 5-v regulator and a levelchanger for I/O's, the 5v supply two pots at the throttlebody, and two pots at the ACC-pedal.

The code should self calibrate the throttlebody for it's min/max values and use these values at each boot-up.
The calibration of the pedal should be done once and stored in memory. (Initiated by one I/O or similiar).
The resting point of the throttlebody is not 0 in the full travel range. unpowered the position of the throttle is about 5 degrees. 
so the tb should be able to go both ways, inputs from megasquirt 3x and pedal to set setpoint.
the input from MS is 0-5v for "idleadvance".

About the dual pots, these should be checked and throw en error code if outside set limits. 
say for example one wire is broken or pot is dead it should kill the throttlebody and halt the enginge.
