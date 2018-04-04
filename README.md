# Closed-Loop-SPINDLE-CONTROL-GRBL
Developing a closed loop spindle controller to run on arduino and maintain spindle rpm to the set point requested by GRBL (via reading pwm output)

Desired inputs and outputs

Inputs
0-5V PWM signal from GRBL to creat rpm setpoint for PID loop
0-5V Signal from potentiometer to allow for manual rpm setpoint or fine adjustment of GRBL derived set point.
0-5V Signal from IR optical sensor to detect actual spindle rpm
Emergancy Stop

Outputs

PWM signal to drive motor controller
Overload protection
