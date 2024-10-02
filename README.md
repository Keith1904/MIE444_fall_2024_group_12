# MIE444_fall_2024_group_12
Repository for control code for MIE444.

general - Interfaces with all other classes and coordinates all operations.

motorSergeant - Issues all actuation commands to motors.
radioOperator - Communicates with Bluetooth mnodule.
recon.py - Collects sensor data and forwards it to scout and sentry.
scout - Performs Monte Carlo localization algorithm and instructs motorSergeant on where to go.
sentry - Responsible for avoiding obstacles and issuing emergency stops.

NO PUSHING TO MAIN
