# ArduinoRobot
Assembling and coding a simple robot with the Arduino environment.

Robot description:

    Hardware:
        1 Arduino UNO
        2 single channel encoders
        1 L298 motor driver
        2 bi-directionnal motors + trailing wheel (differential drive)
        1 HC-SR04 distance sensor
        1 battery pack (x6 AA batteries)
    Software:
        Coded in Arduino IDE

Notes:

    Risk of short circuit on L298 if not careful with the state (HIGH or LOW) applied to its input pins. Risk of short circuit.
    Enable inputs on L298 are always active.
