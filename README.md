### Wall2---Face-Mask-Collector
##1. User Manual

#1.1. Required Hardware and Software

To begin this user manual, the components needed to build the prototype are listed below:

Mechanical Components:

• 2 MDF boards, 6mm thick, 600 x 440 mm (one for each floor).

• 2 rigid couplings, 5mm to 8mm.

• 2 rigid couplings, 8mm to 8mm.

• 4 rigid couplings, 6mm to 8mm.

• 4 Monster Truck toy wheels for sand.

• 4 dowels with nuts.

• 4 axles, 8mm in diameter and 8cm long.

• 2 kg of PLA 3D printing material.

• 1 trash can, 190mm wide.

• Screws and nuts of various diameters.

• 4 L-type couplings for Pololu DC motors.

• 4 Bearing Mounts

Electrical Components:

• ESP32 dev kit

• Arduino MEGA

• Raspberry Pi 4b

• 6 PC817 Optocouplers

• 6 Pre-drilled Breadboards (11cm x 6cm, 6cm x 4cm, 7cm x 3cm x4)
• 12 10kΩ Resistors

• Male Pin Header

• Female Pin Header

• 2 XH-M400 8A Step-Down Voltage Regulators

• 2 L298N H-Bridges

• 4 Pololu 12V DC Motors with Encoders

• 4 MG995 Servo Motors

• 1 ADXL345 Accelerometer-Gyroscope
• 2 11.1V 2200mAh LiPo Batteries

• 1 x 14.8V 5200mAh LiPo battery

• 3 x 2-pin 21x15mm switches

• 2 x XT60 female connectors

• 1 x XT60U-F female connector

• 1 x 640x480 pixel webcam

• 1 x HC-SR04 sonar sensor

• Logic jumper wires and power cables

• 2 x Arduino-type USB cables

• 1 x USB Type-C cable

• Cable ties

Software, knowledge, and computing tools requirements

• Arduino IDE

• 1 computer with a Python programming environment (preferably VS Code)

• Operating system installation software (preferably Rufus)

• Raspberry Pi operating system

• Minimum Python libraries: NumPy, Serial, OpenCV, Imutils, Time, SciPy
• Minimum Arduino Libraries: Simple_MPU6050, Servo.

Other necessary tools

• 3D printer.

• CNC laser cutter.

• Allen wrench set.

• Screwdriver set.

• Pliers.

• Hot glue gun.

#1.2. Installation, assembly, and configuration

Assembly:

• Cut the MDF wood panels with the laser cutter. The first panel is cut according to the design in the Piso1_v1.dxf file, and the second according to the Piso2_v2.dxf file. These pieces correspond to floors one and two of the prototype.

• 3D print the component housings using the following files: motor_housing.ipt for the motors, L298N_Case_1.0.3b.stl and L298N_Case_1.0.3c.stl for the H-bridges, arduino_mega_case.ipt for the Arduino Mega, and battery_housing.ipt and lipo_housing.ipt for the batteries.

• On the board corresponding to the first level, assemble the DC motors with their respective L-shaped brackets. Then, place a 6-8mm coupling between the motor shaft and an 8mm shaft, one bearing per motor, and place them inside their respective housings and screw them in.

• Place the H-bridges inside their respective housings and screw them between the motors (one for every two motors).

• Using a hot glue gun, glue three optocouplers next to each H-bridge. The optocouplers should be arranged in parallel from the H-bridges towards the center of the chassis. • Using the pre-drilled boards, design the PCBs described in Appendix 7.3 and carefully attach them to the chassis as needed.

• Place the Arduino Mega in its housing and assemble it to the chassis in the most suitable location (preferably near the optocouplers).

• Connect the couplings to the wheels and assemble them to the shaft of each motor.

• Place the batteries in their housings and assemble them on the back of the motor.

• Assemble the arm with the respective servos and position them inside according to the arm's schematics presented in the low-level diagram.

• Place the servos on the arm's base in their respective housings and then attach the base to the arm.

• Assemble the arm to the chassis.

Connections:

• Connect an on/off switch to each battery.

• Connect four wires (two power and two ground) to the output of a battery. Connect these wires to the two H-bridges (one power and one ground for each H-bridge).
• Connect six wires from the H-bridges, corresponding to logic pins, to the optocouplers.
• Connect six wires from the optocouplers to a PCB (available in Appendix 7.3). This PCB receives the signals from the optocouplers and connects to the Arduino Mega at the corresponding pins (available in the low-level diagram).
• Connect the motor encoders to the PCB in a way that allows easy connection to the ESP32.
• The ESP32 receives the encoders, a power wire, and a ground wire.
• Once the connections are made and the second level is assembled, connect the Raspberry Pi to the Arduino Mega via serial communication. This will allow you to send information to the Arduino to control the motors.

• Connect the webcam to the Raspberry Pi via serial communication.

Software Installation:

• Go to the official Raspberry Pi website to download the operating system.

• Download the OS called "Raspberry Pi OS (64-bit)".

• Download Rufus from its official website.

• Open Rufus, select the downloaded operating system, and connect the SD card where you will create the partition. Start the installation process.

• Once the process is complete, remove the SD card and insert it into the Raspberry Pi.

• Connect the Raspberry Pi to a monitor via HDMI, and connect a mouse and keyboard. Then, power on the Raspberry Pi.

• Follow the installation instructions.

• Open Rufus, select the downloaded operating system, and connect the mouse and keyboard. • After installing the OS, open a terminal and run the following commands:

$ pip 3 install numpy
$ pip 3 install script
$ pip 3 install multils
$ pip 3 install opencv-python

• Transfer the .py files from the code folder.

• Upload the Wall2_MARK5.ino file to the Arduino Mega and the Wall2_ESP32.ino file to the ESP32 using the Arduino IDE.

• Connect the Arduino Mega and the ESP32 to the Raspberry Pi using a USB connection.

#1.3. Instructions for use
• Turn on the power switch for the Raspberry Pi and the other controllers.

• Turn on the power switches for the motors and servos.

• Run the code “Wall2_MARK5.py”
