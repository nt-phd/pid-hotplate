# DIY Precision Hot Plate Controller

## Overview

In any scientific laboratory, a precision hot plate is an essential piece of equipment. Although sophisticated models equipped with magnetic stirrers can cost upwards of €1000, it is remarkably feasible to convert a simple €20 electric hotplate into a practical and precise heating tool using Arduino. By incorporating a highly accurate PID (Proportional-Integral-Derivative) controller, this DIY solution ensures maximum temperature stability, making it an economical alternative without compromising on functionality.

## Features

- **PID Temperature Control**: Utilizes a PID algorithm to maintain a stable and precise temperature, crucial for sensitive experiments and processes.
- **Serial Communication**: The hot plate can connect to a PC via USB for bidirectional temperature data communication. This feature allows for temperature profiling and real-time temperature plotting.
- **Cost-effective**: Transforms a budget electric hotplate into a high-performance scientific instrument.
- **Easy Configuration**: The system can be controlled using simple text commands through serial communication, which can be sent from any standard terminal software like CoolTerm.
- **Temperature Profiling**: Allows the user to program temperature ramps by simply uploading a text file, ideal for experiments requiring temperature gradients or for reflow soldering SMD components.

## Hardware Requirements

- Arduino Nano
- Adafruit SSD1306 OLED display for real-time temperature feedback
- MAX6675 thermocouple module for high accuracy temperature sensing
- Basic electric hotplate
- Miscellaneous: wires, breadboard, or custom PCB

## Software Requirements

- Arduino IDE for sketch upload
- Optionally, CoolTerm or similar serial terminal software for interaction and plotting

## Installation

1. **Assemble the Hardware**: Connect the MAX6675 thermocouple to the Arduino along with the OLED display and relay controlling the hotplate power.
2. **Load the Sketch**: Open the Arduino IDE, load the provided sketch, and upload it to your Arduino.
3. **Connect to PC**: Connect the Arduino to your PC using a USB cable.
4. **Set Up CoolTerm**: Configure CoolTerm to connect to the correct COM port at 9600 baud rate.
5. **Start Communicating**: Begin sending commands or uploading temperature profiles to control the hot plate.

## Usage

The system reads temperature from the thermocouple, displays it on the OLED, and adjusts the hotplate's heat output to maintain the desired temperature setpoint. Temperature setpoints and ramps can be adjusted by sending specific commands through the serial interface, allowing for manual control or automated temperature profiles.

## Contributing

Contributions to this project are welcome! Feel free to fork the repository, make your changes, and submit a pull request. Whether it's adding new features, fixing bugs, or improving documentation, your help is appreciated.

## License

This project is released under the MIT License. See the `LICENSE` file for more details.

## Acknowledgments

Special thanks to the Arduino community for the invaluable resources and Adafruit for the excellent libraries that made this project possible.

---

Enjoy your new, budget-friendly, and highly accurate DIY hot plate, perfect for scientific experiments or precision crafting such as SMD soldering!
