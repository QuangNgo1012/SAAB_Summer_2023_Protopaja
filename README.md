# SAAB_Summer_2023_Protopaja
Firmware of developed products in the course ELEC-D0301 - Protocamp in Summer 2023


The developed firmware of the project is built on top of the ble_app_uart example template from nRF5 SDK. The source code also take advantage of the Beacon Transmitter Sample Application, and RSSI Viewer app for nRF Connect for Desktop source code.
The developed firmware for our product introduces several significant features and functionalities that distinguish it from the original example template:
•	RSSI Signal Scanning: The firmware incorporates robust RSSI signal scanning capabilities, enabling the device to detect and assess the strength of nearby wireless signals. This functionality forms the foundation for our drone detection mechanism.
•	Drone Detection: Building upon the RSSI scanning capability, the firmware includes a simplified 2.4GHz drone detection module. Through signal analysis and pattern recognition, the device can identify the presence of nearby drones and distinguish them from other wireless sources.
•	Beaconing: This functionality allows the device to transmit periodic signals, which can be utilized for location tracking, proximity sensing, and other context-aware applications.
•	GPIO and PWM Control: To enhance the user experience, the firmware provides GPIO and PWM control features. These capabilities enable the device to trigger alerting mechanisms, such as lights or alarms, when a drone is detected. Users can customize these alerts based on their preferences.
•	Wireless DFU (Device Firmware Update): The firmware includes support for wireless Device Firmware Updates. This feature streamlines the process of updating the device's firmware over-the-air, enhancing convenience and enabling future improvements without requiring physical connections.

These combined features empower our product to serve as an advanced wireless detection and alert system. By intelligently detecting drones, transmitting signals, and offering customizable alerting options, our firmware adds substantial value to the original example template. This aligns with our project's objectives to create a comprehensive solution for drone detection and interaction.
We adopted a modular approach to develop and validate individual functionalities of the product. Subsequently, these separate functionalities were integrated into a unified codebase. In the subsequent section, we will present the results of our code unit testing.
