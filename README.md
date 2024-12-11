# ESP32-C6 Project (Bathroom Thermostat controller)

This is my first IoT project. The goal is to build a device that helps me control the thermostat of my bathroom via an ESP32-C6, and also receive some informations of the thermostat state (via Home Assistant with ZigBee).

* Three buttons will be used in this device
    * One will simply toggle the thermostat from Eco mode to Comfort mode
    * Second will light the embedded RBG led to let me know the state of the thermostat
        * Green in Eco mode
        * Red in Comfort mode
    * Third will be used to reset the device if something went wrong with the ZigBee connection for instance
* The embedded RGB led will stay green or red for 10 seconds when the dedicated button is pressed, hence, this will save power if I decide to use this device on battery