# Digital-Thermometer
Built a Digital Thermometer using C (programming language) and Arduino Uno microcontroller board which:
- Has all its components set up and wired on a breadboard.
- Displays the temperature in degrees Fahrenheit on the LCD with 0.1 degree precision.
- A servo motor is used to rotate a pointer to indicate the current temperature (either local or remote) on a dial that shows temperatures from 40 to 100 degrees Fahrenheit.
- Utilizes a DS18B20 temperature sensor to read both local and remote temperatures.
- Uses two push buttons to select remote or local mode.
- Is able to read the remote temperature through a serial interface (RS-232).
- Uses a tri-state buffer to eliminate contention on RX line.
- Depending on the temperature, the red and green LEDs are programmed to blink using Timer Interrupts.
- Has a rotary encoder which can be used to set the temperature threshold.
- Alerts the user through a tone generated by the buzzer and displays a message on the LCD attached to Arduino Uno board whenever the temperature goes beyond the threshold set by the user.
- The temperature threshold is stored in the Arduino's EEPROM non-volatile memory so the values are retained even if the power is turned off.
- The programming utilizes Timer interrupts instead of busy-looping in order to check for all the interrupts and changes.