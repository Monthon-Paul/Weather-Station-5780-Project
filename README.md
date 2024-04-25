Weather Station Mini Project  
by Monthon Paul, Issei Sawano and Hong Chen  

The main part of our project is having two senors sending the following information: temperature, humidity and pressure to the ESP32.  
These information are sent via I2C and the display should cycle though each one every few seconds.  

For the display, we are using a OLED display which receives information from the ESP32.  
This is how we choose to display the information sent by the sensors and received by the ESP32.  

The second part of our project is having keyboard presses switch to the display that you want.  
We decided to have: P for pressure, T for temperature and H for humidity.  
The keyboard presses are sent via UART and the keyboard presses are read using event driven interrupts.  

The three lab components used are I2C, UART and interrupts.  


