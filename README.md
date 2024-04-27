Weather Station Mini Project  
by Monthon Paul, Issei Sawano and Hong Chen  

The main part of our project is having two senors sending the following information: temperature, humidity and pressure to the ESP32.  
These information are sent via I2C and the display should cycle though each one every few seconds.  

For the display, we are using a OLED display (SSD1306) which receives information from the ESP32.  
This is how we choose to display the information sent by the sensors and received by the ESP32.  

The second part of our project is having keyboard presses switch to the display that you want.  
We decided to have: P for pressure, T for temperature and H for humidity.  
The keyboard presses are sent via UART and the keyboard presses are read using event driven interrupts.  

The three lab components used are I2C, UART and interrupts.

### Hardware Required

* An ESP32-S3 development board
* An STM32f072 development MCU
* An SSD1306 OLED LCD, with I2C interface
* An BMP280 with I2C interface
* DHT11 sensor
* An USB cable for power supply and programming

### Hardware Connection

The connection between ESP Board and the LCD is as follows:

```text
      ESP Board                       OLED LCD (I2C)
+------------------+              +-------------------+
|               GND+--------------+GND                |
|                  |              |                   |
|               3V3+--------------+VCC                |
|                  |              |                   |
|               SDA+--------------+SDA                |
|                  |              |                   |
|               SCL+--------------+SCL                |
+------------------+              +-------------------+
```

The connection between ESP Board and the BMP280 is as follows:

```text
      ESP Board                       BMP280 (I2C)
+------------------+              +-------------------+
|               GND+--------------+GND                |
|                  |              |                   |
|               3V3+--------------+3Vo                |
|                  |              |                   |
|               SDA+--------------+SDI                |
|                  |              |                   |
|               SCL+--------------+SCK                |
+------------------+              +-------------------+
```

The connection between ESP Board and the DHT11 is as follows:

```text
      ESP Board                        STM32 Board
+------------------+              +-------------------+
|               GND+--------------+GND                |
|                  |              |                   |
|               3V3+--------------+3V3                |
|                  |              |                   |
|              PIN5+--------------+GPIO               |
|                  |              |S                  |
+------------------+              +-------------------+
```

The connection between ESP Board and the STM32 is as follows:

```text
      ESP Board                           DHT11
+------------------+              +-------------------+
|                  |              |                   |
|               GND+--------------+GND                |
|                  |              |                   |
|              PB10+--------------+RX                 |
|                  |              |                   |
+------------------+              +-------------------+
```
## Libraries

[BMP280](https://github.com/utkumaden/esp-idf-bmx280)

[DHT11](https://github.com/Anacron-sec/esp32-DHT11)


