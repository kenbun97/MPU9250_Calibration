# MPU9250_Calibration
This program was developed with the intention to communicate with and calibrate the MPU9250 Sensor. Gyroscope and acceleration data can also be read twice per second for easy debugging or other needs.

# Usage:
This program was developed and used on a Raspberry Pi. The MPU's I/O pins were wired as follows:
  MPU      RasPi
  VCC  ->  3V3
  GND  ->  GND
  SCL  ->  SCL1
  SDA  ->  SDA1
  EOA  ->  ~
  ECL  ->  ~
  ADO  ->  ~
  INT  ->  ~
  NCS  ->  ~
  FSYNC->  ~
