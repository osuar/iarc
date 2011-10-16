/**

@author Daniel Sidlauskas Miller

Example of I2C communication with the inertial measurement unit utilized by OSURC Autonomous Aerial Team

*/

/**Include standard library for basic logic functions, avr compiler for xmega constants, and twi master driver because that is what is utilized for I2C communication*/
#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"

/**Define I2C addresses, 7 bit address right shifted because drivers accept that format*/
#define ACCEL (0xA6 >> 1)

/**Define register addresses within Accelerometer*/
#define READ 0x32
#define CONFIG 0x2D
#define SETTING 0x08


