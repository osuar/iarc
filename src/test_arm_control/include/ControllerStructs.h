#ifndef CONTROLLERSTRUCTS_H
#define CONTROLLERSTRUCTS_H

/** @file
  * @brief Defines several structs to be used by controllers.
  */

namespace osurc {

namespace controller {

/** @brief Contains information about new gyro data.
  */
struct GyroData {
	/** @brief The approximate time at which this data was taken.
	  * In ns.
	  */
	int64_t timestamp;

	/** @brief The angular position of the arm.
	  * This is measured off of the x axis.
	  */
	double  pos;

	/** @brief The rotational velocity of the arm.
	  * In rad/s
	  */
	double  rate;
};

/** @brief Contains information about new accelerometer data.
  */
struct AccelData {
	/** @brief The approximate time at which this data was taken.
	  * In ns.
	  */
	int64_t timestamp;

	/** @brief The x acceleration.
	  * In m/s^2
	  */
	double  xAccel;
	double  yAccel;
	double  zAccel;
}

}

}

#endif // CONTROLLERSTRUCTS_H
