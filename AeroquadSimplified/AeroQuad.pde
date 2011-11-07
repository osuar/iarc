//All includes were left alone.
#include <EEPROM.h>
#include <Wire.h>
#include "AeroQuad.h"
#include "I2C.h"
#include "PID.h"
#include "AQMath.h"
#include "Receiver.h"
#include "DataAcquisition.h"
#include "Accel.h"
#include "Gyro.h"
#include "Motors.h"

// Most all object declerations were left intact, but I assume that we will make and declare our own objects/variables.
// Create objects defined from Configuration Section above
#ifdef AeroQuad_v1
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_v1_IDG
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_v18
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_Mini
  Accel_AeroQuadMini accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  Receiver_AeroQuadMega receiver;
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_v2
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

#ifdef ArduCopter
  Gyro_ArduCopter gyro;
  Accel_ArduCopter accel;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
#endif

#ifdef AeroQuad_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  #include "FlightAngle.h"
//  FlightAngle_CompFilter tempFlightAngle;
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

#ifdef AeroQuadMega_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM tempFlightAngle;
  FlightAngle *flightAngle = &tempFlightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

#ifdef APM_OP_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM tempFlightAngle;
  FlightAngle *flightAngle = &tempFlightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif

  void (*processFlightControl)() = &processFlightControlPlusMode;


// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {


//Most setup and initializing code was deleted because I assume Daniel already has code for this that is specific to the XMEGA. Some code was left to structure the different things that will need to be initialized. Also might be good for us to modulate them like they are done below.


	motors.initialize();
	gyro.initialize();
	accel.initialize();
  
  	gyro.autoZero();
  	zeroIntegralError();

	flightAngle->initialize(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
 
	// Integral Limit for attitude mode
	// This overrides default set in readEEPROM()
	// Set for 1/2 max attitude command (+/-0.75 radians)
	// Rate integral not used for now
	PID[LEVELROLL].windupGuard = 0.375;
	PID[LEVELPITCH].windupGuard = 0.375;

  
    	altitude.initialize();
  

    	osd.initialize();
	
	previousTime = micros();	//Assuming Daniel has his own method of keeping loops on a schedule.
}



//The main running loop:

void loop () {
	
	//Daniels scheduler code will go here to maintain a 50Hz task loop.


        // measure critical sensors
	// Daniel has said he can measure, smooth and filter the sensor data himself. That would happen here.
        gyro.measure();
        accel.measure();
        
        //Send filtered and smoothed IMU data to the calculate function.
	// ****************** Calculate Absolute Angle *****************       
	flightAngle->calculate(gyro.getData(ROLL),                         \
                                 gyro.getData(PITCH),                      \
                                 gyro.getData(YAW),                        \
                                 accel.getData(XAXIS),                     \
                                 accel.getData(YAXIS),                     \
                                 accel.getData(ZAXIS),                     \
                                 0.0,                                      \
                                 0.0,                                      \
                                 0.0);
     
      
      // Combines external pilot commands and measured sensor data to generate motor commands
	processFlightControl();
       
	
	//They originally had the altitude measure and process in slower task loops. Not sure if we will want to do the same or not.	
	altitude.measure(); 
	processAltitudeHold();

	//OSD update was also in a slower task loop.
	osd.update();
  
}
