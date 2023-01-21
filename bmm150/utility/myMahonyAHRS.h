//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef myMahonyAHRS_h
#define myMahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

//volatile float twoKp;			// 2 * proportional gain (Kp)
//volatile float twoKi;			// 2 * integral gain (Ki)
//volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float myKp;
extern volatile float myKi;
extern volatile float q[4];

namespace myIMU {
//---------------------------------------------------------------------------------------------------
// Function declarations
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltaT=0.04);
//void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float *pitch,float *roll,float *yaw, float deltaT=0.04);
float invSqrt(float x);
}
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
