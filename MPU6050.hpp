//===============================================================================//
// Name			: MPU6050.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.0
// Description	: Inertial device driver for the SparkFun MPU6050 inertial sensor
//===============================================================================//

#include "device.hpp"

using namespace std;

#ifndef MPU6050_HPP_
#define MPU6050_HPP_

//! derivate class "MPU6050", driver of the SparkFun MPU6050 inertial sensor
class MPU6050: public Device
{
    private:
        static const int CODED_RANGE = 65535;         //!< range which sensed accelerations are mapped in 
        static const float SENSING_RANGE = 39.2266;   //!< sensing range of the sensor: [-2g; +2g]
        
	public:
        //! constructor
        //! (call to Device::Device constructor)
        //! @param[in] n    name of the device
        MPU6050(string n): Device(n){}
        
        //! extract actual acceleration values from an offline sample
        //! @param[in] &line    one line transmitted by the device
        //! @return             matrix with the tri-axial acceleration values in m/s^2
        mat extractActual(string &line)
        {
            // file format:
            // device_flag[int] ax[int] ay[int] az[int] gx[int] gy[int] gz[int] motion_flag[int]
            
            // read one line
            //DEBUG:cout <<"Line: " <<line <<endl;
            std::stringstream ss;
            ss<<line;
            int d, m, ax, ay, az, gx, gy, gz;
            ss>>d >>ax >>ay >>az >>gx >>gy >>gz >>m;
            
            // extract the acceleration values from the coded sample
            mat actualSample;
			actualSample <<ax <<ay <<az;
            mat noisySample = zeros<mat>(1, 3);
            noisySample(0, 0) = (actualSample(0, 0) / CODED_RANGE) * (SENSING_RANGE);
            noisySample(0, 1) = (actualSample(0, 1) / CODED_RANGE) * (SENSING_RANGE);
            noisySample(0, 2) = (actualSample(0, 2) / CODED_RANGE) * (SENSING_RANGE);
            //DEBUG:cout <<"noisySample: " <<noisySample(0,0) <<", "
            //DEBUG:                       <<noisySample(0,1) <<", "
            //DEBUG:                       <<noisySample(0,2) <<endl;
            
            return noisySample;
		}
        
        //! destructor
		~MPU6050()
		{
			//DEBUG:cout<<endl <<"Destroying MPU6050 object" <<endl;
		}
};

#endif
