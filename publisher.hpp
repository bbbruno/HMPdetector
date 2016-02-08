//===============================================================================//
// Name			: publisher.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.0
// Description	: Interface for the publishing middleware (virtual base class)
//===============================================================================//

#include <string>

using namespace std;

#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

//! base class "Publisher" for the interface to the information publishing middleware
class Publisher
{
	public:
		string name;			//!< name of the middleware
        
        //! constructor
        //! @param[in] n    name of the device
		Publisher(string n)
        {
            name = n;
            //DEBUG:cout<<"Publisher: " <<name <<endl;
        }

		//! print publishing middleware information
		void printInfo()
        {
            cout<<"Publisher: " <<name <<endl;
        }
        
        //! publish information
        //! @param[in] key     key of the information to be published
        //! @param[in] value   value of the information to be published
        virtual void publish(const string key, const string value) = 0;
};

#endif
