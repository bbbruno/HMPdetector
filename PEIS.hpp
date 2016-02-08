//===============================================================================//
// Name			: PEIS.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.0
// Description	: Interface for the PEIS middleware
//===============================================================================//

#include "publisher.h"

#ifdef __cplusplus
extern "C"
{
#endif	
	#include <peiskernel/peiskernel_mt.h>
#ifdef __cplusplus
}
#endif

using namespace std;

#ifndef PEIS_HPP_
#define PEIS_HPP_

//! derivate class "PEIS", interface for the PEIS middleware
class PEIS: public Publisher
{
	public:
		//! constructor
        //! (call to Publisher::Publisher constructor)
        //! @param[in] n    name of the publisher
        PEIS(string n): Publisher(n){}
        
        //! publish information
        //! @param[in] key     key of the information to be published
        //! @param[in] value   value of the information to be published
        void publish(const string key, const string value)
        {
            string peisKey = "HMPdetector." + key;
            peiskmt_setStringTuple(peisKey.c_str(), value);
        }
};

#endif