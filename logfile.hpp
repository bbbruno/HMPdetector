//===============================================================================//
// Name			: logfile.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.0
// Description	: Interface for the log file
//===============================================================================//

#include <fstream>

#include "publisher.hpp"

using namespace std;

#ifndef LOGFILE_HPP_
#define LOGFILE_HPP_

//! derivate class "LogFile", interface for the log file
class LogFile: public Publisher
{
	public:
		//! constructor
        //! (call to Publisher::Publisher constructor)
        //! @param[in] n    name of the publisher
        LogFile(string n): Publisher(n){}
        
        //! publish information
        //! @param[in] key     key of the information to be published
        //! @param[in] value   value of the information to be published
        void publish(const string key, const string value)
        {
            ofstream outputFile;
	        outputFile.open(name.c_str(), ios_base::app);
            outputFile<<key <<" " <<value <<endl;
            outputFile.close();
        }
};

#endif