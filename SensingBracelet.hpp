//===============================================================================//
// Name			: SensingBracelet.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.1
// Description	: Human Motion Primitives reasoner module (on-line / off-line)
//===============================================================================//

#include <iostream>
#include <string>
#include <vector>

#ifdef __cplusplus
extern "C"
{
#endif	
	#include <peiskernel/peiskernel_mt.h>
#ifdef __cplusplus
}
#endif

using namespace std;

#ifndef SENSINGBRACELET_HPP_
#define SENSINGBRACELET_HPP_

//! definition of the class "Interval" for the recognition of an HMP
class Interval
{
	public:
		string HMPname;			//!< name of the monitored HMP
		bool open;	   			//!< flag for open/closed interval
		int start[2];			//!< starting sample of the interval:
								//!< [starting_sample;starting_sample]
		int end[2];	   			//!< ending sample of the interval:
				   				//!< 1. open --> [current;+inf]
				   				//!< 2. closed --> [ending_sample;ending_sample]
		float possibility;		//!< highest possibility value within the interval
		float refPossibility;	//!< starting sample possibility value
		int risingTime;			//!< sample[highest_possibility] - starting_sample

		//! constructor
		Interval(string HMPn);

		//! set the starting sample
		void setStart(int sample);

		//! set the ending sample
		void setEnd(int sample);

		//! start a new interval
		void startInterval(float &this_pos, int &nSamples, float &past_pos);

		//! print the interval status
		void printStatus();

		//! destructor
		~Interval()
		{
			//DEBUG: cout<<endl <<"Destroying Interval object" <<endl;
		}
};

//! definition of the class "SensingBracelet"
class SensingBracelet
{
	protected:
		//! publish the SensingBracelet tuples on PEIS
		void publishSensingBracelet(int& i);

		//! report the SensingBracelet tuples on a file
		void reportSensingBracelet(int& i, string rF);

	public:
		string datasetFolder;		//!< folder containing the models
		int nbM;					//!< number of considered models
		vector<Interval> setI;		//!< set of open intervals

		//! constructor
		SensingBracelet(string dF);

		//! print set information
		void printSetStatus();

		//! set all the reasoner variables and initialize the intervals
		void buildSet(string dF);

		// update the interval of activation for one model (a-posteriori)
		void updateInterval(int& i,int& nS,float &p,float &pp,int f,string rF);

        // on-line update of interval of activation (quick & dirty)
		void simpleInterval(int& i,int& nS,float &p,float &pp);

		//! perform off-line analysis of pre-recorded models possibilities
		void offlineSensingBracelet(string path, string testFile);

		//! perform on-line full analysis of a data stream (classifier inside)
		void onlineSensingBracelet(char* port);

		//! destructor
		~SensingBracelet()
		{
			setI.clear();
			//DEBUG: cout<<endl <<"Destroying SensingBracelet object" <<endl;
		}
};

#endif
