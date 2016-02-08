//===============================================================================//
// Name			: SensingBracelet.cpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.1
// Description	: Human Motion Primitives reasoner module (on-line / off-line)
//===============================================================================//

#include <fstream>
#include <limits>

#include "classifier.hpp"
#include "SensingBracelet.hpp"
#include "libs/SerialStream.h"

#define pINF std::numeric_limits<int>::max();

using namespace boost::posix_time;

//! constructor
//! @param HMPn:	name of the motion primitive
Interval::Interval(string HMPn)
{
	//DEBUG: cout<<endl <<"Creating Interval object" <<endl;
	HMPname = HMPn;
	open = false;
	start[0] = -1;
	start[1] = -1;
	end[0] = -1;
	end[1] = pINF;
	possibility = 0;
	refPossibility = 0;
	risingTime = 0;
}

//! set the starting sample
//! @param sample:	starting sample
//! @return:		---
void Interval::setStart(int sample)
{
	start[0] = sample;
	start[1] = sample;
}

//! set the ending sample
//! @param sample:	ending sample
//! @return:		---
void Interval::setEnd(int sample)
{
	end[0] = sample;
	end[1] = sample;
}

//! start a new interval
//! @param &this_pos:	possibility of the monitored model for the current sample
//! @param &nSamples:	index of the current sample
//! @param &past_pos:	possibility of the monitored model for the previous sample
//! @return:			---
void Interval::startInterval(float &this_pos, int &nSamples, float &past_pos)
{
	open = true;
	start[0] = nSamples;
	start[1] = nSamples;
	end[0] = 0;
	end[1] = pINF;
	possibility = this_pos;
	refPossibility = past_pos;
	risingTime = 1;
}

//! print the interval status
//! @param:		---
//! @return:	---
void Interval::printStatus()
{
	cout<<"HMPname = " <<HMPname <<endl;
	cout<<"status = " <<open <<endl;
	cout<<"starting sample = [" <<start[0] <<";" <<start[1] <<"]" <<endl;
	cout<<"ending sample = [" <<end[0] <<";" <<end[1] <<"]" <<endl;
	cout<<"maximum possibility = " <<possibility <<endl;
	cout<<"starting possibility = " <<refPossibility <<endl;
	cout<<"rising time = " <<risingTime <<endl;
}

//! constructor
//! @param dF:	folder containing the modelling dataset
SensingBracelet::SensingBracelet(string dF)
{
	string one_HMPn;
	float temp1;
	float temp2;
	float temp3;
	Interval *one_interval;

	datasetFolder = "./Models/" + dF + "/";
	string fileName = datasetFolder + "Classifierconfig.txt";
	//DEBUG: cout<<"config file: " <<fileName <<endl;
	ifstream configFile(fileName.c_str());
	configFile >>nbM;
	//DEBUG: cout<<"nbM: " <<nbM <<endl;
	for(int i=0; i< nbM; i++)
	{
		configFile>>one_HMPn >>temp1 >>temp2 >>temp3;
		cout<<"Interval: " <<one_HMPn <<endl;
		one_interval = new Interval(one_HMPn);
		//DEBUG: one_interval->printStatus();
		setI.push_back(*one_interval);
	}
	configFile.close();
}

//! print set information
//! @return:	---
void SensingBracelet::printSetStatus()
{
	for(int i=0; i<nbM; i++)
		setI[i].printStatus();
}

//! set all the SensingBracelet variables and initialize the intervals
//! @param dF:		name of the dataset to be considered
//! @return:		---
void SensingBracelet::buildSet(string dF)
{
	string one_HMPn;
	float temp1;
	float temp2;
	float temp3;
	Interval *one_interval;

	// delete the existing intervals
	setI.clear();

	// load the new set of intervals
	datasetFolder = "./Models/" + dF + "/";
	string fileName = datasetFolder + "Classifierconfig.txt";
	//DEBUG: cout<<"config file: " <<fileName <<endl;
	ifstream configFile(fileName.c_str());
	configFile >>nbM;
	//DEBUG: cout<<"nbM: " <<nbM <<endl;
	for(int i=0; i< nbM; i++)
	{
		configFile>>one_HMPn >>temp1 >>temp2 >>temp3;
		cout<<"Interval: " <<one_HMPn <<endl;
		one_interval = new Interval(one_HMPn);
		//DEBUG: one_interval->printStatus();
		setI.push_back(*one_interval);
	}
	configFile.close();
}

//! publish the SensingBracelet tuples on PEIS
//! @param &i:	reference to the index of the interval to be published
//! @return:	---
void SensingBracelet::publishSensingBracelet(int& i)
{
	// HMP.[HMPname]
	string IntervalName = "Bracelet.HMP." + setI[i].HMPname;
	const char* sIntervalName = IntervalName.c_str();
	//DEBUG: cout<<"IntervalName: " <<sIntervalName <<endl;
	stringstream ntos;
	ntos<<setI[i].start[0];
	string IntervalData;
	IntervalData = "[" + ntos.str() + ";";
	ntos.str(string());
	ntos<<setI[i].start[1];
	IntervalData = IntervalData + ntos.str() + "]-[";
	ntos.str(string());
	ntos<<setI[i].end[0];
	IntervalData = IntervalData + ntos.str() + ";";
	ntos.str(string());
	ntos<<setI[i].end[1];
	IntervalData = IntervalData + ntos.str() + "]: ";
	ntos.str(string());
	ntos<<setI[i].possibility;
	IntervalData = IntervalData + ntos.str();
	const char* sIntervalData = IntervalData.c_str();
	//DEBUG: cout<<"IntervalData: " <<sIntervalData <<endl;
	peiskmt_setStringTuple(sIntervalName, sIntervalData);
}

//! report the SensingBracelet tuples on a file
//! @param &i:	reference to the index of the interval to be published
//! @param rF:	name of the result file
//! @return:	---
void SensingBracelet::reportSensingBracelet(int& i, string rF)
{
	// create result file
	ofstream outputFile;
	outputFile.open(rF.c_str(), ios_base::app);

	// HMP.[HMPname]
	string IntervalName = "HMP." + setI[i].HMPname;
	outputFile<<IntervalName.c_str() <<" "
			  <<"[" <<setI[i].start[0] <<";" <<setI[i].start[1] <<"]-"
			  <<"[" <<setI[i].end[0] <<";" <<setI[i].end[1] <<"]: "
			  <<setI[i].possibility <<endl;
	
	// close the result file
	outputFile.close();
}

//! update the interval of activation for one model (a-posteriori)
//! @param &i:			reference to the index of the considered model
//! @param &nSamples:	reference to the current sample
//! @param &p:			reference to the model possibility at current sample
//! @param &pp:			reference to the model possibility at previous sample
//! @param f:			flag for the publishing method (0: file, 1:PEIS)
//! @param rF:			name of the result file
//! @return:			---
void SensingBracelet::updateInterval(int& i,int& nS,float &p,float &pp,int f,string rF)
{
	// 1) active
	if(p > 0)
	{
		// 1.1) new active: create a new interval
		if(setI[i].open == false)
		{
			setI[i].startInterval(p, nS, pp);
			cout<<setI[i].HMPname <<": open interval" <<endl;
		}
		// 1.2.1) already active and rising
		else if(p >= setI[i].possibility)
		{
			setI[i].end[0] = nS;
			setI[i].possibility = p;
			setI[i].risingTime++;
			//DEBUG: cout<<"Rising" <<endl;
		}
		// 1.2.2) already active and descending
		else
		{
			setI[i].risingTime--;
			// if the descent is not smooth, the interval is NOT valid
			if(p > pp)
			{
				setI[i].open = false;
				cout<<setI[i].HMPname <<": false positive - NO bell shape" <<endl;
			}
			// check the simmetry of the possibility curve
			if((setI[i].risingTime == 0) && (p < setI[i].refPossibility))
			{
				setI[i].open = false;
				cout<<setI[i].HMPname <<": false positive - NO symmetry" <<endl;
			}
			// if the above constraints are satisfied, publish the result on PEIS
			if((setI[i].risingTime >= 0) && (p == setI[i].refPossibility))
			{
				// define the ending time of the interval
				setI[i].setEnd(setI[i].end[0]);

				// publish the interval info
				if (f == 0)
					// on file
					reportSensingBracelet(i, rF);
				if (f == 1)
					// on PEIS
					publishSensingBracelet(i);

				// close the interval
				setI[i].open = false;
			}
		}
	}
	// 2) inactive
	// (if the interval should be closed: open == true && refPossibility == 0)
	else if((setI[i].open == true) && (setI[i].refPossibility == 0))
	{
		// define the ending time of the interval
		setI[i].setEnd(setI[i].end[0]);

		// publish the interval info
		if (f == 0)
			// on file
			reportSensingBracelet(i, rF);
		if (f == 1)
			// on PEIS
			publishSensingBracelet(i);
		
		// close the interval
		setI[i].open = false;
	}
}

// on-line update of interval of activation (quick & dirty)
//! @param &i:			reference to the index of the considered model
//! @param &nSamples:	reference to the current sample
//! @param &p:			reference to the model possibility at current sample
//! @param &pp:			reference to the model possibility at previous sample
//! @return:			---
void SensingBracelet::simpleInterval(int& i,int& nS,float &p,float &pp)
{
	// 1) active
	if(p > 0.8)
	{
		// 1.1) new active: create a new interval
		if(setI[i].open == false)
		{
			setI[i].startInterval(p, nS, pp);
			cout<<"I see: " <<setI[i].HMPname <<endl;
            string IntervalName = "Bracelet.HMP." + setI[i].HMPname;
            stringstream ntos;
	        ntos<<p;
	        string IntervalData;
	        IntervalData = ntos.str();
            peiskmt_setStringTuple(IntervalName.c_str(), IntervalData.c_str());
		}
		// 1.2) already active - update ending time of the interval
		else
		{
			setI[i].end[0] = nS;
            if(p >= setI[i].possibility)
			    setI[i].possibility = p;
            string IntervalName = "Bracelet.HMP." + setI[i].HMPname;
            stringstream ntos;
	        ntos<<p;
	        string IntervalData;
	        IntervalData = ntos.str();
            peiskmt_setStringTuple(IntervalName.c_str(), IntervalData.c_str());
		}
	}
	// 2) inactive
	// (if the interval should be closed: open == true)
	else if(setI[i].open == true)
	{
		// define the ending time of the interval
		setI[i].setEnd(setI[i].end[0]);

		// update the info on PEIS
        string IntervalName = "Bracelet.HMP." + setI[i].HMPname;
        peiskmt_setStringTuple(IntervalName.c_str(), "0");
		
		// close the interval
		setI[i].open = false;
	}
}

//! perform off-line analysis of pre-recorded models possibilities
//! @param path:		path to the possibilities & results file
//! @param testFile:	name of the possibilities file to be analyzed
//! @return:			---
void SensingBracelet::offlineSensingBracelet(string path, string testFile)
{
	vector<float> possibilities;		// models possibilities
	vector<float> past_possibilities;	// models previous possibilities
	int nSamples = 0;					// number of acquired samples

	// create result file (same folder of the possibilities file)
	string rF = path + "Rres_" + testFile;

	// initialize the possibilities and PREVIOUSpossibilities
	for(int i = 0; i < nbM; i++)
	{
		possibilities.push_back(0);
		past_possibilities.push_back(0);
	}

	// read & analyze recorded possibilities one line at the time
	string pFile = path + testFile;
	ifstream pf(pFile.c_str());
	cout <<"Reading possibilities in: " <<pFile <<endl;
	while (!pf.eof())
	{
		// update the values of the past_possibilities
		for(int i = 0; i < nbM; i++)
			past_possibilities[i] = possibilities[i];

		// read the new possibilities
		for(int i = 0; i < nbM; i++)
		{
			pf>>possibilities[i];
			//DEBUG: cout<<possibilities[i] <<" ";
		}
		//DEBUG: cout<<endl;
		if (!pf)
			break;

		// extract/update the intervals of activation for each activity
		for(int i = 0; i < nbM; i++)
			updateInterval(i,nSamples,possibilities[i],past_possibilities[i],0,rF);

		nSamples = nSamples + 1;
	}
	pf.close();

	// analyze intervals that are still open at the end of the file
	for (int i = 0; i < nbM; i++)
	{
		if((setI[i].open == true) && (setI[i].refPossibility == 0))
		{
			// define the ending time of the interval
			setI[i].setEnd(setI[i].end[0]);

			// publish the interval info on file
			reportSensingBracelet(i, rF);
		
			// close the interval
			setI[i].open = false;
		}
	}
}

//! perform on-line full analysis of a data stream (classifier inside)
//! @param port:	USB port for data acquisition
//! @return:		---
void SensingBracelet::onlineSensingBracelet(char* port)
{
	int nSamples = 0;			// number of acquired samples
	string sample;				// current sample acquired via USB
	mat actsample;				// current sample in matrix format
	int ax, ay, az;				// accelerometer current sample components
	int gx, gy, gz;				// gyroscope current sample components
	char dev;					// flag --> device type
	string motion;				// flag --> level of motion at the wrist
	vector<float> poss;			// models possibilities
	vector<float> past_poss;	// models previous possibilities

	string waste = " ";

	// instantiate and initialize a Classifier
	string dF = datasetFolder.substr(0,datasetFolder.length()-1);
	dF = dF.substr(9);
	Classifier hC(dF);
	mat window = zeros<mat>(hC.window_size, 3);
	mat gravity = zeros<mat>(hC.window_size, 3);
	mat body = zeros<mat>(hC.window_size, 3);

	// initialize the possibilities and past possibilities
	for(int i = 0; i < nbM; i++)
	{
		poss.push_back(0);
		past_poss.push_back(0);
	}
	
	// set up the serial communication (read-only)
	SerialOptions options;
	options.setDevice(port);
	options.setBaudrate(9600);
	options.setTimeout(seconds(1));
	options.setParity(SerialOptions::noparity);
	options.setCsize(8);
	options.setFlowControl(SerialOptions::noflow);
	options.setStopBits(SerialOptions::one);
	SerialStream serial(options);
	serial.exceptions(ios::badbit | ios::failbit);

	// extract known activities intervals from the stream of raw acceleration data
	while(1)
	{
		try
		{
			// read the current sample
			getline(serial,sample);
			istringstream stream(sample);
			stream >>dev >>ax >> ay >>az >>gx >>gy >>gz >>motion;
			actsample <<ax <<ay <<az;
			//DEBUG: cout<<"Acquired: " <<ax <<" " <<ay <<" " <<az <<" ";

			// update the window of samples to be analyzed
			hC.createWindow(actsample, window, hC.window_size, nSamples);
			if (nSamples >= hC.window_size)
			{
				// analyze the window and compute the models possibilities
				for(int i = 0; i < nbM; i++)
					past_poss[i] = poss[i];
				hC.analyzeWindow(window, gravity, body);
				hC.compareAll(gravity, body, poss);

				// publish the dynamic tuples
				hC.publishDynamic(poss);

                /**************************************************************
                // ACCURATE A-POSTERIORI ACTIVITY ANALYSIS
				// extract/update the intervals of activation for each activity
				for(int i = 0; i < nbM; i++)
					updateInterval(i,nSamples,poss[i],past_poss[i],1,waste);
                **************************************************************/

                // QUICK AND DIRTY ANALYSIS
                // extract/update the intervals of activation for each activity
				for(int i = 0; i < nbM; i++)
					simpleInterval(i,nSamples,poss[i],past_poss[i]);
			}
		}
		catch(TimeoutException&)
		{
			serial.clear();
			cerr<<"Timeout occurred"<<endl;
		}
	}
}
