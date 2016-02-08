//===============================================================================//
// Name			: HMPdetector.cpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 2.0
// Description	: Human Motion Primitives, Posture and Fall detection system
//===============================================================================//

#include <getopt.h>

#include "creator.hpp"
#include "classifier.hpp"
#include "device.hpp"
#include "MPU6050.hpp"
#include "publisher.hpp"
#include "logfile.hpp"
#include "SensingBracelet.hpp"
#include "libs/SerialStream.h"

using namespace boost::posix_time;

//! Program help 
void print_help()
{
	cout<<endl;
    cout<<"\t\t -------------- HMP DETECTOR --------------" <<endl;
	cout<<"Typical calls use the following instructions:" <<endl;
	cout<<"01) -h --help \t\t\t   : program help." <<endl;
    cout<<"02) -m --model [dataset] \t   : [dataset] models creation." <<endl;
	cout<<"03) -l --load [dataset] \t   : load models in [dataset]." <<endl;
	cout<<"04) -v --validate [model] [set] [n]:"
		<<" validate [model] with [n] trials of [set]." <<endl;
	cout<<"05) -t --test [trial] \t\t   :"
		<<" off-line classification of [trial]." <<endl;
	cout<<"06) -c --classify [port] \t   :"
		<<" on-line classification of [port] stream." <<endl;
	cout<<"07) -r --reason [path] [possFile]  :"
		<<" off-line reasoning on [path]/[possFile]." <<endl;
	cout<<"08) -B --Bracelet [port] \t   :"
		<<" on-line HMP analysis on [port] stream." <<endl;
	cout<<"09) -b --belt [port] \t\t   :"
		<<" on-line posture and fall detection in [port] stream." <<endl;
	cout<<"10) -w --wearable [port] \t   :"
		<<" on-line full analysis of [port] stream." <<endl;

	cout<<endl;
	cout<<"Functions calls examples:" <<endl;
	cout<<"01)   ./HMPdetector -h" <<endl;
	cout<<"02.1) ./HMPdetector -m" <<endl;
	cout<<"02.2) ./HMPdetector -m Ovada" <<endl;
	cout<<"03)   ./HMPdetector -l Ovada" <<endl;
	cout<<"04)   ./HMPdetector -v climb Ovada 6" <<endl;
	cout<<"05)   ./HMPdetector -t drink_drink_stand_sit_drink.txt" <<endl;
	cout<<"06)   ./HMPdetector -c /dev/ttyUSB0" <<endl;
	cout<<"07)   ./HMPdetector -r "
		<<"./Results/longTest/ res_drink_drink_stand_sit_drink.txt" <<endl;
	cout<<"08)   ./HMPdetector -B /dev/ttyUSB0" <<endl;
	cout<<"09)   ./HMPdetector -b /dev/ttyUSB0" <<endl;
	cout<<"10)   ./HMPdetector -w /dev/ttyUSB0" <<endl;

	cout<<endl;
	cout<<"Enjoy!"<<endl;

	cout<<endl;
}
/*
//! perform on-line full analysis of a data stream
void completeWearable(char* port, SensingBracelet oneBr, SensingBelt oneBe)
{
	string raw_data;			// current raw data acquired via USB
	int nS = 0;					// number of acquired samples
	mat actsample;				// current sample in matrix format
	int ax, ay, az;				// accelerometer current sample components
	int gx, gy, gz;				// gyroscope current sample components
	char dev;					// flag --> device type
	string motion;				// flag --> level of motion at the wrist
	vector<float> poss;			// models possibilities
	vector<float> past_poss;	// models previous possibilities

	string waste = " ";

	// instantiate and initialize a Classifier
	string dF = oneBr.datasetFolder.substr(0,oneBr.datasetFolder.length()-1);
	dF = dF.substr(9);
	Classifier hC(dF);
	mat window = zeros<mat>(hC.window_size, 3);
	mat gravity = zeros<mat>(hC.window_size, 3);
	mat body = zeros<mat>(hC.window_size, 3);

	// initialize the possibilities and past possibilities
	for(int i = 0; i < oneBr.nbM; i++)
	{
		poss.push_back(0);
		past_poss.push_back(0);
	}

	// setup the serial communication (read-only)
	SerialOptions options;
	options.setDevice(port);
	options.setBaudrate(9600);
	options.setTimeout(seconds(0));
	options.setParity(SerialOptions::noparity);
	options.setCsize(8);
	options.setFlowControl(SerialOptions::noflow);
	options.setStopBits(SerialOptions::one);
	SerialStream serial(options);
	serial.exceptions(ios::badbit | ios::failbit);

	// classify the stream of raw acceleration & gyroscope data
	while(peiskmt_isRunning())
	{
		try
		{
			// acquire the data received by the XBee
			getline(serial,raw_data);
			switch (raw_data[0])
			{
				// call the SensingBelt if the raw_data begin with "F"
				// --> data format: F Fall [Motion]
				case 'F':
					oneBe.publishFall(raw_data);
					break;
                // call the SensingBelt if the raw_data begin with "P"
				// --> data format: P [Angle]
				case 'P':
					oneBe.publishPosture(raw_data);
					break;
				// call the Reasoner if the raw_data begin with "H"
				// --> data format: H [ax] [ay] [ax] [gx] [gy] [gz] [Motion]
				case 'H':
					istringstream stream(raw_data);
					stream >>dev >>ax >> ay >>az >>gx >>gy >>gz >>motion;
					actsample <<ax <<ay <<az;

					// update the window of samples to be analyzed
					hC.createWindow(actsample, window, hC.window_size, nS);
					if (nS >= hC.window_size)
					{
						// analyze the window and compute the models possibilities
						for(int i = 0; i < oneBr.nbM; i++)
							past_poss[i] = poss[i];
						hC.analyzeWindow(window, gravity, body);
						hC.compareAll(gravity, body, poss);

						// publish the dynamic tuples
						hC.publishDynamic(poss);

						// extract/update activation intervals for each activity
						for(int i = 0; i < oneBr.nbM; i++)
                            oneBr.simpleInterval(i,nS,poss[i],past_poss[i]);
					}
					break;
			}
		}
		catch(TimeoutException&)
		{
			serial.clear();
			cerr<<"Timeout occurred"<<endl;
		}
	}
}
*/

int main(int argc, char* argv[])
{
    // default setup choices
    cout<<endl;
    Device* dev = new MPU6050("SparkFun MPU6050");
    cout<<"Default "; dev->printInfo();
	string dF = "Sweden";
    cout<<"Default Dataset: " <<dF <<endl;
    Publisher* p = new LogFile("log.txt");
    cout<<"Default "; p->printInfo();
    
    // instantiate & initialize the HMPdetector components
	Creator oneCreator(dF, dev);
    Classifier oneClassifier(dF, dev, p);
    cout<<endl <<"Initialization phase of HMPdetector: DONE" <<endl;
    
    /*
	// instantiate & initialize the PEIS component
	peiskmt_initialize(&argc, argv);
	// retrieve componentID from the component
	int componentID = peiskmt_peisid();
	printf("componentID: %d\n", componentID);

	Classifier one_classifier(dF);
	//DEBUG: one_classifier.printSetInfo();
	SensingBracelet one_sensingBracelet(dF);
	//DEBUG: one_sensingBracelet.printSetStatus();
    */

    // available options (short-form)
	//const char *short_options = "v:r:wbBctl:mh";
    const char *short_options = "v:::mhE";
	// available options (long-form)
	static struct option long_options[] = 
	{
		{"validate", required_argument, 0, 'v'},
		//{"reason", required_argument, 0, 'r'},
		//{"wearable", required_argument, 0, 'w'},
		//{"Bracelet", required_argument, 0, 'B'},
		//{"classify", required_argument, 0, 'c'},
		//{"test", required_argument, 0, 't'},
		//{"load", required_argument, 0, 'l'},
		{"model", optional_argument, 0, 'm'},
		{"help", no_argument, 0, 'h'},
        {"EXIT", no_argument, 0, 'E'},
		{0, 0, 0, 0} //required line
	};

	// retrieve & execute the chosen option
    //!\todo make it possible to have multiple iterations
	char c;
	do 
	{
		c = getopt_long(argc, argv, short_options, long_options, NULL);
		switch (c)
		{
            case 'E':
                return EXIT_SUCCESS;
			case 'h':
				print_help();
				break;
			case 'm':
				if(argv[2])
                {
                    cout<<"Modelling folder: " <<argv[2] <<endl;
                    oneCreator.driver->printInfo();
					oneCreator.setDatasetFolder(argv[2]);
                }             				
				oneCreator.generateAllModels();
				cout<<"Created dataset in: "<<oneCreator.datasetFolder <<endl;
				break;
            /*    
			case 'l':			
				oneClassifier.buildSet(argv[2], argv[3], argv[4]);
				cout<<"Loaded dataset from: "<<oneClassifier.datasetFolder <<endl;
				break;
            */
			case 'v':
				oneClassifier.validateModel(argv[2], argv[3], atoi(argv[4]));
				cout<<"results in: ./Results/" <<argv[3] <<"/" <<endl;
				break;
            /*
			case 't':
				one_classifier.longTest(argv[2]);
				cout<<"results in: ./Results/longTest/" <<endl;
				return EXIT_SUCCESS;
				break;
			case 'c':
				cout<<"use 'tupleview' to monitor the system" <<endl;
				one_classifier.onlineTest(argv[2]);
				return EXIT_SUCCESS;
				break;
			case 'r':
				one_sensingBracelet.offlineSensingBracelet(argv[2], argv[3]);
				cout<<"results in: " <<argv[2] <<endl;
				return EXIT_SUCCESS;
				break;
			case 'B':
				cout<<"use 'tupleview' to monitor the system" <<endl;
				one_sensingBracelet.onlineSensingBracelet(argv[2]);
				return EXIT_SUCCESS;
				break;
			case 'b':
				cout<<"use 'tupleview' to monitor the system" <<endl;
				one_sensingBelt.standaloneBelt(argv[2]);
				return EXIT_SUCCESS;
				break;
			case 'w':
				cout<<"use 'tupleview' to monitor the system" <<endl;
				completeWearable(argv[2], one_sensingBracelet, one_sensingBelt);
				return EXIT_SUCCESS;
				break;
            */
		}
	}while (c != -1);
}
