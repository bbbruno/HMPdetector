//===============================================================================//
// Name			: classifier.hpp
// Author(s)	: Barbara Bruno, Antonello Scalmato
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 2.0
// Description	: Human Motion Primitives classifier module (on-line / off-line)
//===============================================================================//

#include <fstream>

#include "classifier.hpp"
#include "libs/SerialStream.h"

using namespace arma;
using namespace boost::posix_time;

//! constructor with variables initialization
//! @param[in] HMPn	name of the motion primitive (within the dataset)
//! @param[in] gW	weight of gravity feature for classification
//! @param[in] bW	weight of body acc. feature for classification
//! @param[in] th	max distance for possible motion occurrence
DYmodel::DYmodel(string HMPn, float gW, float bW, float th)
{
	// initialize the class variables
	cout<<"Loading model: " <<HMPn <<"...";
	HMPname = HMPn;
	gravityWeight = gW;
	bodyWeight = bW;
	threshold = th;

	// load the model (initialization of gP, gS, bP, bS)
	bP = loadMu(HMPname, "Body");		//DEBUG: cout<<"MuBody-";
	bS = loadSigma(HMPname, "Body");	//DEBUG: cout<<"SigmaBody-";
	gP = loadMu(HMPname, "Gravity");	//DEBUG: cout<<"MuGravity-";
	gS = loadSigma(HMPname, "Gravity");	//DEBUG: cout<<"SigmaGravity-";

	// compute the size of the model
	size = gP.n_cols;

	cout<<"DONE"<<endl;
}

//! print model information
void DYmodel::printInfo()
{
	cout<<"DYmodel object information:" <<endl;
	cout<<"HMPname = " <<HMPname <<endl;
	cout<<"gravityWeight = " <<gravityWeight <<endl;
	cout<<"bodyWeight = " <<bodyWeight <<endl;
	cout<<"threshold = " <<threshold <<endl;
	cout<<"size = " <<size <<endl;
}

//!\todo replace C-style reading from file with fstream 

//! load the expected points (Mu) of one feature
//! @param[in] HMPname		name of the model (within the dataset)
//! @param[in] component	name of the feature
//! @return     			matrix of the expected points of the feature
mat DYmodel::loadMu(string HMPname, string component)
{
	int row;
	int col;

	string fileName = HMPname + "Mu" + component + ".txt";
	FILE *pf = fopen(fileName.c_str(), "r");
	fscanf(pf, "%d,%d\n", &col, &row);
	mat mod = zeros<mat>(row, col);
	for (int r = 0; r < row; r++)
	{
		int c;
		float fr;
		for (c = 0; c < col - 1; c++)
		{
			fscanf(pf, "%f,", &fr);
			mod(r, c) = fr;
		}
		fscanf(pf, "%f\n", &fr);
		mod(r, c) = fr;
	}
	mod = mod.t();
	fclose(pf);

	return mod;
}

//!\todo replace C-style reading from file with fstream 

//! load the expected variances (Sigma) of one feature
//! @param[in] HMPname		name of the model (within the dataset)
//! @param[in] component	name of the feature
//! @return			        matrix of the expected variances of the feature
cube DYmodel::loadSigma(string HMPname, string component)
{
    int row;
	int col;
	int slice;

    string fileName = HMPname + "Sigma" + component + ".txt";
	FILE *pf = fopen(fileName.c_str(), "r");
	fscanf(pf, "%d,%d,%d\n", &row, &col, &slice);
	cube mod = zeros<cube>(row, col, slice);
	for (int s = 0; s < slice; s++)
	{
		for (int r = 0; r < row; r++)
		{
			int c;
			float fr;
			for (c = 0; c < col - 1; c++)
			{
				fscanf(pf, "%f,", &fr);
				mod(r, c, s) = fr;
			}
			fscanf(pf, "%f\n", &fr);
			mod(r, c, s) = fr;
		}
	}
	fclose(pf);

	return mod;
}

//! set all the model variables and load the model
//! @param[in] HMPn	name of the motion primitive (within the dataset)
//! @param[in] gW	weight of gravity feature for classification
//! @param[in] bW	weight of body acc. feature for classification
//! @param[in] th	max distance for possible motion occurrence
void DYmodel::build(string HMPn, float gW, float bW, float th)
{
	// initialize the class variables
	cout<<"Loading model: " <<HMPn <<"...";
	HMPname = HMPn;
	gravityWeight = gW;
	bodyWeight = bW;
	threshold = th;

	// load the model (initialization of gP, gS, bP, bS)
	bP = loadMu(HMPname, "Body");		//DEBUG: cout<<"MuBody-";
	bS = loadSigma(HMPname, "Body");	//DEBUG: cout<<"SigmaBody-";
	gP = loadMu(HMPname, "Gravity");	//DEBUG: cout<<"MuGravity-";
	gS = loadSigma(HMPname, "Gravity");	//DEBUG: cout<<"SigmaGravity-";

	// compute the size of the model
	size = gP.n_cols;

	cout<<"DONE"<<endl;
}

//! constructor
//! @param[in] dF	folder containing the modelling dataset
//! @param[in] dev  driver for the device used for the dataset collection
//! @param[in] p    interface for the publishing middleware
Classifier::Classifier(string dF, Device* dev, Publisher* p)
{
	string one_HMPn;
	float one_gW;
	float one_bW;
	float one_th;
	DYmodel *one_model;

	datasetFolder = "./Models/" + dF + "/";
    driver = dev;
    //DEBUG:driver->printInfo();
    pub = p;
    pub->printInfo();
	string fileName = datasetFolder + "Classifierconfig.txt";
	//DEBUG:cout<<"config file: " <<fileName <<endl;
	ifstream configFile(fileName.c_str());
	configFile >>nbM;
	//DEBUG:cout<<"nbM: " <<nbM <<endl;
	for(int i=0; i< nbM; i++)
	{
		configFile>>one_HMPn >>one_gW >>one_bW >>one_th;
		one_HMPn = datasetFolder + one_HMPn;
		//DEBUG:cout<<"DYmodel: " <<one_HMPn <<endl;
		one_model = new DYmodel(one_HMPn, one_gW, one_bW, one_th);
		//DEBUG:one_model->printInfo();
		set.push_back(*one_model);
	}
	configFile.close();

	// compute the size of the window
    cout<<"HMP models loaded. Defining window size as: ";
	int temp_ws = set[0].size;
	for(int i=1; i< nbM; i++)
	{
		if(set[i].size > temp_ws)
			temp_ws = set[i].size;
	}
	window_size = temp_ws;
    cout<<window_size <<endl;

	// publish the static information (number & names of models)
	publishStatic();
}

//! print set information
void Classifier::printSetInfo()
{
	for(int i=0; i<nbM; i++)
		set[i].printInfo();
}

//! create a window of samples
//! @param[in] &one_sample	    reference to the sample to be added to the window
//! @param[in,out] &window		reference to the window
//! @param[in] &N			    reference to the size of the window
//! @param[in,out] &numWritten  reference to the number of samples in the window
void Classifier::createWindow(mat &one_sample, mat &window, int &N, int &numWritten)
{
	// update the window content
	if (numWritten < N)
	{
		window.row(numWritten) = one_sample.row(0);
		numWritten = numWritten + 1;
	}
	else
	{
		for (int i = 0; i < N - 1; i++)
			window.row(i) = window.row(i + 1);
		window.row(N - 1) = one_sample;
		numWritten = numWritten + 1;
	}
}

//! get gravity and body acc. components of the window
//! @param[in] &window		reference to the window
//! @param[out] &gravity	reference to the gravity comp. extracted from the window
//! @param[out] &body		reference to the body acc. comp. extracted from the window
void Classifier::analyzeWindow(mat &window, mat &gravity, mat &body)
{
	// perform median filtering to reduce the noise
	int n = 3;
	mat clean_window = window.t();
	medianFilter(clean_window, n);
	clean_window = clean_window.t();

	// discriminate between gravity and body acc. components
	mat tempgr = clean_window.t();
	gravity = ChebyshevFilter(tempgr);
	gravity = gravity.t();
	body = clean_window - gravity;
}

//! compute (trial)point-to-(model)point Mahalanobis distance
//! @param[in] index		index of the points (in trial and model) to be compared
//! @param[in] &trial		reference to the trial
//! @param[in] &model		reference to the model
//! @param[in] &variance	reference to the model variance
//! @return 			    Mahalanobis distance between trial-point and model-point
float Classifier::mahalanobisDist(int index,mat &trial,mat &model,cube &variance)
{
	mat difference = trial.col(index) - model.col(index);
	mat distance = (difference.t() * (variance.slice(index)).i()) * difference;

	return distance(0,0);
}

//! compute the overall distance between the trial and one model
//! @param[in] &Tgravity	reference to the gravity component of the trial
//! @param[in] &Tbody		reference to the body acc. component of the trial
//! @param[in] &MODEL		reference to the model
//! @return 			    Mahalanobis overall distance between trial and model
float Classifier::compareOne(mat &Tgravity, mat &Tbody, DYmodel &MODEL)
{
	// extract the subwindow of interest from the trial (same size of the model)
	mat gravity = Tgravity.rows(0, MODEL.size-1);
	mat body = Tbody.rows(0, MODEL.size-1);

	// acquire the relevant data from the model class
	mat MODELgP = MODEL.gP;
	cube MODELgS = MODEL.gS;
	mat MODELbP = MODEL.bP;
	cube MODELbS = MODEL.bS;

	// discard the "time" row from the models
	int numPoints = MODELgS.n_slices;
	gravity = gravity.t();
	body = body.t();

	mat reference_G = zeros<mat>(3, MODELgP.n_cols);
	mat reference_B = zeros<mat>(3, MODELbP.n_cols);
	for (int i = 0; i < 3; i++)
	{
		reference_G.row(i) = MODELgP.row(i + 1);
		reference_B.row(i) = MODELbP.row(i + 1);
	}

	// compute the components distances (gravity; body acc.)
	mat dist = zeros<mat>(numPoints,2);
	for (int i = 0; i < numPoints; i++)
	{
		dist(i,0) = mahalanobisDist(i, gravity, reference_G, MODELgS);
		dist(i,1) = mahalanobisDist(i, body, reference_B, MODELbS);
	}

	// compute the overall distance
	float distanceG = mean(dist.col(0));
	float distanceB = mean(dist.col(1));
	float overall = (MODEL.gravityWeight*distanceG)+(MODEL.bodyWeight*distanceB);

	return overall;
}

//! compute the matching possibility of all the models
//! @param[in] &gravity         reference to the gravity component of the trial
//! @param[in] &body			reference to the body acc. component of the trial
//! @param[out] &possibilities	reference to the models possibilities
void Classifier::compareAll(mat &gravity,mat &body, vector<float> &possibilities)
{
	float distance[nbM];

	// compare the features of the trial with those of each model
	for(int i = 0; i < nbM; i++)
    {
		distance[i] = compareOne(gravity, body, set[i]);
        //DEBUG: cout<<distance[i] <<endl;
    }

	// compute the possibilities from the trial_to_model distances
	for(int i = 0; i < nbM; i++)
	{
		possibilities[i] = 1 - (distance[i] / set[i].threshold);
		if (possibilities[i] < 0)
			possibilities[i] = 0;
	}
}

//! set all the classifier variables and load the models
//! @param[in] dF	name of the dataset to be loaded
//! @param[in] dev  driver for the device used for the dataset collection
//! @param[in] p    interface for the publishing middleware
void Classifier::buildSet(string dF, Device* dev, Publisher* p)
{
	string one_HMPn;
	float one_gW;
	float one_bW;
	float one_th;
	DYmodel *one_model;

	// delete the existing models
	set.clear();

	// load the new set of models
	datasetFolder = "./Models/" + dF + "/";
    driver = dev;
    //DEBUG:driver->printInfo();
    pub = p;
    pub->printInfo();
	string fileName = datasetFolder + "Classifierconfig.txt";
	//DEBUG:cout<<"config file: " <<fileName <<endl;
	ifstream configFile(fileName.c_str());
	configFile >>nbM;
	//DEBUG:cout<<"nbM: " <<nbM <<endl;
	for(int i=0; i< nbM; i++)
	{
		configFile>>one_HMPn >>one_gW >>one_bW >>one_th;
		one_HMPn = datasetFolder + one_HMPn;
		//DEBUG:cout<<"DYmodel: " <<one_HMPn <<endl;
		one_model = new DYmodel(one_HMPn, one_gW, one_bW, one_th);
		//DEBUG:one_model->printInfo();
		set.push_back(*one_model);
	}
	configFile.close();
    
    // compute the size of the window
    cout<<"HMP models loaded. Defining window size as: ";
	int temp_ws = set[0].size;
	for(int i=1; i< nbM; i++)
	{
		if(set[i].size > temp_ws)
			temp_ws = set[i].size;
	}
	window_size = temp_ws;
    cout<<window_size <<endl;

	// publish the static information (number & names of models)
	publishStatic();
}

//! test one file (off-line)
//! @param[in] testFile 	name of the test file
//! @param[in] resultFile	name of the result file
void Classifier::singleTest(string testFile, string resultFile)
{
	int nSamples = 0;				// number of samples acquired by the system
	mat actualSample;				// current sample in matrix format
	vector<float> possibilities;	// models possibilities

	mat window = zeros<mat>(window_size, 3);
	mat gravity = zeros<mat>(window_size, 3);
	mat body = zeros<mat>(window_size, 3);

	// initialize the possibilities
	for (int i = 0; i < nbM; i++)
		possibilities.push_back(0);

	// create result file
	ofstream outputFile;
	outputFile.open(resultFile.c_str());

	// read recorded data
    ifstream tf(testFile.c_str());
    cout <<"Reading trial: " <<testFile <<endl;
    for (string line; std::getline(tf, line); )
    {
        //DEBUG:cout<<"Line: " <<line <<endl;
        actualSample = driver->extractActual(line);
        createWindow(actualSample, window, window_size, nSamples);
		if (nSamples >= window_size)
		{
			analyzeWindow(window, gravity, body);
			compareAll(gravity, body, possibilities);
			
			// report the possibility values in the results file
			for (int i = 0; i < nbM; i++)
				outputFile<<possibilities[i] <<" ";
			outputFile<<endl;
		}
	}
	tf.close();
	outputFile.close();
}

//! validate one model with given validation trials
//! @param[in] model		name of the model to be validated
//! @param[in] dataset		name of the referring dataset
//! @param[in] numTrials	number of validation trials to be used
void Classifier::validateModel(string model, string dataset, int numTrials)
{
	// analyze all validating trials one by one
	for (int i = 0; i < numTrials; i++)
	{
		stringstream itos;
		itos<<i+1;
		string trial = model + "_test (" + itos.str() + ").txt";
		string tf = "Validation/" + dataset + "/" + trial;
		string rf = "Results/" + dataset + "/res_" + trial;
		singleTest(tf, rf);
  	}
}

//! test one recorded file
//! @param[in] testFile	name of the test file
void Classifier::longTest(string testFile)
{
	string tf = "Validation/longTest/" + testFile;
	string rf = "Results/longTest/res_" + testFile;
	singleTest(tf, rf);
}

//! publish the static information (loaded HMPs)
void Classifier::publishStatic()
{
	// HMP.numModels
	stringstream ntos;
	ntos<<nbM;
	const char* sNumModels = ntos.str().c_str();
    pub->publish("numModels", sNumModels);
	//DEBUG: cout<<"numModels: " <<sNumModels <<endl;

	//HMP.nameModels
	string short_name;
	string allNames;
	int nRemove = datasetFolder.size();
	for(int i=0; i<nbM; i++)
	{
		short_name = set[i].HMPname.erase(0,nRemove);
		//DEBUG: cout<<short_name <<endl;
		allNames = allNames + short_name + " ";
	}
	const char* sNameModels = allNames.c_str();
	pub->publish("nameModels", sNameModels);
	//DEBUG: cout<<"nameModels: " <<sNameModels <<endl;
}

//! publish the dynamic information (recognition results)
//! @param[in] &possibilities	reference to the models possibilities
void Classifier::publishDynamic(vector<float> &possibilities)
{
	// HMP.possibilities
	string p;
	for(int i=0; i<nbM; i++)
	{
		stringstream ptos;
		ptos<<possibilities[i];
		p = p + " " + ptos.str();
	}
	const char* sPossibilities = p.c_str();
	pub->publish("possibilities", sPossibilities);
	//DEBUG: cout<<"possibilities: " <<sPossibilities <<endl;

	// identify the models with highest and second-highest possibility
	int best = 0;
	int secondBest = 0;
	for (int i = 1; i < nbM; i++)
	{
		if (possibilities[i] > possibilities[best])
		{
			secondBest = best;
			best = i;
		}
		else if (possibilities[i] > possibilities[secondBest])
			secondBest = i;
	}
	if (possibilities[best] == 0)
		best = -1;
	if (possibilities[secondBest] == 0)
		secondBest = -1;

	// HMP.highest
	string highest;				
	if (best == -1)
		highest = "NONE";
	else
		highest = set[best].HMPname;
	const char* sHighest = highest.c_str();
	pub->publish("highest", sHighest);
	//DEBUG: cout<<"highest: " <<sHighest <<endl;

	// HMP.other
	float other;
	if (best == -1)
		other = 1;
	else
		other = 1 - possibilities[best];
	stringstream str_other;
	str_other<<other;
	const char* sOther = str_other.str().c_str();
	pub->publish("other", sOther);
	//DEBUG: cout<<"highest: " <<sHighest <<" other: " <<sOther <<endl;

	// HMP.entropy
	float entropy;
	if (best == -1)
		entropy = -1;
	else if (secondBest == -1)
		entropy = possibilities[best];
	else
		entropy = possibilities[best] - possibilities[secondBest];
	stringstream str_entropy;
	str_entropy<<entropy;
	const char* sEntropy = str_entropy.str().c_str();
	pub->publish("entropy", sEntropy);
	//DEBUG: cout<<"highest: " <<sHighest <<" entropy: " <<sEntropy <<endl;
}

/*
//! classify real-time raw acceleration samples acquired via USB
//! @param port:	USB port for data acquisition
//! @return:		---
void Classifier::onlineTest(char* port)
{
	int nSamples = 0;				// number of samples acquired by the system
	string sample;					// current sample acquired via USB
	mat actsample;					// current sample in matrix format
	int ax, ay, az;					// accelerometer current sample components
	int gx, gy, gz;					// gyroscope current sample components
	char dev;						// flag --> device type
	string motion;					// flag --> level of motion at the wrist
	vector<float> possibilities;	// models possibilities

	mat window = zeros<mat>(window_size, 3);
	mat gravity = zeros<mat>(window_size, 3);
	mat body = zeros<mat>(window_size, 3);

	// initialize the possibilities
	for (int i = 0; i < nbM; i++)
		possibilities.push_back(0);
	
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
	
	// classify the stream of raw acceleration & gyroscope data
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
			createWindow(actsample, window, window_size, nSamples);
			if (nSamples >= window_size)
			{
				// analyze the window and compute the models possibilities
				analyzeWindow(window, gravity, body);
				compareAll(gravity, body, possibilities);

				// publish the dynamic tuples
				publishDynamic(possibilities); 
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