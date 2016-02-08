//===============================================================================//
// Name			: creator.cpp
// Author(s)	: Barbara Bruno, Antonello Scalmato
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 2.0
// Description	: Human Motion Primitives models creator module (off-line only)
//===============================================================================//

#include <fstream>

#include "creator.hpp"
#include "libs/GMM+GMR/gmr.h"

//! constructor of class STmodel
//! @param[in] n	name of the motion primitive
//! @param[in] nbMT	number of trials in the modelling folder
//! @param[in] nbGG	number of Gaussians modelling gravity
//! @param[in] nbBG	number of Gaussians modelling body acc.
STmodel::STmodel(string n, int nbMT, int nbGG, int nbBG)
{
	name = n;
	nbModellingTrials = nbMT;
	nbGravityGaussians = nbGG;
	nbBodyGaussians = nbBG;
}

//! print model information
void STmodel::printInfo()
{
	cout<<"STmodel object information:" <<endl;
	cout<<"name = " <<name <<endl;
	cout<<"nbModellingTrials = " <<nbModellingTrials <<endl;
	cout<<"nbGravityGaussians = " <<nbGravityGaussians <<endl;
	cout<<"nbBodyGaussians = " <<nbBodyGaussians <<endl;
}

//! constructor of class Creator
//! @param[in] dF	folder containing the modelling dataset
//! @param[in] dev  driver for the device used for the dataset collection
Creator::Creator(string dF, Device* dev)
{
	datasetFolder = "./Models/" + dF + "/";
    //DEBUG:cout<<"modelling dataset in folder: " <<datasetFolder <<endl;
    driver = dev;
    //DEBUG:driver->printInfo();
}

//! set dataset folder
//! @param[in] dF	folder containing the modelling dataset
void Creator::setDatasetFolder(string dF)
{
	datasetFolder = "./Models/" + dF + "/";
    //DEBUG:cout<<"modelling dataset in folder: " <<datasetFolder <<endl;
}

//! concatenate the trials of the modelling dataset along the three axes
//! @param[in] &actual_sample	one sample from one modelling trial
//! @param[out] &set			set of all the modelling trials samples
void Creator::createSet(mat &actual_sample, mat &set)
{    
	set = join_cols(set, actual_sample);
}

//! extract gravity and body acc. components from the dataset
//! @param[in] name		    name of the motion primitive
//! @param[in] nTr			number of trials in the modelling dataset
//! @param[out] &totGravity reference to the gravity set of the modelling trials
//! @param[out] &totBody	reference to the body acc. set of the modelling trials
void Creator::getFeatures(string name, int nTr, mat &totGravity, mat &totBody)
{
	string fileName;	// name of one trial of the modelling dataset

	for (int i = 0; i < nTr; i++)
	{        
        // read all the modelling trials and concatenate the values along the axes
		stringstream itos;
		itos <<i+1;
		fileName = datasetFolder + name + "/mod (" + itos.str() + ").txt";
		cout<<"Open modelling trial: " <<fileName <<endl;
        mat set;
		mat gravity;
		mat body;
        ifstream trialFile(fileName.c_str());
        for (string line; std::getline(trialFile, line); )
        {
            //DEBUG:cout<<"Line: " <<line <<endl;
            mat actualSample;
            actualSample = driver->extractActual(line);
            createSet(actualSample, set);
        }
        
		// reduce the noise on the sets by median filtering
		int size = 3;
		mat clean_set = set.t();
		medianFilter(clean_set, size);
		clean_set = clean_set.t();

		// separate gravity and body acc. by Chebyshev II low-pass filtering
		mat tempgr = clean_set.t();
		gravity = ChebyshevFilter(tempgr);
		gravity = gravity.t();
		body = clean_set - gravity;

		// create the datasets
		mat time = createInterval(1, gravity.n_rows);
		mat tmpGrav = join_rows(time, gravity);
		mat tmpBody = join_rows(time, body);
		totGravity = join_cols(totGravity, tmpGrav);
		totBody = join_cols(totBody, tmpBody);
	}
}

//! create the model of one motion primitive (with GMM+GMR)
//! @param[in] &motion	motion primitive settings object
void Creator::generateModel(STmodel &motion)
{
	mat totGravity;
	mat totBody;
	string GMMgravity;
	string GMMbody;
	string MuGr;
	string MuBo;
	string SGr;
	string SBo;

	// create the gravity and body acc. datasets
	cout<<endl <<"Creating the gravity and body acceleration datasets" <<endl;
	getFeatures(motion.name, motion.nbModellingTrials, totGravity, totBody);

	// create the GMM+GMR model of the gravity component
	{
		cout<<endl <<"GMM+GMR model of the gravity component" <<endl;
		GaussianMixture gg;
		int nbVar = totGravity.n_cols;
		int nbData = totGravity.n_rows;
		nbData = (int) (nbData / motion.nbModellingTrials);
		cout<<"Number of samples in the modelling trials: " <<nbData <<endl;

		// GMM phase
		cout<<endl <<"GMM...";
		gg.initEM_TimeSplitMat(motion.nbGravityGaussians, totGravity);
		gg.doEM(totGravity);
		GMMgravity = datasetFolder + motion.name + "GMMgravity.txt";
		gg.saveParams(GMMgravity.c_str());
		cout <<"done" <<endl;

		// GMR phase
		cout<<endl <<"GMR...";
		Vector inC(1), outC(nbVar - 1);
		// input data for regression: time	
		inC(0) = 0;
		// output data for regression: tri-axial acceleration
		for (int i = 0; i < nbVar - 1; i++)
			outC(i) = (float) (i + 1);
		mat oneCol = createInterval(1,nbData);
		Matrix *inData = new Matrix(oneCol);
		Matrix *outSigma;
		outSigma = new Matrix[nbData];
		Matrix outData = gg.doRegression(*inData, outSigma, inC, outC);
		MuGr = datasetFolder + motion.name + "MuGravity.txt";
		SGr = datasetFolder + motion.name + "SigmaGravity.txt";
		gg.saveRegressionResult(MuGr.c_str(),SGr.c_str(),*inData,outData,outSigma);
		cout <<"done" <<endl;
	}

	// create the GMM+GMR model of the body acc. component
	{
		cout<<endl <<"GMM+GMR model of the body acc. component" <<endl;
		GaussianMixture gb;
		int nbVar = totBody.n_cols;
		int nbData = totBody.n_rows;
		nbData = (int) (nbData / motion.nbModellingTrials);
		cout<<"Number of samples in the modelling trials: " <<nbData <<endl;
	
		// GMM phase
		cout<<endl <<"GMM...";
		gb.initEM_TimeSplitMat(motion.nbBodyGaussians, totBody);
		gb.doEM(totBody);
		GMMbody = datasetFolder + motion.name + "GMMbody.txt";
		gb.saveParams(GMMbody.c_str());
		cout <<"done" <<endl;

		// GMR phase
		cout<<endl <<"GMR...";
		Vector inC(1), outC(nbVar - 1);
		// input data for regression: time	
		inC(0) = 0;
		// output data for regression: tri-axial acceleration
		for (int i = 0; i < nbVar - 1; i++)
			outC(i) = (float) (i + 1);
		mat oneCol = createInterval(1,nbData);
		Matrix *inData = new Matrix(oneCol);
		Matrix *outSigma;
		outSigma = new Matrix[nbData];
		Matrix outData = gb.doRegression(*inData, outSigma, inC, outC);
		MuBo = datasetFolder + motion.name + "MuBody.txt";
		SBo = datasetFolder + motion.name + "SigmaBody.txt";
		gb.saveRegressionResult(MuBo.c_str(),SBo.c_str(),*inData,outData,outSigma);
		cout <<"done" <<endl;
	}
}

//! create the models of all motion primitives
void Creator::generateAllModels()
{
		string one_n;
		int one_nbMT;
		int one_nbGG;
		int one_nbBG;

		string fileName = datasetFolder + "HMPconfig.txt";
		//DEBUG:cout<<"config file: " <<fileName <<endl;
		ifstream configFile(fileName.c_str());
		while (!configFile.eof())
		{
			configFile>>one_n >>one_nbMT >>one_nbGG >>one_nbBG;
			if (!configFile)
				break;
			//DEBUG:cout<<"Generating model: " <<one_n <<endl;
			STmodel one_HMP(one_n,one_nbMT,one_nbGG,one_nbBG);
			//DEBUG:one_HMP.printInfo();
			generateModel(one_HMP);
		}
		configFile.close();
}
