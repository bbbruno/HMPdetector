//===============================================================================//
// Name			: classifier.hpp
// Author(s)	: Barbara Bruno, Antonello Scalmato
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 2.0
// Description	: Human Motion Primitives classifier module (on-line / off-line)
//===============================================================================//

#include <vector>

#include "device.hpp"
#include "publisher.hpp"
#include "utils.hpp"

using namespace arma;
using namespace std;

#ifndef CLASSIFIER_HPP_
#define CLASSIFIER_HPP_

//! class "model" of an HMP - dynamic classification parameters
class DYmodel
{
	private:
		//! load the expected points (Mu) of one feature
		mat loadMu(string name, string component);

		//! load the expected variances (Sigma) of one feature
		cube loadSigma(string name, string component);

	public:
		string HMPname;			//!< name of the HMP within the dataset
		int size;				//!< number of samples in the model
		float gravityWeight;	//!< weight of gravity feature for classification
		float bodyWeight;		//!< weight of body acc. feature for classification
		float threshold;		//!< max distance for possible motion occurrence
		mat gP;					//!< gravity expected points
		cube gS;				//!< gravity set of covariance matrices
		mat bP;					//!< body acc. expected points
		cube bS;				//!< body acc. set of covariance matrices

		//! constructor
		DYmodel()
		{
			//DEBUG:cout<<endl <<"Creating DYmodel object" <<endl;
		}

		//! constructor with variables initialization
		DYmodel(string HMPn, float gW, float bW, float th);

		//! print model information
		void printInfo();

		//! set all the model variables and load the model
		void build(string HMPn, float gW, float bW, float th);

		//! destructor
		~DYmodel()
		{
			//DEBUG:cout<<endl <<"Destroying DYmodel object" <<endl;
		}
};

//!\test test all

//! class "Classifier" for offline and online recognition of HMP
class Classifier
{
	private:
		//! compute (trial)point-to-(model)point Mahalanobis distance
		float mahalanobisDist(int index,mat &trial,mat &model,cube &variance);

		//! compute the overall distance between the trial and one model
		float compareOne(mat &Tgravity, mat &Tbody, DYmodel &MODEL);

		//! test one file (off-line)
		void singleTest(string testFile, string resultFile);

	protected:
		//! publish the static information (loaded HMPs)
		void publishStatic();

	public:
		string datasetFolder;   //!< folder containing the models
        Device* driver;         //!< driver for the device used for the dataset collection
        Publisher* pub;         //!< interface for the publishing middleware
		int nbM;			    //!< number of considered models
		vector<DYmodel> set;	//!< set of considered models
		int window_size;		//!< size of the largest stored model

		//! constructor
		Classifier(string dF, Device* dev, Publisher* p);

		//! print set information
		void printSetInfo();

		//! set all the classifier variables and load the models
		void buildSet(string dF, Device* dev, Publisher* p);

		//! create a window of samples
		void createWindow(mat &one_sample, mat &window, int &N, int &numWritten);

		//! get gravity and body acc. components of the window
		void analyzeWindow(mat &window, mat &gravity, mat &body);

		//! compute the matching possibility of all the models
		void compareAll(mat &gravity,mat &body, vector<float> &possibilities);

		//! validate one model with given validation trials
		void validateModel(string model, string dataset, int numTrials);

		//! test one recorded file
		void longTest(string testFile);

		//! publish the dynamic information (recognition results)
		void publishDynamic(vector<float> &possibilities);

		//! classify real-time raw acceleration samples acquired via USB
		//void onlineTest(char* port);

		//! destructor
		~Classifier()
		{
			set.clear();
			//DEBUG:cout<<endl <<"Destroying Classifier object" <<endl;
		}
};

#endif
