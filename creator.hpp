//===============================================================================//
// Name			: creator.hpp
// Author(s)	: Barbara Bruno, Antonello Scalmato
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 2.0
// Description	: Human Motion Primitives models creator module (off-line only)
//===============================================================================//

#include <string>

#include "device.hpp"
#include "utils.hpp"

using namespace std;

#ifndef CREATOR_HPP_
#define CREATOR_HPP_

//! class "model" of an HMP - static modelling parameters
class STmodel
{
	public:
		string name;			//!< name of the motion primitive
		int nbModellingTrials;	//!< number of trials in the modelling folder
		int nbGravityGaussians;	//!< number of Gaussians modelling gravity
		int nbBodyGaussians;	//!< number of Gaussians modelling body acc.

		//! constructor
		STmodel(string n, int nbMT, int nbGG, int nbBG);

		//! print model information
		void printInfo();

		//! destructor
		~STmodel()
		{
			//DEBUG:cout<<endl <<"Destroying STmodel object" <<endl;
		}
};

//!\todo add examples for the Creator functions
//!\todo integrate in Creator the library for automatically syncing the dataset trials
//!\todo add model plotting capabilities

//!\test test all

//! class "Creator" to create and manage HMP models
class Creator
{
	private:
		//! concatenate the trials of the modelling dataset along the three axes
		void createSet(mat &actual_sample, mat &set);
        
        //! extract gravity and body acc. components from the dataset
		void getFeatures(string name, int nTr, mat &totGravity, mat &totBody);

		//! create the model of one motion primitive (with GMM+GMR)
		void generateModel(STmodel &motion);
        
	public:                
		string datasetFolder;		//!< folder containing the modelling dataset
        Device* driver;             //!< driver for the device used for the dataset collection

		//! constructor
		Creator(string dF, Device* dev);

		//! set dataset folder
		void setDatasetFolder(string dF);

		//! create the models of all motion primitives
		void generateAllModels();		

		//! destructor
		~Creator()
		{
			//DEBUG:cout<<endl <<"Destroying Creator object" <<endl;
		}
};

#endif
