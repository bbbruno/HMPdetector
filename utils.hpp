//===============================================================================//
// Name			: utils.hpp
// Author(s)	: Barbara Bruno, Antonello Scalmato
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.1
// Description	: Frequently used functions (for Creator and Classifier)
//===============================================================================//

#include <armadillo>

using namespace arma;

#ifndef UTILS_HPP_
#define UTILS_HPP_

//===============================================================================//
// BASIC MATRIX-HANDLING FUNCTIONS

//! create a row-vector of the form: start:1:stop
mat createInterval(int start, int stop);

//! convert a matrix in MAT format to float format
float** matToFloat(mat &matrix);

//! convert a matrix in float format to MAT format
mat floatToMat(float** matrix, int Nrows, int Ncols);
//===============================================================================//

//===============================================================================//
// FILTERING FUNCTIONS

//! compute the median value of a vector
double median(rowvec &vector);

//! perform median filtering on a matrix
void medianFilter(mat &matrix, int size);

//! apply ChebyshevI filter on a matrix
mat ChebyshevFilter(mat matrix);
//===============================================================================//

#endif
