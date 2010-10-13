#include "FastHog3DComputer.h"

#include <iostream>
#include <cmath>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <numeric/functions.h>

#include "FastVideoGradientComputer.h"

using std::cout;
using std::cerr;
using std::endl;


const double FastHog3DComputer::goldenRatio = 1.6180339887;
const double FastHog3DComputer::projectionThresholdIcosahedron = 0.74535599249992989801;
const double FastHog3DComputer::projectionThresholdDodecahedron = 0.44721359549995792770;

void FastHog3DComputer::initDodecahedron()
{
	// init the 6 axes of the dodocahedron
	_projectionMatrix = Matrix(6, 3);
	_projectionMatrix(0, 0) = 0;
	_projectionMatrix(0, 1) = 1;
	_projectionMatrix(0, 2) = goldenRatio;
	
	_projectionMatrix(1, 0) = 0;
	_projectionMatrix(1, 1) = -1;
	_projectionMatrix(1, 2) = goldenRatio;
	
	_projectionMatrix(2, 0) = 1;
	_projectionMatrix(2, 1) = goldenRatio;
	_projectionMatrix(2, 2) = 0;

	_projectionMatrix(3, 0) = -1;
	_projectionMatrix(3, 1) = goldenRatio;
	_projectionMatrix(3, 2) = 0;

	_projectionMatrix(4, 0) = goldenRatio;
	_projectionMatrix(4, 1) = 0;
	_projectionMatrix(4, 2) = 1;

	_projectionMatrix(5, 0) = -goldenRatio;
	_projectionMatrix(5, 1) = 0;
	_projectionMatrix(5, 2) = 1;
	
	_projectionThreshold = projectionThresholdDodecahedron * sqrt(1 + goldenRatio * goldenRatio); // sqrt(1 + goldenRatio^2) is the length of one axis vector
}

void FastHog3DComputer::initIcosahedron()
{
	// init the 10 axes of the icosahedron 
	_projectionMatrix = Matrix(10, 3);
	_projectionMatrix(0, 0) = 1;
	_projectionMatrix(0, 1) = 1;
	_projectionMatrix(0, 2) = 1;
	
	_projectionMatrix(1, 0) = -1;
	_projectionMatrix(1, 1) = -1;
	_projectionMatrix(1, 2) = 1;

	_projectionMatrix(2, 0) = -1;
	_projectionMatrix(2, 1) = 1;
	_projectionMatrix(2, 2) = 1;

	_projectionMatrix(3, 0) = 1;
	_projectionMatrix(3, 1) = -1;
	_projectionMatrix(3, 2) = 1;

	_projectionMatrix(4, 0) = 0;
	_projectionMatrix(4, 1) = 1 / goldenRatio;
	_projectionMatrix(4, 2) = goldenRatio;

	_projectionMatrix(5, 0) = 0;
	_projectionMatrix(5, 1) = -1 / goldenRatio;
	_projectionMatrix(5, 2) = goldenRatio;

	_projectionMatrix(6, 0) = 1 / goldenRatio;
	_projectionMatrix(6, 1) = goldenRatio;
	_projectionMatrix(6, 2) = 0;

	_projectionMatrix(7, 0) = 1 / goldenRatio;
	_projectionMatrix(7, 1) = -1 * goldenRatio;
	_projectionMatrix(7, 2) = 0;

	_projectionMatrix(8, 0) = goldenRatio;
	_projectionMatrix(8, 1) = 0;
	_projectionMatrix(8, 2) = 1 / goldenRatio;

	_projectionMatrix(9, 0) = -1 * goldenRatio;
	_projectionMatrix(9, 1) = 0;
	_projectionMatrix(9, 2) = 1 / goldenRatio;

	_projectionThreshold = projectionThresholdIcosahedron * sqrt(3); // sqrt(3) is the length of one axis vector
}

void FastHog3DComputer::initWeights()
{
	// init weights for each pixel with 1
	std::size_t nPixels = getNCellsXY() * getNCellsXY() * getNCellsT() * getNPixelsPerDim() * getNPixelsPerDim() * getNPixelsPerDim();
	_weights = VectorType(nPixels);
	for (int i = 0; i < _weights.size(); ++i)
		_weights[i] = 1;

	// pre-computed Gaussian weights only if wanted
	if (!_gaussWeighting)
		return;

	// compute weights; distance is measured in 'super pixels'
	// in this way it is invariant to the actual descriptor size
	double meanXY = 0.5 * (_nCellsXY * _nPixelsPerDim - 1);
	double meanT = 0.5 * (_nCellsT * _nPixelsPerDim - 1);
	double inv2SigmaSqXY = 0.5 / (meanXY * meanXY); // meanXY is half of the desc width/height = stdDevXY
	double inv2SigmaSqT = 0.5 / (meanT * meanT); // meanT is half of the desc length = stdDevT
	//cout << "# nCellsXY: " << _nCellsXY << " nCellsT: " << _nCellsT << " nPixelsPerDim: " << _nPixelsPerDim << endl;
	//cout << "# meanXY: " << meanXY << endl;
	//cout << "# meanT: " << meanT << endl;
	//cout << "# inv2SigmaSqXY: " << inv2SigmaSqXY << endl;
	//cout << "# inv2SigmaSqT: " << inv2SigmaSqT << endl;

	std::size_t iPix = 0;
	for (std::size_t iX = 0; iX < _nCellsXY; ++iX)
		for (std::size_t iY = 0; iY < _nCellsXY; ++iY)
			for (std::size_t iT = 0; iT < _nCellsT; ++iT)
				for (std::size_t iXPix = 0; iXPix < _nPixelsPerDim; ++iXPix)
					for (std::size_t iYPix = 0; iYPix < _nPixelsPerDim; ++iYPix)
						for (std::size_t iTPix = 0; iTPix < _nPixelsPerDim; ++iTPix, ++iPix) {
							assert(iPix < _weights.size());
							double distX = meanXY - (iX * _nPixelsPerDim + iXPix);
							double distY = meanXY - (iY * _nPixelsPerDim + iYPix);
							double distT = meanT - (iT * _nPixelsPerDim + iTPix);
							_weights[iPix] = exp(-inv2SigmaSqXY * distX * distX - inv2SigmaSqXY * distY * distY - inv2SigmaSqT * distT * distT);
						}
}

FastHog3DComputer::VectorType
FastHog3DComputer::getPlatonicHog3D(const VectorType& gradient) const
{
	// get the correct histogram size for either full or half orientation
	std::size_t histSize = _projectionMatrix.size1();
	if (_fullOrientation)
		histSize *= 2;
	VectorType histogram(histSize);
	histogram.clear();

	// check whether the gradient vector is not 0
	double normGradientVec = boost::numeric::ublas::norm_2(gradient);
	if (fuzzyEqual(normGradientVec, 0.0))
		return histogram;
		
	// project the gradient to all directions of the platonic solid
	VectorType projectionVec(_projectionMatrix.size1());
	boost::numeric::ublas::axpy_prod(_projectionMatrix, gradient, projectionVec, true);
	double threshold = _projectionThreshold * normGradientVec; //BUG: ... * boost::numeric::ublas::norm_inf(projectionVec);

	// create the histogram
	for (std::size_t i = 0; i < projectionVec.size(); ++i) {
		if (projectionVec[i] > threshold)
			histogram[i] = projectionVec[i] - threshold;
		else if (projectionVec[i] < -threshold)
			histogram[(i + projectionVec.size()) % histogram.size()] += -threshold - projectionVec[i];
	}
	
	// normalize the histogram such that the norm of the gradient is
	// distributed completely in the histogram
	double normHistogram = boost::numeric::ublas::norm_1(histogram);
	histogram *= normGradientVec / normHistogram;

	return histogram;
}

FastHog3DComputer::VectorType
FastHog3DComputer::getPolarHog3D(const VectorType& gradient) const
{
	// get the correct histogram size
	std::size_t histSize = _nPolarBinsXY * _nPolarBinsT;
	VectorType histogram(histSize);
	histogram.clear();

	// check whether the gradient vector is not 0
	double normGradientVec = boost::numeric::ublas::norm_2(gradient);
	if (fuzzyEqual(normGradientVec, 0.0))
		return histogram;

	// some general variables
	double binWidthDegXY = (_fullOrientation ? 360.0 : 180.0) / _nPolarBinsXY;
	double binWidthDegT = 180.0 / _nPolarBinsT;

	// get the orientations, note we shift the orientations since we do not want
	// the center of a bin to align with 0 degree for an orientation limited to 180 degree
	double orientationXY = cvFastArctan(gradient[0], gradient[1]);
	double orientationT = acos(gradient[2] / normGradientVec) / M_PI * 180;
	if (!_fullOrientation && orientationXY >= 180) {
		orientationXY -= 180;
		orientationT = 180 - orientationT;
	}
	
	// find the correct bin (for XY orientation)
	double fBinXY = (orientationXY) / binWidthDegXY;
	int iBinXY = static_cast<int>(roundf(fBinXY)) % _nPolarBinsXY;
	int iBin2XY = (fBinXY - iBinXY) > 0 ? // the second closest bin for interpolation
			(iBinXY + 1) % _nPolarBinsXY : (iBinXY - 1 + _nPolarBinsXY) % _nPolarBinsXY;
	assert(iBinXY >= 0 && iBinXY < _nPolarBinsXY);

	// find the correct bin (for T orientation) 
	// (we need a small offset such that the bin center will not be at degree 0)
	double fBinT = (orientationT - 0.5 * binWidthDegT) / binWidthDegT;
	int iBinT = static_cast<int>(roundf(fBinT)) % _nPolarBinsT;
	int iBin2T(0); // the second closest bin for interpolation
	if (fBinT < 0)
		iBin2T = 0;
	else if (fBinT > _nPolarBinsT - 1)
		iBin2T = _nPolarBinsT - 1;
	else 
		iBin2T = (fBinT - iBinT) > 0 ? (iBinT + 1) : (iBinT - 1);
	assert(iBinT >= 0 && iBinT < _nPolarBinsT);

//cout << "orientation XY: " << orientationXY << endl;
//cout << "orientation T: " << orientationT << endl;
//cout << "fBinXY: " << fBinXY << endl;
//cout << "fBinT: " << fBinT << endl;

	// compute the weights for distributing the magnitude between the four closest bins
	// (use linear interpolation)
	double weightXY = 1 - std::min<double>(fabs(fBinXY - iBinXY), _nPolarBinsXY - fBinXY);
	double weight2XY = 1 - weightXY;
	double weightT = 1 - std::min<double>(fabs(fBinT - iBinT), _nPolarBinsT - fBinT);
	double weight2T = 1 - weightT;
	
	// distribute gradient magnitude/norm into histogram
	histogram[iBinXY * _nPolarBinsT + iBinT] = weightXY * weightT * normGradientVec;
	histogram[iBin2XY * _nPolarBinsT + iBinT] = weight2XY * weightT * normGradientVec;
	histogram[iBinXY * _nPolarBinsT + iBin2T] = weightXY * weight2T * normGradientVec;
	histogram[iBin2XY * _nPolarBinsT + iBin2T] = weight2XY * weight2T * normGradientVec;

	return histogram;
}

FastHog3DComputer::VectorType 
FastHog3DComputer::getHog3D(const Box3D& orgBox) const
{
	// init some variables
	double xStart = orgBox.x;
	double yStart = orgBox.y;
	double tStart = orgBox.t;
	double cellWidth = orgBox.width / _nCellsXY;
	double cellHeight = orgBox.height / _nCellsXY;
	double cellLength = orgBox.length / _nCellsT;
	double strideFactor = 1;
	if (_overlappingCells) {
		cellWidth = 2 * orgBox.width / (_nCellsXY + 1);
		cellHeight = 2 * orgBox.height / (_nCellsXY + 1);
		cellLength = 2 * orgBox.length / (_nCellsT + 1);
		strideFactor = 0.5;
	}
	double hyperPixWidth = cellWidth / _nPixelsPerDim;
	double hyperPixHeight = cellHeight / _nPixelsPerDim;
	double hyperPixLength = cellLength / _nPixelsPerDim;
	double epsilon = 0.02 * _nPixelsPerDim * _nPixelsPerDim * _nPixelsPerDim;
	
	// computation
//	cout << "# " << orgBox.x << " " << orgBox.y << " " << orgBox.t << " " << orgBox.width << " " << orgBox.height << " " << orgBox.length << endl;
	Box3D box(0, 0, 0, hyperPixWidth, hyperPixHeight, hyperPixLength);
	std::size_t histSize = _quantization == PolarBinning ? _nPolarBinsXY * _nPolarBinsT : _projectionMatrix.size1();
	if (_fullOrientation && _quantization != PolarBinning)
		histSize *= 2;
	FastVideoGradientComputer::VectorType vec(_nCellsXY * _nCellsXY * _nCellsT * histSize);
	std::size_t iCellStart(0);
	std::size_t iPix = 0;
	double normSum(0);
	for (std::size_t iX = 0; iX < _nCellsXY; ++iX)
		for (std::size_t iY = 0; iY < _nCellsXY; ++iY)
			for (std::size_t iT = 0; iT < _nCellsT; ++iT) {
				// compute the histogram based on the hyper pixels
				VectorType cellHist(histSize);
				cellHist.clear();
//				cout << "# cell: " << iX << " " << iY << " " << iT << " width: " << cellWidth << " height: " << cellHeight << " length: " << cellLength << endl;
				
				for (std::size_t iXPix = 0; iXPix < _nPixelsPerDim; ++iXPix)
					for (std::size_t iYPix = 0; iYPix < _nPixelsPerDim; ++iYPix)
						for (std::size_t iTPix = 0; iTPix < _nPixelsPerDim; ++iTPix, ++iPix) {
							// prepare the exact position of the cell
							box.x = xStart + iX * strideFactor * cellWidth + iXPix * hyperPixWidth;
							box.y = yStart + iY * strideFactor * cellHeight + iYPix * hyperPixHeight;
							box.t = tStart + iT * strideFactor * cellLength + iTPix * hyperPixLength;
//							cout << "#   " << iXPix << "-" << iYPix << "-" << iTPix << endl;
//							cout << "#     w: " << _weights[iPix] << endl;
//							cout << "#     " << box.x << " " << box.y << " " << box.t << " " << box.width << " " << box.height << " " << box.length << endl; 

							// compute the descriptor for the cell
							FastVideoGradientComputer::VectorType gradientVec = _gradientComputer->getGradientVector(box);
//							cout << "#   " << gradientVec << endl;

							// compute HOG histogram
							VectorType pixHist = _quantization == PolarBinning ? getPolarHog3D(gradientVec) : getPlatonicHog3D(gradientVec);
//							cout << "#   " << pixHist << endl;
							boost::numeric::ublas::noalias(cellHist) += _weights[iPix] * pixHist;
						}

				// normalize cell histogram if necessary
				if (_normCells) {
					double norm = _l1Norm ? boost::numeric::ublas::norm_1(cellHist) : boost::numeric::ublas::norm_2(cellHist);
					normSum += norm;
					cellHist /= norm + epsilon;
//					cout << "#   cell norm: " << norm << endl;
				}
				// concatenate cells
				for (std::size_t j = 0; j < cellHist.size(); ++j)
					vec[iCellStart + j] = cellHist[j];
				iCellStart += cellHist.size();
//				cout << "#   cell hist:  " << cellHist << endl;

// dump for setting threshold
//cout << "# " << cellWidth * cellHeight * cellLength;
//for (std::size_t j = 0; j < cellHist.size(); ++j)
//	cout << " " << cellHist[j];
//cout << endl;

			}

	// set vector size to 0 if cells have been normalized an its
	// norm1 is below the given threshold
	if (_normCells) {
		if (normSum < _normThreshold)
			vec.resize(0);
	}
	// normalize the vector .. set its size to 0 if the norm 
	// is lower than the given threshold
	else {
		double norm = _l1Norm ? boost::numeric::ublas::norm_1(vec) : boost::numeric::ublas::norm_2(vec);
		if (norm > _normThreshold) {
			//cout << "# vec: " << vec << endl;
			//cout << "# norm: " << norm << endl;
			vec /= norm;
			//cout << "# vec2: " << vec << endl;
			
			// cut values and renormalize
			for (std::size_t j = 0; j < vec.size(); ++j)
				vec[j] = std::min(vec[j], _cutZero);
			norm = boost::numeric::ublas::norm_2(vec);
			vec /= norm;
			//cout << "# norm2: " << norm << endl;
			//cout << "# vec3: " << vec << endl;
		}
		else {
//			cout << "# norm too small: " << norm << endl;
			vec.resize(0);
		}
	}
	return vec;
}
