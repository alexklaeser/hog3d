#ifndef FASTHOG3DCOMPUTER_H_
#define FASTHOG3DCOMPUTER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "Box3D.h"

// forward declarations
class FastVideoGradientComputer;


class FastHog3DComputer
{
public:
	typedef boost::numeric::ublas::vector<double> VectorType;
	
	typedef enum { Icosahedron, Dodecahedron, PolarBinning } QuantizationType;
	
protected:
	typedef boost::numeric::ublas::matrix<double> Matrix;


	static const double goldenRatio;
	static const double projectionThresholdIcosahedron;
	static const double projectionThresholdDodecahedron;
	
	FastVideoGradientComputer* _gradientComputer;
	QuantizationType _quantization;
	std::size_t _nPolarBinsXY;
	std::size_t _nPolarBinsT;
	std::size_t _nCellsXY;
	std::size_t _nCellsT;
	std::size_t _nPixelsPerDim;
	double _normThreshold;
	double _cutZero;
	Matrix _projectionMatrix;
	double _projectionThreshold;
	bool _fullOrientation;
	VectorType _weights;
	bool _gaussWeighting;
	bool _overlappingCells;
	bool _normCells;
	bool _l1Norm;
	
	void initIcosahedron();
	
	void initDodecahedron(); 

	void initWeights();
	
	VectorType getPlatonicHog3D(const VectorType& gradient) const;
	
	VectorType getPolarHog3D(const VectorType& gradient) const;
	
public:
	FastHog3DComputer(FastVideoGradientComputer* gradientComputer = 0, QuantizationType quantization = Icosahedron, 
			std::size_t nPolarBinsXY = 6, std::size_t nPolarBinsT = 3,
			std::size_t nCellsXY = 4, std::size_t nCellsT = 3, std::size_t nPixelsPerDim = 3, 
			double normThreshold = 0.1, double cutZero = 0.25, bool fullOrientation = true, 
			bool gaussWeighting = false, bool overlappingCells = false, bool normCells = false, bool l1Norm = false)
		: _gradientComputer(gradientComputer), _quantization(quantization), 
		_nPolarBinsXY(nPolarBinsXY), _nPolarBinsT(nPolarBinsT), 
		_nCellsXY(nCellsXY), _nCellsT(nCellsT), _nPixelsPerDim(nPixelsPerDim),
		_normThreshold(normThreshold), _cutZero(cutZero), _fullOrientation(fullOrientation),
		_gaussWeighting(gaussWeighting), _overlappingCells(overlappingCells), 
		_normCells(normCells), _l1Norm(l1Norm)
	{
		initWeights();
		if (Icosahedron == quantization)
			initIcosahedron();
		else
			initDodecahedron();
	}
	
	virtual ~FastHog3DComputer()
	{ }
	
	QuantizationType getQuantization() const
	{
		return _quantization;
	}

	void setQuantization(QuantizationType q)
	{
		// same quantization, nothing to do
		if (q == _quantization)
			 return;

		// update the matrices
		_quantization = q;
		if (Icosahedron == q)
			initIcosahedron();
		else if (Dodecahedron == q)
			initDodecahedron();
	}

    std::size_t getNPolarBinsXY() const
    {
        return _nPolarBinsXY;
    }

    void setNPolarBinsXY(std::size_t nBins)
    {
        _nPolarBinsXY = nBins;
    }
	
    std::size_t getNPolarBinsT() const
    {
        return _nPolarBinsT;
    }

    void setNPolarBinsT(std::size_t nBins)
    {
        _nPolarBinsT = nBins;
    }
	
	std::size_t getNCellsXY() const
	{
		return _nCellsXY;
	}
	
	void setNCellsXY(std::size_t n)
	{
		_nCellsXY = n;
	}
	
	std::size_t getNCellsT() const
	{
		return _nCellsT;
	}
	
	void setNCellsT(std::size_t n)
	{
		_nCellsT = n;
	}
	
	std::size_t getNPixelsPerDim() const
	{
		return _nPixelsPerDim;
	}
	
	void setNPixelsPerDim(std::size_t n)
	{
		_nPixelsPerDim = n;
	}
	
	bool hasFullOrientation() const
	{
		return _fullOrientation;
	}
	
	void setFullOrientation(bool f)
	{
		_fullOrientation = f;
	}
	
	double getNormThreshold() const
	{
		return _normThreshold;
	}
	
	void setNormThreshold(double t)
	{
		_normThreshold = t;
	}
	
	double getCutZero() const
	{
		return _cutZero;
	}
	
	void setCutZero(double c)
	{
		_cutZero = c;
	}

	void setGaussWeighting(bool gaussWeighting) {
		_gaussWeighting = gaussWeighting;
	}
	
	bool hasGaussWeighting() const {
		return _gaussWeighting;
	}
	
	void setOverlappingCells(bool overlappingCells) {
		_overlappingCells = overlappingCells;
	}
	
	bool hasOverlappingCells() const {
		return _overlappingCells;
	}
	
	void setNormCells(bool normCells) {
		_normCells = normCells;
	}
	
	bool hasNormCells() const {
		return _normCells;
	}
	
	void setL1Norm(bool l1Norm) {
		_l1Norm = l1Norm;
	}
	
	bool hasL1Norm() const {
		return _l1Norm;
	}
	
	std::size_t getHistogramSize() const
	{
		return _projectionMatrix.size1() * 2;
	}
	
	std::size_t getDescSize() const
	{
		std::size_t histSize = _projectionMatrix.size1() * (hasFullOrientation() ? 2 : 1);
		if (PolarBinning == _quantization)
			histSize = _nPolarBinsT * _nPolarBinsXY;
		return getNCellsXY() * getNCellsXY() * getNCellsT() * histSize;
	}
	
	double getMinLength() const
	{
		return _nPixelsPerDim * _nCellsT;
	}
	
	double getMinWidth() const
	{
		return _nPixelsPerDim * _nCellsXY;
	}
	
	double getMinHeight() const
	{
		return _nPixelsPerDim * _nCellsXY;
	}

	const FastVideoGradientComputer* getGradientComputer() const
	{
		return _gradientComputer;
	}

	FastVideoGradientComputer* getGradientComputer()
	{
		return _gradientComputer;
	}

	void setGradientComputer(FastVideoGradientComputer* gradientComputer)
	{
		_gradientComputer = gradientComputer;
	}

	VectorType getHog3D(const Box3D& orgBox) const;
};

#endif /*FASTHOG3DCOMPUTER_H_*/
