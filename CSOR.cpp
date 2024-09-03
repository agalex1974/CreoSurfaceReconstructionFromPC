#include "CSOR.h"
#include <math.h>
#include <algorithm>
#include <execution>
#include <openblas/cblas.h>
#include <lapacke.h>

using namespace alglib;

CSOR::CSOR(const CPointCloud& pointcloud) :
	pcaMatrix(4, 4),
	icpMatrix(4, 4),
	cylindricalCoordinates(pointcloud.GetSize()),
	cartesianCoordinates(pointcloud.GetSize())
{
	icpMatrix.Identity();
	auto pcAligned = pointcloud.alignPointCloud(pcaMatrix);
	int count = 0;
	for (const auto& pnt : *pcAligned)
		cartesianCoordinates[count++] = CPointCloud::PointToVector(pnt);
	getCylindricalCoordinates();
}

void CSOR::getCylindricalCoordinates()
{
	for (int count = 0; count < cylindricalCoordinates.GetSize(); count++)
	{
		cylindricalCoordinates[count] = EuclideanToCylindrical(cartesianCoordinates[count]);
	}
}

void CSOR::getCartesianCoordinates()
{
	for (int count = 0; count < cartesianCoordinates.GetSize(); count++)
	{
		cartesianCoordinates[count] = CylindricalToEucledian(cartesianCoordinates[count]);
	}
}

CVector<double> CSOR::EuclideanToCylindrical(const CVector<double>& inVector)
{
	const double pi = 3.14159265359;
	CVector<double> outVector(3);
	outVector[0] = sqrt(inVector[0] * inVector[0] + inVector[1] * inVector[1]);
	outVector[1] = atan(inVector[1] / inVector[0]);
	outVector[2] = inVector[2];
	if (outVector[0] * cos(outVector[1]) * inVector[0] < 0) outVector[1] += pi;
	return outVector;
}

CVector<double> CSOR::CylindricalToEucledian(const CVector<double>& inVector, bool homogenous)
{
	CVector<double> outVector(4);
	outVector[0] = inVector[0] * cos(inVector[1]);
	outVector[1] = inVector[0] * sin(inVector[1]);
	outVector[2] = inVector[2];
	if (homogenous) outVector[3] = 1.0;
	else outVector[3] = 0.0;
	return outVector;
}

void CSOR::ExtractAxisAndGeneratrix(CVector<CPointEx3D>& curvePoints, CVector<CPointEx3D>& symmetricCurvePoints, CPointEx3D& axis)
{
	barycentricinterpolant p;
	findAlignedGeneratrix(p);
	double old = 1e10;
	double current = FindBestRigidTransformation(p);
	while (fabs(old - current) > 1e-7)
	{
		old = current;
		findAlignedGeneratrix(p);
		current = FindBestRigidTransformation(p);
	}
	getCurvePointsAndAxis(axis, curvePoints, symmetricCurvePoints, p);
}

void CSOR::findAxisAlignedSORExtents(double& min, double& max)
{
	max = -1e10;
	min = 1e10;
	for (int i = 0; i < cylindricalCoordinates.GetSize(); i++)
	{
		if (cylindricalCoordinates[i][2] > max) max = cylindricalCoordinates[i][2];
		if (cylindricalCoordinates[i][2] < min) min = cylindricalCoordinates[i][2];
	}
}

bool CSOR::sortFunction(pnt p1, pnt p2)
{
	return p1.x < p2.x;
}

void CSOR::getCurvePointsAndAxis(CPointEx3D& axis, CVector<CPointEx3D>& curvePoints, CVector<CPointEx3D>& symmetricCurvePoints, const barycentricinterpolant& p)
{
	double min, max;
	findAxisAlignedSORExtents(min, max);
	double step = 0.001;
	double current = min;
	while (current < max)
	{
		double v = barycentriccalc(p, current);
		curvePoints.Add(CPointEx3D(v, 0.0, current));
		current += step;
	}
	CMatrix<double> inversePCA = inverseTransformation(pcaMatrix);
	CMatrix<double> inverseICP = inverseTransformation(icpMatrix);
	CMatrix<double> inverse = Mult(inversePCA, inverseICP);
	symmetricCurvePoints.ReSize(curvePoints.GetSize());
	for (int i = 0; i < curvePoints.GetSize(); i++)
	{
		symmetricCurvePoints[i] = CPointCloud::VectorToPoint(Mult(inverse,
			CPointCloud::PointToVector(CPointEx3D(-curvePoints[i][0], curvePoints[i][1], curvePoints[i][2]))));
		curvePoints[i] = CPointCloud::VectorToPoint(Mult(inverse, CPointCloud::PointToVector(curvePoints[i])));
		
	}
	axis = CPointCloud::VectorToPoint(Mult(inverse, CPointCloud::PointToVector(CPointEx3D(0., 0., 1.), false)));
}

CMatrix<double> CSOR::inverseTransformation(const CMatrix<double>& transformationIn)
{
	CMatrix<double> inverse(4, 4);
	inverse.Identity();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			inverse(i, j) = transformationIn(j, i);
		}
	}
	CVector<double> trans(4);
	trans[0] = transformationIn(0, 3);
	trans[1] = transformationIn(1, 3);
	trans[2] = transformationIn(2, 3);
	trans[3] = 0.0;
	trans = Mult(inverse, trans);
	inverse(0, 3) = -trans[0];
	inverse(1, 3) = -trans[1];
	inverse(2, 3) = -trans[2];
	return inverse;
}

void CSOR::findAlignedGeneratrix(barycentricinterpolant& p)
{
	real_1d_array x;
	real_1d_array y;
	double* xr = new double[cylindricalCoordinates.GetSize()];
	double* yr = new double[cylindricalCoordinates.GetSize()];
	pnt* pnts = new pnt[cylindricalCoordinates.GetSize()];
	for (int i = 0; i < cylindricalCoordinates.GetSize(); i++)
	{
		pnts[i].x = cylindricalCoordinates[i][2];
		pnts[i].y = cylindricalCoordinates[i][0];
	}
	sort(std::execution::par_unseq, pnts, pnts + cylindricalCoordinates.GetSize(), sortFunction);
	for (int i = 0; i < cylindricalCoordinates.GetSize(); i++)
	{
		xr[i] = pnts[i].x;
		yr[i] = pnts[i].y;
	}
	x.setcontent(cylindricalCoordinates.GetSize(), xr);
	y.setcontent(cylindricalCoordinates.GetSize(), yr);
	delete[] xr;
	delete[] yr;
	delete[] pnts;

	ae_int_t m = 14;
	ae_int_t info;
	polynomialfitreport rep;

	//
	// Fitting without individual weights
	//
	// NOTE: result is returned as barycentricinterpolant structure.
	//       if you want to get representation in the power basis,
	//       you can use barycentricbar2pow() function to convert
	//       from barycentric to power representation (see docs for 
	//       POLINT subpackage for more info).
	//
	polynomialfit(x, y, m, info, p, rep);
}

double CSOR::FindBestRigidTransformation(const barycentricinterpolant& p)
{
	CVector<CPointEx3D> modelPnts(cartesianCoordinates.GetSize());
	CVector<CPointEx3D> sorPnts(cartesianCoordinates.GetSize());
#pragma omp parallel for
	for (int i = 0; i < modelPnts.GetSize(); i++)
	{
		double ri = barycentriccalc(p, cylindricalCoordinates[i][2]);
		double xi = ri * cos(cylindricalCoordinates[i][1]);
		double yi = ri * sin(cylindricalCoordinates[i][1]);
		modelPnts[i] = CPointEx3D(xi, yi, cylindricalCoordinates[i][2]);
		ri = cylindricalCoordinates[i][0];
		xi = ri * cos(cylindricalCoordinates[i][1]);
		yi = ri * sin(cylindricalCoordinates[i][1]);
		sorPnts[i] = CPointEx3D(xi, yi, cylindricalCoordinates[i][2]);
	}
	auto mp = CPointCloud::FindBaryCenter(sorPnts);
	auto mx = CPointCloud::FindBaryCenter(modelPnts);
	double* P, * X, * C;
	int m, k;
	double alpha = 1.0, beta = 0.0;
	m = 3;
	k = sorPnts.GetSize();
	int n = 3;
	P = (double*)malloc(m * k * sizeof(double));
	X = (double*)malloc(m * k * sizeof(double));
	C = (double*)malloc(m * n * sizeof(double));
	if (P == NULL || X == NULL || C == NULL) {
		printf("\n ERROR: Can't allocate memory for matrices. Aborting... \n\n");
		free(P);
		free(X);
		free(C);
		exit(0);
	}
	double n_inv = 1.0 / sqrt(sorPnts.GetSize());
#pragma omp parallel for
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < k; j++)
		{
			P[j + i * k] = n_inv * (sorPnts[j][i] - mp[i]);
			X[j + i * k] = n_inv * (modelPnts[j][i] - mx[i]);
		}
	}
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		m, n, k, alpha, P, k, X, k, beta, C, n);
	double A[9];
	double trace = 0.0;
	CMatrix<double> LL(3, 3);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			A[j + 3 * i] = C[j + 3 * i] - C[i + 3 * j];
			LL(i, j) = C[j + 3 * i] + C[i + 3 * j];
		}
		trace += C[i + 3 * i];
	}
	for (int i = 0; i < 3; i++) LL(i, i) -= trace;
	free(P);
	free(X);
	free(C);
	double delta[3];
	delta[0] = A[2 + 3 * 1];
	delta[1] = A[0 + 3 * 2];
	delta[2] = A[1 + 3 * 0];
	double Q[16] = { trace,    delta[0],        delta[1],         delta[2],
					 delta[0], LL(0,0), LL(0, 1), LL(0, 2),
					 delta[1], LL(1,0), LL(1, 1), LL(1, 2),
					 delta[2], LL(2,0), LL(2, 1), LL(2, 2) };
	n = 4;
	int lda = 4, info;
	/* Local arrays */
	double w[4];
	/* Solve eigenproblem */
	info = LAPACKE_dsyev(LAPACK_ROW_MAJOR, 'V', 'U', n, Q, lda, w);
	/* Check for convergence */
	if (info > 0) {
		printf("The algorithm failed to compute eigenvalues.\n");
		exit(1);
	}
	double q[4];
	for (int i = 0; i < 4; i++) q[i] = Q[3 + 4 * i];
	CMatrix<double> T(4, 4);
	T.SetVal(0.);
	T(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	T(0, 1) = 2 * (q[1] * q[2] - q[0] * q[3]);
	T(0, 2) = 2 * (q[1] * q[3] + q[0] * q[2]);
	T(1, 0) = 2 * (q[1] * q[2] + q[0] * q[3]);
	T(1, 1) = q[0] * q[0] + q[2] * q[2] - q[1] * q[1] - q[3] * q[3];
	T(1, 2) = 2 * (q[2] * q[3] - q[0] * q[1]);
	T(2, 0) = 2 * (q[1] * q[3] - q[0] * q[2]);
	T(2, 1) = 2 * (q[2] * q[3] + q[0] * q[1]);
	T(2, 2) = q[0] * q[0] + q[3] * q[3] - q[1] * q[1] - q[2] * q[2];
	CVector<double> qt = Mult(T, CPointCloud::PointToVector(mp));
	qt[0] = mx[0] - qt[0];
	qt[1] = mx[1] - qt[1];
	qt[2] = mx[2] - qt[2];
	T(0, 3) = qt[0];
	T(1, 3) = qt[1];
	T(2, 3) = qt[2];
	T(3, 3) = 1.0;
#pragma omp parallel for
	for (int i = 0; i < sorPnts.GetSize(); i++)
	{
		cartesianCoordinates[i] = Mult(T, cartesianCoordinates[i]);
	}
	CVector<CVector<double>> old = cylindricalCoordinates;
	getCylindricalCoordinates();
	icpMatrix = Mult(T, icpMatrix);
	double mse = 0.0;
	for (int i = 0; i < sorPnts.GetSize(); i++)
	{
		mse += (cartesianCoordinates[i][0] - modelPnts[i][0]) * (cartesianCoordinates[i][0] - modelPnts[i][0]) +
			(cartesianCoordinates[i][1] - modelPnts[i][1]) * (cartesianCoordinates[i][1] - modelPnts[i][1]) +
			(cartesianCoordinates[i][2] - modelPnts[i][2]) * (cartesianCoordinates[i][2] - modelPnts[i][2]);
	}
	mse *= 1.0 / sorPnts.GetSize();
	std::cout << mse << std::endl;
	return mse;
}

CMatrix<double> CSOR::GetTransormationMatrix() const
{
	return Mult(inverseTransformation(pcaMatrix), inverseTransformation(icpMatrix));
}

spPointCloud CSOR::reconstructSOR(const CVector<CPointEx3D>& generatrix, const CPointEx3D& axis, const CPointEx3D& bc)
{
	//we are going to proceed with steps of 10 degrees
	spPointCloud sorPoints = std::make_shared<CPointCloud>(36 * generatrix.GetSize());
	for (int i = 0; i < generatrix.GetSize(); i++)
	{
		(*sorPoints)[i].first = generatrix[i] - bc;
	}
	int start = generatrix.GetSize();
	for (int angle = 10; angle < 360; angle += 10)
	{
		CMatrix<double> R = findRotationMatrixAroundAnAxisOfRevolution(angle, axis);
		for (int i = 0; i < generatrix.GetSize(); i++)
		{
			(*sorPoints)[i + start].first = CPointCloud::VectorToPoint(Mult(R, CPointCloud::PointToVector((*sorPoints)[i].first)));
		}
		start += generatrix.GetSize();
	}
	for (int i = 0; i < sorPoints->GetSize(); i++)
	{
		(*sorPoints)[i].first += bc;
	}
	return sorPoints;
}

CMatrix<double> CSOR::findRotationMatrixAroundAnAxisOfRevolution(double angle, const CPointEx3D& axis)
{
	const double u = axis[0], v = axis[1], w = axis[2];
	CMatrix<double> rotationMatrix(4, 4);
	const double L = u * u + v * v + w * w;
	angle = angle * M_PI / 180.0; //converting to radian value
	const double u2 = u * u;
	const double v2 = v * v;
	const double w2 = w * w;

	rotationMatrix(0, 0) = (u2 + (v2 + w2) * cos(angle)) / L;
	rotationMatrix(0, 1) = (u * v * (1 - cos(angle)) - w * sqrt(L) * sin(angle)) / L;
	rotationMatrix(0, 2) = (u * w * (1 - cos(angle)) + v * sqrt(L) * sin(angle)) / L;
	rotationMatrix(0, 3) = 0.0;

	rotationMatrix(1, 0) = (u * v * (1 - cos(angle)) + w * sqrt(L) * sin(angle)) / L;
	rotationMatrix(1, 1) = (v2 + (u2 + w2) * cos(angle)) / L;
	rotationMatrix(1, 2) = (v * w * (1 - cos(angle)) - u * sqrt(L) * sin(angle)) / L;
	rotationMatrix(1, 3) = 0.0;

	rotationMatrix(2, 0) = (u * w * (1 - cos(angle)) - v * sqrt(L) * sin(angle)) / L;
	rotationMatrix(2, 1) = (v * w * (1 - cos(angle)) + u * sqrt(L) * sin(angle)) / L;
	rotationMatrix(2, 2) = (w2 + (u2 + v2) * cos(angle)) / L;
	rotationMatrix(2, 3) = 0.0;

	rotationMatrix(3, 0) = 0.0;
	rotationMatrix(3, 1) = 0.0;
	rotationMatrix(3, 2) = 0.0;
	rotationMatrix(3, 3) = 1.0;

	return rotationMatrix;
}
