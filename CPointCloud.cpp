//#include <mkl.h>
#include <cstdio>
#include <cstdlib>
#include <map>
#include "CPointCloud.h"
#include "NurbsLibSt.h"
#include <iostream>
//#include <boost/math/distributions/normal.hpp>
#include <numeric>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

void CPointCloud::createUmbrellaFromTriangulation(std::vector<std::map<int, UmbrellaElement>>& umbrellas) const
{
	FILE* file = fopen("vase_sor_smoothed.off", "r");
	char off[10];
	int numbPnts, numbTris, numbEdges;
	fscanf(file, "%s", off);
	fscanf(file, "%d %d %d", &numbPnts, &numbTris, &numbEdges);
	//FILE* file1 = fopen("info.txt", "w");
	//fprintf(file1, "%d %d\n", numbPnts, numbTris);
	for (int i = 0; i < numbPnts; i++)
	{
		float x, y, z;
		fscanf(file, "%f %f %f", &x, &y, &z);
	}
	umbrellas.resize(numbPnts);



	for (int t = 0; t < numbTris; t++)
	{
		int numb, i, j, k;
		fscanf(file, "%d %d %d %d", &numb, &i, &j, &k);
		for (int u = 0; u < 3; u++) {
			if (umbrellas[i].find(j) != umbrellas[i].end())
			{
				umbrellas[i][j].indexNext = k;
			}
			else
			{
				umbrellas[i][j] = { j, -1, k };
			}
			if (umbrellas[j].find(i) != umbrellas[j].end())
			{
				umbrellas[j][i].indexPrevious = k;
			}
			else
			{
				umbrellas[j][i] = { i, k, -1 };
			}
			int i1 = i, j1 = j, k1 = k;
			i = j1; j = k1; k = i1;
		}
	}
	fclose(file);
}
/*
 * for (int u = 0; u < 3; u++) {
			//if (i < numbPnts && i >= 0) {
				if (umbrellas[i].find(j) != umbrellas[i].end())
				{
					umbrellas[i][j].indexNext = k;
					if (umbrellas[i].find(k) != umbrellas[i].end())
					{
						umbrellas[i][k].indexPrevious = j;
					}
					else
					{
						umbrellas[i][k] = { k, j, -1 };
					}
				}
				else
				{
					umbrellas[i][j] = { j, -1, k };
					if (umbrellas[i].find(k) != umbrellas[i].end())
					{
						umbrellas[i][k].indexPrevious = j;
					}
					else
					{
						umbrellas[i][k] = { k, j, -1 };
					}
				}
			//}
			int i1 = i, j1 = j, k1 = k;
			i = j1; j = k1; k = i1;
		}
 */
spPointCloud CPointCloud::CreatePointCloudFromFileWithNormals(const char* fileName, bool onlyPnts)
{
	int countPoints = 0;
	FILE* file = fopen(fileName, "r");
	CPointEx3D pnt, nrm;
	spPointCloud pc;
	if (!onlyPnts)
	{
		while (fscanf(file, "%lf %lf %lf %lf %lf %lf", &pnt[0], &pnt[1], &pnt[2], &nrm[0], &nrm[1], &nrm[2]) == 6)
			countPoints++;
		rewind(file);
		pc = std::make_shared<CPointCloud>(countPoints);
		int currentIndex = 0;
		while (fscanf(file, "%lf %lf %lf %lf %lf %lf", &pnt[0], &pnt[1], &pnt[2], &nrm[0], &nrm[1], &nrm[2]) == 6)
		{
			(*pc)[currentIndex].first = pnt;
			(*pc)[currentIndex++].second = nrm;
		}
	}
	else
	{
		while (fscanf(file, "%lf %lf %lf", &pnt[0], &pnt[1], &pnt[2]) == 3)
			countPoints++;
		rewind(file);
		pc = std::make_shared<CPointCloud>(countPoints);
		int currentIndex = 0;
		while (fscanf(file, "%lf %lf %lf", &pnt[0], &pnt[1], &pnt[2]) == 3)
		{
			(*pc)[currentIndex].first = pnt;
			(*pc)[currentIndex++].second = nrm;
		}
	}
	fclose(file);
	return pc;
}

CPointCloud::CPointCloud(const char* file, bool onlyPnts)
{
	auto pc = CreatePointCloudFromFileWithNormals(file, onlyPnts);
	*this = *pc;
}

void CPointCloud::savePointCloud(const char* fileName, bool withNormals) const
{
	FILE* file = fopen(fileName, "w");
	for (int i = 0; i < this->GetSize(); i++)
	{
		if (withNormals)
			fprintf(file, "%lf %lf %lf %lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2],
				normals[i][0], normals[i][1], normals[i][2]);
		else fprintf(file, "%lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]);
	}
	fclose(file);
}

CPointCloud::CPointCloud(const CPointCloud& pcSource):
	CVector<CPointEx3D>(pcSource)
{
	normals = pcSource.normals;
}

CPointCloud& CPointCloud::operator=(const CPointCloud& d)
{
	if (this != &d) {
		CVector<CPointEx3D>::operator=(d);
		normals = d.normals;
	}
	return *this;
}

CPointCloud::CPointCloud(int MaxSize, int GrowRate):
	CVector<CPointEx3D>(MaxSize, GrowRate),
	normals(MaxSize, GrowRate)
{}

CPointEx3D CPointCloud::FindBaryCenter() const
{
	CPointEx3D baryCenter(0., 0., 0.);
	for (int i = 0; i < GetSize(); i++)
		baryCenter += (*this)[i].first;
	if (this->GetSize())
		baryCenter /= this->GetSize();
	return baryCenter;
}

CPointEx3D CPointCloud::FindBaryCenter(const CVector<CPointEx3D>& pnts)
{
	CPointEx3D baryCenter(0., 0., 0.);
	for (int i = 0; i < pnts.GetSize(); i++) baryCenter += pnts[i];
	if (pnts.GetSize())
		baryCenter /= pnts.GetSize();
	return baryCenter;
}

void CPointCloud::Add(const CPointEx3D& point)
{
	CVector<CPointEx3D>::Add(point);
	normals.Add(CPointEx3D(0., 0., 0.));
}

void CPointCloud::Add(const CPointEx3D& point, const CPointEx3D& normal)
{
	CVector<CPointEx3D>::Add(point);
	normals.Add(normal);
}

std::pair<CPointEx3D&, CPointEx3D&> CPointCloud::operator[](int i)
{
	CPointEx3D& a = CVector<CPointEx3D>::operator[](i);
	CPointEx3D& b = normals[i];
	return { a, b };
}

std::pair<CPointEx3D, CPointEx3D> CPointCloud::operator[](int i) const
{
	const CPointEx3D& a = CVector<CPointEx3D>::operator[](i);
	const CPointEx3D& b = normals[i];
	return { a, b };
}

double CPointCloud::AreaOfTriangle(const CPointEx3D& pnt1, const CPointEx3D& pnt2, const CPointEx3D& pnt3)
{
	const double a = (pnt2 - pnt1).norm();
	const double b = (pnt3 - pnt2).norm();
	const double c = (pnt3 - pnt1).norm();
	const double s = (a + b + c) / 2;
	return sqrt(s * (s - a) * (s - b) * (s - c));
}

CPointEx3D CPointCloud::NormalOfTriangle(const CPointEx3D& pnt1, const CPointEx3D& pnt2, const CPointEx3D& pnt3)
{
	CPointEx3D vec12 = pnt2 - pnt1;
	CPointEx3D vec13 = pnt3 - pnt1;
	CPointEx3D normal = cross(vec12, vec13);
	if (normal.norm() == 0.0) return CPointEx3D(0, 0, 0);
	normal.normalize();
	return normal;
}

std::tuple<double, double, CPointEx3D, CPointEx3D> CPointCloud::GabrielTaubinCurvatureTensor(int i, const std::vector<UmbrellaElement>& umbrella, 
	CPointEx3D& normal) const
{	
	CPointEx3D Nvi(0, 0, 0);
	const CPointEx3D& vi = (*this)[i].first;
	for (const auto& umbel : umbrella)
	{
		const CPointEx3D& vj = (*this)[umbel.index].first;
		if (umbel.indexNext >= 0) 
		{
			const CPointEx3D& vk = (*this)[umbel.indexNext].first;
			Nvi += AreaOfTriangle(vi, vj, vk) * NormalOfTriangle(vi, vj, vk);
		}
	}
	Nvi.normalize();
	normal = Nvi;
	std::vector<double> wi(umbrella.size(), 0.0);
	
	for (int j = 0; j < umbrella.size(); j++)
	{
		const CPointEx3D& vj = (*this)[umbrella[j].index].first;
		if (umbrella[j].indexNext >= 0) {
			const CPointEx3D& vk = (*this)[umbrella[j].indexNext].first;
			wi[j] += AreaOfTriangle(vi, vj, vk);
		}
		if (umbrella[j].indexPrevious >= 0) {
			const CPointEx3D& vk = (*this)[umbrella[j].indexPrevious].first;
			wi[j] += AreaOfTriangle(vi, vj, vk);
		}
	}
	
	double sum = std::accumulate(wi.begin(), wi.end(), 0.0);
	std::transform(wi.begin(), wi.end(), wi.begin(), [sum](double x) {return x / sum; });
	CDoubleMatrix I(3, 3);
	I.Identity();
	CDoubleMatrix Nvim(3, 1);
	Nvim(0, 0) = Nvi.x;
	Nvim(1, 0) = Nvi.y;
	Nvim(2, 0) = Nvi.z;
	CDoubleMatrix mat = Mult(Nvim, Nvim.GetTranspose());
	Mult(mat, -1.0);
	::Add(I, mat, mat);
	CDoubleMatrix M(3, 3);
	M.SetVal(0.0);
	for (int j = 0; j < umbrella.size(); j++)
	{
		CPointEx3D vj = (*this)[umbrella[j].index].first;
		CPointEx3D vjvi = vi - vj;
		CDoubleVector vivjVec(3);
		vivjVec[0] = vjvi.x; vivjVec[1] = vjvi.y; vivjVec[2] = vjvi.z;
		CDoubleVector TiTjVec = Mult(mat, vivjVec);
		CPointEx3D TiTj(TiTjVec[0], TiTjVec[1], TiTjVec[2]);
		TiTj.normalize();
		double kij = -2.0 * dot(Nvi, vjvi) / vjvi.sqr_norm();
		CDoubleMatrix TiTjm(3, 1);
		TiTjm(0, 0) = TiTj.x;
		TiTjm(1, 0) = TiTj.y;
		TiTjm(2, 0) = TiTj.z;
		CDoubleMatrix matrix = Mult(TiTjm, TiTjm.GetTranspose());
		Mult(matrix, kij);
		Mult(matrix, wi[j]);
		::Add(M, matrix, M);
	}
	CPointEx3D E1(1.0, 0.0, 0.0);
	double plus = (E1 + Nvi).norm();
	double minus = (E1 - Nvi).norm();
	CPointEx3D Wvi;
	if (plus > minus) Wvi = E1 + Nvi;
	else Wvi = E1 - Nvi;
	Wvi.normalize();
	CDoubleMatrix Wvim(3, 1);
	Wvim(0, 0) = Wvi[0];
	Wvim(1, 0) = Wvi[1];
	Wvim(2, 0) = Wvi[2];
	CDoubleMatrix matrix = Mult(Wvim, Wvim.GetTranspose());
	Mult(matrix, -2.0);
	::Add(I, matrix, matrix);
	CDoubleMatrix Qvi = matrix;
	CDoubleMatrix temp = Mult(matrix.GetTranspose(), M);
	CDoubleMatrix K = Mult(temp, matrix);
	double a = K(1, 1);
	double b = K(2, 2);
	double c = K(1, 2);
	double sintheta, costheta;
	if (c != 0.0) {
		double denominator = sqrt((a - b) * (a - b) + 4 * c * c);
		double sin2theta = 2 * c / denominator;
		double cos2theta = (a - b) / denominator;
		costheta = sqrt((1.0 + cos2theta) / 2.0);
		if (c < 0) costheta = -costheta;
		if (costheta != 0.0)
			sintheta = 0.5 * sin2theta / costheta;
		else sintheta = 1.0;
	}
	else
	{
		if (a > b) {
			sintheta = 0.0;
			costheta = 1.0;
		}
		else
		{
			sintheta = 1.0;
			costheta = 0.0;
		}
	}
	CDoubleMatrix G(2, 2);
	G(0, 0) = costheta;
	G(0, 1) = -sintheta;
	G(1, 0) = sintheta;
	G(1, 1) = costheta;
	CDoubleMatrix M2x2(2, 2);
	M2x2(0, 0) = K(1, 1);
	M2x2(0, 1) = K(1, 2);
	M2x2(1, 0) = K(2, 1);
	M2x2(1, 1) = K(2, 2);
	temp = Mult(G.GetTranspose(), M2x2);
	
	CDoubleMatrix D = Mult(temp, G);
	double k1 = D(0, 0), k2 = D(1, 1);
	CPointEx3D T1per(Qvi(0, 1), Qvi(1, 1), Qvi(2, 1));
	CPointEx3D T2per(Qvi(0, 2), Qvi(1, 2), Qvi(2, 2));
	CPointEx3D T1 = costheta * T1per - sintheta * T2per;
	CPointEx3D T2 = sintheta * T1per + costheta * T2per;
	if (k1 > k2)
	{
		return { k1, k2, T1, T2 };
	}
	else
	{
		return { k2, k1, T2, T1 };
	}
}

std::vector<CPointCloud::UmbrellaElement> CPointCloud::CreateUmbrellaAroundPoint(int originIndex, const std::vector<int>& oneRing) const
{
	const CPointEx3D& origin = (*this)[originIndex].first;
	const CPointEx3D& originNormal = (*this)[originIndex].second;
	CDoubleMatrix transformationMatrix = CreateLocalCoordinateSystem(origin, originNormal, Z_AXIS);
	CPointEx3DVector umbrellaEdges(oneRing.size());
	for (int i = 0; i < oneRing.size(); i++)
	{
		umbrellaEdges[i] = VectorToPoint(Mult(transformationMatrix, PointToVector((*this)[oneRing[i]].first)));
		umbrellaEdges[i][2] = 0.0;
	}
	std::vector<UmbrellaElement> umbrella(oneRing.size());
	for (int i = 0; i < oneRing.size(); i++)
	{
		UmbrellaElement umbrellaEllement;
		umbrellaEllement.index = oneRing[i];
		std::vector<sortElement> elementsCCW;
		std::vector<sortElement> elementsCW;
		for (int j = 0; j < oneRing.size(); j++)
		{
			if (i != j) 
			{
				if (cross(umbrellaEdges[i], umbrellaEdges[j]).z > 0)
				{
					double costheta = dot(umbrellaEdges[i], umbrellaEdges[j]) / (umbrellaEdges[i].norm() * umbrellaEdges[j].norm());
					if (costheta > -0.6) {
						elementsCCW.push_back({ -costheta, oneRing[j] });
					}
				}
				else
				{
					double costheta = dot(umbrellaEdges[i], umbrellaEdges[j]) / (umbrellaEdges[i].norm() * umbrellaEdges[j].norm());
					if (costheta > -0.6) {
						elementsCW.push_back({ -costheta, oneRing[j] });
					}
				}
			}
		}
		if (elementsCCW.size() > 0) {
			sort(elementsCCW.begin(), elementsCCW.end(), sortFunction);
			umbrellaEllement.indexNext = elementsCCW[0].index;
		}
		else umbrellaEllement.indexNext = -1;
		if (elementsCW.size() > 0) {
			sort(elementsCW.begin(), elementsCW.end(), sortFunction);
			umbrellaEllement.indexPrevious = elementsCW[0].index;
		}
		else umbrellaEllement.indexPrevious = -1;
		umbrella[i] = umbrellaEllement;
	}
	return umbrella;
}

CDoubleMatrix CPointCloud::CreateLocalCoordinateSystem(const CPointEx3D& o, const CPointEx3D& d, Axis axis)
{
	char indexSet[3];
	switch (axis)
	{
		case X_AXIS:
		{
			indexSet[0] = 1;
			indexSet[1] = 2;
			indexSet[2] = 0;
			break;
		}
		case Y_AXIS:
		{
			indexSet[0] = 2;
			indexSet[1] = 0;
			indexSet[2] = 1;
			break;
		}
		case Z_AXIS:
		{
			indexSet[0] = 0;
			indexSet[1] = 1;
			indexSet[2] = 2;
		}
	}
	CPointEx3D firstPerpendicular = GetNormalizedPerpendicularVectorToVector(d);
	CPointEx3D secondPerpendicular = cross(d, firstPerpendicular);
	secondPerpendicular.normalize();
	CDoubleMatrix transformationMatrix(4, 4);
	transformationMatrix.Identity();
	transformationMatrix(indexSet[0], 0) = firstPerpendicular[0];
	transformationMatrix(indexSet[0], 1) = firstPerpendicular[1];
	transformationMatrix(indexSet[0], 2) = firstPerpendicular[2];
	transformationMatrix(indexSet[1], 0) = secondPerpendicular[0];
	transformationMatrix(indexSet[1], 1) = secondPerpendicular[1];
	transformationMatrix(indexSet[1], 2) = secondPerpendicular[2];
	transformationMatrix(indexSet[2], 0) = d[0];
	transformationMatrix(indexSet[2], 1) = d[1];
	transformationMatrix(indexSet[2], 2) = d[2];
	auto t = Mult(transformationMatrix, PointToVector(o));
	transformationMatrix(0, 3) = -t[0];
	transformationMatrix(1, 3) = -t[1];
	transformationMatrix(2, 3) = -t[2];
	return transformationMatrix;
}

void CPointCloud::FindPrincipalAxes(CMatrix<double>& mat, char zIndex) const
{
	auto bc = FindBaryCenter();
	double* A, * C;
	int m, k;
	double alpha = 1.0, beta = 0.0;
	m = 3;
	k = this->GetSize();
	int n = 3;
	A = (double*)mkl_malloc(m * k * sizeof(double), 64);
	C = (double*)mkl_malloc(m * n * sizeof(double), 64);
	if (A == NULL || C == NULL) {
		printf("\n ERROR: Can't allocate memory for matrices. Aborting... \n\n");
		mkl_free(A);
		mkl_free(C);
		return;
	}
	double n_inv = 1.0 / sqrt(this->GetSize());
#pragma omp parallel for
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < k; j++)
		{
			A[j + i * k] = n_inv * ((*this)[j].first[i] - bc[i]);
		}
	}
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		m, n, k, alpha, A, k, A, k, beta, C, n);
	MKL_INT lda = n, info;
	/* Local arrays */
	double w[3];
	/* Solve eigenproblem */
	info = LAPACKE_dsyev(LAPACK_ROW_MAJOR, 'V', 'U', n, C, lda, w);
	/* Check for convergence */
	if (info > 0) {
		printf("The algorithm failed to compute eigenvalues.\n");
		exit(1);
	}
	///////////////////////////////////////////////////////////////////////
	//
	mat.SetVal(0.);
	char indexSet[3];
	switch(zIndex)
	{
		case 0:
		{
			indexSet[0] = 1;
			indexSet[1] = 2;
			indexSet[2] = 0;
			break;
		}
		case 1:
		{
			indexSet[0] = 2;
			indexSet[1] = 0;
			indexSet[2] = 1;
			break;
		}
		case 2:
		{
			indexSet[0] = 0;
			indexSet[1] = 1;
			indexSet[2] = 2;
		}
	}
	mat(3, 3) = 1.0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
		{
			mat(indexSet[j], i) = C[j + i * 3];
		}
	}
	FILE* file = fopen("RotationMatrixPCA.txt", "w");
	fprintf(file, "%lf %lf %lf\n", mat(0,0), mat(0,1), mat(0,2));
	fprintf(file, "%lf %lf %lf\n", mat(1, 0), mat(1, 1), mat(1, 2));
	fprintf(file, "%lf %lf %lf\n", mat(2, 0), mat(2, 1), mat(2, 2));
	fclose(file);
	auto res = Mult(mat, PointToVector(bc));
	bc = VectorToPoint(res);
	mat(0, 3) = -bc[0];
	mat(1, 3) = -bc[1];
	mat(2, 3) = -bc[2];
	mat(3, 3) = 1.0;
	mkl_free(A);
	mkl_free(C);
}

bool CPointCloud::sortFunction(sortElement element1, sortElement element2)
{
	return element1.value < element2.value;
}

CVector<double> CPointCloud::PointToVector(const CPointEx3D& pnt, bool homogenous)
{
	CVector<double> vec(4);
	vec[0] = pnt[0];
	vec[1] = pnt[1];
	vec[2] = pnt[2];
	if (homogenous) vec[3] = 1.0;
	else vec[3] = 0.0;
	return vec;
}

CPointEx3D CPointCloud::VectorToPoint(const CVector<double>& vec)
{
	CPointEx3D pnt;
	pnt[0] = vec[0];
	pnt[1] = vec[1];
	pnt[2] = vec[2];
	return pnt;
}

spPointCloud CPointCloud::rotatePointCloud(const CMatrix<double>& mat) const
{
	spPointCloud pc = std::make_shared<CPointCloud>(this->GetSize());
	for (int i = 0; i < this->GetSize(); i++)
	{
		auto pnt = Mult(mat, PointToVector((*this)[i].first));
		auto nrm = Mult(mat, PointToVector((*this)[i].second, false));
		(*pc)[i].first = VectorToPoint(pnt);
		(*pc)[i].second = VectorToPoint(nrm);
	}
	return pc;
}

spPointCloud CPointCloud::alignPointCloud(CMatrix<double>& mat, char zIndex) const
{
	//CMatrix<double> mat(4, 4);
	FindPrincipalAxes(mat, zIndex);
	return rotatePointCloud(mat);
}

CPointEx3D* CPointCloud::begin()
{
	return m_pData;
}

CPointEx3D* CPointCloud::end()
{
	if (m_pData)
		return &m_pData[this->GetSize()];
	return nullptr;
}

const CPointEx3D* CPointCloud::begin() const
{
	return m_pData;
}

const CPointEx3D* CPointCloud::end() const
{
	if (m_pData)
		return &m_pData[this->GetSize()];
	return nullptr;
}

spPointCloud CPointCloud::addNoiseToPointCloud(double standard_deviation) const
{
	spPointCloud pc = std::make_shared<CPointCloud>(this->GetSize());
	*pc = *this;
	
	boost::mt19937 rng; rng.seed(static_cast<unsigned int> (time(0)));
	boost::normal_distribution<> nd(0, standard_deviation);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
	for (auto& pnt : *pc) {
		pnt[0] += var_nor();
		pnt[1] += var_nor();
		pnt[2] += var_nor();
	}
	return pc;
}

void CPointCloud::scaleToUnity(CVector<CPointEx3D>& points)
{
	double maxx = -1e10;
	double minx =  1e10;
	double maxy = -1e10;
	double miny = 1e10;
	double maxz = -1e10;
	double minz = 1e10;
	for (int i = 0; i < points.GetSize(); i++)
	{
		if (points[i].x > maxx) maxx = points[i].x;
		if (points[i].x < minx) minx = points[i].x;
		if (points[i].y > maxy) maxy = points[i].y;
		if (points[i].y < miny) miny = points[i].y;
		if (points[i].z > maxz) maxz = points[i].z;
		if (points[i].z < minz) minz = points[i].z;
	}

	double max = maxx;
	if (max < maxy) max = maxy;
	if (max < maxz) max = maxz;

	double min = minx;
	if (min > miny) min = miny;
	if (min > minz) min = minz;
	for (int i = 0; i < points.GetSize(); i++)
	{
		points[i].x = (points[i].x - min) / (max - min);
		points[i].y = (points[i].y - min) / (max - min);
		points[i].z = (points[i].z - min) / (max - min);
	}

	auto bc = FindBaryCenter(points);
	for (int i = 0; i < points.GetSize(); i++)
	{
		points[i] -= bc;
	}
}


std::shared_ptr<CMatrix<double>> CPointCloud::getSlippableVectors(const CPointEx3D& bc) const
{
	//auto bc = FindBaryCenter();
	CVector<CPointEx3D> crossPoints(GetSize());
	std::shared_ptr<CMatrix<double>> slippableVectors;
	int count = 0;
	for (int i = 0; i < GetSize(); i++)
	{
		crossPoints[i] = (*this)[i].first - bc;	
	}
	scaleToUnity(crossPoints);
	for (int i = 0; i < GetSize(); i++)
	{
		crossPoints[i] = cross(crossPoints[i], (*this)[i].second);
	}

	double* A, * C;
	int m, k;
	double alpha = 1.0, beta = 0.0;
	m = 6;
	k = GetSize();
	int n = 6;
	A = (double*)mkl_malloc(m * k * sizeof(double), 64);
	C = (double*)mkl_malloc(m * n * sizeof(double), 64);
	if (A == NULL || C == NULL) {
		printf("\n ERROR: Can't allocate memory for matrices. Aborting... \n\n");
		mkl_free(A);
		mkl_free(C);
		return nullptr;
	}
	double n_inv = 1.0 / sqrt(this->GetSize());
#pragma omp parallel for
	for (int j = 0; j < k; j++)
	{
		A[j + 0 * k] = crossPoints[j][0];
		A[j + 1 * k] = crossPoints[j][1];
		A[j + 2 * k] = crossPoints[j][2];
		A[j + 3 * k] = (*this)[j].second[0];
		A[j + 4 * k] = (*this)[j].second[1];
		A[j + 5 * k] = (*this)[j].second[2];
	}

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		m, n, k, alpha, A, k, A, k, beta, C, n);
	MKL_INT lda = n, info;
	
	double w[6];
	
	info = LAPACKE_dsyev(LAPACK_ROW_MAJOR, 'V', 'U', n, C, lda, w);
	
	if (info > 0) {
		printf("The algorithm failed to compute eigenvalues.\n");
		exit(1);
	}
	
	count = 0;
	double eps = 1e-10;
	while (w[5] / (w[count++] + eps) > 400){}
	count--;
	if (count > 0) {
		FILE* file = fopen("eigen_values.txt", "w");
		for (int i = 0; i < count; i++)
		{
			fprintf(file, "%lf ", w[i]);
		}
		fclose(file);
		slippableVectors = std::make_shared<CMatrix<double>>(6, count);
		for (int dimCount = 0; dimCount < 6; dimCount++)
		{
			for (int vectorCount = 0; vectorCount < count; vectorCount++)
			{
				(*slippableVectors)(dimCount, vectorCount) = C[vectorCount + dimCount * 6];
			}
		}
	}
	mkl_free(A);
	mkl_free(C);
	return slippableVectors;
}

CPointEx3D CPointCloud::GetNormalizedPerpendicularVectorToVector(const CPointEx3D& inVector)
{
	double max = fabs(inVector[0]);
	int cordIndex = 0;

	if (max < fabs(inVector[1]))
	{
		cordIndex = 1;
		max = fabs(inVector[1]);
	}

	if (max < fabs(inVector[2]))
	{
		cordIndex = 2;
	}
	CPointEx3D outVector;
	outVector[0] = 1.0;
	outVector[1] = 1.0;
	outVector[2] = 1.0;

	switch (cordIndex)
	{
	case 0:
		outVector[0] = (-inVector[1] * outVector[1] - inVector[2] * outVector[2]) / inVector[0];
		break;
	case 1:
		outVector[1] = (-inVector[0] * outVector[0] - inVector[2] * outVector[2]) / inVector[1];
		break;
	case 2:
		outVector[2] = (-inVector[0] * outVector[0] - inVector[1] * outVector[1]) / inVector[2];
		break;
	}
	outVector.normalize();
	return outVector;
}

CDoubleMatrix CPointCloud::CreatePatchLocalCoordinateSystem(const CPointEx3D& p, const CPointEx3D& n, 
	CPointEx3D& firstPerpendicular, CPointEx3D& secondPerpendicular)
{
	firstPerpendicular = GetNormalizedPerpendicularVectorToVector(n);
	secondPerpendicular = cross(n, firstPerpendicular);
	secondPerpendicular.normalize();
	CDoubleMatrix transformationMatrix(4, 4);
	transformationMatrix.Identity();
	transformationMatrix(0, 0) = firstPerpendicular[0];
	transformationMatrix(0, 1) = firstPerpendicular[1];
	transformationMatrix(0, 2) = firstPerpendicular[2];
	transformationMatrix(1, 0) = secondPerpendicular[0];
	transformationMatrix(1, 1) = secondPerpendicular[1];
	transformationMatrix(1, 2) = secondPerpendicular[2];
	transformationMatrix(2, 0) = n[0];
	transformationMatrix(2, 1) = n[1];
	transformationMatrix(2, 2) = n[2];
	auto t = Mult(transformationMatrix, PointToVector(p));
	transformationMatrix(0, 3) = -t[0];
	transformationMatrix(1, 3) = -t[1];
	transformationMatrix(2, 3) = -t[2];
	return transformationMatrix;
}

std::pair<double, double> CPointCloud::GetCurvaturesOfLocalPatch(const CPointEx3D& p, const CPointEx3D& n, const CPointCloud& pc, 
	CPointEx3D& maximumDirection, CPointEx3D& minimumDirection)
{
	CPointEx3D Xu, Xv;
	auto transformationMatrix = CreatePatchLocalCoordinateSystem(p, n, Xu, Xv);
	CPointCloud pcTransformed(pc.GetSize());
	for (int i = 0; i < pc.GetSize(); i++)
	{
		pcTransformed[i].first  = VectorToPoint(Mult(transformationMatrix, PointToVector(pc[i].first)));
		pcTransformed[i].second = VectorToPoint(Mult(transformationMatrix, PointToVector(pc[i].second, false)));
	}
	CDoubleMatrix U(3 * pc.GetSize(), 7);
	CDoubleMatrix d(3 * pc.GetSize(), 1);
	int count = 0;
	for (int i = 0; i < 3 * pc.GetSize(); i += 3)
	{
		double xi = pcTransformed[count].first[0], yi = pcTransformed[count].first[1], zi = pcTransformed[count].first[2];
		double ai = pcTransformed[count].second[0], bi = pcTransformed[count].second[1], ci = pcTransformed[count++].second[2];
		if (fabs(ci) < 1e-8) ci = 1e-8;
		double ki = 2. / (xi * xi + yi * yi);
		U(i, 0) = ki * 0.5 * xi * xi;
		U(i, 1) = ki * xi * yi;
		U(i, 2) = ki * 0.5 * yi * yi;
		U(i, 3) = ki * xi * xi * xi;
		U(i, 4) = ki * xi * xi * yi;
		U(i, 5) = ki * xi * yi * yi;
		U(i, 6) = ki * yi * yi * yi;
		U(i + 1, 0) = ki * xi;
		U(i + 1, 1) = ki * yi;
		U(i + 1, 2) = 0.0;
		U(i + 1, 3) = 3.0 * ki * xi * xi;
		U(i + 1, 4) = 2.0 * ki * xi * yi;
		U(i + 1, 5) = ki * yi * yi;
		U(i + 1, 6) = 0.0;
		U(i + 2, 0) = 0.0;
		U(i + 2, 1) = ki * xi;
		U(i + 2, 2) = ki * yi;
		U(i + 2, 3) = 0.0;
		U(i + 2, 4) = ki * xi * xi;
		U(i + 2, 5) = 2.0 * ki * xi * yi;
		U(i + 2, 6) = 3.0 * ki * yi * yi;
		d(i, 0) = ki * zi;
		d(i + 1, 0) = -ki * ai / ci;
		d(i + 2, 0) = -ki * bi / ci;
	}
	LIN_SYS_MKL::SolveLeastSquares(U, d);
	double W[4] = {d[0], d[1], d[1], d[2]};
	MKL_INT lda = 2, info;
	/* Local arrays */
	double l[2];
	/* Solve eigenproblem */
	info = LAPACKE_dsyev(LAPACK_ROW_MAJOR, 'V', 'U', 2, W, lda, l);
	/* Check for convergence */
	if (info > 0) {
		printf("The algorithm failed to compute eigenvalues.\n");
		exit(1);
	}
	minimumDirection = W[0] * Xu + W[2] * Xv;
	maximumDirection = W[1] * Xu + W[3] * Xv;
	CPointEx3D nframe = cross(maximumDirection, minimumDirection);
	if (dot(nframe, n) < 0)
	{
		std::swap(l[0], l[1]);
		l[0] = -l[0];
		l[1] = -l[1];
		CPointEx3D temp = maximumDirection;
		maximumDirection = minimumDirection;
		minimumDirection = maximumDirection;
	}
	return { l[1], l[0] };
}

std::tuple<double, double, double> CPointCloud::PerformPCAOnPatch(const CPointEx3D& p, const CVector<CPointEx3D>& patch, CPointEx3D& normal)
{
	std::vector<double> d(patch.GetSize());
	std::vector<double> w(patch.GetSize());
	for (int i = 0; i < patch.GetSize(); i++)
	{
		d[i] = (p - patch[i]).norm();
	}
	double invsumw = 1.0/ std::accumulate(d.begin(), d.end(), 0.0);
	for (int i = 0; i < patch.GetSize(); i++)
	{
		if (d[i] == 0.0) w[i] = 1.0;
		else w[i] = invsumw / d[i];
	}
	CPointEx3D pw(0.0, 0.0, 0.0);
	for (int i = 0; i < patch.GetSize(); i++)
	{
		pw += w[i] * patch[i];
	}
	pw /= std::accumulate(w.begin(), w.end(), 0.0);
	double alpha = 1.0, beta = 0.0;
	int m = 3;
	int k = patch.GetSize();
	int n = 3;
	double* A = (double*)mkl_malloc(m * k * sizeof(double), 64);
	double* C = (double*)mkl_malloc(m * n * sizeof(double), 64);
	if (A == NULL || C == NULL) {
		printf("\n ERROR: Can't allocate memory for matrices. Aborting... \n\n");
		mkl_free(A);
		mkl_free(C);
		exit(0);
	}
	double k_inv = 1.0 / sqrt(patch.GetSize());
#pragma omp parallel for
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < k; j++)
		{
			A[j + i * k] = k_inv * std::pow(w[j], 0.25) * (patch[j][i] - pw[i]);
		}
	}
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		m, n, k, alpha, A, k, A, k, beta, C, n);
	MKL_INT lda = n, info;
	/* Local arrays */
	double l[3];
	/* Solve eigenproblem */
	info = LAPACKE_dsyev(LAPACK_ROW_MAJOR, 'V', 'U', n, C, lda, l);
	normal = CPointEx3D(C[0], C[3], C[6]);
	mkl_free(A);
	mkl_free(C);
	return std::make_tuple(l[0], l[1], l[2]);
}
