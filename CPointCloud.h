#ifndef POINT_CLOUD
#define POINT_CLOUD
#include <memory>
#include "MCLSEXST.H"
#include <vector>
#include <map>
class CPointCloud;
using spPointCloud = std::shared_ptr<CPointCloud>;
class CPointCloud : public CVector<CPointEx3D>
{
public:
	enum Axis
	{
		X_AXIS,
		Y_AXIS,
		Z_AXIS
	};
	struct UmbrellaElement
	{
		int index;
		int indexPrevious;
		int indexNext;
	};
	struct sortElement
	{
		double value;
		int index;
	};
	void createUmbrellaFromTriangulation(std::vector<std::map<int, UmbrellaElement>>& umbrellas) const;
	CPointCloud(int MaxSize = 0, int GrowRate = 6);
	CPointCloud(const CPointCloud& pcSource);
	CPointCloud(const char* file, bool onlyPnts = false);
	CPointCloud& operator=(const CPointCloud& d);
	std::pair<CPointEx3D&, CPointEx3D&> operator[](int i);
	std::pair<CPointEx3D, CPointEx3D> operator[](int i) const;
	void Add(const CPointEx3D& point);
	void Add(const CPointEx3D& point, const CPointEx3D& normal);	
	void savePointCloud(const char* fileName, bool withNormals = true) const;
	spPointCloud alignPointCloud(CMatrix<double>& R, char zIndex = 2) const;
	CPointEx3D FindBaryCenter() const;
	static CPointEx3D FindBaryCenter(const CVector<CPointEx3D>& pnts);
	void FindPrincipalAxes(CMatrix<double>& R, char zIndex) const;
	spPointCloud rotatePointCloud(const CMatrix<double>& mat) const;
	spPointCloud addNoiseToPointCloud(double standard_deviation) const;
	static spPointCloud CreatePointCloudFromFileWithNormals(const char* fileName, bool onlyPnts = false);
	static CVector<double> PointToVector(const CPointEx3D& pnt, bool homogenous = true);
	static CPointEx3D VectorToPoint(const CVector<double>& pnt);
	std::shared_ptr<CMatrix<double>> getSlippableVectors(const CPointEx3D& bc) const;
	static void scaleToUnity(CVector<CPointEx3D>& points);
	static CPointEx3D GetNormalizedPerpendicularVectorToVector(const CPointEx3D& inVector);
	static std::tuple<double, double, double> PerformPCAOnPatch(const CPointEx3D& p, const CVector<CPointEx3D>& patch, CPointEx3D& normal);
	static CDoubleMatrix CreateLocalCoordinateSystem(const CPointEx3D& o, const CPointEx3D& d, Axis axis);
	std::vector<UmbrellaElement> CreateUmbrellaAroundPoint(int originIndex, const std::vector<int>& oneRing) const;
	std::tuple<double, double, CPointEx3D, CPointEx3D> GabrielTaubinCurvatureTensor(int i, const std::vector<UmbrellaElement>& umbrellas, 
		CPointEx3D& normal) const;
	CPointEx3D* begin();
	CPointEx3D* end();
	const CPointEx3D* begin() const;
	const CPointEx3D* end() const;
	void scale(double lambda);
	static bool sortFunction(sortElement element1, sortElement element2);
	bool hasNormals()
	{
		return fabs((*this)[0].second.norm() - 1.0) < 1e-6;
	}
	static double AreaOfTriangle(const CPointEx3D& pnt1, const CPointEx3D& pnt2, const CPointEx3D& pnt3);
	static CPointEx3D NormalOfTriangle(const CPointEx3D& pnt1, const CPointEx3D& pnt2, const CPointEx3D& pnt3);
	
protected:
	
	CVector<CPointEx3D> normals;
	static CDoubleMatrix CreatePatchLocalCoordinateSystem(const CPointEx3D& p, const CPointEx3D& n,
		CPointEx3D& firstPerpendicular, CPointEx3D& secondPerpendicular);
};
#endif