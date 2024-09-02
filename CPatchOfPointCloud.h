#ifndef CPATCH_OF_POINT_CLOUD
#define CPATCH_OF_POINT_CLOUD

#include "CPointCloud.h"
#include <set>

class CPatchOfPointCloud : public CPointCloud
{
public:
	CPatchOfPointCloud(spPointCloud pointCloud, std::shared_ptr<std::set<int>> indexSet);
	CPatchOfPointCloud(const CPatchOfPointCloud& patch);
	void MergeWith(std::shared_ptr<CPatchOfPointCloud>, const CPointCloud& pointCloud);
	double checkCompatibility(std::shared_ptr<CPatchOfPointCloud> patch, int k, double tolerance);
	bool IndexInPointCloud(int index) const;
	bool CheckIfNeighbor(std::shared_ptr<CPatchOfPointCloud> patchToCompare);
	CPatchOfPointCloud& operator=(const CPatchOfPointCloud& patch);
private:
	std::shared_ptr<std::set<int>> indexSet;
	std::shared_ptr<CMatrix<double>> slippableMotions;
};


#endif

