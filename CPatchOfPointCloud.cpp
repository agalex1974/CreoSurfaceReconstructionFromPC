#include "CPatchOfPointCloud.h"

CPatchOfPointCloud::CPatchOfPointCloud(spPointCloud pointCloud, std::shared_ptr<std::set<int>> indexSet):
	CPointCloud(*pointCloud),
	indexSet(indexSet)
{
	slippableMotions = getSlippableVectors(FindBaryCenter());
}

CPatchOfPointCloud::CPatchOfPointCloud(const CPatchOfPointCloud& patch):
	CPointCloud(patch)
{
	indexSet = std::make_shared<std::set<int>>(*(patch.indexSet));
	slippableMotions = getSlippableVectors(FindBaryCenter());
}

CPatchOfPointCloud& CPatchOfPointCloud::operator=(const CPatchOfPointCloud& patch)
{
	if (this != &patch) {
		CPointCloud::operator=(patch);
		indexSet = std::make_shared<std::set<int>>(*(patch.indexSet));
		slippableMotions = getSlippableVectors(FindBaryCenter());
	}
	return *this;
}

bool CPatchOfPointCloud::CheckIfNeighbor(std::shared_ptr<CPatchOfPointCloud> patchToCompare)
{
	auto& indexSetToMerge = patchToCompare->indexSet;
	for (auto it = indexSetToMerge->begin(); it != indexSetToMerge->end(); it++)
		if (IndexInPointCloud(*it))
		{
			return true;
		}
	return false;
}

void CPatchOfPointCloud::MergeWith(std::shared_ptr<CPatchOfPointCloud> patchToBeMerged, const CPointCloud& pointCloud)
{
	auto indexSetToMerge = patchToBeMerged->indexSet;
	for (auto it = indexSetToMerge->begin(); it != indexSetToMerge->end(); it++)
		if (indexSet->insert(*it).second)
		{
			Add(pointCloud[*it].first, pointCloud[*it].second);
		}
	slippableMotions = getSlippableVectors(FindBaryCenter());
}

bool CPatchOfPointCloud::IndexInPointCloud(int index) const
{
	return indexSet->find(index) != indexSet->end();
}
