#include "CCreoPointCloud.h"
#include <ProImportfeat.h>
#include <ProIntfData.h>
#include <ProArray.h>
#include <ProSelbuffer.h>
#include <ProUtil.h>
#include <vector>
#include <set>
#include <queue>
#include <omp.h>
#include <ProSolid.h>
#include <stack>
#include <chrono>

//a *.xyz (coordinate,normal) file
CCreoPointCloud::CCreoPointCloud(ProPath filePath, const char* pntCloudFile, bool onlyPnts):
	CPointCloud(pntCloudFile, onlyPnts)
{
	void* vp = (void*)filePath;
	ProMdl model;
	ProError err = ProMdlCurrentGet(&model);
	ProIntfDataSource id_source;
	err = ProIntfDataSourceInit(PRO_INTF_PTS, vp, &id_source);
	err = ProImportfeatCreate((ProSolid)model, &id_source, NULL, NULL, &featurePntCloud);
	point_cloud_id = featurePntCloud.id;
	ProSelection selection;
	ProModelitem modelitem;
	ProSelectionAlloc(NULL, &featurePntCloud, &selection);
	err = ProSelectionModelitemGet(selection, &modelitem);
	point_cloud_internal_id = modelitem.id;
	ProSelectionFree(&selection);
	ProIntfDataSourceClean(&id_source);
	std::vector<Point_3> points;
	std::vector<int> indices;
	for (int i = 0; i < GetSize(); i++)
	{
		points.push_back(Point_3((*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]));
		indices.push_back(i);
	}
	pointCloudSpatialTree = std::make_shared<Tree>(boost::make_zip_iterator(boost::make_tuple(points.begin(), indices.begin())),
		boost::make_zip_iterator(boost::make_tuple(points.end(), indices.end())));

	Distance tr_dist;
	meanDistance = 0.0;
	for (int i = 0; i < GetSize(); i++) {
		K_neighbor_search search(*pointCloudSpatialTree, points[i], 5);
		for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++) {
			meanDistance += tr_dist.inverse_of_transformed_distance(it->second);
		}
	}
	meanDistance /= 5 * GetSize();
}

std::vector<int> CCreoPointCloud::GabrielkNeighborhood(int i, int k, const std::vector<std::vector<int>>& EGG)
{
	std::vector<int> output;
	std::set<int> ringsSet;
	std::queue <int> neighborQueue;
	std::map<int, int> neighborMap;

	output.push_back(i);
	ringsSet.insert(i);
	neighborQueue.push(i);
	neighborMap[i] = 0;

	while(!neighborQueue.empty())
	{
		int index = neighborQueue.front();
		neighborQueue.pop();
		if (neighborMap[index] < k) {
			for (const auto& el : EGG[index])
			{
				if (ringsSet.insert(el).second)
				{
					output.push_back(el);
					neighborQueue.push(el);
					neighborMap[el] = neighborMap[index] + 1;
				}
			}
		}
	}	
	return output;
}

spPointCloud CCreoPointCloud::GetPointsWithinRadiusFromPoint(const CPointEx3D& pnt, double timesOfDensity, std::set<int>& indexSet, int excludeIndex) const
{
	spPointCloud outputPointCloud = std::make_shared<CPointCloud>();
	const Point_3 center(pnt.x, pnt.y, pnt.z);
	const Fuzzy_sphere fs(center, timesOfDensity * meanDistance, 0.0);
	std::vector<Point_and_int> result;
	pointCloudSpatialTree->search(std::back_insert_iterator<std::vector<Point_and_int>>(result), fs);
	for (auto pnt : result)
	{
		const int pntIndex = pnt.get<1>();
		if (pntIndex != excludeIndex) 
		{
			indexSet.insert(pntIndex);
			outputPointCloud->Add((*this)[pntIndex].first, (*this)[pntIndex].second);
		}
	}
	return outputPointCloud;
}

//i is the index order in NNs not the index of the queryPoint
//we need to check all points 1..i-1
bool CCreoPointCloud::isEllipicGabrielNeighbor(int i, const std::vector<int>& NNs, double a) const
{
	const CPointEx3D& p  = (*this)[NNs[0]].first;
	const CPointEx3D& qi = (*this)[NNs[i]].first;
	const CPointEx3D origin = 0.5 * (p + qi);
	const double d = (qi - p).norm() / 2.0;
	CPointEx3D localXaxis = qi - p;
	localXaxis.normalize();
	CDoubleMatrix transformationMatrix = CreateLocalCoordinateSystem(origin, localXaxis, X_AXIS);
	for (int j = 1; j < i; j++)
	{
		CPointEx3D pnt = VectorToPoint(Mult(transformationMatrix, PointToVector((*this)[NNs[j]].first)));
		double x = pnt.x, y = pnt.y, z = pnt.z;
		double ellipsoidValue = x * x + y * y / (a * a) + z * z / (a * a);
		if (ellipsoidValue < d * d) return false;
	}
	return true;
}

std::vector<double> CCreoPointCloud::get_cotan_weights(int i, const std::vector<UmbrellaElement>& ring) const
{
	std::vector<double> weights(ring.size());

	const CPointEx3D c_pos = (*this)[i].first;
	double sum = 0.;
	int nb_edges = ring.size();
	weights.resize(nb_edges);
	for (int e = 0; e < nb_edges; ++e)
	{
		double cotan1 = 0.0, cotan2 = 0.0;
		if (ring[e].indexPrevious >= 0) {
			CPointEx3D v1 = c_pos - (*this)[ring[e].indexPrevious].first;
			CPointEx3D v2 = (*this)[ring[e].index].first - (*this)[ring[e].indexPrevious].first;
			cotan1 = (dot(v1, v2)) / (1e-6 + (cross(v1, v2)).norm());
		}
		if (ring[e].indexNext >= 0) {
			CPointEx3D v3 = c_pos - (*this)[ring[e].indexNext].first;
			CPointEx3D v4 = (*this)[ring[e].index].first - (*this)[ring[e].indexNext].first;
			cotan2 = (dot(v3, v4)) / (1e-6 + (cross(v3, v4)).norm());
		}
		double w = (cotan1 + cotan2) * 0.5;
		weights[e] = w;
	}
	return weights;
}

void CCreoPointCloud::save_smoothed_tringulation()
{
	FILE* file = fopen("vase_sor_noise.off", "r");
	char off[10];
	int numbPnts, numbTris, numbEdges;
	fscanf(file, "%s", off);
	fscanf(file, "%d %d %d", &numbPnts, &numbTris, &numbEdges);
	FILE* fileOut = fopen("vase_sor_smoothed.off", "w");
	fprintf(fileOut, "OFF\n%d %d 0\n", numbPnts, numbTris);
	for (int i = 0; i < numbPnts; i++)
	{
		float x, y, z;
		fscanf(file, "%f %f %f", &x, &y, &z);
		fprintf(fileOut, "%lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]);
	}
	
	for (int t = 0; t < numbTris; t++)
	{
		int numb, i, j, k;
		fscanf(file, "%d %d %d %d", &numb, &i, &j, &k);
		fprintf(fileOut, "3 %d %d %d\n", i, j, k);
	}
	fclose(file);
	fclose(fileOut);
}

void CCreoPointCloud::smooth_normal_field(int nb_iter)
{
	int neighborCount = 30;
	unsigned nb_vertices = unsigned(GetSize());

	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);
	std::vector<std::vector<double>> thetaValues(nb_vertices);
	std::vector<std::vector<int>> neighbors(nb_vertices);

	double sigmaP = 1.0;
	double sigmaN = 1.0;
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].second;
		destination[i] = (*this)[i].second;
	}
	
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		auto indexes = GetKNearestNeighborIndex((*this)[i].first, neighborCount);
		double max_distance = -1.0;
		for (int n = 0; n < indexes.size(); n++)
		{
			int idx = indexes[n];
			neighbors[i].push_back(idx);
			double distance = ((*this)[i].first - (*this)[idx].first).norm();
			if (max_distance < distance) max_distance = distance;
		}
		sigmaP = max_distance;
		for (int n = 0; n < indexes.size(); n++)
		{
			int idx = indexes[n];
			double r2 = ((*this)[i].first - (*this)[idx].first).sqr_norm();
			double theta = exp(-r2 / (sigmaP * sigmaP));
			thetaValues[i].push_back(theta);
		}
	}
	const double pi = 3.14159265359;
	double theta = 15 * pi / 180.0;
	for (int k = 0; k < nb_iter; k++)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			CPointEx3D sumVector(0., 0., 0.);
			double sum = 0.0;
			for (int n = 0; n < neighborCount; n++)
			{
				int neigh = neighbors[i][n];
				double value = 1.0 - dot(source[i], source[neigh]);
				value /= 1 - cos(theta);
				double psi = exp(-value * value);
				sumVector += psi * thetaValues[i][n] * source[neigh];
				sum += psi * thetaValues[i][n];
			}
			destination[i] = 1.0 / sum * sumVector;
		}
	}

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		(*this)[i].second = destination[i];
	}

}

void CCreoPointCloud::bilateral_iterative(int nb_iter, double sigmaC, double sigmaS, double gabrielRatio)
{
	smooth_normal_field(15);
	unsigned nb_vertices = unsigned(GetSize());
	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].first;
		destination[i] = (*this)[i].first;
	}

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	updateSpatialTree();
	std::vector<std::vector<int>> umbrellas(GetSize());

	//calculateEGG(umbrellas, gabrielRatio);
	for (int i = 0; i < GetSize(); i++)
	{
		auto indexes = GetKNearestNeighborIndex((*this)[i].first, 30);
		for (auto idx : indexes)
		{
			if (idx != i) umbrellas[i].push_back(idx);
		}
	}
	
	CPointEx3D* src_vertices = source.data();
	CPointEx3D* dst_vertices = destination.data();
	for (int k = 0; k < nb_iter; k++)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			double sum = 0.;
			double normalizer = 0.;
			size_t nb_neighs = umbrellas[i].size();
			std::vector<double> offsets(nb_neighs);
			double max_distance = -1.0;
			for (size_t n = 0; n < nb_neighs; n++)
			{
				int neigh = umbrellas[i][n];
				offsets[n] = abs(dot((*this)[i].second, src_vertices[i] - src_vertices[neigh]));
				double t = (src_vertices[i] - src_vertices[neigh]).norm();
				if (t > max_distance)
				{
					max_distance = t;
				}
			}
			sigmaC = max_distance / 2;
			double mean_offset = std::accumulate(offsets.begin(), offsets.end(), 0.0) / nb_neighs;
			sigmaS = 0.0;
			for (size_t n = 0; n < nb_neighs; n++)
			{
				sigmaS += (offsets[n] - mean_offset) * (offsets[n] - mean_offset);
			}
			sigmaS = sqrt(sigmaS / nb_neighs) + 1e-6;
			for (size_t n = 0; n < nb_neighs; n++)
			{
				int neigh = umbrellas[i][n];
				double t = (src_vertices[i] - src_vertices[neigh]).norm();
				double h = dot((*this)[i].second, src_vertices[neigh] - src_vertices[i]);
				double wc = exp(-t * t / (2.0 * sigmaC * sigmaC));
				double ws = exp(-h * h / (2.0 * sigmaS * sigmaS));
				sum += wc * ws * h;
				normalizer += wc * ws;
			}
			dst_vertices[i] = src_vertices[i] + (sum / (normalizer + 1e-6)) * (*this)[i].second;
		}
		std::swap(dst_vertices, src_vertices);
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	FILE* file = fopen("LocalBilateralSmoothingTime.txt", "w");
	fprintf(file, "Time taken:%lf", time);
	fclose(file);

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		(*this)[i].first = destination[i];
	}

	updateSpatialTree();
	ComputeNormalsWithPCA(false);
}

/*void CCreoPointCloud::smooth_iterative(int nb_iter, double lambda, double mu, double gabrielRatio, bool isEdgeAware, bool doMollification)
{
	const bool isPhotogrammetryData = false;
	//if (isEdgeAware) smooth_normal_field(10);
	unsigned nb_vertices = unsigned(GetSize());
	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);
	std::vector<CPointEx3D> previous_destination(nb_vertices);
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].first;
		destination[i] = (*this)[i].first;
		previous_destination[i] = (*this)[i].first;
	}

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	updateSpatialTree();
	CPointEx3D* src_vertices = source.data();
	CPointEx3D* dst_vertices = destination.data();
	std::vector<std::vector<int>> umbrellas(GetSize());
	calculateEGG(umbrellas, gabrielRatio);
	double scale[2] = { lambda, mu };
	double max_deviation = 0.0;
	int k;
	double previous_mean;
	for (k = 0; k < 2 * nb_iter; k++)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			CPointEx3D cog(0., 0., 0.);
			float sum = 0.;
			size_t nb_neighs = umbrellas[i].size();
			for (size_t n = 0; n < nb_neighs; n++)
			{
				int neigh = umbrellas[i][n];
				double w;
				double norm2 = (src_vertices[neigh] - src_vertices[i]).sqr_norm();
				if (isEdgeAware)
				{
					double dotproduct = dot((*this)[i].second, (*this)[neigh].second);
					double power = dotproduct * dotproduct * norm2;
					w = exp(-power);
				}
				else
				{
					if (isPhotogrammetryData)
					{
						w = k % 2 ? 1.0 / norm2 : 1.0;
					}
					else {
						double weightExp = exp(-norm2);
						w = weightExp;
					}
				}
				cog += w * (src_vertices[neigh] - src_vertices[i]);
				sum += w;
				dst_vertices[i] = src_vertices[i] + cog / sum * scale[k % 2];
			}
		}
		std::swap(dst_vertices, src_vertices);		
	}
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	FILE* file = fopen("TaubinSmoothingTime.txt", "w");
	fprintf(file, "Time taken:%lf\n", time);
	fprintf(file, "maximum deviation:%lf\n", max_deviation);
	fprintf(file, "iterations taken:%d\n", k);
	fclose(file);
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		src_vertices[i] = (*this)[i].first;
		(*this)[i].first = destination[i];
	}

	updateSpatialTree();
	ComputeNormalsWithPCA(false);

	if (doMollification)
	{
#pragma omp parallel for
		for (int i = 0; i < GetSize(); i++)
		{
			(*this)[i].first = src_vertices[i];
		}
	}
}*/

void CCreoPointCloud::smooth_iterative(int nb_iter, double lambda, double mu, double gabrielRatio, bool isEdgeAware, bool doMollification)
{
	const bool isPhotogrammetryData = false;
	//if (isEdgeAware) smooth_normal_field(10);
	unsigned nb_vertices = unsigned(GetSize());
	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);
	
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].first;
		destination[i] = (*this)[i].first;
	}

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	updateSpatialTree();
	CPointEx3D* src_vertices = source.data();
	CPointEx3D* dst_vertices = destination.data();
	std::vector<std::vector<int>> umbrellas(GetSize());
	calculateEGG(umbrellas, gabrielRatio);
	double scale[2] = { lambda, mu };
	for (int k = 0; k < 2 * nb_iter; k++)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			CPointEx3D cog(0., 0., 0.);
			float sum = 0.;
			size_t nb_neighs = umbrellas[i].size();
			for (size_t n = 0; n < nb_neighs; n++)
			{
				int neigh = umbrellas[i][n];
				double norm2 = (src_vertices[neigh] - src_vertices[i]).sqr_norm();
				double weightExp = exp(-norm2);
				double w = weightExp;
				cog += w * (src_vertices[neigh] - src_vertices[i]);
				sum += w;
			}
			dst_vertices[i] = src_vertices[i] + cog / sum * scale[k % 2];
		}
		std::swap(dst_vertices, src_vertices);
	}
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	FILE* file = fopen("TaubinSmoothingTime.txt", "w");
	fprintf(file, "Time taken:%lf", time);
	fclose(file);
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		src_vertices[i] = (*this)[i].first;
		(*this)[i].first = destination[i];
	}
	
	updateSpatialTree();
	ComputeNormalsWithPCA(false);

	if (doMollification)
	{
#pragma omp parallel for
		for (int i = 0; i < GetSize(); i++)
		{
			(*this)[i].first = src_vertices[i];
		}
	}
}

/*void CCreoPointCloud::smooth_iterative(int nb_iter, double lambda, double mu, double gabrielRatio, bool isEdgeAware, bool doMollification)
{
	if (isEdgeAware) smooth_normal_field(10);
	unsigned nb_vertices = unsigned(GetSize());
	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].first;
		destination[i] = (*this)[i].first;
	}

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	//updateSpatialTree();
	CPointEx3D* src_vertices = source.data();
	CPointEx3D* dst_vertices = destination.data();
	std::vector<std::vector<int>> umbrellas(GetSize());
	calculateEGG(umbrellas, gabrielRatio);


	double scale[2] = { lambda, mu };

	for (int k = 0; k < 2 * nb_iter; k++)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			CPointEx3D cog(0., 0., 0.);
			float sum = 0.;
			size_t nb_neighs = umbrellas[i].size();
			for (size_t n = 0; n < nb_neighs; n++)
			{
				int neigh = umbrellas[i][n];
				double w;
				if (isEdgeAware)
				{
					double power = dot((*this)[i].second, src_vertices[i] - src_vertices[neigh]) * dot((*this)[i].second, src_vertices[i] - src_vertices[neigh]);
					w = exp(-power);
				}
				//else {
				//	double norm2 = (src_vertices[neigh] - src_vertices[i]).norm() + 1e-7;
				//	w = k % 2 ? 1.0 / norm2 : 1.0 / norm2;
				//}
				else
				{
					double norm = (src_vertices[neigh] - src_vertices[i]).norm();
					double weightExp = exp(-norm * norm);
					w = weightExp;
				}
				cog += w * (src_vertices[neigh] - src_vertices[i]);
				sum += w;
				dst_vertices[i] = src_vertices[i] + cog / sum * scale[k % 2];
			}
		}
		std::swap(dst_vertices, src_vertices);
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	FILE* file = fopen("TaubinSmoothingTime.txt", "w");
	fprintf(file, "Time taken:%lf", time);
	fclose(file);
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		src_vertices[i] = (*this)[i].first;
		(*this)[i].first = destination[i];
	}

	updateSpatialTree();
	ComputeNormalsWithPCA(false);

	if (doMollification)
	{
#pragma omp parallel for
		for (int i = 0; i < GetSize(); i++)
		{
			(*this)[i].first = src_vertices[i];
		}
	}
}*/

/*void CCreoPointCloud::smooth_iterative(int nb_iter, double lambda, double mu, double gabrielRatio, bool isEdgeAware, bool doMollification)
{
	if (isEdgeAware) smooth_normal_field(10);
	unsigned nb_vertices = unsigned(GetSize());
	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].first;
		destination[i] = (*this)[i].first;
	}

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	//updateSpatialTree();

	if (cachePoints)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			(*this)[i].first = (*cachePoints)[i];
		}
	}
	cachePoints = nullptr;
	CPointEx3D* src_vertices = source.data();
	CPointEx3D* dst_vertices = destination.data();
	if (doMollification)
	{
		std::vector<std::vector<int>> umbrellas(GetSize());
		calculateEGG(umbrellas, gabrielRatio);
		for (int k = 0; k < 17; k++) {
#pragma omp parallel for
			for (int i = 0; i < nb_vertices; i++)
			{
				auto neighbors = GetKNearestNeighborIndex((*this)[i].first, 24);
				double max_distance = -1.0;
				for (auto neigh : neighbors)
				{
					double t = (src_vertices[i] - src_vertices[neigh]).norm();
					if (t > max_distance)
					{
						max_distance = 1.0;
					}
				}
				double sigmaC = max_distance;
				CPointEx3D vec_sum(0., 0., 0.);
				double sum = 0.0;
				for (auto neigh : neighbors)
				{
					double norm2 = (src_vertices[i] - src_vertices[neigh]).sqr_norm();
					double kernel = exp(-norm2 / (sigmaC * sigmaC));

					vec_sum += kernel * src_vertices[neigh];
					sum += kernel;
				}
				vec_sum += src_vertices[i];
				sum += 1;
				dst_vertices[i] = vec_sum / sum;
			}
			std::swap(dst_vertices, src_vertices);
		}
	}

	else {
		std::vector<std::vector<int>> umbrellas(GetSize());
		calculateEGG(umbrellas, gabrielRatio);


		double scale[2] = { lambda, mu };

		for (int k = 0; k < 2 * nb_iter; k++)
		{
#pragma omp parallel for
			for (int i = 0; i < nb_vertices; i++)
			{
				CPointEx3D cog(0., 0., 0.);
				float sum = 0.;
				size_t nb_neighs = umbrellas[i].size();
				double max_distance = -1.0;
				for (size_t n = 0; n < nb_neighs; n++)
				{
					int neigh = umbrellas[i][n];

					double t = (src_vertices[i] - src_vertices[neigh]).norm();
					if (t > max_distance)
					{
						max_distance = t;
					}
				}
				double sigmaC = max_distance;
				for (size_t n = 0; n < nb_neighs; n++)
				{
					int neigh = umbrellas[i][n];
					double w;
					if (isEdgeAware)
					{
						double power = dot((*this)[i].second, src_vertices[i] - src_vertices[neigh]) * dot((*this)[i].second, src_vertices[i] - src_vertices[neigh]);
						w = exp(-power);
					}
					//else {
					//	double norm2 = (src_vertices[neigh] - src_vertices[i]).norm() + 1e-7;
					//	w = k % 2 ? 1.0 / norm2 : 1.0 / norm2;
					//}
					else
					{
						double norm = (src_vertices[neigh] - src_vertices[i]).norm();
						double weightExp = exp(-norm * norm / (sigmaC * sigmaC));
						w = weightExp;
					}
					cog += w * (src_vertices[neigh] - src_vertices[i]);
					sum += w;
					dst_vertices[i] = src_vertices[i] + cog / sum * scale[k % 2];
				}
			}
			std::swap(dst_vertices, src_vertices);
		}

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		FILE* file = fopen("TaubinSmoothingTime.txt", "w");
		fprintf(file, "Time taken:%lf", time);
		fclose(file);
	}
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		src_vertices[i] = (*this)[i].first;
		(*this)[i].first = destination[i];
	}

	updateSpatialTree();
	ComputeNormalsWithPCA(false);

	if (doMollification)
	{
		cachePoints = std::make_shared<std::vector<CPointEx3D>>(nb_vertices);
#pragma omp parallel for
		for (int i = 0; i < GetSize(); i++)
		{
			(*this)[i].first = src_vertices[i];
			(*cachePoints)[i] = destination[i];
		}
	}
}*/

/*void CCreoPointCloud::smooth_iterative(int nb_iter, double lambda, double mu, double gabrielRatio, bool isEdgeAware, bool doMollification)
{
	unsigned nb_vertices = unsigned(GetSize());
	std::vector<CPointEx3D> source(nb_vertices);
	std::vector<CPointEx3D> destination(nb_vertices);

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		source[i] = (*this)[i].first;
		destination[i] = (*this)[i].first;
	}

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	updateSpatialTree();
	std::vector<std::vector<int>> umbrellas(GetSize());
	calculateEGG(umbrellas, 0.65);

	CPointEx3D* src_vertices = source.data();
	CPointEx3D* dst_vertices = destination.data();
	double scale[2] = { 0.64, -0.65 };

	for (int k = 0; k < 2 * nb_iter; k++)
	{
#pragma omp parallel for
		for (int i = 0; i < nb_vertices; i++)
		{
			CPointEx3D cog(0., 0., 0.);
			float sum = 0.;
			size_t nb_neighs = umbrellas[i].size();
			double max_distance = -1.0;
			

			for (size_t n = 0; n < nb_neighs; n++)
			{
				int neigh = umbrellas[i][n];
				
				double norm = (src_vertices[neigh] - src_vertices[i]).norm() + 1e-7;
				//double w = k % 2 ? 1.0 / (norm * norm): 1.0 / norm;
				//double power = dot((*this)[i].second, src_vertices[i] - src_vertices[neigh]) * (src_vertices[i] - src_vertices[neigh]);
				double weightExp = exp(-norm * norm);
				double w = weightExp;
				cog += w * (src_vertices[neigh] - src_vertices[i]);
				sum += w;
			}
			dst_vertices[i] = src_vertices[i] + cog / sum * scale[k % 2];
		}
		std::swap(dst_vertices, src_vertices);
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	FILE* file = fopen("TaubinSmoothingTime.txt", "w");
	fprintf(file, "Time taken:%lf", time);
	fclose(file);

#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		(*this)[i].first = destination[i];
	}

	updateSpatialTree();
	ComputeNormalsWithPCA(false);
}*/

//We assume that the EGG vector is initialized
void CCreoPointCloud::calculateEGG(std::vector<std::vector<int>>& EGG, double ratio) const
{
	int pointsCount = EGG.size();
#pragma omp parallel for
	for (int i = 0; i < pointsCount; i++)
	{
		auto NNs = GetKNearestNeighborIndex((*this)[i].first, 40);
		for (int j = 1; j < NNs.size(); j++)
		{
			if (isEllipicGabrielNeighbor(j, NNs, ratio))
				EGG[i].push_back(NNs[j]);
		}
	}
}

std::vector<int> CCreoPointCloud::GetKNearestNeighborIndex(const CPointEx3D& pnt, int K) const
{
	std::vector<int> indexes;
	const Point_3 point(pnt.x, pnt.y, pnt.z);
	K_neighbor_search search(*pointCloudSpatialTree, point, K);
	for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++) 
	{
		indexes.push_back(boost::get<1>(it->first));
	}
	return indexes;
}

CCreoPointCloud::~CCreoPointCloud()
{
	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	ProError status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &point_cloud_internal_id, 1, opt, 1);
	status = ProTreetoolRefresh(part);
	status = ProSolidDisplay((ProSolid)part);
}

spPointCloud CCreoPointCloud::pointCloudFromSelection(std::set<int>& indexes) const
{
	spPointCloud outputPointCloud = std::make_shared<CPointCloud>();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray*)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);

	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			const Point_3 point(xyz[0], xyz[1], xyz[2]);
			K_neighbor_search search(*pointCloudSpatialTree, point, 1);
			K_neighbor_search::iterator it = search.begin();
			const int pointIndex = boost::get<1>(it->first);
			outputPointCloud->Add((*this)[pointIndex].first, (*this)[pointIndex].second);
			indexes.insert(pointIndex);
		}
	}
	ProArrayFree((ProArray*)&p_sel);
	return outputPointCloud;
}

CCreoPointCloud::patchMap CCreoPointCloud::createPatchMap() const
{
	patchMap pMap = std::make_shared<map<int, std::shared_ptr<CPatchOfPointCloud>>>();
	auto start_time = std::chrono::high_resolution_clock::now();
	std::set<int> index_total_set;
	int counter = 0;
	for (int i = 0; i < GetSize(); i++)
	{
		auto time = std::chrono::high_resolution_clock::now() - start_time;
		if (time / std::chrono::milliseconds(1) > 3000)
		{
			ProError err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_PROGRESS_NAME));
			err = ProUIDialogCreate(const_cast<char*>(UI_GUI_PROGRESS_NAME), const_cast<char*>(UI_GUI_PROGRESS_NAME));
			int ui_status;
			err = ProUIDialogActivate(const_cast<char*>(UI_GUI_PROGRESS_NAME), &ui_status);
			start_time = std::chrono::high_resolution_clock::now();
		}
		int percent = (int)((1.0 * i) / GetSize() * 50);
		setProgress(percent);
		if (index_total_set.find(i) == index_total_set.end()) {
			auto indexSet = std::make_shared<set<int>>();
			auto patch = GetPointsWithinRadiusFromPoint((*this)[i].first, 7, *indexSet);
			(*pMap)[counter++] = std::make_shared<CPatchOfPointCloud>(patch, indexSet);
			for (auto it = indexSet->begin(); it != indexSet->end(); it++)
				index_total_set.insert(*it);
		}
	}
	return pMap;
}

CCreoPointCloud::pPairPatchQueue CCreoPointCloud::CreateInitialQueue(patchMap pMap, int k, double tolerance) const
{
	pPairPatchQueue pQueue = std::make_shared<pair_patch_queue>();
	auto start_time = std::chrono::high_resolution_clock::now();
	
	for (int i = 0; i < pMap->size() - 1; i++)
	{
		int percent = (int)((1.0 * i) / pMap->size() * 16);
		for (int j = i+1; j < pMap->size(); j++)
		{
			auto time = std::chrono::high_resolution_clock::now() - start_time;
			if (time / std::chrono::milliseconds(1) > 2000)
			{
				ProError err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_PROGRESS_NAME));
				err = ProUIDialogCreate(const_cast<char*>(UI_GUI_PROGRESS_NAME), const_cast<char*>(UI_GUI_PROGRESS_NAME));
				int ui_status;
				err = ProUIDialogActivate(const_cast<char*>(UI_GUI_PROGRESS_NAME), &ui_status);
				start_time = std::chrono::high_resolution_clock::now();
			}
			setProgress(50 + (3 - k) * 16 + percent);
			if ((*pMap)[i] && (*pMap)[j] && patchesAreNeighboors((*pMap)[i], (*pMap)[j]))
			{
				double score = (*pMap)[i]->checkCompatibility((*pMap)[j], k, tolerance);
				if (score >= 0) {
					//omp_set_lock(&writelock);
					pQueue->push({ score, {i, j} });
					//omp_unset_lock(&writelock);
				}
			}
		} 
	}
	return pQueue;
}

bool CCreoPointCloud::pair_patch_queue::eraseElement(pair<int, int> patchPair)
{
	for (auto it = c.begin(); it != c.end(); it++)
	{
		if (it->second == patchPair) {
			c.erase(it);
			return true;
		}
	}
	return false;
}

void CCreoPointCloud::SegmentPointCloud(std::vector<std::vector<int>>& regions)
{
	ProError err = ProUIDialogCreate(const_cast<char*>(UI_GUI_PROGRESS_NAME), const_cast<char*>(UI_GUI_PROGRESS_NAME));
	int ui_status;
	err = ProUIDialogActivate(const_cast<char*>(UI_GUI_PROGRESS_NAME), &ui_status);
	auto pMap = createPatchMap();
	int k = 3;
	while (k > 0)
	{
		auto pQueue = CreateInitialQueue(pMap, k, 0.4);
		//auto element = pQueue->top();
		auto start_time = std::chrono::high_resolution_clock::now();
		
		while (!pQueue->empty() && pQueue->top().first > 0.05)
		{
			const auto pair = pQueue->top().second;
			pQueue->pop();
			const int numbPatches = pMap->size(); const int i = pair.first; const int j = pair.second;
			auto newMap = std::make_shared<CPatchOfPointCloud>(*(*pMap)[i]);
			newMap->MergeWith((*pMap)[j], *this);
			for (int patchIndex = 0; patchIndex < numbPatches; patchIndex++)
			{
				if ((*pMap)[patchIndex] && patchIndex != i && patchIndex != j) {
					if (patchesAreNeighboors((*pMap)[patchIndex], (*pMap)[i]) || patchesAreNeighboors((*pMap)[patchIndex], (*pMap)[j]))
					{
						auto erase_pair = std::make_pair(patchIndex, i);
						if (patchIndex > i)
						{
							erase_pair.first = i;
							erase_pair.second = patchIndex;
						}
						pQueue->eraseElement(erase_pair);
						double score = newMap->checkCompatibility((*pMap)[patchIndex], k, 0.4);
						if (score >= 0)
						{
							pQueue->push({ score, erase_pair });
						}
						erase_pair.first = patchIndex;
						erase_pair.second = j;
						if (patchIndex > j)
						{
							erase_pair.first = j;
							erase_pair.second = patchIndex;
						}
						pQueue->eraseElement(erase_pair);
					}
					auto time = std::chrono::high_resolution_clock::now() - start_time;
					if (time / std::chrono::milliseconds(1) > 2000)
					{
						ProError err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_PROGRESS_NAME));
						err = ProUIDialogCreate(const_cast<char*>(UI_GUI_PROGRESS_NAME), const_cast<char*>(UI_GUI_PROGRESS_NAME));
						int ui_status;
						err = ProUIDialogActivate(const_cast<char*>(UI_GUI_PROGRESS_NAME), &ui_status);
						start_time = std::chrono::high_resolution_clock::now();
					}
					setProgress(50 + (3 - k) * 16 + 16);
				}
			}
			(*pMap)[i] = newMap;
			(*pMap)[j] = nullptr;
		}
		k--;
	}
	
	err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_PROGRESS_NAME));
	FILE* file = fopen("segmentNumbers.txt", "w");
	std::tuple<int, int, int> color[8];
	color[0] = std::make_tuple(255, 0, 0);
	color[1] = std::make_tuple(0, 255, 0);
	color[2] = std::make_tuple(0, 0, 255);
	color[3] = std::make_tuple(255, 255, 0);
	color[4] = std::make_tuple(255, 0, 255);
	color[5] = std::make_tuple(0, 255, 0);
	color[6] = std::make_tuple(255, 255, 255);
	color[7] = std::make_tuple(128, 128, 255);

	int counter = 0;
	for (int i = 0; i < pMap->size(); i++)
	{
		if ((*pMap)[i]) {
			for (int j = 0; j < (*pMap)[i]->GetSize(); j++)
				fprintf(file, "%lf %lf %lf %d %d %d\n", (*(*pMap)[i])[j].first.x, (*(*pMap)[i])[j].first.y, (*(*pMap)[i])[j].first.z,
					std::get<0>(color[counter]), std::get<1>(color[counter]), std::get<2>(color[counter]));
			counter++;
		}
	}
	
	fclose(file);
}

ProError CCreoPointCloud::setProgress(int value)
{
	ProError err;
	err = ProUIProgressbarIntegerSet(const_cast<char*>(UI_GUI_PROGRESS_NAME),
		const_cast<char*>(UI_PROGRESS_BAR), value);
	return err;
}

bool CCreoPointCloud::patchesAreNeighboors(std::shared_ptr<CPatchOfPointCloud> patch1, std::shared_ptr<CPatchOfPointCloud> patch2) const
{
	std::shared_ptr<CPatchOfPointCloud> patchMinimum = patch1;
	std::shared_ptr<CPatchOfPointCloud> patchMaximum = patch2;
	
	if (patch1->GetSize() > patch2->GetSize())
	{
		patchMinimum = patch2;
		patchMaximum = patch1;
	}
	return patchMaximum->CheckIfNeighbor(patchMinimum);
}

double CCreoPointCloud::GetMedianFactor(const CPointEx3D& p, const CPointEx3D& n, const CPointCloud& patch) const{
	const int patchCount = patch.GetSize();
	std::vector<double> projs(patchCount);
	for (int i = 0; i < patchCount; i++)
	{
		projs[i] = dot(patch[i].first - p, n);
	}
	std::sort(projs.begin(), projs.end());
	return projs[patchCount / 2];
}

double CCreoPointCloud::GetBilateralFactor(const CPointEx3D& p, const CPointEx3D& n, const CPointCloud& patch) const
{
	//computing sigmac
	std::vector<double> norms(patch.GetSize());
	std::vector<double> errors(patch.GetSize());
	const int patchCount = patch.GetSize();
#pragma omp parallel for
	for (int i = 0; i < patchCount; i++)
	{
		norms[i] = (p - patch[i].first).norm();
		errors[i] = dot(p - patch[i].first, n);
	}
	const double sigmac = *std::max_element(norms.begin(), norms.end());
	const double meanError = std::accumulate(errors.begin(), errors.end(), 0.0) / patch.GetSize();
	double meanValue = 0;
	for (int i = 0; i < patchCount; i++)
	{
		meanValue += (errors[i] - meanError) * (errors[i] - meanError);
	}
	meanValue /= (patchCount - 1);
	const double sigmas = sqrt(meanValue);
	auto wc = [sigmac](double x) {return exp(-x * x / (2 * sigmac * sigmac)); };
	auto ws = [sigmas](double x) {return exp(-x * x / (2 * sigmas * sigmas)); };
	std::vector<double> weights(patchCount);
#pragma omp parallel for
	for (int i = 0; i < patchCount; i++)
	{
		weights[i] = wc(norms[i]) * ws(fabs(dot(n, patch[i].second) - 1.0));
	}
	meanValue = 0.0;
	for (int i = 0; i < patchCount; i++)
	{
		meanValue += weights[i] * errors[i];
	}
	return meanValue / std::accumulate(weights.begin(), weights.end(), 0.0);
}


void CCreoPointCloud::createMeshTriangular()
{
	FILE* file = fopen("outputMeshTriangular.off", "w");
	struct Triangle
	{
		int i, j, k;
	};
	std::vector<std::vector<int>> EGG(GetSize());
	std::vector<std::map<int, UmbrellaElement>> umbrellas;
	createUmbrellaFromTriangulation(umbrellas);
	
	std::vector<UmbrellaElement> umbrella;
	for (auto [key, umbel] : umbrellas[1000])
	{
		umbrella.push_back(umbel);
	}
	
	std::vector<Triangle> umbrellaFan;
	for (auto umbel : umbrella)
	{
		if (umbel.indexNext >= 0)
		{
			umbrellaFan.push_back({ 1000, umbel.index, umbel.indexNext });
		}
	}

	fprintf(file, "OFF\n");
	fprintf(file, "%d %d 0\n", GetSize(), umbrellaFan.size());
	for (int i = 0; i < GetSize(); i++)
	{
		fprintf(file, "%lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]);
	}
	for (auto tri : umbrellaFan)
	{
		fprintf(file, "3 %d %d %d\n", tri.i, tri.j, tri.k);
	}
	fclose(file);
}

void CCreoPointCloud::CreateMesh()
{
	FILE* file = fopen("outputMesh.off", "w");
	struct Triangle
	{
		int i, j, k;
	};
	std::vector<std::vector<int>> EGG(GetSize());
	calculateEGG(EGG);
	
	auto umbrella = CreateUmbrellaAroundPoint(1000, EGG[1000]);

	std::vector<Triangle> umbrellaFan;
	for (auto umbel : umbrella)
	{
		if (umbel.indexNext >= 0)
		{
			umbrellaFan.push_back({ 1000, umbel.index, umbel.indexNext });
		}
	}

	fprintf(file, "OFF\n");
	fprintf(file, "%d %d 0\n", GetSize(), umbrellaFan.size());
	for (int i = 0; i < GetSize(); i++)
	{
		fprintf(file, "%lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]);
	}
	for (auto tri : umbrellaFan)
	{
		fprintf(file, "3 %d %d %d\n", tri.i, tri.j, tri.k);
	}
	fclose(file);
}



PrincipalCurvatures CCreoPointCloud::computePrincipalCurvatures(std::shared_ptr<CPointEx3DVector> maximumDirection, 
	std::shared_ptr<CPointEx3DVector> minimumDirection) const
{
	std::shared_ptr<std::vector<double>> kmax = std::make_shared<std::vector<double>>(GetSize());
	std::shared_ptr<std::vector<double>> kmin = std::make_shared<std::vector<double>>(GetSize());
	if (maximumDirection) maximumDirection->ReSize(GetSize());
	if (minimumDirection) minimumDirection->ReSize(GetSize());
	double percent = 0.0;
	std::vector<void*> parameters(6);
	parameters[0] = &*kmax;
	parameters[1] = &*kmin;
	parameters[2] = maximumDirection ? &*maximumDirection : nullptr;
	parameters[3] = minimumDirection ? &*minimumDirection : nullptr;
	parameters[4] = (void*)this;
	parameters[5] = &percent;

	auto threadToExecute = [](LPVOID lpParameter)->DWORD
	{
		std::vector<void*>* parameterVector = (std::vector<void*>*)lpParameter;
		std::vector<double>* kmax = (std::vector<double>*)(*parameterVector)[0];
		std::vector<double>* kmin = (std::vector<double>*)(*parameterVector)[1];
		CPointEx3DVector* maximumDirections = (CPointEx3DVector*)(*parameterVector)[2];
		CPointEx3DVector* minimumDirections = (CPointEx3DVector*)(*parameterVector)[3];
		CCreoPointCloud* pc = (CCreoPointCloud*)(*parameterVector)[4];
		double* percent = (double*)(*parameterVector)[5];

		std::vector<std::vector<int>> EGG(pc->GetSize());
		pc->calculateEGG(EGG);

		double thres = 0.1;
		int counter = 0;
#pragma omp parallel for
		for (int i = 0; i < pc->GetSize(); i++)
		{
			//std::set<int> indexset;
			//auto patch = pc->GetPointsWithinRadiusFromPoint((*pc)[i].first, 3.5, indexset, i);
			/*std::set<int> indexSet;
			for (auto index : EGG[i])
			{
				indexSet.insert(index);
			}
			for (auto index : indexSet)
			{
				for (auto indexIndex : EGG[index])
					if (indexIndex != i)
						indexSet.insert(indexIndex);
			}
			for (auto index : indexSet)
			{
				for (auto indexIndex : EGG[index])
					if (indexIndex != i)
						indexSet.insert(indexIndex);
			}*/
			auto indexes = pc->GabrielkNeighborhood(i, 4, EGG);
			auto patch = std::make_shared<CPointCloud>();
			for (auto index : indexes)
			{
				if (index != i)
					patch->Add((*pc)[index].first, (*pc)[index].second);
			}
			CPointEx3D maximumDirection, minimumDirection;
			auto curvatures = GetCurvaturesOfLocalPatch((*pc)[i].first, (*pc)[i].second, *patch, maximumDirection, minimumDirection);
#pragma omp critical
			{
				double currentPercent = (1.0 * counter) / pc->GetSize();
				if (currentPercent > thres)
				{
					*percent = thres;
					thres += 0.1;
				}
				counter++;
			}
			(*kmax)[i] = curvatures.first;
			(*kmin)[i] = curvatures.second;
			if (maximumDirections) (*maximumDirections)[i] = maximumDirection;
			if (minimumDirections) (*minimumDirections)[i] = minimumDirection;
		}
		*percent = 1.0;
		return 0;
	};
	HANDLE hThread = CreateThread(
		NULL,    // Thread attributes
		0,       // Stack size (0 = use default)
		threadToExecute, // Thread start address
		&parameters,    // Parameter to pass to the thread
		0,       // Creation flags
		NULL);   // Thread id

	showProgressBar("Curvature computation", percent);

	// Wait for thread to finish execution
	WaitForSingleObject(hThread, INFINITE);

	// Thread handle must be closed when no longer needed
	CloseHandle(hThread);
	FILE * file;
	/*file = fopen("curv.txt", "r");
	for (int i = 0; i < GetSize(); i++)
	{
		double k11, k22, k11x, k11y, k11z, k22x, k22y, k22z;
		fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &k11, &k22, &k11x, &k11y, &k11z, &k22x, &k22y, &k22z);
		(*kmin)[i] = k11; (*kmax)[i] = k22;
		if (maximumDirection) (*maximumDirection)[i] = CPointEx3D(k11x, k11y, k11z);
		if (minimumDirection) (*minimumDirection)[i] = CPointEx3D(k22x, k22y, k22z);
	}
	fclose(file);*/

	std::vector<double> curvedness(GetSize());
	for (int i = 0; i < GetSize(); i++)
	{
		double k1 = (*kmin)[i];
		double k2 = (*kmax)[i];
		curvedness[i] = sqrt(k1 * k1 + k2 * k2);
		//if (curvedness[i] > 8) curvedness[i] = 8;
	}
	double meanCurvedness = std::accumulate(curvedness.begin(), curvedness.end(), 0.0) / GetSize();
	double curvednessDeviation = 0.0;
	for (auto c : curvedness)
	{
		curvednessDeviation += (c - meanCurvedness) * (c - meanCurvedness);
	}
	curvednessDeviation = sqrt(curvednessDeviation / GetSize());
	file = fopen("curvatureCharacteristics.txt", "w");
	fprintf(file, "%lf %lf %lf", meanCurvedness, curvednessDeviation, meanCurvedness + curvednessDeviation);
	fclose(file);
	for (int i = 0; i < GetSize(); i++)
	{
		if (curvedness[i] > meanCurvedness + curvednessDeviation / 2.0)
			curvedness[i] = meanCurvedness + curvednessDeviation / 2.0;
		//if (curvedness[i] > 8) curvedness[i] = 8;
	}
	double threshold = meanCurvedness + curvednessDeviation / 4.0;
	for (int i = 0; i < GetSize(); i++)
	{
		if (curvedness[i] > meanCurvedness + curvednessDeviation / 2.0)
			curvedness[i] = meanCurvedness + curvednessDeviation / 2.0;
		//if (curvedness[i] > 8) curvedness[i] = 8;
	}

	vector<double> shapeIndex(GetSize(), 1e10);
	for (int i = 0; i < GetSize(); i++)
	{
		double s = (-2.0 / M_PI) * atan(((*kmax)[i] + (*kmin)[i]) / ((*kmax)[i] - (*kmin)[i]));
		shapeIndex[i] = s;
	}
	
	file = fopen("shapeIndexValues.txt","w");
	for (int i = 0; i < GetSize(); i++)
	{
		fprintf(file, "%lf %lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2], shapeIndex[i]);
	}
	fclose(file);

	int colorTable[10][3] =
	{
		{255, 0, 0},   //0
		{0, 255, 0},   //1
		{0, 0, 255},   //2
		{255, 255, 0}, //3
		{0, 255, 255}, //4
		{255, 0, 255},
		{128, 255, 255},
		{255, 128, 255},
		{255, 255, 128},
		{128, 128, 255}
	};

	file = fopen("shapeIndex.xyz", "w");
	for (int i = 0; i < GetSize(); i++)
	{
		int color;
		double s = shapeIndex[i];

		if (s >= -1.0 && s < -7.0 / 8) color = 0;
		else if (s >= -7.0 / 8 && s < -5.0 / 8) color = 1;
		else if (s >= -5.0 / 8 && s < -3.0 / 8) color = 2;
		else if (s >= -3.0 / 8 && s < -1.0 / 8) color = 3;
		else if (s >= -1.0 / 8 && s < 1.0 / 8) color = 4;
		else if (s >= 1.0 / 8 && s < 3.0 / 8) color = 5;
		else if (s >= 3.0 / 8 && s < 5.0 / 8) color = 6;
		else if (s >= 5.0 / 8 && s < 7.0 / 8) color = 7;
		else if (s >= 7.0 / 8 && s <= 1.0) color = 8;
		fprintf(file, "%lf %lf %lf %d %d %d\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2],
			colorTable[color][0], colorTable[color][1], colorTable[color][2]);
	}
	fclose(file);

	file = fopen("principalCurvatures.txt", "w");
	for (int i = 0; i < GetSize(); i++)
	{
		fprintf(file, "%lf %lf\n", (*kmin)[i], (*kmax)[i]);
	}
	fclose(file);

	std::vector<double> meanShiftedShapeIndex(GetSize());
	for (int counter = 0; counter < 3; counter++) {
#pragma omp parallel for
		for (int i = 0; i < GetSize(); i++)
		{
			std::set<int> indexset;
			CPointEx3D pnt = (*this)[i].first;
			double shapeIndext = shapeIndex[i];
			double shapeIndextp1 = shapeIndex[i];
			int excludeIndex = i;
			do {
				shapeIndext = shapeIndextp1;
				double sumSumShape = shapeIndext;
				CPointEx3D sumPnt = pnt;
				double counter = 1;
				auto patch = GetPointsWithinRadiusFromPoint(pnt, 3.0, indexset, excludeIndex);
				excludeIndex = -1;
				for (int j = 0; j < patch->GetSize(); j++)
				{
					if (fabs(shapeIndext - shapeIndex[j]) < 0.15)
					{
						sumSumShape += shapeIndex[j];
						sumPnt += (*this)[i].first;
						counter++;
					}
				}
				shapeIndextp1 = sumSumShape / counter;
				pnt = 1.0 / counter * sumPnt;
			} while (fabs(shapeIndextp1 - shapeIndext) > 1e-6);
			meanShiftedShapeIndex[i] = shapeIndextp1;
		}
		shapeIndex = meanShiftedShapeIndex;
	}

	file = fopen("meanShapeIndexValues.txt", "w");
	for (int i = 0; i < GetSize(); i++)
	{
		fprintf(file, "%lf %lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2], meanShiftedShapeIndex[i]);
	}
	fclose(file);
	

	file = fopen("shapeIndexShifted.xyz", "w");
	for (int i = 0; i < GetSize(); i++)
	{
		int color;
		double s = meanShiftedShapeIndex[i];
			
		if (s >= -1 && s < -7.0 / 8) color = 0;
		else if (s >= -7.0/8 && s < -5.0 / 8) color = 1;
		else if (s >= -5.0 / 8 && s < -3.0 / 8) color = 2;
		else if (s >= -3.0 / 8 && s < -1.0 / 8) color = 3;
		else if (s >= -1.0 / 8 && s <  1.0 / 8) color = 4;
		else if (s >=  1.0 / 8 && s <  3.0 / 8) color = 5;
		else if (s >=  3.0 / 8 && s <  5.0 / 8) color = 6;
		else if (s >=  5.0 / 8 && s <  7.0 / 8) color = 7;
		else if (s >=  7.0 / 8 && s <= 1.0) color = 8;
		fprintf(file, "%lf %lf %lf %d %d %d\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2], 
				colorTable[color][0], colorTable[color][1], colorTable[color][2]);
	}
	fclose(file);


	/*double threshold = 50;// meanCurvedness + 0.5 * sqrt(curvednessDeviation / GetSize());
	auto gaussian = [](double x, double s, double m) {return exp(-(x - m) * (x - m) / (s * s)); };
	auto cnew = curvedness;
	std::transform(cnew.begin(), cnew.end(), cnew.begin(),
		[threshold, gaussian](double x)
		{
			if (x < threshold)
				return x;
			return x * gaussian(x, 0.5, threshold);
		});
	*/
	
	file = fopen("curvatures.txt", "w");
	for (int i = 0; i < GetSize(); i++)
	{
		fprintf(file, "%lf %lf %lf %lf\n", (*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2], curvedness[i]);
	}
	fclose(file);

	return { kmax, kmin };
}

std::shared_ptr<CCreoPointCloud> CCreoPointCloud::sampleUniformlyByUpsampling() const
{
	// Reads a .xyz point set file in points[]
	CGALPointList points(this->GetSize());
#pragma omp parallel for
	for (int i = 0; i < this->GetSize(); i++)
	{
		points[i] = {
			Point_3((*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]),
			Vector_3((*this)[i].second[0], (*this)[i].second[1], (*this)[i].second[2])
		};
	}
	double percent = 0.0;
	//parameters

	std::vector<void*> parameters(3);
	double density_temp = meanDistance;
	parameters[0] = &points;
	parameters[1] = &density_temp;
	parameters[2] = &percent;
	auto threadToExecute = [](LPVOID lpParameter)->DWORD
	{
		std::vector<void*>* parametetVector = (std::vector<void*>*)lpParameter;
		CGALPointList* points = (CGALPointList*)(*parametetVector)[0];
		double* density = (double*)(*parametetVector)[1];
		double* percent = (double*)(*parametetVector)[2];
		//Algorithm parameters
		const double sharpness_angle = 25;   // control sharpness of the result.
		const double edge_sensitivity = 0;    // higher values will sample more points near the edges
		const double neighbor_radius = 6 * *density;  // initial size of neighborhood.
		const std::size_t number_of_output_points = (int)(points->size() * 4);
		//Run algorithm
		CGAL::edge_aware_upsample_point_set<Concurrency_tag>(
			*points,
			std::back_inserter(*points),
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<CGALPointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<CGALPointVectorPair>()).
			sharpness_angle(sharpness_angle).
			edge_sensitivity(edge_sensitivity).
			neighbor_radius(neighbor_radius).
			number_of_output_points(number_of_output_points).
			callback([percent](double a) {*percent = a; return true; }));
		return 0;
	};
	HANDLE hThread = CreateThread(
		NULL,    // Thread attributes
		0,       // Stack size (0 = use default)
		threadToExecute, // Thread start address
		&parameters,    // Parameter to pass to the thread
		0,       // Creation flags
		NULL);   // Thread id

	showProgressBar("Uniform resampling", percent);

	// Wait for thread to finish execution
	WaitForSingleObject(hThread, INFINITE);

	// Thread handle must be closed when no longer needed
	CloseHandle(hThread);

	FILE* filexyz = fopen("uniform_sampled.xyz", "w");
	FILE* filepts = fopen("uniform_sampled.pts", "w");
	for (int i = 0; i < points.size(); i++)
	{
		fprintf(filexyz, "%lf %lf %lf %lf %lf %lf\n", 
			points[i].first.x(), points[i].first.y(), points[i].first.z(),
			points[i].second.x(), points[i].second.y(), points[i].second.z());
		fprintf(filepts, "%lf %lf %lf\n",
			points[i].first.x(), points[i].first.y(), points[i].first.z());
	}
	fclose(filexyz);
	fclose(filepts);
	ProPath creoFile = L"uniform_sampled.pts";
	return std::make_shared<CCreoPointCloud>(creoFile, "uniform_sampled.xyz");
}

std::shared_ptr<CCreoPointCloud> CCreoPointCloud::wlop() const
{
	// Reads a .xyz point set file in points[]
	CGALPointList points(this->GetSize());
#pragma omp parallel for
	for (int i = 0; i < this->GetSize(); i++)
	{
		points[i] = {
			Point_3((*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]),
			Vector_3((*this)[i].second[0], (*this)[i].second[1], (*this)[i].second[2])
		};
	}
	double percent = 0.0;
	//parameters

	std::vector<void*> parameters(4);
	double density_temp = meanDistance;
	parameters[0] = &points;
	parameters[1] = &density_temp;
	parameters[2] = &percent;
	std::vector<Point_3> output;
	parameters[3] = &output;
	auto threadToExecute = [](LPVOID lpParameter)->DWORD
	{
		std::vector<void*>* parametetVector = (std::vector<void*>*)lpParameter;
		CGALPointList* points = (CGALPointList*)(*parametetVector)[0];
		double* density = (double*)(*parametetVector)[1];
		double* percent = (double*)(*parametetVector)[2];
		std::vector<Point_3>* output = (std::vector<Point_3>*)(*parametetVector)[3];
		//Algorithm parameters
		const double percentOfPoints = 95;
		//Run algorithm
		CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>(
			*points,
			std::back_inserter(*output),
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<CGALPointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<CGALPointVectorPair>()).
			select_percentage(percentOfPoints).
			number_of_iterations(150).
			require_uniform_sampling(true).
			callback([percent](double a) {*percent = a; return true; }));
		return 0;
	};
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	HANDLE hThread = CreateThread(
		NULL,    // Thread attributes
		0,       // Stack size (0 = use default)
		threadToExecute, // Thread start address
		&parameters,    // Parameter to pass to the thread
		0,       // Creation flags
		NULL);   // Thread id

	showProgressBar("WLOP Projection", percent);

	// Wait for thread to finish execution
	WaitForSingleObject(hThread, INFINITE);

	// Thread handle must be closed when no longer needed
	CloseHandle(hThread);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

	FILE* file = fopen("WLOPSmoothingTime.txt", "w");
	fprintf(file, "Time taken:%lf", time);
	fclose(file);
	
	FILE* filepts = fopen("wlop_sampled.pts", "w");
	for (int i = 0; i < points.size(); i++)
	{
		fprintf(filepts, "%lf %lf %lf\n",
			output[i].x(), output[i].y(), output[i].z());
	}
	
	fclose(filepts);
	ProPath creoFile = L"wlop_sampled.pts";
	return std::make_shared<CCreoPointCloud>(creoFile, "wlop_sampled.pts", true);
}


std::shared_ptr<CCreoPointCloud> CCreoPointCloud::sampleUniformly(){
	// Reads a .xyz point set file in points[]
	std::vector<Point_3> points(this->GetSize());
	for (int i = 0; i < this->GetSize(); i++)
	{
		points[i] = Point_3((*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]);
	}
	std::vector<Point_3> output;
	double percent = 0.0;
	//parameters
	std::vector<void*> parameters(4);
	parameters[0] = &points;
	parameters[1] = &output;
	parameters[2] = &meanDistance;
	parameters[3] = &percent;
	auto threadToExecute = [](LPVOID lpParameter)->DWORD
	{
		std::vector<void*>* parametetVector = (std::vector<void*>*)lpParameter;
		std::vector<Point_3>* points = (std::vector<Point_3>*)(*parametetVector)[0];
		std::vector<Point_3>* output = (std::vector<Point_3>*)(*parametetVector)[1];
		double* density = (double*)(*parametetVector)[2];
		double* percent = (double*)(*parametetVector)[3];
		const double retain_percentage = 99;   // percentage of points to retain.
		const double neighbor_radius = 5.0 * *density;   // neighbors size.
		CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>
			(*points, std::back_inserter(*output),
				CGAL::parameters::select_percentage(retain_percentage).
				neighbor_radius(neighbor_radius).require_uniform_sampling(true).callback([percent](double a) {*percent = a; return true; }));
		return 0;
	};
	HANDLE hThread = CreateThread(
		NULL,    // Thread attributes
		0,       // Stack size (0 = use default)
		threadToExecute, // Thread start address
		&parameters,    // Parameter to pass to the thread
		0,       // Creation flags
		NULL);   // Thread id

	showProgressBar("Uniform resampling", percent);
	
	// Wait for thread to finish execution
	WaitForSingleObject(hThread, INFINITE);

	// Thread handle must be closed when no longer needed
	CloseHandle(hThread);
	
	FILE* file = fopen("uniform_sampled.pts", "w");
	for (int i = 0; i < output.size(); i++)
	{
		fprintf(file, "%lf %lf %lf\n", output[i].x(), output[i].y(), output[i].z());
	}
	fclose(file);
	ProPath creoFile = L"uniform_sampled.pts";
	return std::make_shared<CCreoPointCloud>(creoFile, "uniform_sampled.pts");
}

void CCreoPointCloud::showProgressBar(const char* title, double& percent)
{
	auto start_time = std::chrono::high_resolution_clock::now();
	ProError err = ProUIDialogCreate(const_cast<char*>(UI_GUI_PROGRESS_NAME), const_cast<char*>(UI_GUI_PROGRESS_NAME));
	wchar_t creoTitle[512];
	ProStringToWstring(creoTitle, (char*)title);
	ProUIDialogTitleSet(const_cast<char*>(UI_GUI_PROGRESS_NAME), creoTitle);
	int ui_status;
	err = ProUIDialogActivate(const_cast<char*>(UI_GUI_PROGRESS_NAME), &ui_status);
	while (percent < 1.0) {
		auto time = std::chrono::high_resolution_clock::now() - start_time;
		if (time / std::chrono::milliseconds(1) > 3000)
		{
			ProError err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_PROGRESS_NAME));
			err = ProUIDialogCreate(const_cast<char*>(UI_GUI_PROGRESS_NAME), const_cast<char*>(UI_GUI_PROGRESS_NAME));
			ProUIDialogTitleSet(const_cast<char*>(UI_GUI_PROGRESS_NAME), creoTitle);
			int ui_status;
			err = ProUIDialogActivate(const_cast<char*>(UI_GUI_PROGRESS_NAME), &ui_status);
			start_time = std::chrono::high_resolution_clock::now();
		}
		int per = (int)(percent * 100);
		setProgress(per);
	}
	err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_PROGRESS_NAME));
}

void CCreoPointCloud::updateSpatialTree()
{
	std::vector<Point_3> points(GetSize());
	std::vector<int> indices(GetSize());
#pragma omp parallel for
	for (int i = 0; i < GetSize(); i++)
	{
		points[i] = Point_3((*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]);
		indices[i] = i;
	}
	pointCloudSpatialTree = std::make_shared<Tree>(boost::make_zip_iterator(boost::make_tuple(points.begin(), indices.begin())),
		boost::make_zip_iterator(boost::make_tuple(points.end(), indices.end())));
}


void CCreoPointCloud::SmoothPointCloudCGALBilateral()
{
	CGALPointList cgalPntLst(this->GetSize());
#pragma omp parallel for
	for (int i = 0; i < this->GetSize(); i++)
	{
		cgalPntLst[i] = {Point_3((*this)[i].first[0], (*this)[i].first[1], (*this)[i].first[2]),
						 Vector_3((*this)[i].second[0], (*this)[i].second[1], (*this)[i].second[2])};
	}
	double percent = 0;
	std::vector<void*> parameters(2);
	parameters[0] = &cgalPntLst;
	parameters[1] = &percent;
	auto threadToExecute = [](LPVOID lpParameter)->DWORD
	{
		std::vector<void*>* parametetVector = (std::vector<void*>*)lpParameter;
		CGALPointList* cgalPntLst = (CGALPointList*)(*parametetVector)[0];
		double* percent = (double*)(*parametetVector)[1];
		int k = 120;                 // size of neighborhood. The bigger the smoother the result will be.
									 // This value should bigger than 1.
		double sharpness_angle = 25;  // control sharpness of the result.
									 // The bigger the smoother the result will be
		int iter_number = 1;         // number of times the projection is applied
		for (int i = 0; i < iter_number; ++i)
		{
			/* double error = */
			CGAL::bilateral_smooth_point_set <Concurrency_tag>(
				*cgalPntLst,
				k,
				CGAL::parameters::point_map(CGAL::First_of_pair_property_map<CGALPointVectorPair>())
				.normal_map(CGAL::Second_of_pair_property_map<CGALPointVectorPair>())
				.sharpness_angle(sharpness_angle).callback([percent, i, iter_number](double a) {*percent = a/iter_number + i/(1.0 * iter_number); return true; }));
		}
		return 0;
	};
	auto start = std::chrono::high_resolution_clock::now();
	ComputeNormalsWithPCA(false);
	HANDLE hThread = CreateThread(
		NULL,    // Thread attributes
		0,       // Stack size (0 = use default)
		threadToExecute, // Thread start address
		&parameters,    // Parameter to pass to the thread
		0,       // Creation flags
		NULL);   // Thread id

	showProgressBar("Noise reduction", percent);

	// Wait for thread to finish execution
	WaitForSingleObject(hThread, INFINITE);

	// Thread handle must be closed when no longer needed
	CloseHandle(hThread);

	auto stop = std::chrono::high_resolution_clock::now();

	// Get duration. Substart timepoints to
	// get durarion. To cast it to proper unit
	// use duration cast method
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	FILE* file = fopen("Bilateral Duration", "w");
	fprintf(file, "Time taken: %lld ms", duration.count());
	fclose(file);
	
	// Algorithm parameters
#pragma omp parallel for
	for (int i = 0; i < this->GetSize(); i++)
	{
		(*this)[i].first = CPointEx3D(cgalPntLst[i].first.x(), cgalPntLst[i].first.y(), cgalPntLst[i].first.z());
		(*this)[i].second = CPointEx3D(cgalPntLst[i].second.x(), cgalPntLst[i].second.y(), cgalPntLst[i].second.z());
	}
	updateSpatialTree();
	regenerateFeature();
}

void CCreoPointCloud::regenerateFeature()
{
	ProMdl model;
	ProError err = ProMdlCurrentGet(&model);
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	ProError status = ProMdlCurrentGet(&model);
	status = ProFeatureDelete((ProSolid)model, &point_cloud_internal_id, 1, opt, 1);
	status = ProTreetoolRefresh(model);
	status = ProSolidDisplay((ProSolid)model);

	this->savePointCloud("CGALsmoothing.pts", false);
	const wchar_t* filePath = L"CGALsmoothing.pts";
	void* vp = (void*)filePath;
	
	ProIntfDataSource id_source;
	err = ProIntfDataSourceInit(PRO_INTF_PTS, vp, &id_source);
	ProFeature feature;
	err = ProImportfeatCreate((ProSolid)model, &id_source, NULL, NULL, &feature);
	point_cloud_id = feature.id;
	ProSelection selection;
	ProModelitem modelitem;
	ProSelectionAlloc(NULL, &feature, &selection);
	err = ProSelectionModelitemGet(selection, &modelitem);
	point_cloud_internal_id = modelitem.id;
	ProSelectionFree(&selection);
	ProIntfDataSourceClean(&id_source);
}