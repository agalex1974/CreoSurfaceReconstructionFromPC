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


ProError CCreoPointCloud::setProgress(int value)
{
	ProError err;
	err = ProUIProgressbarIntegerSet(const_cast<char*>(UI_GUI_PROGRESS_NAME),
		const_cast<char*>(UI_PROGRESS_BAR), value);
	return err;
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