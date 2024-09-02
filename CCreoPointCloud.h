#ifndef CREO_POINT_CLOUD_H
#define CREO_POINT_CLOUD_H

#undef min
#include <queue>
#include <ProMdl.h>
#include <ProFeature.h>
#include "CPointCloud.h"
// CGAL HEADERS///////////////////////////////////
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <boost/iterator/zip_iterator.hpp>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/property_map.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
///////////////////////////////////////////////
#include <ProUI.h>
#include <ProUIDialog.h>
#include <ProUIProgressbar.h>
#include "CPatchOfPointCloud.h"

/// <summary>
// CGAL DEFINITIONS
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
//typedef CGAL::Monge_via_jet_fitting<Kernel> My_Monge_via_jet_fitting;
//typedef My_Monge_via_jet_fitting::Monge_form     My_Monge_form;
typedef boost::tuple<Point_3, int> Point_and_int;
typedef CGAL::Search_traits_3<Kernel>                       Traits_base;
typedef CGAL::Search_traits_adapter<Point_and_int,
	CGAL::Nth_of_tuple_property_map<0, Point_and_int>,
	Traits_base> Traits;
typedef CGAL::Random_points_in_cube_3<Point_3>       Random_points_iterator;
typedef CGAL::Counting_iterator<Random_points_iterator> N_Random_points_iterator;
typedef CGAL::Kd_tree<Traits> Tree;
typedef CGAL::Fuzzy_sphere<Traits> Fuzzy_sphere;
typedef CGAL::Fuzzy_iso_box<Traits> Fuzzy_iso_box;
typedef CGAL::Orthogonal_k_neighbor_search<Traits>          K_neighbor_search;
typedef K_neighbor_search::Distance                         Distance;
typedef CGAL::Parallel_if_available_tag Concurrency_tag;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point_3, Vector_3> CGALPointVectorPair;
typedef std::vector<CGALPointVectorPair> CGALPointList;

/// </summary>

using PrincipalCurvatures = std::pair<std::shared_ptr<std::vector<double>>, std::shared_ptr<std::vector<double>>>;

class CCreoPointCloud : public CPointCloud
{
public:
	double getSamplingDensity() { return meanDistance; }
	CCreoPointCloud(ProPath filePath, const char* pntCloudFile, bool onlyPnts = false);
	~CCreoPointCloud();
	spPointCloud GetPointsWithinRadiusFromPoint(const CPointEx3D& pnt, double timesOfDensity, std::set<int>& indexSet, int excludeIndex = -1) const;
	std::vector<int> GetKNearestNeighborIndex(const CPointEx3D& pnt, int K) const;
	spPointCloud pointCloudFromSelection(std::set<int>& indexes) const;
	//static DWORD WINAPI SegmentPointCloud(LPVOID lpParameter);
	//static void SegmentPointCloudCallback(void* lpParameter);
	void SegmentPointCloud(std::vector<std::vector<int>>& regions);
	static ProError setProgress(int value);
	PrincipalCurvatures computePrincipalCurvatures(std::shared_ptr<CPointEx3DVector> maximumDirection = nullptr,
		std::shared_ptr<CPointEx3DVector> minimumDirection = nullptr) const;
	bool isEllipicGabrielNeighbor(int i, const std::vector<int>& NNs, double a) const;
	std::vector<int> GabrielkNeighborhood(int i, int k, const std::vector<std::vector<int>>& EGG);
	void calculateEGG(std::vector<std::vector<int>>& EGG, double ratio = 0.65) const;
	void ComputeNormalsWithPCA(bool use_existing_orientation = false);
	std::shared_ptr<CCreoPointCloud> SmoothPointCloud();
	void SmoothPointCloudCGALBilateral();
	std::shared_ptr<CCreoPointCloud> sampleUniformly();
	ProFeature& GetPntCloudFeature(){ return featurePntCloud; }
	PrincipalCurvatures GetTaubinCurvatures(std::shared_ptr<CPointEx3DVector> maximumDirection = nullptr,
		std::shared_ptr<CPointEx3DVector> minimumDirection = nullptr) const;
	PrincipalCurvatures GetTaubinCurvaturesFromPointCloud(std::shared_ptr<CPointEx3DVector> maximumDirection = nullptr,
		std::shared_ptr<CPointEx3DVector> minimumDirection = nullptr) const;
	void CreateMesh();
	void createMeshTriangular();
	std::vector<double> get_cotan_weights(int i, const std::vector<UmbrellaElement>& ring) const;
	void smooth_iterative(int nb_iter, double lambda, double mu, double gabrielRatio, bool isEdgeAware, bool doMollification);
	void bilateral_iterative(int nb_iter, double sigmaC, double sigmaS, double gabrielRatio);
	void smooth_normal_field(int nb_iter);
	void save_smoothed_tringulation();
	PrincipalCurvatures computeCGALPrincipalCurvatures(std::shared_ptr<CPointEx3DVector> maximumDirection = nullptr,
		std::shared_ptr<CPointEx3DVector> minimumDirection = nullptr) const;
	std::shared_ptr<CCreoPointCloud> sampleUniformlyByUpsampling() const;
	std::shared_ptr<CCreoPointCloud> wlop() const;
private:
	std::shared_ptr<std::vector<CPointEx3D>> cachePoints;
	ProFeature featurePntCloud;
	class pair_patch_queue : public priority_queue<pair<double, pair<int, int>>>
	{
	public:
		bool eraseElement(pair<int, int> patchPair);
	};
	void updateSpatialTree();
	void regenerateFeature();
	using constString = const char* const;
	using patchMap = std::shared_ptr<map<int, std::shared_ptr<CPatchOfPointCloud>>>;
	using pPairPatchQueue = std::shared_ptr<pair_patch_queue>;
	// Declare user interface constants to tie them with retrieving the
	static constexpr constString UI_GUI_PROGRESS_NAME = "progress";
	static constexpr constString UI_PROGRESS_BAR = "ProgressBar";
	static void showProgressBar(const char* title, double& progress);
	patchMap createPatchMap() const;
	pPairPatchQueue CreateInitialQueue(patchMap pMap, int k, double tolerance) const;
	bool patchesAreNeighboors(std::shared_ptr<CPatchOfPointCloud> patch1, std::shared_ptr<CPatchOfPointCloud> patch2) const;
	double GetBilateralFactor(const CPointEx3D& p, const CPointEx3D& n, const CPointCloud& patch) const;
	double GetMedianFactor(const CPointEx3D& p, const CPointEx3D& n, const CPointCloud& patch) const;
	double meanDistance;
	std::shared_ptr<Tree> pointCloudSpatialTree;
	int point_cloud_internal_id;
	int point_cloud_id;	
};

#endif