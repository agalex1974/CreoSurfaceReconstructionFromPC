#ifndef GUI_INTERFACE
#define GUI_INTERFACE

#include <map>
#include <utility>
#include "GuiInterface.h"
#include "RansacPrimitive.h"
#include "BSplineSketch.h"
#include "CPointCloud.h"
#include "HelperStructures.h"

class GuiInterface
{
private:
	using constString = const char* const;
	// Declare user interface constants to tie them with retrieving the
	static constexpr constString UI_GUI_NAME = "ransacdialog";
	static constexpr constString UI_PLANE_GRAB_POINTS = "PlaneGrabPoints";
	static constexpr constString UI_CYLINDER_GRAB_POINTS = "CylinderGrabPoints";
	static constexpr constString UI_CYLINDER_GRAB_STARTING_POINT = "CylinderSelectStartingPoint";
	static constexpr constString UI_CYLINDER_GRAB_ENDING_POINT = "CylinderSelectTerminationPoint";
	static constexpr constString UI_GENERAL_GRAB_REFERENCE_PLANE = "GrabReferencePlane";
	static constexpr constString UI_GENERAL_GRAB_INNER_POINT = "GrabGeneralInnerPoint";
	static constexpr constString UI_GENERAL_GRAB_EXTENT_POINT = "GrabGeneralExtentPoint";
	static constexpr constString UI_INPUT_EPSILON = "epsilon";
	static constexpr constString UI_INPUT_BITMAP_EPSILON = "bitmapEpsilon";
	static constexpr constString UI_INPUT_NORMAL_THRESHOLD = "normalThresh"; 
	static constexpr constString UI_INPUT_MINIMUM_SUPPORT = "minSupport";
	static constexpr constString UI_INPUT_PROBABILITY = "probability";
	static constexpr constString UI_OUTPUT_PLANE_POINT_COUNT = "PlanePointCount";
	static constexpr constString UI_OUTPUT_CYLINDER_POINT_COUNT = "CylinderPointCount";
	static constexpr constString UI_CREATE_PLANE_POINT_COUNT = "PlanePointCount";
	static constexpr constString UI_TAB_NAME = "PrimitivesType";
	static constexpr constString UI_TAB_NAME_PLANE = "PlaneLayout";
	static constexpr constString UI_TAB_NAME_CYLINDER = "CylinderLayout";
	static constexpr constString UI_TAB_NAME_GENERAL_EXTRUSION = "GeneralLayout";
	static constexpr constString UI_CREATE_PRIMITIVE = "CreatePrimitives";
	static constexpr constString UI_ACCEPT_PRIMITIVE = "AcceptPrimitive";
	static constexpr constString UI_PRIMITIVE_LIST = "PrimitiveList";
	static constexpr constString UI_PRIMITIVE_LIST_POPUP = "ransac_cyliinder_popup";
	static constexpr constString UI_CYLINDER_STARTING_POINT_INFO = "CylinderStartingPointInfo";
	static constexpr constString UI_CYLINDER_ENDING_POINT_INFO = "CylinderEndingPointInfo";
	static constexpr constString UI_GENERAL_EXTRUSION_PLANE_INFO = "ReferencePlaneName";
	static constexpr constString UI_GENERAL_INNER_POINT_INFO = "InnerGeneralPointInfo";
	static constexpr constString UI_GENERAL_EXTENT_POINT_INFO = "ExtentGeneralPointInfo";
	static constexpr constString UI_PRIMITIVE_CHANGE_DIRECTION = "PrimitiveChangeDirection";
	static constexpr constString UI_POINT_CLOUD_LOAD = "LoadModel";

	static void closeAction(char* dialog, char *component, ProAppData appdata);
	std::vector<Pro3dPnt> pnts_grabbed_plane; // Should be erased? Probably.
	static void on_push_button_plane_grab_points(char* dialog, char *component, ProAppData appdata);
	static void on_push_button_cylinder_grab_points(char* dialog, char *component, ProAppData appdata);
	static void on_push_button_cylinder_grab_start_point(char* dialog, char *component, ProAppData appdata);
	static void on_push_button_cylinder_grab_end_point(char* dialog, char *component, ProAppData appdata);
	static void on_push_button_general_grab_reference_plane(char* dialog, char *component, ProAppData appdata);
	static void on_push_button_general_grab_inner_point(char* dialog, char *component, ProAppData appdata);
	static void on_push_button_general_grab_extent_point(char* dialog, char *component, ProAppData appdata);
	static void on_element_select(char* dialog, char *component, ProAppData appdata);
	static void set_default_values();
	
	std::vector<CPointEx3D> buffer_pnt_plane;						/**< The points that are selected by the user on the plane dialog */
	std::vector<CPointEx3D> buffer_pnt_cylinder;					/**< The points that are selected by the user on the cylinder dialog */
	CPointEx3D cylinder_starting_point;							/**< The cylinder start point picked by the user on the cylinder dialog */
	CPointEx3D cylinder_ending_point;								/**< The cylinder end point picked by the user on the cylinder dialog */
	CPointEx3D general_extrusion_inner_point;						/**< The inner point selected by the user denoting the point through which
																	 the plane on which the projection of point will be made in order to create
																	 the general extrusion silhouette */
	double general_extrusion_distance_threshold;				/**< The general extrusion distance from the projection plane that a point should have
																	 in order to belong on the extrusion silhouette */
	std::vector<CPointEx3D> general_extrusion_projected_points;	/**< The points that will define the extrusion's silhouette and are below the threshold */
	double sampling_distance;
	static ProError setPlanePointCount(int value);
	static ProError setCylinderPointCount(int value);
	static ProError SetCylinderTerminatingPointInfo(char* info);
	static ProError SetCylinderStartingPointInfo(char* info);
	static ProError SetGeneralExtrusionPlaneName(const char* string_name);
	static ProError SetGeneralExtrusionInnerPointInfo(const char* string_name);
	static ProError SetGeneralExtrusionExtentPointInfo(const char* info);
	static void CreatePlanePrimitive();
	static void CreateCylinderPrimitive();
	static void CreateGeneralExtrusion();
	static void on_create_primitive();
	static void on_accept_primitive();
	static void on_primitive_change_direction();
	static void on_point_cloud_load();
	
	//The primitives in the list;
	std::shared_ptr<RansacPrimitive> active_primitive;
	map<std::string, std::shared_ptr<RansacPrimitive>> primitive_list;
	map<int, std::shared_ptr<RansacPrimitive>> accepted_primitives;
	int accepted_planes_count;
	int accepted_cylinders_count;
	wchar_t** list_labels;
	char** list_names;
	int list_size;
	std::shared_ptr<RansacPrimitive> generalExtrusionReferencePlane;
	CPointEx3D general_extent_point;
	std::shared_ptr<BSplineSketch> pBSplineSketch;
	ProFeature feat; // the point cloud feature
public:
	ProFeature* getPointCloudFeature()
	{
		return &feat;
	}
	static double signed_distance_point_from_surface(const CPointEx3D& normal, const CPointEx3D& point_on_plane, const CPointEx3D& query_point);
	void readPointCloud(const char* filePath);
	std::shared_ptr<RansacPrimitive>& GetActivePrimitive();
	std::map<std::string, std::shared_ptr<RansacPrimitive>>& GetPrimitiveList();
	std::map<int, std::shared_ptr<RansacPrimitive>>& GetAcceptedPrimitiveList();
	wchar_t**& Get_List_Labels();
	char**& Get_List_Names();
	int& GetListSize();
	int& GetAcceptedPlanesCount();
	int& GetAcceptedCylindersCount();
	void primitiveListClear();
	void erasePointCloudFromCreo();
	static void UIListClear();
	static ProError setEpsilon(double value);
	static double getEpsilon(ProError& status);
	static ProError setBitmapEpsilon(double value);
	static double getBitmapEpsilon(ProError& status);
	static ProError setNormalThreshold(double value);
	static double getNormalThreshold(ProError& status);
	static ProError setMinimumSupport(int value);
	static int getMinimumSupport(ProError& status);
	static ProError setProbability(double value);
	static double getProbability(ProError& status);
	CPointEx3D& getGeneralExtentPoint();
	std::vector<CPointEx3D>& getBufferPntPlane();
	std::vector<CPointEx3D>& getBufferPntCylinder();
	std::shared_ptr<RansacPrimitive>& getGeneralProtrusionReferencePlane();
	std::vector<CPointEx3D>& getExternalProtrusionPointsToProject();
	double& getSamplingDensity()
	{
		return sampling_distance;
	}
	void CreateSectionOnPointCloud();
	void EraseBSplineSketch()
	{
		pBSplineSketch.reset();
	}
	ProFeature& getFeature()
	{
		return feat;
	}
	void unhighlighPointCloud();
	bool isInThreshold(const Pro3dPnt& pnt);
	double& getGeneralExtrusionDistanceThreshold();
	CPointEx3D& GetCylinderStartPoint();
	CPointEx3D& GetCylinderEndPoint();
	CPointEx3D& GetGeneralExtrusionInnerPoint();
	int& GetPointCloudInternalId();
	GuiInterface();
	static int GuiActivate();
	
	~GuiInterface();
};

#endif
