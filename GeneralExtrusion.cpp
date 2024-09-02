#include "GeneralExtrusion.h"

#include <MCLSEXST.H>
#include <NurbsLibSt.h>
#include <string>

#include "UtilMatrix.h"
#include <ProAxis.h>
#include "bcrust.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <boost/iterator/zip_iterator.hpp>

////////kd_tree//////
typedef CGAL::Exact_predicates_inexact_constructions_kernel	K;
typedef CGAL::Search_traits_2<K> Traits_base;
typedef K::Point_2                   Point_22;
typedef K::Vector_2                  Vector_22;
typedef boost::tuple<Point_22, int> Point_and_int;
typedef CGAL::Search_traits_adapter<Point_and_int,
	CGAL::Nth_of_tuple_property_map<0, Point_and_int>,
	Traits_base> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits>          K_neighbor_search;
typedef K_neighbor_search::Distance                         Distance;
typedef CGAL::Kd_tree<Traits> Tree;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point_22, Vector_22> CGALPointVectorPair;
typedef std::vector<CGALPointVectorPair> CGALPointList;

void GeneralExtrusion::GetPerpendicularVector(const Pro3dPnt& normal, Pro3dPnt& outVector)
{
	double max = fabs(normal[0]);
	int cordIndex = 0;

	if (max < fabs(normal[1]))
	{
		cordIndex = 1;
		max = fabs(normal[1]);
	}

	if (max < fabs(normal[2]))
	{
		cordIndex = 2;
	}
	outVector[0] = 1.0;
	outVector[1] = 1.0;
	outVector[2] = 1.0;

	switch (cordIndex)
	{
	case 0:
		outVector[0] = (-normal[1] * outVector[1] - normal[2] * outVector[2]) / normal[0];
		break;
	case 1:
		outVector[1] = (-normal[0] * outVector[0] - normal[2] * outVector[2]) / normal[1];
		break;
	case 2:
		outVector[2] = (-normal[0] * outVector[0] - normal[1] * outVector[1]) / normal[2];
		break;
	}
	NormalizeVector(outVector);
}

double getRadius(const std::vector<CPointEx>& points_in)
{
	std::vector<Point_22> points;
	std::vector<int> indices;
	for (int i = 0; i < points_in.size(); i++)
	{
		points.push_back(Point_22(points_in[i].x, points_in[i].y));
		indices.push_back(i);
	}
	Tree tree(boost::make_zip_iterator(boost::make_tuple(points.begin(), indices.begin())),
		boost::make_zip_iterator(boost::make_tuple(points.end(), indices.end())));

	Distance tr_dist;
	double meanRadius = 0.0;
	for (int i = 0; i < points.size(); i++) {
		K_neighbor_search search(tree, points[i], 20);
		K_neighbor_search::iterator it = search.begin();
		for (int j = 0; j < 19; j++) it++;
		Point_22 p1 = boost::get<0>(it->first);
		meanRadius += sqrt((points[i].x()-p1.x()) * (points[i].x() - p1.x()) + (points[i].y() - p1.y()) * (points[i].y() - p1.y()));
	}
	return meanRadius / points.size();
}

void GeneralExtrusion::NormalizeVector(Pro3dPnt& normal)
{
	double norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
	normal[0] /= norm; normal[1] /= norm; normal[2] /= norm;
}

void GeneralExtrusion::crossProduct(const Pro3dPnt& vect_A, const Pro3dPnt& vect_B, Pro3dPnt& cross_P)
{
	cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
	cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
	cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
	NormalizeVector(cross_P);
}

GeneralExtrusion::GeneralExtrusion(const ProPoint3d& normal, const Pro3dPnt& inner_point, const std::vector<CPointEx3D>& threshold_pnts, int sketch_plane_id,
	double depth, bool flipDirection, const char* name)
	:sketch_plane_id(sketch_plane_id)
{
	std::string extrusionName(name);
	std::string innerPointNane = extrusionName + "_Inner_Pnt";
	innerPnt = std::make_shared<CDatumPoint>(inner_point, innerPointNane.c_str());
	
	std::string orientationAxisNane = extrusionName + "_Orient_Axis";
	std::string orientationPntName = extrusionName + "_Orient_Pnt";
	Pro3dPnt orientationNormal;
	GetPerpendicularVector(normal, orientationNormal);
	Pro3dPnt point_on_normal;
	point_on_normal[0] = inner_point[0] + 0.1 * orientationNormal[0];
	point_on_normal[1] = inner_point[1] + 0.1 * orientationNormal[1];
	point_on_normal[2] = inner_point[2] + 0.1 * orientationNormal[2];
	orientationPnt = std::make_shared<CDatumPoint>(point_on_normal, orientationPntName.c_str());
	orientationAxis = std::make_shared<CDatumAxis>(*innerPnt, *orientationPnt, orientationAxisNane.c_str());
	orientationPlane = std::make_shared<CDatumPlane>(*orientationPnt, *orientationAxis, "orientation_plane");
	std::string cuttingAxisNane = extrusionName + "_Cutng_Axis";
	std::string cuttingPntNane = extrusionName  + "_Cutng_Pnt";
	Pro3dPnt cuttingNormal;
	crossProduct(normal, orientationNormal, cuttingNormal);
	point_on_normal[0] = inner_point[0] + 0.1 * cuttingNormal[0];
	point_on_normal[1] = inner_point[1] + 0.1 * cuttingNormal[1];
	point_on_normal[2] = inner_point[2] + 0.1 * cuttingNormal[2];
	cuttingPnt  = std::make_shared<CDatumPoint>(point_on_normal, cuttingPntNane.c_str());
	cuttingAxis = std::make_shared<CDatumAxis>(*innerPnt, *cuttingPnt, cuttingAxisNane.c_str());
	
	if (flipDirection) changeExtrusionDirectionBeforeCreation();
	createGeneralExtrusion(depth, threshold_pnts, name);
}

double GeneralExtrusion::distance(const CPointEx& pnt1, const CPointEx& pnt2)
{
	return sqrt((pnt1.x - pnt2.x)*(pnt1.x - pnt2.x) + (pnt1.y - pnt2.y)*(pnt1.y - pnt2.y));
}

ProError GeneralExtrusion::GeneralExtrusionSection(
	ProSection section,		/*In : the section */
	Parameter *params,
	int* extrusionId,
	int* orientation_plane_projection_id,
	int* cutting_plane_projection_id)
	/*     params [0] - threshold points */
	/*     params [1] - vertical axis */
	/*     params [2] - vertical axis */
{
	Pro2dSplinedef		spline;
	Pro2dEntdef			*ent;
	Pro2dLinedef		*line;
	ProError			err;
	ProWSecerror		errors;

	/*----------------------------------------------------------*\
	Create entities
	\*----------------------------------------------------------*/

	err = ProSectionEntityFromProjection(section, params[1].r, orientation_plane_projection_id);
	err = ProSectionEntityFromProjection(section, params[2].r, cutting_plane_projection_id);
	err = ProSectionEntityGet(section, *orientation_plane_projection_id, &ent);
	line = (Pro2dLinedef*)ent;

	ProMatrix location_matrix;
	ProMatrix inv;

	// Get the transformation matrix from Sketch -> World
	ProSectionLocationGet(section, location_matrix);
	// Get the transformation matrix from World -> Sketch
	ProUtilMatrixInvert(location_matrix, inv);

	std::vector<CPointEx3D>* thres_pnts = (std::vector<CPointEx3D>*)params[0].p;
	std::vector<CPointEx> projected_points;
	// transform from world -> sketch 
	for (const auto& pp : *thres_pnts)
	{
		Pro2dPnt projectedPointOnSketch;
		projectedPointOnSketch[0] = pp.x * inv[0][0] + pp.y * inv[1][0] + pp.z * inv[2][0] + inv[3][0];
		projectedPointOnSketch[1] = pp.x * inv[0][1] + pp.y * inv[1][1] + pp.z * inv[2][1] + inv[3][1];
		projected_points.push_back({projectedPointOnSketch[0], projectedPointOnSketch[1]});
	}
	std::vector<int> valid_points(projected_points.size(), 1);
	// Discard points below a threshold
	double radius = getRadius(projected_points);
	for (int i = 0; i < projected_points.size(); i++)
	{
		if (!valid_points[i]) continue;
		for (int j = i+1; j < projected_points.size(); j++)
		{
			if (!valid_points[j]) continue;
			if (distance(projected_points[i], projected_points[j]) < radius)
			{
				valid_points[j] = 0;
			}
		}
	}

	std::vector<CPointEx> valid_projected_points;
	for (int i = 0; i < projected_points.size(); i++)
	{
		if (valid_points[i])
		{
			valid_projected_points.push_back(projected_points[i]);
		}
	}

	// Calculate a polyline
	std::vector<std::vector<CPointEx>> poly_lines;
	// Calculate a polyline
	calculate_b_crust(valid_projected_points, poly_lines);
	for (auto& poly_line : poly_lines) {
		int n = poly_line.size();
		spline.n_points = n + 1;
		spline.tangency_type = PRO_2D_SPLINE_TAN_NONE;
		spline.point_arr = (Pro2dPnt*)malloc((n + 1) * sizeof(Pro2dPnt));
		spline.type = PRO_2D_SPLINE;
		Pro2dPnt d;
		for (int i = 0; i < n; i++)
		{
			d[0] = poly_line[i].x - line->end1[0];
			d[1] = poly_line[i].y - line->end1[1];
			spline.point_arr[i][0] = line->end1[0] + d[0];
			spline.point_arr[i][1] = line->end1[1] + d[1];
		}
		d[0] = poly_line[0].x - line->end1[0];
		d[1] = poly_line[0].y - line->end1[1];
		spline.point_arr[n][0] = line->end1[0] + d[0];
		spline.point_arr[n][1] = line->end1[1] + d[1];
		err = ProSectionEntityAdd(section, (Pro2dEntdef*)&spline, extrusionId);
	}
	/*----------------------------------------------------------*\
	Solve section
	\*----------------------------------------------------------*/
	err = ProSecerrorAlloc(&errors);
	err = ProSectionAutodim(section, &errors);
	err = ProSecerrorFree(&errors);

	return (err);
}

ProError GeneralExtrusion::GeneralExtrusionSectionPhilipp(
	ProSection section,		/*In : the section */
	Parameter *params,
	int* extrusionId,
	int* orientation_plane_projection_id,
	int* cutting_plane_projection_id)
	/*     params [0] - threshold points */
	/*     params [1] - vertical surface */
	/*     params [2] - vertical surface */
{
	Pro2dSplinedef		spline;
	Pro2dEntdef			*ent1, *ent2;
	Pro2dLinedef		*line1, *line2;
	ProError			err;
	ProWSecerror		errors;
	Pro2dBSplinedef		bSpline;

	/*----------------------------------------------------------*\
	Create entities
	\*----------------------------------------------------------*/
	
	err = ProSectionEntityFromProjection(section, params[1].r, orientation_plane_projection_id);
	err = ProSectionEntityFromProjection(section, params[2].r, cutting_plane_projection_id);
	err = ProSectionEntityGet(section, *orientation_plane_projection_id, &ent1);
	line1 = (Pro2dLinedef*)ent1;
	err = ProSectionEntityGet(section, *cutting_plane_projection_id, &ent2);
	line2 = (Pro2dLinedef*)ent2;

	ProMatrix location_matrix;
	ProMatrix inv;

	// Get the transformation matrix from Sketch -> World
	ProSectionLocationGet(section, location_matrix);
	// Get the transformation matrix from World -> Sketch
	ProUtilMatrixInvert(location_matrix, inv);

	std::vector<CPointEx3D>* thres_pnts = (std::vector<CPointEx3D>*)params[0].p;
	std::vector<CPointEx> projected_points;
	// transform from world -> sketch 
	for (const auto& pp : *thres_pnts)
	{
		Pro2dPnt projectedPointOnSketch;
		projectedPointOnSketch[0] = pp.x * inv[0][0] + pp.y * inv[1][0] + pp.z * inv[2][0] + inv[3][0];
		projectedPointOnSketch[1] = pp.x * inv[0][1] + pp.y * inv[1][1] + pp.z * inv[2][1] + inv[3][1];
		projected_points.push_back({ projectedPointOnSketch[0], projectedPointOnSketch[1] });
	}
	std::vector<int> valid_points(projected_points.size(), 1);
	// Discard points below a threshold
	for (int i = 0; i < projected_points.size(); i++)
	{
		if (!valid_points[i]) continue;
		for (int j = i + 1; j < projected_points.size(); j++)
		{
			if (!valid_points[j]) continue;
			if (distance(projected_points[i], projected_points[j]) < 0.01) // allow denser cloud
			{
				valid_points[j] = 0;
			}
		}
	}

	std::vector<CPointEx> valid_projected_points;
	for (int i = 0; i < projected_points.size(); i++)
	{
		if (valid_points[i])
		{
			valid_projected_points.push_back(projected_points[i]);
		}
	}

	// Filip you need to replace from here down for your Bspline
	// using Pro2dBSplinedef data structure and define its values with your software.
	// The points to be interpolated are valid_projected_points 2D points.
	// The approach used now is not industrial since when noise is present we need
	// your least squares approach.

	// We shall use b_crust to create a "polyline" structure of the datapoints
	// Then we shall use LQR approx to define theg curve approximation to this polyline with TOL accuracy.

	std::vector<std::vector<CPointEx>> poly_lines;
	// Calculate a polyline
	
	calculate_b_crust(valid_projected_points, poly_lines);
	for (auto& poly_line : poly_lines) {
		int n = poly_line.size();
		spline.n_points = n + 1;
		spline.tangency_type = PRO_2D_SPLINE_TAN_NONE;
		spline.point_arr = (Pro2dPnt*)malloc((n + 1) * sizeof(Pro2dPnt));
		spline.type = PRO_2D_SPLINE;
		Pro2dPnt d;
		CPointExVector dataPoints; // for NURBS lib
		for (int i = 0; i < n; i++)
		{
			d[0] = poly_line[i].x - line1->end1[0];
			d[1] = poly_line[i].y - line2->end1[1];
			spline.point_arr[i][0] = line1->end1[0] + d[0];
			spline.point_arr[i][1] = line2->end1[1] + d[1];
			dataPoints.Add(CPointEx(spline.point_arr[i][0], spline.point_arr[i][1]));
		}
		d[0] = poly_line[0].x - line1->end1[0];
		d[1] = poly_line[0].y - line2->end1[1];
		spline.point_arr[n][0] = line1->end1[0] + d[0];
		spline.point_arr[n][1] = line2->end1[1] + d[1];
		dataPoints.Add(CPointEx(spline.point_arr[n][0], spline.point_arr[n][1]));
		//	err = ProSectionEntityAdd(section, (Pro2dEntdef*)&spline, extrusionId);

		// define curve2D
		CCurve2D curve2D;
		int nDegree = 2,
			nInitQtrlPoints = n / 4 + 1, // will xcompute one forth of data points as control points
			nQtrlPoints = nInitQtrlPoints - 1;
		double TOL = 1e-3; // this should be on GUI
		CPointExVector QtrlPoints(nInitQtrlPoints);	QtrlPoints.SetVal(CPointEx(spline.point_arr[0][0], spline.point_arr[0][1]));
		/*	bool bRes = curve2D.Init(2, QtrlPoints);
			std::string str = string_format("curve2D.Init = %s\n\nData Points = %d\nDegree = %d\nInit CtrlPoints = %d\nFinal CtrlPoints = %d\nNum Knots = %d\nTOL = %.6f",
				bRes ? "Succeed!" : "Failed!", dataPoints.GetSize(), curve2D.GetDegree(), nInitQtrlPoints, curve2D.GetCtrlDim(), curve2D.GetKnotDim(), TOL);
			MsgBoxInfo(str);
			*/

		bool bRes = curve2D.InitLSQApprox(nDegree, nQtrlPoints, dataPoints, TOL);

		/*std::string str = string_format("curve2D.InitLSQApprox = %s\n\nData Points = %d\nDegree = %d\nInit CtrlPoints = %d\nFinal CtrlPoints = %d\nNum Knots = %d\nTOL = %.6f",
			bRes ? "Succeed!" : "Failed!", dataPoints.GetSize(), curve2D.GetDegree(), nInitQtrlPoints, curve2D.GetCtrlDim(), curve2D.GetKnotDim(), TOL);*/
			//MsgBoxInfo(str);


			// define a ProBSpline and copy data
		bSpline.degree = curve2D.GetDegree();
		bSpline.num_c_points = curve2D.GetCtrlDim();
		bSpline.num_knots = curve2D.GetKnotDim();
		bSpline.weights = NULL;
		bSpline.type = PRO_2D_B_SPLINE;
		bSpline.c_pnts = new Pro2dPnt[bSpline.num_c_points];
		for (int i = 0; i < bSpline.num_c_points; i++)
		{
			bSpline.c_pnts[i][0] = curve2D.m_ctrPoints[i].x;
			bSpline.c_pnts[i][1] = curve2D.m_ctrPoints[i].y;
		}

		bSpline.params = new double[bSpline.num_knots];
		for (int i = 0; i < bSpline.num_knots; i++)
		{
			bSpline.params[i] = curve2D.GetKnotU()[i];
		}

		// add bspline to section
		err = ProSectionEntityAdd(section, (Pro2dEntdef*)&bSpline, extrusionId);
		//str = string_format("%s\nProSectionEntityAdd = %d", err == 0 ? "No error!" : "Error!", err);
		//MsgBoxInfo(str);
	}

	/*----------------------------------------------------------*\
	Solve section
	\*----------------------------------------------------------*/
	err = ProSecerrorAlloc(&errors);
	err = ProSectionAutodim(section, &errors);
	err = ProSecerrorFree(&errors);

	return (err);
}
void GeneralExtrusion::createGeneralExtrusion(double depth, const std::vector<CPointEx3D>& pnts, const char* name)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	setTheDrawingPlaneReference(model, sketch_plane_id);
	setTheDrawingPlaneOrientation(model, orientationPlane->GetId());
	
	setExtrusionDepth(depth);
	setDrawingFunction(GeneralExtrusionSection);
	
	Parameter params[3];
	params[0].p = (void*)&pnts;

	ProAxis axis1;
	ProModelitem modelitem1;
	ProError err = ProAxisInit((ProSolid)model, orientationAxis->GetAxisId(), &axis1);
	err = ProAxisToGeomitem((ProSolid)model, axis1, (ProGeomitem*)&modelitem1);
	err = ProSelectionAlloc(NULL, &modelitem1, &params[2].r);

	ProAxis axis2;
	ProModelitem modelitem2;
	err = ProAxisInit((ProSolid)model, cuttingAxis->GetAxisId(), &axis2);
	err = ProAxisToGeomitem((ProSolid)model, axis2, (ProGeomitem*)&modelitem2);
	err = ProSelectionAlloc(NULL, &modelitem2, &params[1].r);
	createExtrusion(model, params, &sketch_id, name, &orientation_id, &cutting_id);
	status = ProSelectionFree(&params[1].r);
	status = ProSelectionFree(&params[2].r);
}
