#include <string>
#include "UtilityHelperClass.h"
#include "Cylinder.h"
#include "UtilMatrix.h"

double CylindricalExtrusion::innerProduct(const Pro3dPnt& vec1, const Pro3dPnt& vec2)
{
	return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}

void CylindricalExtrusion::projection(const Pro3dPnt& start, const Pro3dPnt& axis, const Pro3dPnt& vecin, Pro3dPnt& vecOut)
{
	Pro3dPnt vec;
	vec[0] = vecin[0] - start[0];
	vec[1] = vecin[1] - start[1];
	vec[2] = vecin[2] - start[2];
	double t = innerProduct(vec, axis);
	vecOut[0] = start[0] + t * axis[0];
	vecOut[1] = start[1] + t * axis[1];
	vecOut[2] = start[2] + t * axis[2];
}

CylindricalExtrusion::CylindricalExtrusion(const Pro3dPnt& normalCylinder, double radius,
	const Pro3dPnt& pnt1, const Pro3dPnt& pnt2, const Pro3dPnt& pointOnAxis, const char* name)
{
	// normalize normal
	Pro3dPnt normal;
	normal[0] = normalCylinder[0]; normal[1] = normalCylinder[1]; normal[2] = normalCylinder[2];
	NormalizeVector(normal);
	
	// we will draw the cylinder as an extrusion in the direction of the normal
	Pro3dPnt pnt12;
	pnt12[0] = pnt2[0] - pnt1[0];
	pnt12[1] = pnt2[1] - pnt1[1];
	pnt12[2] = pnt2[2] - pnt1[2];
	Pro3dPnt pntOnPlane;
	pntOnPlane[0] = pnt1[0];
	pntOnPlane[1] = pnt1[1];
	pntOnPlane[2] = pnt1[2];
	if (pnt12[0] * normal[0] + pnt12[1] * normal[1] + pnt12[2] * normal[2] < 0)
	{
		pntOnPlane[0] = pnt2[0];
		pntOnPlane[1] = pnt2[1];
		pntOnPlane[2] = pnt2[2];
	}

	std::string cylinderName(name);
	std::string sketchNane = cylinderName + "_Sketch_Plane";
	drawingPlane = std::make_shared<CDatumPlane>(normal, pntOnPlane, sketchNane.c_str());

	std::string innerPointName = cylinderName + "_Inner_Pnt";
	datumPointOnAxis = std::make_shared<CDatumPoint>(pointOnAxis, innerPointName.c_str());

	std::string orientationAxisNane = cylinderName + "_Orient_Axis";
	std::string orientationPntName = cylinderName + "_Orient_Pnt";
	Pro3dPnt orientationNormal;
	GetPerpendicularVector(normal, orientationNormal);
	Pro3dPnt point_on_normal;
	point_on_normal[0] = pointOnAxis[0] + 0.1 * orientationNormal[0];
	point_on_normal[1] = pointOnAxis[1] + 0.1 * orientationNormal[1];
	point_on_normal[2] = pointOnAxis[2] + 0.1 * orientationNormal[2];
	orientationPnt = std::make_shared<CDatumPoint>(point_on_normal, orientationPntName.c_str());
	orientationAxis = std::make_shared<CDatumAxis>(*datumPointOnAxis, *orientationPnt, orientationAxisNane.c_str());

	std::string cuttingAxisNane = cylinderName + "_Cutng_Axis";
	std::string cuttingPntNane = cylinderName  + "_Cutng_Pnt";
	Pro3dPnt cuttingNormal;
	crossProduct(normal, orientationNormal, cuttingNormal);
	point_on_normal[0] = pointOnAxis[0] + 0.1 * cuttingNormal[0];
	point_on_normal[1] = pointOnAxis[1] + 0.1 * cuttingNormal[1];
	point_on_normal[2] = pointOnAxis[2] + 0.1 * cuttingNormal[2];
	cuttingPnt = std::make_shared<CDatumPoint>(point_on_normal, cuttingPntNane.c_str());
	cuttingAxis = std::make_shared<CDatumAxis>(*datumPointOnAxis, *cuttingPnt, cuttingAxisNane.c_str());
	std::string orientationPlaneName = cylinderName + "_Orient_Plane";
	orientationPlane = std::make_shared<CDatumPlane>(*orientationPnt, *orientationAxis, orientationPlaneName.c_str());
	double norm1 = innerProduct(normal, orientationNormal);
	double norm2 = innerProduct(normal, cuttingNormal);
	double norm3 = innerProduct(orientationNormal, cuttingNormal);
	
	Pro3dPnt pnt1Projection;
	projection(pointOnAxis, normal, pnt1, pnt1Projection);
	Pro3dPnt pnt2Projection;
	projection(pointOnAxis, normal, pnt2, pnt2Projection);
	Pro3dPnt vec;
	vec[0] = pnt2Projection[0] - pnt1Projection[0];
	vec[1] = pnt2Projection[1] - pnt1Projection[1];
	vec[2] = pnt2Projection[2] - pnt1Projection[2];
	double length = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
	createCylinder(length, radius, name);
}

void CylindricalExtrusion::GetPerpendicularVector(const Pro3dPnt& normal, Pro3dPnt& outVector)
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

void CylindricalExtrusion::NormalizeVector(Pro3dPnt& normal)
{
	double norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
	normal[0] /= norm; normal[1] /= norm; normal[2] /= norm;
}

void CylindricalExtrusion::crossProduct(const Pro3dPnt& vect_A, const Pro3dPnt& vect_B, Pro3dPnt& cross_P)
{
	cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
	cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
	cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
	NormalizeVector(cross_P);
}

bool CylindricalExtrusion::LineIntersection(Pro2dPnt A, Pro2dPnt B, Pro2dPnt C, Pro2dPnt D, Pro2dPnt I)
{
	// Line AB represented as a1x + b1y = c1 
	double a1 = B[1] - A[1];
	double b1 = A[0] - B[0];
	double c1 = a1 * A[0] + b1 * A[1];

	// Line CD represented as a2x + b2y = c2 
	double a2 = D[1] - C[1];
	double b2 = C[0] - D[0];
	double c2 = a2 * C[0] + b2 * C[1];

	double determinant = a1 * b2 - a2 * b1;

	if (determinant == 0.0)
	{
		// The lines are parallel.
		return false;
	}

	I[0] = (b2*c1 - b1 * c2) / determinant;
	I[1] = (a1*c2 - a2 * c1) / determinant;
	return true;
}

ProError CylindricalExtrusion::CircleSection(
	ProSection section,		/*In : the section */
	Parameter *params,
	int* circleId,
	int* orientation_plane_projection_id, 
	int* cutting_plane_projection_id)
	/*     params [0] - radius */
	/*     params [1] - vertical surface */
	/*     params [2] - vertical surface */
{
	Pro2dCircledef		circle;
	Pro2dEntdef			*ent;
	Pro2dLinedef		*line;
	ProError			err;
	ProWSecerror		errors;
	
	/*----------------------------------------------------------*\
	Create entities
	\*----------------------------------------------------------*/
	circle.type = PRO_2D_CIRCLE;
	
	err = ProSectionEntityFromProjection(section, params[1].r, orientation_plane_projection_id);
	err = ProSectionEntityFromProjection(section, params[2].r, cutting_plane_projection_id);
	err = ProSectionEntityGet(section, *orientation_plane_projection_id, &ent);
	line = (Pro2dLinedef*)ent;
	
	circle.center[0] = line->end1[0];
	circle.center[1] = line->end1[1];
	circle.radius = params[0].d;

	err = ProSectionEntityAdd(section, (Pro2dEntdef*)&circle, circleId);
		
	/*----------------------------------------------------------*\
	Solve section
	\*----------------------------------------------------------*/
	err = ProSecerrorAlloc(&errors);
	err = ProSectionAutodim(section, &errors);
	err = ProSecerrorFree(&errors);
	
	return (err);
}

void CylindricalExtrusion::createCylinder(double height, double radius, const char* name)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	setTheDrawingPlaneReference(model, drawingPlane->GetId());
	setTheDrawingPlaneOrientation(model, orientationPlane->GetId());
	setExtrusionDepth(height);
	setDrawingFunction(CircleSection);
	Parameter params[3];
	params[0].d = radius;
	ProAxis axis1;
	ProModelitem modelitem1;

	ProError err = ProAxisInit((ProSolid)model, orientationAxis->GetAxisId(), &axis1);
	err= ProAxisToGeomitem((ProSolid)model, axis1, (ProGeomitem*)&modelitem1);
	err = ProSelectionAlloc(NULL, &modelitem1, &params[2].r);
	
	ProAxis axis2;
	ProModelitem modelitem2;

	err = ProAxisInit((ProSolid)model, cuttingAxis->GetAxisId(), &axis2);
	err = ProAxisToGeomitem((ProSolid)model, axis2, (ProGeomitem*)&modelitem2);
	err = ProSelectionAlloc(NULL, &modelitem2, &params[1].r);
	createExtrusion(model, params, &circleId, name, &orientation_plane_projection_id, &cutting_plane_projection_id);
	status = ProSelectionFree(&params[1].r);
	status = ProSelectionFree(&params[2].r);
}

void CylindricalExtrusion::SetName(const char* name)
{
	std::string sketchName = std::string(name) + "_Sketch_Plane";
	drawingPlane->SetName(sketchName.c_str());

	std::string orientationName = std::string(name) + "_Orient_Plane";
	//orientationPlane->SetName(orientationName.c_str());

	std::string cuttingName = std::string(name) + "_Cutng_Plane";
	//cuttingPlane->SetName(cuttingName.c_str());

	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProModelitem modelitem;
	status = ProModelitemInit(model, extrusionId, PRO_FEATURE, &modelitem);
	ProName itemName;
	ProStringToWstring(itemName, (char*)name);
	status = ProModelitemNameSet(&modelitem, itemName);
}
