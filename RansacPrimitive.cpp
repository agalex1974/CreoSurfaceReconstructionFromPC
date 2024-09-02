#include "HelperStructures.h"
#include "RansacPrimitive.h"

RansacPlanePrimitive::RansacPlanePrimitive(const CPointEx3D& normal, double d, const CPointEx3D& near_point):
	RansacPrimitive(PLANE),
	normal(normal),
	d(d),
	near_point(near_point)
{}

void RansacPlanePrimitive::Create(const char* name)
{
	Pro3dPnt normalPro;
	normalPro[0] = normal.x;
	normalPro[1] = normal.y;
	normalPro[2] = normal.z;
	Pro3dPnt nearPoint;
	nearPoint[0] = near_point.x;
	nearPoint[1] = near_point.y;
	nearPoint[2] = near_point.z;
	datumPlane = std::make_unique<CDatumPlane>(nearPoint, normalPro, d, name);
}

CPointEx3D RansacPlanePrimitive::GetPointOnPlane() const
{
	double max = fabs(normal.x);
	int cordIndex = 0;

	if (max < fabs(normal.y))
	{
		cordIndex = 1;
		max = fabs(normal.y);
	}

	if (max < fabs(normal.z))
	{
		cordIndex = 2;
	}
	CPointEx3D outPnt;
	outPnt.x = near_point.x;
	outPnt.y = near_point.y;
	outPnt.z = near_point.z;

	switch (cordIndex)
	{
	case 0:
		outPnt.x = (-normal.y * outPnt.y - normal.z * outPnt.z - d) / normal.x;
		break;
	case 1:
		outPnt.y = (-normal.x * outPnt.x - normal.z * outPnt.z - d) / normal.y;
		break;
	case 2:
		outPnt.z = (-normal.x * outPnt.x - normal.y * outPnt.y - d) / normal.z;
		break;
	}
	return outPnt;
}

int RansacPlanePrimitive::GetPrimitiveCreoId()
{
	if (datumPlane) return datumPlane->GetInternalId();
	return -1;
}

int RansacPlanePrimitive::GetPrimitiveFeatureId()
{
	if (datumPlane) return datumPlane->GetId();
	return -1;
}

const CPointEx3D& RansacPlanePrimitive::GetNormal()
{
	return normal;
}

void RansacPlanePrimitive::Destroy()
{
	datumPlane.reset();
}

void RansacPlanePrimitive::SetName(const char* name)
{
	datumPlane->SetName(name);
}

RansacCylinderPrimitive::RansacCylinderPrimitive(const CPointEx3D& axisDirection, const CPointEx3D& axisPosition,
	const CPointEx3D& start_point, const CPointEx3D& end_point, double radius):
	RansacPrimitive(CYLINDER),
	axisDirection(axisDirection),
	axisPosition(axisPosition),
	start_point(start_point),
	end_point(end_point),
	radius(radius)
{}

void RansacCylinderPrimitive::Create(const char* name)
{
	Pro3dPnt normal;
	Pro3dPnt start;
	Pro3dPnt finish;
	Pro3dPnt pntOnAxis;
	normal[0] = axisDirection.x; normal[1] = axisDirection.y; normal[2] = axisDirection.z;
	pntOnAxis[0] = axisPosition.x; pntOnAxis[1] = axisPosition.y; pntOnAxis[2] = axisPosition.z;
	start[0] = start_point.x; start[1] = start_point.y; start[2] = start_point.z;
	finish[0] = end_point.x; finish[1] = end_point.y; finish[2] = end_point.z;
	cylinder = std::make_unique<CylindricalExtrusion>(normal, radius, start, finish, pntOnAxis, name);
}

void RansacCylinderPrimitive::Destroy()
{
	cylinder.reset();
}

void RansacCylinderPrimitive::SetName(const char* name)
{
	cylinder->SetName(name);
}

void RansacCylinderPrimitive::ChangeDirection()
{
	cylinder->changeDirectionOfExtrusion();
}

int RansacCylinderPrimitive::GetPrimitiveCreoId()
{
	if (cylinder) return cylinder->getExtrusionInternalId();
	return -1;
}

int RansacCylinderPrimitive::GetPrimitiveFeatureId()
{
	if (cylinder) return cylinder->getExtrusionId();
	return -1;
}

ExtrudePrimitive::ExtrudePrimitive(const CPointEx3D& normal, const CPointEx3D& inner_point, const std::vector<CPointEx3D>& threshold_pnts, int sketch_plane_id,
	bool flipDirection, double Depth):
	RansacPrimitive(GENERAL),
	normal(normal),
	inner_point(inner_point),
	threshold_pnts(threshold_pnts),
	sketch_plane_id(sketch_plane_id),
	flipDirection(flipDirection),
	depth(Depth)
{}

void ExtrudePrimitive::Create(const char* name)
{
	Pro3dPnt proNormal;
	proNormal[0] = normal.x; proNormal[1] = normal.y; proNormal[2] = normal.z;
	Pro3dPnt proInnerPnt;
	proInnerPnt[0] = inner_point.x; proInnerPnt[1] = inner_point.y; proInnerPnt[2] = inner_point.z;
	generalExtrusion = std::make_unique<GeneralExtrusion>(proNormal, proInnerPnt, threshold_pnts, sketch_plane_id, depth, flipDirection, name);
}

void ExtrudePrimitive::Destroy()
{
	generalExtrusion.reset();
}

void ExtrudePrimitive::SetName(const char* name)
{
	//ExtrudePrimitive->SetName(name);
}

void ExtrudePrimitive::ChangeDirection()
{
	if (generalExtrusion) generalExtrusion->changeDirectionOfExtrusion();
}

int ExtrudePrimitive::GetPrimitiveCreoId()
{
	if (generalExtrusion) return generalExtrusion->getExtrusionInternalId();
	return -1;
}

int ExtrudePrimitive::GetPrimitiveFeatureId()
{
	if (generalExtrusion) return generalExtrusion->getExtrusionId();
	return -1;
}
