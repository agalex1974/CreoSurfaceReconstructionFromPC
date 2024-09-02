#include "CCreoPlane.h"

CCreoPlane::CCreoPlane(const Pro3dPnt& normal, const Pro3dPnt& pntOnPlane, const char* name):
	CDatumPlane(normal, pntOnPlane, name),
	cPlane()
{
	CPointEx3D cNormal(normal[0], normal[1], normal[2]);
	CPointEx3D cPoint(pntOnPlane[0], pntOnPlane[1], pntOnPlane[2]);
	cPlane.InitPointNormal(cPoint, cNormal);
}

CCreoPlane::CCreoPlane(const CPlane& cplane, const CPointEx3D& pointNearPlane, char* name):
	CDatumPlane(pointNearPlane[0], pointNearPlane[1], pointNearPlane[2], cplane.GetPlaneNormal()[0], cplane.GetPlaneNormal()[1], cplane.GetPlaneNormal()[2], cplane.GetOrigin(), name),
	cPlane(cplane)
{}

const CPlane& CCreoPlane::getCPlane() const
{
	return cPlane;
}