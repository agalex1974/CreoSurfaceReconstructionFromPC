#ifndef CCREOPLANE_H
#define CCREOPLANE_H

#include "NurbsLibSt.h"
#include "CDatumPlane.h"

class CCreoPlane: public CDatumPlane
{
private:
	CPlane cPlane;
public:
	CCreoPlane(const Pro3dPnt& normal, const Pro3dPnt& pntOnPlane, const char* name);
	CCreoPlane(const CPlane& cplane, const CPointEx3D& pointNearPlane, char* name);
	const CPlane& getCPlane() const;
};

#endif
