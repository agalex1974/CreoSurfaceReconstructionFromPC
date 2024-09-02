#ifndef RANSAC_PRIMITIVES
#define RANSAC_PRIMITIVES

#include <memory>
#include <vector>
#include <ProFeature.h>
#include <ProFeatType.h>
#include <ProDtmPnt.h>
#include "HelperStructures.h"
#include "CDatumPoint.h"
#include "CDatumAxis.h"
#include "CDatumPlane.h"
#include "Cylinder.h"
#include "GeneralExtrusion.h"

class RansacPrimitive
{
public:
	enum PrimitiveType
	{
		PLANE,
		CYLINDER,
		GENERAL
	};
	RansacPrimitive(PrimitiveType type):
		type(type){}
	virtual ~RansacPrimitive(){};
	virtual void Create(const char* name) = 0;
	virtual void Destroy() = 0;
	virtual void SetName(const char* name) = 0;
	virtual void ChangeDirection(){}
	virtual int GetPrimitiveCreoId() = 0;
	virtual int GetPrimitiveFeatureId() = 0;
	PrimitiveType GetType()
	{
		return type;
	};
protected:
	PrimitiveType type;
};

class RansacPlanePrimitive: public RansacPrimitive
{
public:
	RansacPlanePrimitive(const CPointEx3D& normal, double d, const CPointEx3D& near_point);
	void Create(const char* name) override;
	void Destroy() override;
	void SetName(const char* name) override;
	int GetPrimitiveCreoId() override;
	int GetPrimitiveFeatureId() override;
	const CPointEx3D& GetNormal();
	CPointEx3D GetPointOnPlane() const;
protected:
	std::unique_ptr<CDatumPlane> datumPlane;
	CPointEx3D normal;
	CPointEx3D near_point;
	double d;
};

//cylinder.AxisDirection()[0], cylinder.AxisDirection()[1], cylinder.AxisDirection()[2],
//cylinder.AxisPosition()[0], cylinder.AxisPosition()[1], cylinder.AxisPosition()[2], cylinder.Radius()

class RansacCylinderPrimitive : public RansacPrimitive
{
public:
	RansacCylinderPrimitive(const CPointEx3D& axisDirection, const CPointEx3D& axisPosition, 
		const CPointEx3D& start_point, const CPointEx3D& end_point, double radius);
	void Create(const char* name) override;
	void Destroy() override;
	void SetName(const char* name) override;
	void ChangeDirection() override;
	int GetPrimitiveCreoId() override;
	int GetPrimitiveFeatureId() override;
protected:
	std::unique_ptr<CylindricalExtrusion> cylinder;
	CPointEx3D axisDirection;
	CPointEx3D axisPosition;
	CPointEx3D start_point;
	CPointEx3D end_point;
	double radius;
};

class ExtrudePrimitive : public RansacPrimitive
{
public:
	ExtrudePrimitive(const CPointEx3D& normal, const CPointEx3D& inner_point, const std::vector<CPointEx3D>& threshold_pnts,
		int sketch_plane_id, bool flipDirection, double depth);
	void Create(const char* name) override;
	void Destroy() override;
	void SetName(const char* name) override;
	void ChangeDirection() override;
	int GetPrimitiveCreoId() override;
	int GetPrimitiveFeatureId() override;
private:
	std::unique_ptr<GeneralExtrusion> generalExtrusion;
	CPointEx3D normal;
	CPointEx3D inner_point;
	std::vector<CPointEx3D> threshold_pnts;
	int sketch_plane_id;
	bool flipDirection;
	double depth;
};
#endif