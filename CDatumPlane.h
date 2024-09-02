#ifndef DATUM_PLANE_H
#define DATUM_PLANE_H

#ifndef DECLSPEC 
#define DECLSPEC 
#endif

#include <iostream>
#include <memory>
#include <string>
#include <ProToolkit.h>
#include <ProMdl.h>
#include <ProModelitem.h>
#include <ProGeomitem.h>
#include <ProCsys.h>
#include <ProSolid.h>
#include <ProFeature.h>
#include <ProFeatType.h>
#include <ProDtmPnt.h>
#include <ProUtil.h>
#include <ProAxis.h>
#include "HelperStructures.h"
#include "CDatumPoint.h"
#include "CDatumAxis.h"

class CDatumPlane
{
public:
	CDatumPlane(const CDatumPoint& pnt, const CDatumAxis& axis, const char* name);
	//pnt: the point near the plane
	//normal: the normal of the plane (a, b, c)
	//d: the signed distance.
	//the equation of the plane: a*x + b*y + c*z + d = 0
	CDatumPlane(const Pro3dPnt& pnt, const Pro3dPnt& normal, double d, const char* name);
	CDatumPlane(double x, double y, double z, double nx, double ny, double nz, double d, const char* name);
	//pnt: Here we assume the point(0, 0, 0)
	//normal: the normal of the plane (a, b, c)
	//d: the signed distance.
	//the equation of the plane: a*x + b*y + c*z + d = 0
	CDatumPlane(const Pro3dPnt& normal, double d, const char* name);
	//normal: the normal of the plane (a, b, c)
	//the point on the plane
	CDatumPlane(const Pro3dPnt& normal, const Pro3dPnt& pntOnPlane, const char* name);

	/**
	 * Delete the current plane from Creo
	 */
	void DeletePlane();
	~CDatumPlane();
	/**
	 * Get the plane Id 
	 *
	 * @return the plane Id
	 */
	int GetId() const;
	/**
	 * Get the plane axis Id that defines it
	 *
	 * @return the axis intern Id
	 */
	int GetNormalId() const;
	/**
	 * Get the plane Id that is internal in Creo's UI
	 *
	 * @return the axis intern Id
	 */
	int GetInternalId() const;
	/**
	 * Set the name of the datum plane as displayed in the UI
	 *
	 * @param name The name of the datum plane as displayed in the UI
	 */
	void SetName(const char* name);
	void printPlaneEquation(ProVector& normal);
private:
	/**
	 * Define the Creo primitives that create the datum plane
	 *
	 * @param pnt The point which belong on the plane
	 * @param normal The normal to the plane
	 * @param d The distance from the origin
	 * @param name The name of the plane
	 */
	void definePrimitives(const Pro3dPnt& pnt, const Pro3dPnt& normal, double d, const char* name);
	/**
	 * Get the point on the plane given a point close to the plane
	 *
	 * @param pnt The point close to the plane
	 * @param normal The normal to the plane
	 * @param d The distance from the origin
	 * @param outPnt The point on the plane
	 */
	void GetPointOnPlane(const Pro3dPnt& pnt, const Pro3dPnt& normal, double d, Pro3dPnt outPnt) const;
	// pnt1 the point that the  
	std::unique_ptr<CDatumPoint> pnt1;				/**< The point on the plane */
	std::unique_ptr<CDatumPoint> pnt2;				/**< A point on the direction of the normal */
	std::unique_ptr<CDatumAxis> axis;		/**< The axis that is the normal to the plane */
	ElemTreeData dtm_tree[9] = {
		/* 0 */ {0, PRO_E_FEATURE_TREE, {(ProValueDataType)-1}},
		/* 1 */ {1, PRO_E_FEATURE_TYPE, {PRO_VALUE_TYPE_INT, PRO_FEAT_DATUM}},
		/* 2 */ {1, PRO_E_DTMPLN_CONSTRAINTS, {(ProValueDataType)-1}},
		/* 3 */ {2, PRO_E_DTMPLN_CONSTRAINT, {(ProValueDataType)-1}},
		/* 4 */ {3, PRO_E_DTMPLN_CONSTR_TYPE, {PRO_VALUE_TYPE_INT, PRO_DTMPLN_NORM}},
		/* 5 */ {3, PRO_E_DTMPLN_CONSTR_REF, {PRO_VALUE_TYPE_SELECTION, NULL}},
		/* 6 */ {2, PRO_E_DTMPLN_CONSTRAINT, {(ProValueDataType)-1}},
		/* 7 */ {3, PRO_E_DTMPLN_CONSTR_TYPE, {PRO_VALUE_TYPE_INT, PRO_DTMPLN_THRU}},
		/* 8 */ {3, PRO_E_DTMPLN_CONSTR_REF, {PRO_VALUE_TYPE_SELECTION}},
	};
	/**
	 * Define the primitives in the feature tree
	 *
	 * @param pnt The point on the plane
	 * @param axis The normal of the plane
	 */
	void definePrimitive(const CDatumPoint& pnt, const CDatumAxis& axis);
	/**
	 * Create the plane feature
	 *
	 * @param name The name of the feature
	 */
	int datumPlaneInit(const char* name);
	
	int plane_id;				/**< The point feature Id */
	int plane_internal_id;		/**< The point's Creo internal Id */
	ProPoint3d normalTest;
};

#endif

