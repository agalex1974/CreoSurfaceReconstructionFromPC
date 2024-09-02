#ifndef CYLINDER_H
#define CYLINDER_H

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

#include "Extrusion.h"
#include "CDatumPoint.h"
#include "CDatumAxis.h"
#include "CDatumPlane.h"

class CylindricalExtrusion : public Extrusion
{
public:
	/**
	 * Constructor of the Cylindrical Extrusion
	 *
	 * @param normal The axis directional normal
	 * @param radius The radius of the cylindrical extrusion
	 * @param pnt1 The first point extent of the cylindrical extrusion
	 * @param pnt2 The second point extent of the cylindrical extrusion
	 * @param pointOnAxis A 3D point on the Axis
	 * @param name The name of the extrusion
	 */
	CylindricalExtrusion(const Pro3dPnt& normal, double radius,
		const Pro3dPnt& pnt1, const Pro3dPnt& pnt2, const Pro3dPnt& pointOnAxis, const char* name);
	/**
	 * Set the name of the Extrusion
	 *
	 * @param name The name of the extrusion
	 */
	void SetName(const char* name);
	
private:
	/**
	 * The function callback to draw a circle on the section plane to be used as the extrusion sketched feature
	 *
	 * @param section The sketch plane to draw
	 * @param params The parameters needed for sketching the circle section
	 * @param circleId The id of the sketched circle
	 * @param orientation_plane_projection_id the id of the first reference of the coordinate system
	 * @param cutting_plane_projection_id the id of the second reference of the coordinate system
	 * @return The code whether the operation were successful or not
	 */
	static ProError CircleSection(ProSection section, Parameter *params, int* circleId, int* orientation_plane_projection_id, int* cutting_plane_projection_id);
	/**
	 * Project a point on the axis with reference the start point
	 *
	 * @param start The reference point on the axis
	 * @param axis The axis normalized directional vector
	 * @param vecin The point to be projected
	 * @param vecOut The projected point 
	 */
	static void projection(const Pro3dPnt& start, const Pro3dPnt& axis, const Pro3dPnt& vecin, Pro3dPnt& vecOut);
	/**
	 * The inner product of two vectors
	 *
	 * @param vec1 The first vector
	 * @param vec The second vector
	 * @return The inner product result
	 */
	static double innerProduct(const Pro3dPnt& vec1, const Pro3dPnt& vec);
	/**
	 * Normalize a given vector
	 *
	 * @param normal the input vector
	 */
	static void NormalizeVector(Pro3dPnt& normal);
	/**
	 * Cross Product of two vectors
	 *
	 * @param vect_A the first input vector
	 * @param vect_B the second input vector
	 * @param cross_P The output cross product
	 */
	static void crossProduct(const Pro3dPnt& vect_A, const Pro3dPnt& vect_B, Pro3dPnt& cross_P);
	/**
	 * Get a perpendicular vector to the normal vector
	 *
	 * @param normal the normal vector
	 * @param outVector the perpendicular vector
	 */
	static void GetPerpendicularVector(const Pro3dPnt& normal, Pro3dPnt& outVector);
	/**
	 * The intersection of tow 2D lines
	 *
	 * @param A the first 2D point of vector AB
	 * @param B the second 2D point of vector BB
	 * @param C the first 2D point of vector CD
	 * @param D the second 2D point of vector CD
	 * @param I The intersection point
	 */
	static bool LineIntersection(Pro2dPnt A, Pro2dPnt B, Pro2dPnt C, Pro2dPnt D, Pro2dPnt I);
	/**
	 * Create the cylindrical extrusion
	 *
	 * @param height The height of the extrusion
	 * @param radius The radius of the cylindrical extrusion
	 * @param name The name of the cylindrical extrusion
	 */
	void createCylinder(double height, double radius, const char* name);
	
	// The helping primitives for the creation of the extrusion
	std::shared_ptr<CDatumPlane> drawingPlane;				/**< The drawing plane */
	std::shared_ptr<CDatumPoint> datumPointOnAxis;			/**< The datum point which is on the axis of the cylinder */
	std::shared_ptr<CDatumAxis> orientationAxis;	/**< An axis which is perpendicular to the cylinder axis */
	std::shared_ptr<CDatumPlane> orientationPlane;
	std::shared_ptr<CDatumPoint> orientationPnt;				/**< The point which define the orientationAxis datumPointOnAxis->orientationPnt */
	std::shared_ptr<CDatumAxis> cuttingAxis;		/**< An axis which is perpendicular to the cylinder axis and orientation axis */
	std::shared_ptr<CDatumPoint> cuttingPnt;					/**< The point which define the cuttingAxis datumPointOnAxis->cuttingPnt */

	int circleId;											/**< The circle on the sketch id */
	int orientation_plane_projection_id;					/**< The projection id of orientationAxis on the sketch plane defining a reference to the coordinate system */
	int cutting_plane_projection_id;						/**< The projection id of cuttingAxis on the sketch plane defining a reference to the coordinate system */
};

#endif
