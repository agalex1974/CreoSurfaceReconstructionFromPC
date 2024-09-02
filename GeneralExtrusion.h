#ifndef GENERAL_EXTRUSION_H
#define GENERAL_EXTRUSION_H

#include <memory>
#include <vector>
#include "CDatumPoint.h"
#include "CDatumAxis.h"
#include "CDatumPlane.h"
#include "Extrusion.h"
#include "bcrust.h"

class GeneralExtrusion : public Extrusion
{
public:
	/**
	 * General extrusion constructor
	 *
	 * @param normal The normal of the sketching plane
	 * @param inner_point The point inside the extrusion
	 * @param threshold_pnts The points in a threshold projected on the plane that passes through inner_point
	 * @param sketch_plane_id The id of the plane that the B-Spline will be sketched
	 * @param depth The depth (height) of the extrusion
	 * @param flipDirection If the direction of the extrusion should be flipped.
	 * @param name The name of the extrusion.
	 */
	GeneralExtrusion(const ProPoint3d& normal, const Pro3dPnt& inner_point, const std::vector<CPointEx3D>& threshold_pnts, int sketch_plane_id,
		double depth, bool flipDirection, const char* name);
private:
	int sketch_plane_id;									/**< The plane sketch id */
	int sketch_id;											/**< The sketch id */
	int orientation_id;										/**< The orientation id */
	int cutting_id;											/**< Another projection id that define an orthonormal Id on the plane */
	std::shared_ptr<CDatumPoint> innerPnt;					/**< The inner point datum point */
	std::shared_ptr<CDatumAxis> orientationAxis;	/**< The orientation axis id used for projection */
	std::shared_ptr<CDatumPlane> orientationPlane;
	std::shared_ptr<CDatumPoint> orientationPnt;				/**< The point used to create the orientation axis */
	std::shared_ptr<CDatumAxis> cuttingAxis;		/**< An axis whose projection on the sketch will create a reference system */
	std::shared_ptr<CDatumPoint> cuttingPnt;					/**< The point used to create the cuttingAxis */

	/**
	 * The sketching function of the general extrusion
	 * Filip you need to play here with your B-Spline 
	 *
	 * @param section The sketch section plane
	 * @param params The parameters used by the sketching function
	 * @param extrusionId The sketched extrusionId
	 * @param orientation_plane_projection_id The orientation axis id which will be projected on the sketch plane creating a reference
	 * @param cutting_plane_projection_id This cutting axis id which will be projected on the sketch plane creating a reference
	 */
	static ProError GeneralExtrusionSection(ProSection section, Parameter *params, int* extrusionId, int* orientation_plane_projection_id,
		int* cutting_plane_projection_id);

	static ProError GeneralExtrusionSectionPhilipp(ProSection section, Parameter *params, int* extrusionId, int* orientation_plane_projection_id,
		int* cutting_plane_projection_id);
	/**
	 * Get a perpendicular vector to the normal
	 *
	 * @param normal The input normal
	 * @param outVector The output perpendicular vector
	 */
	void GetPerpendicularVector(const Pro3dPnt& normal, Pro3dPnt& outVector);
	/**
	 * Normalize the input vector
	 *
	 * @param normal The input vector
	 */
	void NormalizeVector(Pro3dPnt& normal);
	/**
	 * Calculate the cross product of two vectors
	 *
	 * @param vect_A The first input vector
	 * @param vect_B The second input vector
	 * @param cross_P The output cross product vector
	 */
	void crossProduct(const Pro3dPnt& vect_A, const Pro3dPnt& vect_B, Pro3dPnt& cross_P);
	/**
	 * Create the general extrusion method
	 *
	 * @param depth The depth (height) of the protrusion
	 * @param pnts The points that the protrusion must interpolote
	 * @param name The name of the protrusion 
	 */
	void createGeneralExtrusion(double depth, const std::vector<CPointEx3D>& pnts, const char* name);
	/**
	 * The distance of two 2D points
	 *
	 * @param pnt1 The first 2D point
	 * @oaram pnt2 The second 2D point
	 * @return The distance between pnt1 and pnt2
	 */
	static double distance(const CPointEx& pnt1, const CPointEx& pnt2);
};

#endif
