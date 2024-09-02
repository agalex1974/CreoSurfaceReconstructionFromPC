#ifndef CSOR_H
#define CSOR_H

#include <memory>
#include <ProElemId.h>
#include <ProFeatType.h>
#include <ProFeatForm.h>
#include <ProExtrude.h>
#include <ProStdSection.h>
#include <ProSection.h>
#include <ProRevolve.h>
#include "UtilityHelperClass.h"
#include "HelperStructures.h"
#include "CSOR.h"
#include "CCreoPlane.h"
#include "CCreoPointCloud.h"
#include "CDatumAxis.h"

class CCreoSOR : public CSOR
{
public:
	static void AugmentPointCloud(std::shared_ptr<const CCreoPointCloud>& creoPointCloud, spPointCloud pointCloudToAugment, std::set<int>& indexSet);

	/**
	 * Constructor of Creo Surface of Revolution
	 *
	 * @param pnts The point cloud 
	 */
	CCreoSOR(const CPointCloud& pnts, const char* name);
	static void crossProduct(const Pro3dPnt& vect_A, const Pro3dPnt& vect_B, Pro3dPnt& cross_P);
	static void NormalizeVector(Pro3dPnt& normal);
	ProError getSketchInformation();
	ProError correctAxisInformation();
	~CCreoSOR();
private:
	enum DEFAULT_PLANES {
		RIGHT,
		TOP,
		FRONT
	};
	// The feature element tree of axis of revolution 
	ElemTreeData tree[20] = {
		/* 0 */ {0, PRO_E_FEATURE_TREE, {(ProValueDataType)-1}},
		/* 1 */ {1, PRO_E_FEATURE_TYPE, {PRO_VALUE_TYPE_INT, PRO_FEAT_PROTRUSION}},
		/* 2 */ {1, PRO_E_FEATURE_FORM, {PRO_VALUE_TYPE_INT, PRO_REVOLVE}},
		/* 3 */ {1, PRO_E_EXT_SURF_CUT_SOLID_TYPE, {PRO_VALUE_TYPE_INT, PRO_EXT_FEAT_TYPE_SURFACE}},
		/* 4 */ {1, PRO_E_REMOVE_MATERIAL, {PRO_VALUE_TYPE_INT, PRO_EXT_MATERIAL_ADD}},
		/* 5 */ {1, PRO_E_STD_SECTION,  {(ProValueDataType)-1}},
		/* 6 */ {2, PRO_E_STD_SEC_SETUP_PLANE, {(ProValueDataType)-1}},
		/* 7 */ {3, PRO_E_STD_SEC_PLANE, {PRO_VALUE_TYPE_SELECTION}},
		/* 8 */ {3, PRO_E_STD_SEC_PLANE_VIEW_DIR, {PRO_VALUE_TYPE_INT, PRO_SEC_VIEW_DIR_SIDE_ONE}},
		/* 9 */ {3, PRO_E_STD_SEC_PLANE_ORIENT_DIR, {PRO_VALUE_TYPE_INT, PRO_SEC_ORIENT_DIR_RIGHT}},//here
		/* 10 */{3, PRO_E_STD_SEC_PLANE_ORIENT_REF, {PRO_VALUE_TYPE_SELECTION}},
		/* 11 */{1, PRO_E_FEAT_FORM_IS_THIN, {PRO_VALUE_TYPE_INT, PRO_EXT_FEAT_FORM_NO_THIN}},
		/* 12 */{1, PRO_E_STD_DIRECTION, {PRO_VALUE_TYPE_INT, PRO_EXT_CR_IN_SIDE_ONE}},//here
		/* 13 *///{1, PRO_E_REVOLVE_AXIS_OPT, {PRO_VALUE_TYPE_INT, PRO_REV_AXIS_EXT_REF}},
		/* 14 *///{1, PRO_E_REVOLVE_AXIS, {PRO_VALUE_TYPE_SELECTION}},
		/* 15(13) */{1, PRO_E_REV_ANGLE, {(ProValueDataType)-1} },
		/* 16(14) */{2, PRO_E_REV_ANGLE_FROM, {(ProValueDataType)-1} },
		/* 17(15) */{3, PRO_E_REV_ANGLE_FROM_TYPE, {PRO_VALUE_TYPE_INT, PRO_REV_ANG_FROM_ANGLE} },
		/* 18(16) */{3, PRO_E_REV_ANGLE_FROM_VAL, {PRO_VALUE_TYPE_DOUBLE, 0} },
		/* 19(17) */{2, PRO_E_REV_ANGLE_TO, {(ProValueDataType)-1} },
		/* 20(18) */{3, PRO_E_REV_ANGLE_TO_TYPE, {PRO_VALUE_TYPE_INT, PRO_REV_ANG_TO_ANGLE} },
		/* 21(19) */{3, PRO_E_REV_ANGLE_TO_VAL, {PRO_VALUE_TYPE_DOUBLE, 0} }
	};
	int sorId;			/**< The sor feature id */
	int sorInternalId;	/**< The sor internal id */
	/**
	 * Set the drawing plane sketch plane reference in the feature tree
	 *
	 * @param model Creo's current model
	 * @param surfaceId The plane id
	 */
	void setTheDrawingPlaneReference(ProMdl model, int surfaceId);
	/**
	 * Set the drawing plane reference
	 * choose between RIGHT, TOP, FRONT or none at all
	 *
	 * @param model Creo's current model
	 * @param normal The normal to the plane for choosing RIGHT, TOP, FRONT
	 */
	void setTheDrawingPlaneOrientation(ProMdl model, CPointEx3D normal);
	/**
	 * Create the extrusion from its feature tree
	 *
	 * @param model the current part model session
	 * @param params the params passed inside the creo sketching function
	 * @param sketchId the sketchId
	 * @param featureName The name of the extrusion
	 * @param side_id The side projection on the sketch plane
	 * @param bot_id The bottom projection on the sketch plane
	 */
	bool createExtrusion(ProMdl model, Parameter* params, int* sketchId, const char* featureName, int* side_id, int* bot_id);

	/**
	 * Checks whether the chosen orientation is valid
	 *
	 * @param model The creo model session
	 * @return if it is valid or not
	 */
	bool OrientationReferenceValid(ProMdl model);
	/**
	 * Create the flat surface sketch from its feature tree
	 *
	 * @param model the current part model session
	 * @param params the params passed inside the creo sketching function
	 * @param sketchId the sketchId
	 * @param featureName The name of the extrusion
	 * @param axis_id The axis projection on the sketch plane
	 * @param orth_axis_id The orthogonal to axis projection on the sketch plane
	 */
	bool createSOR(ProMdl model, Parameter* params, int* sketchId, const char* featureName, int* axis_id, int* orth_axis_id, int* center_line_id);

	/**
	 * Create the feature from its element tree
	 *
	 * @param model the current part model session
	 */
	void CreateElementTree(ProMdl model);

	ProError sketchSection(ProSection section, Parameter* params, int* extrusionId, int* orientation_plane_projection_id,
		int* cutting_plane_projection_id, int* ceneter_line_id);
	
	ProError getSketchInformation(ProSection section, int sketch_id, int side_id, int bot_id, int ceneter_line_id);
	ProError correcorrectAxisOnSketchct(ProSection section, int sketch_id, int side_id, int bot_id, int ceneter_line_id);
	void setTheDrawingPlaneOrientation(ProMdl model, int surfaceId);
	void createSOR(const CVector<CPointEx3D>& pnts, const CVector<CPointEx3D>& symPnts, Pro3dPnt normalCreo, CPointEx3D axis, const char* name);
	void setTheAxisReference(ProMdl model, int axisId);
	std::shared_ptr<CDatumAxis> axisOfSymmetry;
	std::shared_ptr<CDatumAxis>	orthogonalToAxisOfSymmetry;
	std::shared_ptr<CCreoPlane> generatrixSketchPlane;
	std::shared_ptr<CCreoPlane> orthogonalPlaneToAxisOfSymmetry;
	CPointEx axisInSketchStart;
	CPointEx axisInSketchEnd;
	int sketch_id, orientation_id, cutting_id, center_line_id;

	static CMatrix<double> getTransformationMatrix(CPointEx3D vecFrom, CPointEx3D vecTo);
};



#endif