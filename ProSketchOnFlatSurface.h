#ifndef PRO_SKETCH_ON_FLAT_SURFACE
#define PRO_SKETCH_ON_FLAT_SURFACE

#include <ProToolkit.h>
#include <ProMdl.h>
#include <ProModelitem.h>
#include <ProGeomitem.h>
#include <ProSolid.h>
#include <ProFeature.h>
#include <ProFeatType.h>
#include <ProDtmCrv.h>
#include <ProStdSection.h>

#include "HelperStructures.h"
#include "MCLSEXST.h"

class ProSketchOnFlatSurface
{
protected:
	using SketchFunctionDefinition = ProError(*)(ProSection section, Parameter *params, int* sketchId, int* side_id, int* bot_id);
	using UpdateSketchFunctionDefinition = ProError(*)(ProSection section, Parameter *params, int* sketchId, int side_id, int bot_id);
	enum DEFAULT_PLANES {
		RIGHT,
		TOP,
		FRONT
	};
private:
	ElemTreeData flat_feature_tree[10] = {
		/* 0 */ {0, PRO_E_FEATURE_TREE, {(ProValueDataType)-1}},
		/* 1 */ {1, PRO_E_FEATURE_TYPE, {PRO_VALUE_TYPE_INT, PRO_FEAT_CURVE}},
		/* 2 */ {1, PRO_E_CURVE_TYPE, {PRO_VALUE_TYPE_INT, PRO_CURVE_TYPE_SKETCHED}},
		/* 3 */ {1, PRO_E_STD_SECTION,  {(ProValueDataType)-1}},
		/* 4 */ {2, PRO_E_STD_SEC_SETUP_PLANE, {(ProValueDataType)-1}},
		/* 5 */ {3, PRO_E_STD_SEC_PLANE, {PRO_VALUE_TYPE_SELECTION}},
		/* 6 */ {3, PRO_E_STD_SEC_PLANE_VIEW_DIR, {PRO_VALUE_TYPE_INT, PRO_SEC_VIEW_DIR_SIDE_ONE}},
		/* 7 */ {3, PRO_E_STD_SEC_PLANE_ORIENT_DIR, {PRO_VALUE_TYPE_INT, PRO_SEC_ORIENT_DIR_RIGHT}},//here
		/* 8 */ {3, PRO_E_STD_SEC_PLANE_ORIENT_REF, {PRO_VALUE_TYPE_SELECTION}},
		/* 9 */ {1, PRO_E_DTMCRV_DISPLAY_HATCH, {PRO_VALUE_TYPE_INT, PRO_B_FALSE}}
	};
	ProFeature feature;
	int flatFeatureId;
	int flatFeatureInternalId;
public:
	ProSketchOnFlatSurface();
	virtual ~ProSketchOnFlatSurface();
	/**
	 * Get the flat surface id
	 *
	 * @return The extrusion id
	 */
	int getId() const;
	/**
	 * Get the extrusion internal id
	 *
	 * @return The extrusion internal id
	 */
	int getInternalId() const;
	bool OrientationReferenceValid(ProMdl model);
	/**
	 * Set the drawing plane sketch plane reference in the feature tree
	 *
	 * @param model Creo's current model
	 * @param surfaceId The plane id
	 */
	void setTheDrawingPlaneReference(ProMdl model, int surfaceId);
protected:
	SketchFunctionDefinition sketchFunction;				/**< The function sketch definition */
	UpdateSketchFunctionDefinition updateSketchFunction;	/**< The update function sketch definition */
	
	/**
	 * Set the drawing plane reference
	 * Currently it is using just the RIGHT datum plane, it should be enhanced to
	 * choose between RIGHT, TOP, FRONT
	 *
	 * @param model Creo's current model
	 * @param surfaceId (not used)
	 */
	void setTheDrawingPlaneOrientation(ProMdl model, int surfaceId);
	/**
	 * Set the drawing plane reference
	 * Currently it is using just the RIGHT datum plane, it should be enhanced to
	 * choose between RIGHT, TOP, FRONT
	 *
	 * @param model Creo's current model
	 * @param normal The normal to the sketch plane
	 */
	void setTheDrawingPlaneOrientation(ProMdl model, CPointEx3D normal);
	/**
	 * Set the drawing function
	 *
	 * @param sketchFunction The sketch function
	 */
	void setDrawingFunction(SketchFunctionDefinition sketchFunction);
	/**
	 * Create the flat surface sketch from its feature tree
	 *
	 * @param model the current part model session
	 * @param params the params passed inside the creo sketching function
	 * @param sketchId the sketchId
	 * @param featureName The name of the extrusion
	 * @param side_id The side projection on the sketch plane
	 * @param bot_id The bottom projection on the sketch plane
	 */
	bool createSketch(ProMdl model, Parameter* params, int* sketchId, const char* featureName, int* side_id, int* bot_id);
	void CreateElementTree(ProMdl model);
};

#endif