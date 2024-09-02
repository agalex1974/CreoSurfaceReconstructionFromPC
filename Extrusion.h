#ifndef EXTRUSION_H
#define EXTRUSION_H

#include <ProElemId.h>
#include <ProFeatType.h>
#include <ProFeatForm.h>
#include <ProExtrude.h>
#include <ProStdSection.h>
#include <ProSection.h>
#include "HelperStructures.h"

class Extrusion
{
public:
	
protected:
	using SketchFunctionDefinition = ProError(*)(ProSection section, Parameter *params, int* sketchId, int* side_id, int* bot_id);
	using UpdateSketchFunctionDefinition = ProError(*)(ProSection section, Parameter *params, int* sketchId, int side_id, int bot_id);
	ElemTreeData tree[19] = {
		/* 0 */ {0, PRO_E_FEATURE_TREE, {(ProValueDataType)-1}},
		/* 1 */ {1, PRO_E_FEATURE_TYPE, {PRO_VALUE_TYPE_INT, PRO_FEAT_PROTRUSION}},
		/* 2 */ {1, PRO_E_FEATURE_FORM, {PRO_VALUE_TYPE_INT, PRO_EXTRUDE}},
		/* 3 */ {1, PRO_E_EXT_SURF_CUT_SOLID_TYPE, {PRO_VALUE_TYPE_INT, PRO_EXT_FEAT_TYPE_SOLID}},
		/* 4 */ {1, PRO_E_REMOVE_MATERIAL, {PRO_VALUE_TYPE_INT, PRO_EXT_MATERIAL_ADD}},
		/* 5 */ {1, PRO_E_STD_SECTION,  {(ProValueDataType)-1}},
		/* 6 */ {2, PRO_E_STD_SEC_SETUP_PLANE, {(ProValueDataType)-1}},
		/* 7 */ {3, PRO_E_STD_SEC_PLANE, {PRO_VALUE_TYPE_SELECTION}},
		/* 8 */ {3, PRO_E_STD_SEC_PLANE_VIEW_DIR, {PRO_VALUE_TYPE_INT, PRO_SEC_VIEW_DIR_SIDE_ONE}},
		/* 9 */ {3, PRO_E_STD_SEC_PLANE_ORIENT_DIR, {PRO_VALUE_TYPE_INT, PRO_SEC_ORIENT_DIR_RIGHT}},//here
		/* 10 */{3, PRO_E_STD_SEC_PLANE_ORIENT_REF, {PRO_VALUE_TYPE_SELECTION}},
		/* 11 */{1, PRO_E_FEAT_FORM_IS_THIN, {PRO_VALUE_TYPE_INT, PRO_EXT_FEAT_FORM_NO_THIN}},
		/* 12 */{1, PRO_E_STD_DIRECTION, {PRO_VALUE_TYPE_INT, PRO_EXT_CR_IN_SIDE_TWO}},//here
		/* 13 */{1, PRO_E_STD_EXT_DEPTH, {(ProValueDataType)-1}},
		/* 14 */{2, PRO_E_EXT_DEPTH_FROM, {(ProValueDataType)-1}},
		/* 15 */{3, PRO_E_EXT_DEPTH_FROM_TYPE, {PRO_VALUE_TYPE_INT, PRO_EXT_DEPTH_FROM_NONE}},
		/* 16 */{2, PRO_E_EXT_DEPTH_TO, {(ProValueDataType)-1}},
		/* 17 */{3, PRO_E_EXT_DEPTH_TO_TYPE, {PRO_VALUE_TYPE_INT, PRO_EXT_DEPTH_TO_BLIND}},
		/* 18 */{3, PRO_E_EXT_DEPTH_TO_VALUE, {PRO_VALUE_TYPE_DOUBLE}}
	};
	/**
	 * Change the direction in the feature tree before the feature is created 
	 */
	void changeExtrusionDirectionBeforeCreation()
	{
		tree[12].data.v.i = PRO_EXT_CR_IN_SIDE_ONE;
	}
	SketchFunctionDefinition sketchFunction;				/**< The function sketch definition */
	UpdateSketchFunctionDefinition updateSketchFunction;	/**< The update function sketch definition */
	bool referencePlaneSet;									/**< Check whether the reference plane is set */
	bool orientationPlaneSet;								/**< Check whether the sketch orientation direction is set */
	bool sketchFunctionSet;									/**< Check whether the sketch function is set */
	ProFeature feature;										/**< The feature definition of the extrusion */
public:
	Extrusion();
	virtual ~Extrusion();
	/**
	 * Change the direction of the extrusion after it is created
	 */
	void changeDirectionOfExtrusion();
	/**
	 * Get the extrusion id
	 *
	 * @return The extrusion id
	 */
	int getExtrusionId();
	/**
	 * Get the extrusion internal id
	 *
	 * @return The extrusion internal id
	 */
	int getExtrusionInternalId();
protected:
	int extrusionId;			/**< The extrusion feature id */
	int extrusionInternalId;	/**< The extrusion internal id */
	/**
	 * Set as orientation reference a normal
	 * DOES NOT WORK DO NOT USE YET!
	 */
	void setTheDrawingPlaneOrientationWithNormal(ProMdl model, int normalId);
	/**
	 * Set the drawing plane sketch plane reference in the feature tree
	 *
	 * @param model Creo's current model
	 * @param surfaceId The plane id
	 */
	void setTheDrawingPlaneReference(ProMdl model, int surfaceId);
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
	 * Set the extrusion depth (height)
	 *
	 * @param value The depth value
	 */
	void setExtrusionDepth(double value);
	/**
	 * Set the drawing function
	 *
	 * @param sketchFunction The sketch function 
	 */
	void setDrawingFunction(SketchFunctionDefinition sketchFunction);
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
	 * Update the extrusion 
	 *
	 * @param model the current part model session
	 * @param params the params passed inside the creo sketching function
	 * @param sketchId the sketchId
	 * @param side_id The side projection on the sketch plane
	 * @param bot_id The bottom projection on the sketch plane
	 */
	bool updateExtrusionSketch(ProMdl model, Parameter* params, int* sketchId, int side_id, int bot_id);
};

#endif
