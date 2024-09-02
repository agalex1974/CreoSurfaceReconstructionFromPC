#include <ProElement.h>
#include <ProFeature.h>
#include <ProModelItem.h>
#include "UtilityHelperClass.h"
#include "Extrusion.h"

#include <ProAxis.h>
#include <ProSolid.h>
#include <ProWindows.h>
#define SIZEOFARR(a) (sizeof(a)/sizeof(a[0]))

//extern FILE* errlog_fp;

Extrusion::Extrusion():
	sketchFunction(nullptr),
	referencePlaneSet(false),
	orientationPlaneSet(false),
	sketchFunctionSet(false)
{}

void Extrusion::setExtrusionDepth(double value)
{
	tree[18].data.v.d = value;
}

void Extrusion::setTheDrawingPlaneReference(ProMdl model, int surfaceId)
{
	
	ProError status;
	status = UtilityHelperClass::ProUtilSelectionFromSurfaceId(model, surfaceId, &tree[7].data.v.r);
	referencePlaneSet = status != PRO_TK_NO_ERROR;
	
}

void Extrusion::setTheDrawingPlaneOrientation(ProMdl model, int surfaceId)
{
	//ProModelitem modelitem;
	ProError status;
	//ProFeature feature;
	//status = ProModelitemByNameInit(model, PRO_SURFACE, (wchar_t*)L"RIGHT", &modelitem);
	//status = ProGeomitemFeatureGet(&modelitem, &feature);
	//status = UtilityHelperClass::ProUtilAxisGeomitem(&feature, PRO_SURFACE, &surfaceId);
	status = UtilityHelperClass::ProUtilSelectionFromSurfaceId(model, surfaceId, &tree[10].data.v.r);
	orientationPlaneSet = status != PRO_TK_NO_ERROR;
}

void Extrusion::setTheDrawingPlaneOrientationWithNormal(ProMdl model, int normalId)
{
	ProAxis axis;
	ProModelitem modelitem;

	ProError err = ProAxisInit((ProSolid)model, normalId, &axis);
	err = ProAxisToGeomitem((ProSolid)model, axis, (ProGeomitem*)&modelitem);
	err = ProSelectionAlloc(NULL, &modelitem, &tree[10].data.v.r);
	orientationPlaneSet = err != PRO_TK_NO_ERROR;
	
}

void Extrusion::setDrawingFunction(SketchFunctionDefinition sketchFunction)
{
	this->sketchFunction = sketchFunction;
	sketchFunctionSet = true;
}

bool Extrusion::updateExtrusionSketch(ProMdl model, Parameter* params, int* sketchId, int side_id, int bot_id)
{
	static ProElempathItem path_items[] = {
		{PRO_ELEM_PATH_ITEM_TYPE_ID, PRO_E_STD_SECTION},
		{PRO_ELEM_PATH_ITEM_TYPE_ID, PRO_E_SKETCHER}
	};

	ProFeature extrusion_feature;
	ProError status = ProFeatureInit(static_cast<ProSolid>(model), extrusionId, &extrusion_feature);
	ProElement elementTree;
	status = ProFeatureElemtreeExtract(&extrusion_feature, nullptr, PRO_FEAT_EXTRACT_NO_OPTS, &elementTree);

	ProElempath				path;
	ProValue				value;
	ProValueData			value_data;
	ProSection				section;
	ProElement				sketch_element;
	
	status = ProElempathAlloc(&path);
	status = ProElempathDataSet(path, path_items, 2);
	status = ProElemtreeElementGet(elementTree, path, &sketch_element);
	status = ProElementValueGet(sketch_element, &value);

	status = ProValueDataGet(value, &value_data);
	section = (ProSection)value_data.v.p;
	status = ProElempathFree(&path);
	//////////////////////////////////////////////////////////
	/*----------------------------------------------------------*\
	Create a section
	\*----------------------------------------------------------*/
	
	status = updateSketchFunction(section, params, sketchId, side_id, bot_id);
	
	/*------------------------------------------------------------*\
	Set section-dependent element values
	\*------------------------------------------------------------*/
	ProValueDataSet(value, &value_data);
	ProElementValueSet(sketch_element, value);

	UtilityHelperClass::ProUtilFeatureSetSectionDependentValues(elementTree, tree,
		SIZEOFARR(tree));
	
	ProFeatureCreateOptions opts[1];

	ProErrorlist errs;
	opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;
	//opts[0] = PRO_FEAT_CR_NO_OPTS;
	status = ProFeatureRedefine(NULL, &feature, elementTree, opts, 1, &errs);
	return status;
}

void Extrusion::changeDirectionOfExtrusion()
{
	static ProElempathItem path_items[] = {
		{PRO_ELEM_PATH_ITEM_TYPE_ID, PRO_E_STD_DIRECTION},
	};

	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	
	ProFeature extrusion_feature;
	status = ProFeatureInit(static_cast<ProSolid>(model), extrusionId, &extrusion_feature);
	ProElement elementTree;
	status = ProFeatureElemtreeExtract(&extrusion_feature, nullptr, PRO_FEAT_EXTRACT_NO_OPTS, &elementTree);
	

	ProElempath				path;
	ProValue				value;
	ProValueData			value_data;
	ProElement				sketchDirection;

	status = ProElempathAlloc(&path);
	status = ProElempathDataSet(path, path_items, 1);
	status = ProElemtreeElementGet(elementTree, path, &sketchDirection);
	status = ProElementValueGet(sketchDirection, &value);
	status = ProValueDataGet(value, &value_data);
	status = ProElempathFree(&path);

	if (value_data.v.i == PRO_EXT_CR_IN_SIDE_TWO)
		value_data.v.i = PRO_EXT_CR_IN_SIDE_ONE;
	else value_data.v.i = PRO_EXT_CR_IN_SIDE_TWO;
	
	ProValueDataSet(value, &value_data);
	ProElementValueSet(sketchDirection, value);

	ProFeatureCreateOptions opts[1];

	ProErrorlist errs;
	//opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;
	opts[0] = PRO_FEAT_CR_NO_OPTS;
	status = ProFeatureRedefine(NULL, &feature, elementTree, opts, 1, &errs);
	status = ProSolidRegenerate((ProSolid)model, 0);
	status = ProWindowRepaint(-1);
}

bool Extrusion::createExtrusion(ProMdl model, Parameter* params, int* sketchId, const char* featureName, int* side_id, int* bot_id)
{
	ProElement				elem_tree;
	ProElement				created_elemtree, sketch_element;

	ProError				err;
	ProModelitem			model_item;
	ProSelection			model_sel;

	ProFeatureCreateOptions opts[1];
	ProErrorlist			errs;
	ProElempath				path;
	ProValue				value;
	ProValueData			value_data;
	ProSection				section;

	int brk = 0;

	static ProElempathItem path_items[] = {
		{PRO_ELEM_PATH_ITEM_TYPE_ID, PRO_E_STD_SECTION},
		{PRO_ELEM_PATH_ITEM_TYPE_ID, PRO_E_SKETCHER}
	};

	do /* Used for exit from middle of block */
	{
		/*----------------------------------------------------------*\
		Create Element Tree
		\*----------------------------------------------------------*/
		err = UtilityHelperClass::ProUtilElemtreeCreate(tree, SIZEOFARR(tree), NULL, &elem_tree);
		if (err != PRO_TK_NO_ERROR)
			break;
		/*----------------------------------------------------------*\
		Create the incomplete protrusion in the current model
		\*----------------------------------------------------------*/
		err = ProMdlToModelitem(model, &model_item);
		err = ProSelectionAlloc(NULL, &model_item, &model_sel);

		opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;
		err = ProFeatureCreate(model_sel, elem_tree, opts, 1, &feature, &errs);
		if (err != PRO_TK_NO_ERROR)
		{
			//ProUtilFeatErrsWrite("ProFeatureCreate", err, elem_tree, &errs);
			break;
		}
		err = ProSelectionFree(&model_sel);
		/*----------------------------------------------------------*\
		Get the initialized section element from the database
		\*----------------------------------------------------------*/
		// Details can be found on page 989 of user manual.
		err = ProElempathAlloc(&path);
		err = ProElempathDataSet(path, path_items, 2);
		err = ProFeatureElemtreeCreate(&feature, &created_elemtree);
		err = ProElemtreeElementGet(created_elemtree, path, &sketch_element);
		err = ProElementValueGet(sketch_element, &value);
		err = ProValueDataGet(value, &value_data);
		section = (ProSection)value_data.v.p;
		err = ProElempathFree(&path);
		//////////////////////////////////////////////////////////
		/*----------------------------------------------------------*\
		Create a section
		\*----------------------------------------------------------*/
		err = sketchFunction(section, params, sketchId, side_id, bot_id);
		if (err != PRO_TK_NO_ERROR)
			break;
		/*------------------------------------------------------------*\
		Set section-dependent element values
		\*------------------------------------------------------------*/
		//UtilityHelperClass::ProUtilFeatureSetSectionDependentValues(created_elemtree, tree,
		//	SIZEOFARR(tree));

		opts[0] = PRO_FEAT_CR_NO_OPTS;
		err = ProFeatureRedefine(NULL, &feature, created_elemtree, opts, 1, &errs);

		extrusionId = feature.id;

		ProSelection selection;
		ProModelitem modelitem;
		ProSelectionAlloc(NULL, &feature, &selection);
		err = ProSelectionModelitemGet(selection, &modelitem);
		extrusionInternalId = modelitem.id;
		ProSelectionFree(&selection);
		
		if (err != PRO_TK_NO_ERROR)
		{
			//ProUtilFeatErrsWrite("ProFeatureRedefine", err, elem_tree, &errs);
			break;
		}
		err = ProElementFree(&elem_tree);
		/*----------------------------------------------------------*\
		Set feature name
		\*----------------------------------------------------------*/
		UtilityHelperClass::ProUtilModelitemNameSet(&feature, const_cast<char*>(featureName));
	} while (brk);

	return (err);
}

Extrusion::~Extrusion()
{
	ProError status = ProSelectionFree(&tree[7].data.v.r);
	status = ProSelectionFree(&tree[10].data.v.r);
	
	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &extrusionInternalId, 1, opt, 1);
}

int Extrusion::getExtrusionId()
{
	return extrusionId;
}

int Extrusion::getExtrusionInternalId()
{
	return extrusionInternalId;
}
