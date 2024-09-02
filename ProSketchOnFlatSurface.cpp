#include "ProSketchOnFlatSurface.h"

#include <cmath>

#include "Extrusion.h"
#include "UtilityHelperClass.h"

#define SIZEOFARR(a) (sizeof(a)/sizeof(a[0]))

ProSketchOnFlatSurface::ProSketchOnFlatSurface()
{
}


void ProSketchOnFlatSurface::CreateElementTree(ProMdl model)
{
	ProModelitem			model_item;
	ProSelection			model_sel;
	ProFeatureCreateOptions opts[1];
	ProElement				elem_tree;
	ProErrorlist			errs;

	ProError err = UtilityHelperClass::ProUtilElemtreeCreate(flat_feature_tree, SIZEOFARR(flat_feature_tree), NULL, &elem_tree);
	/*----------------------------------------------------------*\
	Create the incomplete protrusion in the current model
	\*----------------------------------------------------------*/
	err = ProMdlToModelitem(model, &model_item);
	err = ProSelectionAlloc(NULL, &model_item, &model_sel);

	opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;
	err = ProFeatureCreate(model_sel, elem_tree, opts, 1, &feature, &errs);
	flatFeatureId = feature.id;

	err = ProElementFree(&elem_tree);
	err = ProSelectionFree(&model_sel);
}


bool ProSketchOnFlatSurface::createSketch(ProMdl model, Parameter* params, int* sketchId, const char* featureName, int* side_id, int* bot_id)
{
	ProElement				elem_tree;
	ProElement				sketch_element;

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
		CreateElementTree(model);
		/*----------------------------------------------------------*\
		Get the initialized section element from the database
		\*----------------------------------------------------------*/
		// Details can be found on page 989 of user manual.
		err = ProElempathAlloc(&path);
		err = ProElempathDataSet(path, path_items, 2);
		
		err = ProFeatureElemtreeCreate(&feature, &elem_tree);
		err = ProElemtreeElementGet(elem_tree, path, &sketch_element);
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

		UtilityHelperClass::ProUtilFeatureSetSectionDependentValues(elem_tree, flat_feature_tree,
			SIZEOFARR(flat_feature_tree));

		opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;
		err = ProFeatureRedefine(NULL, &feature, elem_tree, opts, 1, &errs);

		ProSelection selection;
		ProModelitem modelitem;
		ProSelectionAlloc(NULL, &feature, &selection);
		err = ProSelectionModelitemGet(selection, &modelitem);
		flatFeatureInternalId = modelitem.id;
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

void ProSketchOnFlatSurface::setTheDrawingPlaneReference(ProMdl model, int surfaceId)
{
	ProError status;
	status = UtilityHelperClass::ProUtilSelectionFromSurfaceId(model, surfaceId, &flat_feature_tree[5].data.v.r);
}

void ProSketchOnFlatSurface::setTheDrawingPlaneOrientation(ProMdl model, int surfaceId)
{
	ProError status = UtilityHelperClass::ProUtilSelectionFromSurfaceId(model, surfaceId, &flat_feature_tree[8].data.v.r);
}


ProSketchOnFlatSurface::~ProSketchOnFlatSurface()
{
	ProError status = ProSelectionFree(&flat_feature_tree[5].data.v.r);
	status = ProSelectionFree(&flat_feature_tree[8].data.v.r);

	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &flatFeatureInternalId, 1, opt, 1);
}

int ProSketchOnFlatSurface::getId() const
{
	return flatFeatureId;
}

int ProSketchOnFlatSurface::getInternalId() const
{
	return flatFeatureInternalId;
}

void ProSketchOnFlatSurface::setDrawingFunction(SketchFunctionDefinition sketchFunction)
{
	this->sketchFunction = sketchFunction;
}