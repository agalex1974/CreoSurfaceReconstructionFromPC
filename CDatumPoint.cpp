#include <string>
#include "CDatumPoint.h"
#include "UtilityHelperClass.h"

#define SIZEOFARR(a) (sizeof(a)/sizeof(a[0]))

//extern FILE* debuger;

CDatumPoint::CDatumPoint(const Pro3dPnt& pnt, const char* name)
{
	setCsys();
	setCoordinates(pnt[0], pnt[1], pnt[2]);
	ProElement elem_point_tree;
	ProError status = UtilityHelperClass::ProUtilElemtreeCreate(point_feature_tree, SIZEOFARR(point_feature_tree), NULL, &elem_point_tree);
	ProMdl model;
	ProModelitem model_item;
	ProSelection model_sel;
	ProFeature feature;
	ProErrorlist errors;
	status = ProMdlCurrentGet(&model);
	status = ProMdlToModelitem(model, &model_item);
	status = ProSelectionAlloc(NULL, &model_item, &model_sel);
	ProFeatureCreateOptions *opts = 0;
	status = ProArrayAlloc(1, sizeof(ProFeatureCreateOptions),1, (ProArray*)&opts);
	opts[0] = PRO_FEAT_CR_DEFINE_MISS_ELEMS;
	status = ProFeatureWithoptionsCreate(model_sel, elem_point_tree, opts, PRO_REGEN_NO_FLAGS, &feature, &errors);
	status = UtilityHelperClass::ProUtilAxisGeomitem(&feature, PRO_POINT, &point_id);
	ProSelection selection;
	ProModelitem modelitem;
	ProSelectionAlloc(NULL, &feature, &selection);
	status = ProSelectionModelitemGet(selection, &modelitem);
	point_internal_id = modelitem.id;
	status = UtilityHelperClass::ProUtilModelitemNameSet((ProModelitem*)&feature, const_cast<char*>(name));
	status = ProArrayFree((ProArray*)&opts);
	status = ProElementFree(&elem_point_tree);
	status = ProSelectionFree(&selection);
	status = ProSelectionFree(&point_feature_tree[4].data.v.r);
}

void CDatumPoint::setCsys()
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	/////////// Grab the world coordinate system ///////////////////////////
	int csys_id;
	auto act_function = [](ProCsys csys, ProError filt_status, ProAppData app_data)->ProError
	{
		ProMdl model;
		ProError status = ProMdlCurrentGet(&model);
		ProGeomitem modelitem;
		status = ProCsysToGeomitem((ProSolid)model, csys, (ProGeomitem*)&modelitem);
		std::string name;
		status = UtilityHelperClass::ProUtilModelitemNameGet(&modelitem, name);
		if (name == "PRT_CSYS_DEF")
		{
			int& id = *(int*)app_data;
			status = ProCsysIdGet(csys, &id);
		}
		return (ProError)1;
	};
	status = ProSolidCsysVisit((ProSolid)model, act_function,NULL, (ProAppData)&csys_id);
	////////////////////////////////////////////////////////////////////////////////////////////
	ProCsys csys;
	status = ProCsysInit((ProSolid)model, csys_id, &csys);
	ProGeomitem modelitem;
	status = ProCsysToGeomitem((ProSolid)model, csys, (ProGeomitem*)&modelitem);
	status = ProSelectionAlloc(NULL, &modelitem, &point_feature_tree[4].data.v.r);
}

/*void CDatumPoint::setName(const char* name)
{
	ProStringToWstring(point_feature_tree[9].data.v.w, (char*)name);
	ProStringToWstring(point_feature_tree[3].data.v.w, (char*)name);
}*/

void CDatumPoint::setCoordinates(double x, double y, double z)
{
	point_feature_tree[8].data.v.d  = x;
	point_feature_tree[9].data.v.d  = y;
	point_feature_tree[10].data.v.d = z;
}

int CDatumPoint::GetId() const
{
	return point_id;
}

int CDatumPoint::GetInternalId() const
{
	return point_internal_id;
}

CDatumPoint::~CDatumPoint()
{
	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	ProError status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &point_internal_id, 1, opt, 1);
}

void CDatumPoint::SetName(const char* name)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProModelitem modelitem;
	status = ProModelitemInit(model, point_id, PRO_POINT, &modelitem);
	ProName itemName;
	
	ProStringToWstring(itemName, (char*)name);
	status = ProModelitemNameSet(&modelitem, itemName);
}

ProError CDatumPoint::getDatumPointCoordinates(ProPoint3d& pnt) const
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProModelitem modelitem;
	status = ProModelitemInit(model, point_internal_id, PRO_FEATURE, &modelitem);
	auto visit = [](ProGeomitem* p_handle,
					ProError    status,
					ProAppData  app_data) -> ProError
	{
		ProVector* pnt = (ProVector*)app_data;
		ProPoint proPnt;
		ProError err = ProGeomitemToPoint(p_handle, &proPnt);
		err = ProPointCoordGet(proPnt, *pnt);
		return err;
	};
	status = ProFeatureGeomitemVisit(&modelitem, PRO_POINT, (ProGeomitemAction)visit, NULL, (void*)&pnt);
	return status;
}