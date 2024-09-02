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
#include <ProDtmAxis.h>
#include "HelperStructures.h"
#include "CDatumPoint.h"
#include "UtilityHelperClass.h"
#include "CDatumAxis.h"

#include <string>

#define SIZEOFARR(a) (sizeof(a)/sizeof(a[0]))

//extern FILE* debuger;

CDatumAxis::CDatumAxis(const CDatumPoint& pnt1, const CDatumPoint& pnt2, const char* name)
{
	definePointContraints(pnt1, pnt2);
	createAxis(name);
}

CDatumAxis::CDatumAxis(const ProPoint3d& pnt1, const ProPoint3d& pnt2, const char* name)
{
	pntStart = std::make_shared<CDatumPoint>(pnt1, "");
	pntEnd = std::make_shared<CDatumPoint>(pnt2, "");
	
	std::string axisName(name);
	std::string pntName = axisName + "_pnt_1";
	pntStart->SetName(pntName.c_str());
	pntName = axisName + "_pnt_2";
	pntEnd->SetName(pntName.c_str());
	definePointContraints(*pntStart, *pntEnd);
	createAxis(name);
}

int CDatumAxis::createAxis(const char* name)
{
	ProElement elem_axis_tree;
	ProSelection featsel_axis;
	ProFeature feature_axis;
	ProModelitem model_item;
	ProErrorlist axis_errs;
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProFeatureCreateOptions opts[2], *cr_opts;
	opts[0] = PRO_FEAT_CR_NO_OPTS;
	status = ProArrayAlloc(0, sizeof(ProFeatureCreateOptions), 1, (ProArray*)&cr_opts);
	status = ProArrayObjectAdd((ProArray*)&cr_opts, PRO_VALUE_UNUSED, 1, (void*)&(opts[0]));
	status = UtilityHelperClass::ProUtilElemtreeCreate(axis_tree, SIZEOFARR(axis_tree), NULL, &elem_axis_tree);
	status = ProMdlToModelitem(model, &model_item);
	status = ProSelectionAlloc(NULL, &model_item, &featsel_axis);
	status = ProFeatureWithoptionsCreate(featsel_axis, elem_axis_tree, cr_opts, PRO_REGEN_FORCE_REGEN, &feature_axis, &axis_errs);
	status = UtilityHelperClass::ProUtilAxisGeomitem(&feature_axis, PRO_AXIS, &axis_id);
	
	ProSelection selection;
	ProModelitem modelitem;
	ProSelectionAlloc(NULL, &feature_axis, &selection);
	status = ProSelectionModelitemGet(selection, &modelitem);
	axis_internal_id = modelitem.id;
	status = ProArrayFree((ProArray*)&cr_opts);
	status = ProSelectionFree(&featsel_axis);
	status = ProSelectionFree(&axis_tree[5].data.v.r);
	status = ProSelectionFree(&axis_tree[8].data.v.r);
	status = ProElementFree(&elem_axis_tree);
	status = ProSelectionFree(&selection);
	/*-----------------------------------------------------------------*\
	Set datum axis name
	\*-----------------------------------------------------------------*/
	status = UtilityHelperClass::ProUtilModelitemNameSet((ProModelitem*)&feature_axis, const_cast<char*>(name));
	
	return 0;
}

void CDatumAxis::definePointContraints(const CDatumPoint& pnt1, const CDatumPoint& pnt2)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProPoint d_pnt1;
	ProPointInit((ProSolid)model, pnt1.GetId(), &d_pnt1);
	ProPoint d_pnt2;
	ProPointInit((ProSolid)model, pnt2.GetId(), &d_pnt2);
	ProGeomitem modelitem;
	status = ProPointToGeomitem((ProSolid)model, d_pnt1, (ProGeomitem*)&modelitem);
	status = ProSelectionAlloc(NULL, &modelitem, &axis_tree[5].data.v.r);
	status = ProPointToGeomitem((ProSolid)model, d_pnt2, (ProGeomitem*)&modelitem);
	status = ProSelectionAlloc(NULL, &modelitem, &axis_tree[8].data.v.r);
}

int CDatumAxis::GetAxisId() const
{
	return axis_id;
}

int CDatumAxis::GetAxisInternalId() const
{
	return axis_internal_id;
}

CDatumAxis::~CDatumAxis()
{
	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	ProError status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &axis_internal_id, 1, opt, 1);
}

void CDatumAxis::SetName(const char* name)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProModelitem modelitem;
	status = ProModelitemInit(model, axis_id, PRO_AXIS, &modelitem);
	ProName itemName;
	ProStringToWstring(itemName, (char*)name);
	status = ProModelitemNameSet(&modelitem, itemName);
}
