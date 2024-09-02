#include "CDatumPlane.h"
#include "UtilityHelperClass.h"

//extern FILE* debuger;

#define SIZEOFARR(a) (sizeof(a)/sizeof(a[0]))

CDatumPlane::CDatumPlane(const CDatumPoint& pnt, const CDatumAxis& axis, const char* name)
{
	definePrimitive(pnt, axis);
	datumPlaneInit(name);
}

void CDatumPlane::GetPointOnPlane(const Pro3dPnt& pnt, const Pro3dPnt& normal, double d, Pro3dPnt outPnt) const
{
	double max = fabs(normal[0]);
	int cordIndex = 0;

	if (max < fabs(normal[1]))
	{
		cordIndex = 1;
		max = fabs(normal[1]);
	}

	if (max < fabs(normal[2]))
	{
		cordIndex = 2;
	}
	outPnt[0] = pnt[0];
	outPnt[1] = pnt[1];
	outPnt[2] = pnt[2];
	
	switch (cordIndex)
	{
	case 0:
		outPnt[0] = (-normal[1] * outPnt[1] -normal[2] * outPnt[2] - d) / normal[0];
		break;
	case 1:
		outPnt[1] = (-normal[0] * outPnt[0] -normal[2] * outPnt[2] - d) / normal[1];
		break;
	case 2:
		outPnt[2] = (-normal[0] * outPnt[0] -normal[1] * outPnt[1] - d) / normal[2];
		break;
	}
}

void CDatumPlane::definePrimitives(const Pro3dPnt& pnt, const Pro3dPnt& normal, double d, const char* name)
{
	Pro3dPnt pntOnPlane;
	GetPointOnPlane(pnt, normal, d, pntOnPlane);
	Pro3dPnt pntOnNormal;
	pntOnNormal[0] = pntOnPlane[0] + 0.1 * normal[0];
	pntOnNormal[1] = pntOnPlane[1] + 0.1 * normal[1];
	pntOnNormal[2] = pntOnPlane[2] + 0.1 * normal[2];	
	std::string pnt1_name = std::string(name) + "_pnt1";
	pnt1 = std::make_unique<CDatumPoint>(pntOnPlane, pnt1_name.c_str());
	std::string pnt2_name = std::string(name) + "_pnt2";
	pnt2 = std::make_unique<CDatumPoint>(pntOnNormal, pnt2_name.c_str());
	std::string axis_string = std::string(name) + "_axis";
	axis = std::make_unique<CDatumAxis>(*pnt1, *pnt2, axis_string.c_str());
}

CDatumPlane::CDatumPlane(double x, double y, double z, double nx, double ny, double nz, double d, const char* name)
{
	Pro3dPnt pnt;
	pnt[0] = x; pnt[1] = y; pnt[2] = z;
	Pro3dPnt normal;
	normal[0] = nx; normal[1] = ny; normal[2] = nz;
	definePrimitives(pnt, normal, d, name);
	definePrimitive(*pnt1, *axis);
	datumPlaneInit(name);
}

CDatumPlane::CDatumPlane(const Pro3dPnt& pnt, const Pro3dPnt& normal, double d, const char* name)
{
	definePrimitives(pnt, normal, d, name);
	definePrimitive(*pnt1, *axis);
	datumPlaneInit(name);
}

CDatumPlane::CDatumPlane(const Pro3dPnt& normal, double d, const char* name)
{
	Pro3dPnt pnt;
	pnt[0] = 0.0;
	pnt[1] = 0.0;
	pnt[2] = 0.0;
	definePrimitives(pnt, normal, d, name);
	definePrimitive(*pnt1, *axis);
	datumPlaneInit(name);
}

//normal: the normal of the plane (a, b, c)
//the point on the plane
CDatumPlane::CDatumPlane(const Pro3dPnt& normal, const Pro3dPnt& pntOnPlane, const char* name)
{
	double d = -normal[0] * pntOnPlane[0] - normal[1] * pntOnPlane[1] - normal[2] * pntOnPlane[2];
	definePrimitives(pntOnPlane, normal, d, name);
	definePrimitive(*pnt1, *axis);
	datumPlaneInit(name);
}

int CDatumPlane::datumPlaneInit(const char* name)
{
	ProElement elem_axis_tree;
	ProSelection featsel_axis;
	ProFeature feature_plane;
	ProModelitem model_item;
	ProErrorlist plane_errs;
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProFeatureCreateOptions opts[2], *cr_opts;
	opts[0] = PRO_FEAT_CR_NO_OPTS;
	status = ProArrayAlloc(0, sizeof(ProFeatureCreateOptions), 1, (ProArray*)&cr_opts);
	status = ProArrayObjectAdd((ProArray*)&cr_opts, PRO_VALUE_UNUSED, 1, (void*)&(opts[0]));
	status = UtilityHelperClass::ProUtilElemtreeCreate(dtm_tree, SIZEOFARR(dtm_tree), NULL, &elem_axis_tree);
	status = ProMdlToModelitem(model, &model_item);
	status = ProSelectionAlloc(NULL, &model_item, &featsel_axis);
	status = ProFeatureWithoptionsCreate(featsel_axis, elem_axis_tree, cr_opts, PRO_REGEN_FORCE_REGEN, &feature_plane, &plane_errs);
	status = UtilityHelperClass::ProUtilAxisGeomitem(&feature_plane, PRO_SURFACE, &plane_id);
	ProSelection selection;
	ProModelitem modelitem;
	ProSelectionAlloc(NULL, &feature_plane, &selection);
	status = ProSelectionModelitemGet(selection, &modelitem);
	plane_internal_id = modelitem.id;
	status = ProArrayFree((ProArray*)&cr_opts);
	status = ProSelectionFree(&featsel_axis);
	status = ProSelectionFree(&dtm_tree[5].data.v.r);
	status = ProSelectionFree(&dtm_tree[8].data.v.r);
	status = ProElementFree(&elem_axis_tree);
	status = ProSelectionFree(&selection);
	/*-----------------------------------------------------------------*\
	Set datum axis name
	\*-----------------------------------------------------------------*/
	status = UtilityHelperClass::ProUtilModelitemNameSet((ProModelitem*)&feature_plane, const_cast<char*>(name));
	return 0;
}

void CDatumPlane::definePrimitive(const CDatumPoint& pnt, const CDatumAxis& axis)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProGeomitem modelitem;
	
	ProPoint d_pnt;
	status = ProPointInit((ProSolid)model, pnt.GetId(), &d_pnt);
	status = ProPointToGeomitem((ProSolid)model, d_pnt, (ProGeomitem*)&modelitem);
	status = ProSelectionAlloc(NULL, &modelitem, &dtm_tree[8].data.v.r);
	
	ProAxis d_axis;
	status = ProAxisInit((ProSolid)model, axis.GetAxisId(), &d_axis);
	status = ProAxisToGeomitem((ProSolid)model, d_axis, (ProGeomitem*)&modelitem);
	status = ProSelectionAlloc(NULL, &modelitem, &dtm_tree[5].data.v.r);
}

void CDatumPlane::DeletePlane()
{
	
}

CDatumPlane::~CDatumPlane()
{
	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	ProError status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &plane_internal_id, 1, opt, 1);
}

int CDatumPlane::GetId() const
{
	return plane_id;
}

int CDatumPlane::GetInternalId() const
{
	return plane_internal_id;
}

void CDatumPlane::SetName(const char* name)
{
	std::string strname(name);
	strname += "_pnt1";
	pnt1->SetName(strname.c_str());

	strname = std::string(name);
	strname += "_pnt2";
	pnt2->SetName(strname.c_str());

	strname = std::string(name);
	strname += "_axis";
	axis->SetName(strname.c_str());
	
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProModelitem modelitem;
	status = ProModelitemInit(model, plane_id, PRO_SURFACE, &modelitem);
	ProName itemName;
	ProStringToWstring(itemName, (char*)name);
	status = ProModelitemNameSet(&modelitem, itemName);
}

int CDatumPlane::GetNormalId() const
{
	return axis->GetAxisId();
}

void CDatumPlane::printPlaneEquation(ProVector& normal)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	int n = 1;
	if (n) {
		ProModelitem modelitem;
		ProError status = ProModelitemInit(model, plane_internal_id, PRO_FEATURE, &modelitem);

		auto visit = [](ProGeomitem* p_handle,
			ProError    status,
			ProAppData  app_data)->ProError
		{
			ProSurface surface;
			ProError err = ProGeomitemToSurface(p_handle, &surface);
			ProGeomitemdata* pGeomData;
			err = ProSurfaceDataGet(surface, &pGeomData);
			ProVector* normal = (ProVector*)app_data;

			(*normal)[0] = pGeomData->data.p_surface_data->srf_shape.plane.e3[0];
			(*normal)[1] = pGeomData->data.p_surface_data->srf_shape.plane.e3[1];
			(*normal)[2] = pGeomData->data.p_surface_data->srf_shape.plane.e3[2];

			ProGeomitemdataFree(&pGeomData);
			return PRO_TK_NO_ERROR;
		};
		ProFeatureGeomitemVisit(&modelitem, PRO_SURFACE, (ProGeomitemAction)visit, NULL, (void*)&normal);
	}
}