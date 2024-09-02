#include <filesystem>
#include <ProUI.h>
#include <ProUIDialog.h>
#include <ProUIInputpanel.h>
#include <ProUIPushbutton.h>
#include <ProUITab.h>
#include <ProUIList.h>
#include <ProUtil.h>
#include <vector>
#include <memory>
#include <ProFeature.h>
#include <ProFeatType.h>
#include <ProDtmPnt.h>
#include "HelperStructures.h"
#include "CDatumPoint.h"
#include "CDatumAxis.h"
#include "CDatumPlane.h"
#include "GuiInterface.h"
#include <ProFlatSrf.h>
#include <ProDtmCrv.h>

#include <ProArray.h>
#include <ProSelbuffer.h>

#define M_PI 3.1415926535897932384626433832795

#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <Plane.h>
#include <CylinderPrimitiveShape.h>
#include <Cylinder.h>
#include <ProImportfeat.h>
#include <ProIntfData.h>
#include "CCreoPointCloud.h"


extern GuiInterface* gui;
extern CCreoPointCloud* pointCloud;
//extern FILE* debuger;

int GuiInterface::GuiActivate()
{
	ProError err;
	err = ProUIDialogCreate(const_cast<char*>(UI_GUI_NAME), const_cast<char*>(UI_GUI_NAME));
	err = ProUIDialogCloseActionSet(const_cast<char*>(UI_GUI_NAME),
		static_cast<ProUIAction>(closeAction), nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_PLANE_GRAB_POINTS), (ProUIAction)on_push_button_plane_grab_points, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_CYLINDER_GRAB_POINTS), (ProUIAction)on_push_button_cylinder_grab_points, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_CYLINDER_GRAB_STARTING_POINT), (ProUIAction)on_push_button_cylinder_grab_start_point, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_CYLINDER_GRAB_ENDING_POINT), (ProUIAction)on_push_button_cylinder_grab_end_point, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_GENERAL_GRAB_REFERENCE_PLANE), (ProUIAction)on_push_button_general_grab_reference_plane, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
			const_cast<char*>(UI_GENERAL_GRAB_INNER_POINT), (ProUIAction)on_push_button_general_grab_inner_point, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_GENERAL_GRAB_EXTENT_POINT), (ProUIAction)on_push_button_general_grab_extent_point, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_CREATE_PRIMITIVE), (ProUIAction)on_create_primitive, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_ACCEPT_PRIMITIVE), (ProUIAction)on_accept_primitive, nullptr);
	err = ProUIPushbuttonActivateActionSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_PRIMITIVE_CHANGE_DIRECTION), (ProUIAction)on_primitive_change_direction, nullptr);
	

	err = ProUIListSelectActionSet(const_cast<char*>(UI_GUI_NAME), const_cast<char*>(UI_PRIMITIVE_LIST), on_element_select, NULL);
	err = ProUIListPopupmenuSet(const_cast<char*>(UI_GUI_NAME), const_cast<char*>(UI_PRIMITIVE_LIST),
		const_cast<char*>(UI_PRIMITIVE_LIST_POPUP));
	set_default_values();
	UIListClear();
	int ui_status;
	
	err = ProUIDialogActivate(const_cast<char*>(UI_GUI_NAME), &ui_status);
	err = ProUIDialogDestroy(const_cast<char*>(UI_GUI_NAME));
	return 0;
}

void GuiInterface::on_primitive_change_direction()
{
	gui->GetActivePrimitive()->ChangeDirection();
}

void GuiInterface::closeAction(char* dialog, char *component, ProAppData appdata)
{
	gui->GetActivePrimitive() = nullptr;
	gui->primitiveListClear();
	UIListClear();
	ProUIDialogExit(dialog, 0);
}

ProError GuiInterface::setEpsilon(double value)
{
	ProError err;
	err = ProUIInputpanelDoubleSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_EPSILON), value);
	return err;
}

double GuiInterface::getEpsilon(ProError& status)
{
	double value;
	status = ProUIInputpanelDoubleGet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_EPSILON), &value);
	return value;
}

ProError GuiInterface::setBitmapEpsilon(double value)
{
	ProError err;
	err = ProUIInputpanelDoubleSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_BITMAP_EPSILON), value);
	return err;
}
double GuiInterface::getBitmapEpsilon(ProError& status)
{
	double value;
	status = ProUIInputpanelDoubleGet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_BITMAP_EPSILON), &value);
	return value;
}

ProError GuiInterface::setNormalThreshold(double value)
{
	ProError err;
	err = ProUIInputpanelDoubleSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_NORMAL_THRESHOLD), value);
	return err;
}

double GuiInterface::getNormalThreshold(ProError& status)
{
	double value;
	status = ProUIInputpanelDoubleGet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_NORMAL_THRESHOLD), &value);
	return value;
}

ProError GuiInterface::setMinimumSupport(int value)
{
	ProError err;
	err = ProUIInputpanelIntegerSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_MINIMUM_SUPPORT), value);
	return err;
}

ProError PointHighLightAction(
	ProGeomitem geomItem,
	ProError status,
	ProAppData app_data
)
{
	ProPoint pro_point;
	ProGeomitemToPoint(&geomItem, &pro_point);
	ProVector vec;
	ProPointCoordGet(pro_point, vec);
	ProSelection selection;
	ProSelectionAlloc(NULL, &geomItem, &selection);
	if (gui->isInThreshold(vec)) {
		ProSelectionHighlight(selection, PRO_COLOR_SELECTED);
	}
	else
	{
		ProSelectionHighlight(selection, PRO_COLOR_DIMMED);
	}
	ProSelectionFree(&selection);
	return PRO_TK_NO_ERROR;
}

ProError PointUnHighLightAction(
	ProGeomitem geomItem,
	ProError status,
	ProAppData app_data
)
{
	ProSelection selection;
	ProSelectionAlloc(NULL, &geomItem, &selection);	
	ProSelectionUnhighlight(selection);
	ProSelectionFree(&selection);
	return PRO_TK_NO_ERROR;
}

void GuiInterface::unhighlighPointCloud()
{
	ProFeatureGeomitemVisit(&feat, PRO_POINT, (ProGeomitemAction)PointUnHighLightAction,
		NULL, NULL);
}

int GuiInterface::getMinimumSupport(ProError& status)
{
	int value;
	status = ProUIInputpanelIntegerGet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_MINIMUM_SUPPORT), &value);
	return value;
}

ProError GuiInterface::setProbability(double value)
{
	ProError err;
	err = ProUIInputpanelDoubleSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_PROBABILITY), value);
	return err;
}

double GuiInterface::getProbability(ProError& status)
{
	double value;
	status = ProUIInputpanelDoubleGet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_INPUT_PROBABILITY), &value);
	return value;
}

void GuiInterface::set_default_values()
{
	setEpsilon(0.01);
	setBitmapEpsilon(0.02);
	setNormalThreshold(0.9);
	setMinimumSupport(500);
	setProbability(0.001);
}

std::vector<CPointEx3D>& GuiInterface::getBufferPntPlane()
{
	return buffer_pnt_plane;
}

ProError GuiInterface::setPlanePointCount(int value)
{
	ProError err;
	err = ProUIInputpanelIntegerSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_OUTPUT_PLANE_POINT_COUNT), value);
	return err;
}

ProError GuiInterface::setCylinderPointCount(int value)
{
	ProError err;
	err = ProUIInputpanelIntegerSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_OUTPUT_CYLINDER_POINT_COUNT), value);
	return err;
}

ProError GuiInterface::SetCylinderTerminatingPointInfo(char* info)
{
	wchar_t* winfo = new wchar_t[512];
	ProStringToWstring(winfo, info);
	ProError err;
	err = ProUIInputpanelWidestringSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_CYLINDER_ENDING_POINT_INFO), winfo);
	delete[] winfo;
	return err;
}

ProError GuiInterface::SetGeneralExtrusionPlaneName(const char* string_name)
{
	wchar_t* winfo = new wchar_t[512];
	ProStringToWstring(winfo, (char*)string_name);
	ProError err;
	err = ProUIInputpanelWidestringSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_GENERAL_EXTRUSION_PLANE_INFO), winfo);
	delete[] winfo;
	return err;
}


ProError GuiInterface::SetGeneralExtrusionInnerPointInfo(const char* info)
{
	wchar_t* winfo = new wchar_t[512];
	ProStringToWstring(winfo, (char*)info);
	ProError err;
	err = ProUIInputpanelWidestringSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_GENERAL_INNER_POINT_INFO), winfo);
	delete[] winfo;
	return err;
}

ProError GuiInterface::SetGeneralExtrusionExtentPointInfo(const char* info)
{
	wchar_t* winfo = new wchar_t[512];
	ProStringToWstring(winfo, (char*)info);
	ProError err;
	err = ProUIInputpanelWidestringSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_GENERAL_EXTENT_POINT_INFO), winfo);
	delete[] winfo;
	return err;
}


ProError GuiInterface::SetCylinderStartingPointInfo(char* info)
{
	wchar_t* winfo = new wchar_t[512];
	ProStringToWstring(winfo, info);
	ProError err;
	err = ProUIInputpanelWidestringSet(const_cast<char*>(UI_GUI_NAME),
		const_cast<char*>(UI_CYLINDER_STARTING_POINT_INFO), winfo);
	delete[] winfo;
	return err;
}

void GuiInterface::on_element_select(char* dialog, char *component, ProAppData appdata)
{
	int count;
	char** names;
	ProUIListSelectednamesGet(dialog, component, &count, &names);
	if (count == 1)
	{
		if (gui->GetActivePrimitive())
			gui->GetActivePrimitive()->Destroy();
		gui->GetActivePrimitive() = gui->GetPrimitiveList()[std::string(names[0])];
		gui->GetActivePrimitive()->Create(names[0]);
	}
	ProStringarrayFree(names, count);
}

void GuiInterface::on_push_button_plane_grab_points(char* dialog, char *component, ProAppData appdata)
{
	std::vector<CPointEx3D>& buffer_pnts = gui->getBufferPntPlane();
	buffer_pnts.clear();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);
	
	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			buffer_pnts.push_back({xyz[0], xyz[1], xyz[2]});
		}	
	}
	setPlanePointCount((int)buffer_pnts.size());
	ProArrayFree((ProArray *)&p_sel);
}

std::vector<CPointEx3D>& GuiInterface::getBufferPntCylinder()
{
	return buffer_pnt_cylinder;
}

void GuiInterface::on_push_button_cylinder_grab_points(char* dialog, char *component, ProAppData appdata)
{
	std::vector<CPointEx3D>& buffer_pnts = gui->getBufferPntCylinder();
	buffer_pnts.clear();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);

	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			buffer_pnts.push_back({ xyz[0], xyz[1], xyz[2] });
		}
	}
	setCylinderPointCount((int)buffer_pnts.size());
	ProArrayFree((ProArray *)&p_sel);
}

void GuiInterface::on_push_button_cylinder_grab_start_point(char* dialog, char *component, ProAppData appdata)
{
	CPointEx3D& pnt = gui->GetCylinderStartPoint();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);

	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			pnt = CPointEx3D(xyz[0], xyz[1], xyz[2]);
			char pntinfo[512];
			sprintf(pntinfo, "(%f, %f, %f)", pnt.x, pnt.y, pnt.z);
			SetCylinderStartingPointInfo(pntinfo);
			break;
		}
	}
	ProArrayFree((ProArray *)&p_sel);
}

void GuiInterface::on_push_button_general_grab_reference_plane(char* dialog, char* component, ProAppData appdata)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	ProSelection* p_sel;
	status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	status = ProArraySizeGet(p_sel, &n);
	for (int i = 0; i < n; i++)
	{
		ProReference reference;
		status = ProSelectionToReference(p_sel[i], &reference);
		ProType type;
		status = ProReferenceTypeGet(reference, &type);
		int id;
		status = ProReferenceIdGet(reference, &id);
		ProModelitem modelitem;
		status = ProModelitemInit(model, id, PRO_FEATURE, &modelitem);
		auto visit = [](ProGeomitem *p_handle,
						ProError    status,
						ProAppData  app_data)->ProError
		{
			ProSurface surface;
			ProError err = ProGeomitemToSurface(p_handle, &surface);
			
			FILE* file = fopen("plane_type.txt", "w");
			fprintf(file, "get the surface:%d\n", err == PRO_TK_NO_ERROR);
			ProGeomitemdata* pGeomData;
			err = ProSurfaceDataGet(surface, &pGeomData);
			fprintf(file, "Get the surface data:%d\n", err == PRO_TK_NO_ERROR);
			ProVector e1, e2, e3, origin;

			e3[0] = pGeomData->data.p_surface_data->srf_shape.plane.e3[0];
			e3[1] = pGeomData->data.p_surface_data->srf_shape.plane.e3[1];
			e3[2] = pGeomData->data.p_surface_data->srf_shape.plane.e3[2];

			fprintf(file, "%lf %lf %lf\n", e3[0], e3[1], e3[2]);
			fclose(file);
			ProGeomitemdataFree(&pGeomData);
			return PRO_TK_NO_ERROR;
		};
		
		ProFeatureGeomitemVisit(&modelitem, PRO_SURFACE, (ProGeomitemAction)visit, NULL, NULL);
		ProName name;
		ProModelitemNameGet(&modelitem, name);
		char string_name[33];
		ProWstringToString(string_name, name);
		auto iterator = gui->GetAcceptedPrimitiveList().find(modelitem.id);
		if (iterator != gui->GetAcceptedPrimitiveList().end() && iterator->second->GetType() == RansacPrimitive::PLANE)
		{
			std::shared_ptr<RansacPlanePrimitive> plane = dynamic_pointer_cast<RansacPlanePrimitive>(iterator->second);
			ProSurface surf;
			
			FILE* file = fopen("plane.txt", "w");
			ProError err = ProSurfaceInit(model, id, &surf);
			fprintf(file, "%d", err == PRO_TK_NO_ERROR);
			ProGeomitemdata* pGeomData;
			err = ProSurfaceDataGet(surf, &pGeomData);
			fprintf(file, "%d", err == PRO_TK_NO_ERROR);
			fclose(file);
			ProVector e1, e2, e3, origin;
			//e3[0] = pGeomData->data.p_surface_data->srf_shape.plane.e3[0];
			//e3[1] = pGeomData->data.p_surface_data->srf_shape.plane.e3[1];
			//e3[2] = pGeomData->data.p_surface_data->srf_shape.plane.e3[2];
			
			//fprintf(file, "%lf %lf %lf - %lf %lf %lf\n", plane->GetNormal().x, plane->GetNormal().y, plane->GetNormal().z,
			//	e3[0], e3[1], e3[2]);
			
			
			gui->getGeneralProtrusionReferencePlane() = iterator->second;
			SetGeneralExtrusionPlaneName(string_name);
			ProReferenceFree(reference);
			break;
		}
		ProReferenceFree(reference);
	}
	ProArrayFree((ProArray *)&p_sel);
}

std::shared_ptr<RansacPrimitive>& GuiInterface::getGeneralProtrusionReferencePlane()
{
	return generalExtrusionReferencePlane;
}

void GuiInterface::on_push_button_general_grab_inner_point(char* dialog, char *component, ProAppData appdata)
{
	CPointEx3D& pnt = gui->GetGeneralExtrusionInnerPoint();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);
	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			pnt = CPointEx3D(xyz[0], xyz[1], xyz[2]);
			char pntinfo[512];
			sprintf(pntinfo, "(%f, %f, %f)", pnt.x, pnt.y, pnt.z);
			SetGeneralExtrusionInnerPointInfo(pntinfo);
			break;
		}
	}
	if (n > 0) gui->CreateSectionOnPointCloud();
	ProArrayFree((ProArray *)&p_sel);
}

CPointEx3D& GuiInterface::getGeneralExtentPoint()
{
	return general_extent_point;
}

void GuiInterface::on_push_button_general_grab_extent_point(char* dialog, char *component, ProAppData appdata)
{
	CPointEx3D& pnt = gui->getGeneralExtentPoint();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);
	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			pnt = { xyz[0], xyz[1], xyz[2] };
			char pntinfo[512];
			sprintf(pntinfo, "(%f, %f, %f)", pnt.x, pnt.y, pnt.z);
			SetGeneralExtrusionExtentPointInfo(pntinfo);
			break;
		}
	}
	ProArrayFree((ProArray *)&p_sel);
}

void GuiInterface::on_push_button_cylinder_grab_end_point(char* dialog, char *component, ProAppData appdata)
{
	CPointEx3D& pnt = gui->GetCylinderEndPoint();
	ProSelection* p_sel;
	ProError status = ProArrayAlloc(0, sizeof(ProFeature), 1, (ProArray *)&p_sel);
	ProSelbufferSelectionsGet(&p_sel);
	int n;
	Pro3dPnt xyz;
	status = ProArraySizeGet(p_sel, &n);

	for (int i = 0; i < n; i++)
	{
		status = ProSelectionPoint3dGet(p_sel[i], xyz);
		if (status == PRO_TK_NO_ERROR)
		{
			pnt = CPointEx3D(xyz[0], xyz[1], xyz[2]);
			char pntinfo[512];
			sprintf(pntinfo, "(%f, %f, %f)", pnt.x, pnt.y, pnt.z);
			SetCylinderTerminatingPointInfo(pntinfo);
			break;
		}
	}
	ProArrayFree((ProArray *)&p_sel);
}



GuiInterface::GuiInterface()
{
	list_names = NULL;
	list_labels = NULL;
	list_size = 0;
	accepted_planes_count = 0;
	accepted_cylinders_count = 0;
}

void computeBBox(Point* pnts, unsigned size, Vec3f& min, Vec3f& max) {
	min[0] = 1e10; min[1] = 1e10; min[2] = 1e10;
	max[0] = -1e10; max[1] = -1e10; max[2] = -1e10;
	for (unsigned i = 0; i < size; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (min[j] > pnts[i].pos[j]) min[j] = pnts[i].pos[j];
			if (max[j] < pnts[i].pos[j]) max[j] = pnts[i].pos[j];
		}
	}

}

void GuiInterface::on_accept_primitive()
{
	if (gui->GetActivePrimitive())
	{
		switch (gui->GetActivePrimitive()->GetType())
		{
		case RansacPrimitive::PLANE:
		{
			std::string name("Rec_Pln_");
			name += std::to_string(gui->GetAcceptedPlanesCount()++);
			gui->GetActivePrimitive()->SetName(name.c_str());
			gui->GetAcceptedPrimitiveList()[gui->GetActivePrimitive()->GetPrimitiveCreoId()] = gui->GetActivePrimitive();
			break;
			/*RansacPlanePrimitive* plane = (RansacPlanePrimitive*)&*gui->GetActivePrimitive();
			CPointEx3D normal = plane->GetNormal();
			FILE* file = fopen("GeometricPlane.txt", "w");
			fprintf(file, "%lf %lf %lf\n", normal.x, normal.y, normal.z);
				fclose(file);*/
				
		}
		case RansacPrimitive::CYLINDER:
		{
			std::string name("Rec_Cyl_");
			name += std::to_string(gui->GetAcceptedCylindersCount()++);
			gui->GetActivePrimitive()->SetName(name.c_str());
			gui->GetAcceptedPrimitiveList()[gui->GetActivePrimitive()->GetPrimitiveCreoId()] = gui->GetActivePrimitive();
			break;
		}
		case RansacPrimitive::GENERAL:
		{
			//std::string name("Rec_Cyl_");
			//name += std::to_string(gui->GetAcceptedCylindersCount()++);
			//gui->GetActivePrimitive()->SetName(name.c_str());
			gui->GetAcceptedPrimitiveList()[gui->GetActivePrimitive()->GetPrimitiveCreoId()] = gui->GetActivePrimitive();
			break;
		}
		}
		gui->GetActivePrimitive() = nullptr;
		gui->primitiveListClear();
		UIListClear();
		ProUIDialogExit(const_cast<char*>(UI_GUI_NAME), 0);
	}
}

void GuiInterface::on_create_primitive()
{
	char** labelNames;
	int count = 1;
	ProError status = ProUITabSelectednamesGet((char*)UI_GUI_NAME, (char*)UI_TAB_NAME, &count, &labelNames);

	if (!strcmp(labelNames[0], UI_TAB_NAME_PLANE))
	{
		if (gui->getBufferPntPlane().size() > 0)
			CreatePlanePrimitive();
	}

	if (!strcmp(labelNames[0], UI_TAB_NAME_CYLINDER))
	{
		if (gui->getBufferPntCylinder().size() > 0)
			CreateCylinderPrimitive();
	}

	if (!strcmp(labelNames[0], UI_TAB_NAME_GENERAL_EXTRUSION))
	{
		CreateGeneralExtrusion();
	}
	ProStringarrayFree(labelNames, count);
}

void GuiInterface::CreatePlanePrimitive()
{
	gui->primitiveListClear();
	UIListClear();
	const std::vector<CPointEx3D>& buffer_pnts = gui->getBufferPntPlane();
	Point* pnts_norms = new Point[buffer_pnts.size()];
	int i = 0;
	for (const auto& query_pnt : buffer_pnts)
	{
		auto indexSet = pointCloud->GetKNearestNeighborIndex(query_pnt, 1);
		unsigned int nearest_index = indexSet[0];
		pnts_norms[i].pos[0] = (float)(*pointCloud)[nearest_index].first.x;
		pnts_norms[i].pos[1] = (float)(*pointCloud)[nearest_index].first.y;
		pnts_norms[i].pos[2] = (float)(*pointCloud)[nearest_index].first.z;
		pnts_norms[i].normal[0] = (float)(*pointCloud)[nearest_index].second.x;
		pnts_norms[i].normal[1] = (float)(*pointCloud)[nearest_index].second.y;
		pnts_norms[i].normal[2] = (float)(*pointCloud)[nearest_index].second.z;
		i++;
	}
	PointCloud pc(pnts_norms, buffer_pnts.size());
	Vec3f min; Vec3f max;
	computeBBox(pnts_norms, buffer_pnts.size(), min, max);
	pc.setBBox(min, max);
	pc.widenBBox(0.05f);
	RansacShapeDetector::Options ransacOptions;
	ProError status;
	ransacOptions.m_epsilon = (float)gui->getEpsilon(status) * pc.getScale();
	ransacOptions.m_bitmapEpsilon = (float)gui->getBitmapEpsilon(status) * pc.getScale();
	ransacOptions.m_normalThresh = (float)gui->getNormalThreshold(status);
	ransacOptions.m_minSupport = gui->getMinimumSupport(status);
	ransacOptions.m_probability = (float)gui->getProbability(status);

	RansacShapeDetector detector(ransacOptions);
	detector.Add(new PlanePrimitiveShapeConstructor());

	MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>> shapes; 
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);

	char**& list_names = gui->Get_List_Names();
	wchar_t**& list_labels = gui->Get_List_Labels();

	if (shapes.size()) {
		list_names = (char**)malloc(shapes.size() * sizeof(char*));
		list_labels = (wchar_t**)malloc(shapes.size() * sizeof(wchar_t*));
	}
	gui->GetListSize() = shapes.size();
	unsigned sizeprevious = 0;
	for (int s = 0; s < shapes.size(); s++)
	{
		PrimitiveShape* primitiveShape = shapes[s].first;
		PlanePrimitiveShape* planePrimitiveShape = (PlanePrimitiveShape*)primitiveShape;
		const Plane& plane = planePrimitiveShape->Internal();
		CPointEx3D normal;
		normal.x = plane.getNormal()[0];
		normal.y = plane.getNormal()[1];
		normal.z = plane.getNormal()[2];
		double d = -plane.SignedDistToOrigin();
		int i = pc.size() - (sizeprevious + shapes[s].second);
		CPointEx3D near_plane;
		near_plane.x = pc[i].pos[0];
		near_plane.y = pc[i].pos[1];
		near_plane.z = pc[i].pos[2];
		
		list_names[s] = (char*)malloc(1024 * sizeof(char));
		list_labels[s] = (wchar_t*)malloc(1024 * sizeof(wchar_t));
		char label[1024];
		sprintf(list_names[s], "Plane_%d", s);
		gui->GetPrimitiveList()[list_names[s]] = std::make_shared<RansacPlanePrimitive>(normal, d, near_plane);

		sprintf(label, "Plane : %f %f %f %f", normal.x, normal.y, normal.z, d);
		ProStringToWstring(list_labels[s], label);
		sizeprevious += shapes[s].second;
	}
	status = ProUIListNamesSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, gui->GetListSize(), gui->Get_List_Names());
	status = ProUIListLabelsSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, gui->GetListSize(), gui->Get_List_Labels());
	
	delete[] pnts_norms;
}

void GuiInterface::CreateCylinderPrimitive()
{
	gui->primitiveListClear();
	UIListClear();
	const std::vector<CPointEx3D>& buffer_pnts = gui->getBufferPntCylinder();
	Point* pnts_norms = new Point[buffer_pnts.size()];
	int i = 0;
	for (const CPointEx3D& query_pnt : buffer_pnts)
	{
		auto indexSet = pointCloud->GetKNearestNeighborIndex(query_pnt, 1);
		unsigned int nearest_index = indexSet[0];
		pnts_norms[i].pos[0] = (float)(*pointCloud)[nearest_index].first.x;
		pnts_norms[i].pos[1] = (float)(*pointCloud)[nearest_index].first.y;
		pnts_norms[i].pos[2] = (float)(*pointCloud)[nearest_index].first.z;
		pnts_norms[i].normal[0] = (float)(*pointCloud)[nearest_index].second.x;
		pnts_norms[i].normal[1] = (float)(*pointCloud)[nearest_index].second.y;
		pnts_norms[i].normal[2] = (float)(*pointCloud)[nearest_index].second.z;
		i++;
	}
	PointCloud pc(pnts_norms, buffer_pnts.size());
	Vec3f min; Vec3f max;
	computeBBox(pnts_norms, buffer_pnts.size(), min, max);
	pc.setBBox(min, max);
	pc.widenBBox(0.05f);
	RansacShapeDetector::Options ransacOptions;
	ProError status;
	ransacOptions.m_epsilon = (float)gui->getEpsilon(status) * pc.getScale();
	ransacOptions.m_bitmapEpsilon = (float)gui->getBitmapEpsilon(status) * pc.getScale();
	ransacOptions.m_normalThresh = (float)gui->getNormalThreshold(status);
	ransacOptions.m_minSupport = gui->getMinimumSupport(status);
	ransacOptions.m_probability = (float)gui->getProbability(status);

	RansacShapeDetector detector(ransacOptions);
	detector.Add(new CylinderPrimitiveShapeConstructor());

	MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>> shapes;
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);

	char**& list_names = gui->Get_List_Names();
	wchar_t**& list_labels = gui->Get_List_Labels();

	if (shapes.size()) {
		list_names = (char**)malloc(shapes.size() * sizeof(char*));
		list_labels = (wchar_t**)malloc(shapes.size() * sizeof(wchar_t*));
	}
	gui->GetListSize() = shapes.size();
	unsigned sizeprevious = 0;
	for (int s = 0; s < shapes.size(); s++)
	{
		PrimitiveShape* primitiveShape = shapes[s].first;
		CylinderPrimitiveShape* cylinderPrimitiveShape = (CylinderPrimitiveShape*)primitiveShape;
		const Cylinder& cylinder = cylinderPrimitiveShape->Internal();
		CPointEx3D axisDirection;
		axisDirection.x = cylinder.AxisDirection()[0];
		axisDirection.y = cylinder.AxisDirection()[1];
		axisDirection.z = cylinder.AxisDirection()[2];

		CPointEx3D axisPosition;
		axisPosition.x = cylinder.AxisPosition()[0];
		axisPosition.y = cylinder.AxisPosition()[1];
		axisPosition.z = cylinder.AxisPosition()[2];

		double radius = cylinder.Radius();

		CPointEx3D start;
		start.x = gui->GetCylinderStartPoint().x;
		start.y = gui->GetCylinderStartPoint().y;
		start.z = gui->GetCylinderStartPoint().z;

		CPointEx3D end;
		end.x = gui->GetCylinderEndPoint().x;
		end.y = gui->GetCylinderEndPoint().y;
		end.z = gui->GetCylinderEndPoint().z;

		list_names[s] = (char*)malloc(1024 * sizeof(char));
		list_labels[s] = (wchar_t*)malloc(1024 * sizeof(wchar_t));
		char label[1024];
		sprintf(list_names[s], "Cylinder_%d", s);
		gui->GetPrimitiveList()[list_names[s]] = std::make_shared<RansacCylinderPrimitive>(axisDirection, axisPosition, start, end, radius);

		sprintf(label, "Cylinder : (%f, %f, %f) %f", axisDirection.x, axisDirection.y, axisDirection.z, radius);
		ProStringToWstring(list_labels[s], label);
		sizeprevious += shapes[s].second;
	}
	status = ProUIListNamesSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, gui->GetListSize(), gui->Get_List_Names());
	status = ProUIListLabelsSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, gui->GetListSize(), gui->Get_List_Labels());

	delete[] pnts_norms;
}

std::vector<CPointEx3D>& GuiInterface::getExternalProtrusionPointsToProject()
{
	return general_extrusion_projected_points;
}

bool GuiInterface::isInThreshold(const Pro3dPnt& qpnt)
{
	RansacPlanePrimitive* plane = (RansacPlanePrimitive*)&*generalExtrusionReferencePlane;
	CPointEx3D& pnt = general_extrusion_inner_point;
	CPointEx3D inner_pnt = { pnt.x, pnt.y, pnt.z };
	CPointEx3D query_point{qpnt[0], qpnt[1], qpnt[2]};
	const CPointEx3D& normal = plane->GetNormal();
	CPointEx3D pnt_on_plane = plane->GetPointOnPlane();
	double distanceThreshold = 0.1 * fabs(signed_distance_point_from_surface(normal, pnt_on_plane, inner_pnt));
	double distance = fabs(signed_distance_point_from_surface(normal, inner_pnt, query_point));
	return distance < distanceThreshold;	
}

void GuiInterface::CreateSectionOnPointCloud()
{
	RansacPlanePrimitive* plane = (RansacPlanePrimitive*)&*generalExtrusionReferencePlane;
	CPointEx3D& pnt = general_extrusion_inner_point;						
	CPointEx3D inner_pnt = { pnt.x, pnt.y, pnt.z };
	Pro3dPnt proInnerPnt;
	proInnerPnt[0] = inner_pnt.x, proInnerPnt[1] = inner_pnt.y, proInnerPnt[2] = inner_pnt.z;
	
	const CPointEx3D& normal = plane->GetNormal();
	Pro3dPnt proNormal;
	proNormal[0] = normal.x, proNormal[1] = normal.y, proNormal[2] = normal.z;
	CPointEx3D pnt_on_plane = plane->GetPointOnPlane();
	double& distanceThreshold = general_extrusion_distance_threshold = 0.1 * fabs(signed_distance_point_from_surface(normal, pnt_on_plane, inner_pnt));
	general_extrusion_projected_points.clear();
	for (int i = 0; i < pointCloud->GetSize(); i++)
	{
		double distance = fabs(signed_distance_point_from_surface(normal, inner_pnt, (*pointCloud)[i].first));
		if (distance < distanceThreshold)
		{
			general_extrusion_projected_points.push_back((*pointCloud)[i].first);
		}
	}
	pBSplineSketch = std::make_shared<BSplineSketch>(proNormal, proInnerPnt, general_extrusion_projected_points, "sect");
	
	ProFeatureGeomitemVisit(&feat, PRO_POINT, (ProGeomitemAction)PointHighLightAction,
		NULL, NULL);
}

void GuiInterface::CreateGeneralExtrusion()
{
	gui->unhighlighPointCloud();
	gui->EraseBSplineSketch();
	gui->primitiveListClear();
	UIListClear();
	std::vector<CPointEx3D>& pnts_to_project = gui->getExternalProtrusionPointsToProject();
	pnts_to_project.clear();
	CPointEx3D& pnt = gui->GetGeneralExtrusionInnerPoint();
	CPointEx3D inner_pnt = { pnt.x, pnt.y, pnt.z };
	RansacPlanePrimitive* plane = (RansacPlanePrimitive*)&*gui->getGeneralProtrusionReferencePlane();
	const CPointEx3D& normal = plane->GetNormal();
	FILE* file = fopen("GeometricPlane.txt", "w");
	fprintf(file, "%lf %lf %lf\n", normal.x, normal.y, normal.z);
	fclose(file);
	CPointEx3D pnt_on_plane = plane->GetPointOnPlane();
	double& distanceThreshold = gui->getGeneralExtrusionDistanceThreshold() = 0.1 * fabs(signed_distance_point_from_surface(normal, pnt_on_plane, inner_pnt));
	for (int i = 0; i < pointCloud->GetSize(); i++)
	{
		double distance = fabs(signed_distance_point_from_surface(normal, inner_pnt, (*pointCloud)[i].first));
		if (distance < distanceThreshold)
		{
			pnts_to_project.push_back((*pointCloud)[i].first);
		}
	}
	
	const CPointEx3D& extent_point = gui->getGeneralExtentPoint();
	double depth = fabs(signed_distance_point_from_surface(normal, pnt_on_plane, extent_point));
	CPointEx3D direction = { extent_point.x - inner_pnt.x, extent_point.y - inner_pnt.y, extent_point.z - inner_pnt.z };
	bool flipDirection = direction.x * normal.x + direction.y * normal.y + direction.z * normal.z < 0;
	gui->GetListSize() = 1;
	char**& list_names = gui->Get_List_Names();
	wchar_t**& list_labels = gui->Get_List_Labels();

	list_names = (char**)malloc(gui->GetListSize() * sizeof(char*));
	list_labels = (wchar_t**)malloc(gui->GetListSize() * sizeof(wchar_t*));
	
	list_names[0] = (char*)malloc(1024 * sizeof(char));
	list_labels[0] = (wchar_t*)malloc(1024 * sizeof(wchar_t));
	char label[1024];
	sprintf(list_names[0], "GEN_EXTR_%d", 0);
	gui->GetPrimitiveList()[list_names[0]] = std::make_shared<ExtrudePrimitive>(normal, inner_pnt, pnts_to_project, plane->GetPrimitiveFeatureId(), flipDirection, depth);

	sprintf(label, "General Extrusion %d", 0);
	ProStringToWstring(list_labels[0], label);
	
	ProError status = ProUIListNamesSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, gui->GetListSize(), gui->Get_List_Names());
	status = ProUIListLabelsSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, gui->GetListSize(), gui->Get_List_Labels());
}

std::shared_ptr<RansacPrimitive>& GuiInterface::GetActivePrimitive()
{
	return active_primitive;
}

std::map<std::string, std::shared_ptr<RansacPrimitive>>& GuiInterface::GetPrimitiveList()
{
	return primitive_list;
}

void GuiInterface::primitiveListClear()
{
	if (active_primitive)
		active_primitive->Destroy();
	active_primitive = nullptr;
	primitive_list.clear();
}

wchar_t**& GuiInterface::Get_List_Labels()
{
	return list_labels;
}

char**& GuiInterface::Get_List_Names()
{
	return list_names;
}

int& GuiInterface::GetListSize()
{
	return list_size;
}

void GuiInterface::UIListClear()
{
	for (int i = 0; i < gui->GetListSize(); i++)
	{
		free(gui->Get_List_Labels()[i]);
		free(gui->Get_List_Names()[i]);
	}
	free(gui->Get_List_Labels());
	gui->Get_List_Labels() = NULL;
	free(gui->Get_List_Names());
	gui->Get_List_Names() = NULL;
	ProError status = ProUIListNamesSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, 0, gui->Get_List_Names());
	status = ProUIListLabelsSet((char*)UI_GUI_NAME, (char*)UI_PRIMITIVE_LIST, 0, gui->Get_List_Labels());
	gui->GetListSize() = 0;
}

GuiInterface::~GuiInterface()
{
	UIListClear();
}

int& GuiInterface::GetAcceptedCylindersCount()
{
	return accepted_cylinders_count;
}

int& GuiInterface::GetAcceptedPlanesCount()
{
	return accepted_planes_count;
}

std::map<int, std::shared_ptr<RansacPrimitive>>& GuiInterface::GetAcceptedPrimitiveList()
{
	return accepted_primitives;
}

CPointEx3D& GuiInterface::GetCylinderStartPoint()
{
	return cylinder_starting_point;
}

CPointEx3D& GuiInterface::GetCylinderEndPoint()
{
	return cylinder_ending_point;
}

CPointEx3D& GuiInterface::GetGeneralExtrusionInnerPoint()
{
	return general_extrusion_inner_point;
}

double GuiInterface::signed_distance_point_from_surface(const CPointEx3D& normal, const CPointEx3D& point_on_plane, const CPointEx3D& query_point)
{
	CPointEx3D v;
	v.x = query_point.x - point_on_plane.x;
	v.y = query_point.y - point_on_plane.y;
	v.z = query_point.z - point_on_plane.z;
	return normal.x * v.x + normal.y * v.y + normal.z * v.z;
}

double& GuiInterface::getGeneralExtrusionDistanceThreshold()
{
	return general_extrusion_distance_threshold;
}