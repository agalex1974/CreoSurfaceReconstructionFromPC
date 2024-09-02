#include <iostream>
#include <Windows.h>
#include <memory>
#include <thread>
#include <experimental/filesystem>
#include <ProUICmd.h>
#include <ProMenuBar.h>
#include "CCreoSOR.h"
#include "CCreoPointCloud.h"
#include <ctime>
#include <chrono>

#include "BestPlaneFitInterface.h"
#include "GuiInterface.h"
#include "SectionGUI.h"
#include "TaubinDialog.h"

#ifdef __cplusplus
extern "C" {
#endif
	int user_initialize(int	argc, char* argv[], char* version, char* build, wchar_t errbuf[80]);
	void user_terminate();
#ifdef __cplusplus
}
#endif

/*=============================================================*\
  Function: 	ProTKGDAccess
  Purpose:		Determine if the action should be accessible from menus
\*=============================================================*/
static uiCmdAccessState accessFunction(uiCmdAccessMode access_mode)
{
	ProMdl current;
	ProMdlType type;
	ProError status;

	status = ProMdlCurrentGet(&current);
	if (status != PRO_TK_NO_ERROR) return ACCESS_INVISIBLE;

	status = ProMdlTypeGet(current, &type);
	if (status != PRO_TK_NO_ERROR || type != PRO_MDL_PART)
		return ACCESS_INVISIBLE;

	return ACCESS_AVAILABLE;
}
const wchar_t* MSGFIL = L"messages.txt";
std::shared_ptr<CCreoPointCloud> spointCloud;
std::shared_ptr<CCreoSOR> sor;
CCreoPointCloud* pointCloud;
GuiInterface* gui;
SectionGUI* sectionGUI;
BestPlaneFitInterface* bpfGUI;

void on_point_cloud_load()
{
	ProError err;
	ProPath filePath;
	char strFilter[81] = "*.pts";
	ProLine filter;
	ProStringToWstring(filter, strFilter);
	err = ProFileMdlnameOpen(NULL, filter, NULL, NULL, NULL, NULL, filePath);
	char strFilePathPts[260];
	char strFilePathXYZ[260];
	memset(strFilePathPts, 0, 260);
	ProWstringToString(strFilePathPts, filePath);
	if (std::string(strFilePathPts).find(".pts") != std::string::npos) {
		strcpy(strFilePathXYZ, strFilePathPts);
		int length = (int)strlen(strFilePathXYZ);
		strFilePathXYZ[length - 3] = 'x';
		strFilePathXYZ[length - 2] = 'y';
		strFilePathXYZ[length - 1] = 'z';
		if (std::experimental::filesystem::exists(strFilePathXYZ))
		{
			spointCloud = std::make_shared<CCreoPointCloud>(filePath, strFilePathXYZ);
			pointCloud = &*spointCloud;
		}
		else
		{
			strFilePathXYZ[length - 3] = 'p';
			strFilePathXYZ[length - 2] = 't';
			strFilePathXYZ[length - 1] = 's';
			spointCloud = std::make_shared<CCreoPointCloud>(filePath, strFilePathXYZ, true);
			pointCloud = &*spointCloud;
		}
	}
}

void on_sor_create()
{
	if (pointCloud)
	{
		sor = std::make_shared<CCreoSOR>(*pointCloud, "SOR");
	}
}

void label_points()
{
	if (pointCloud)
	{
		//std::vector<std::vector<int>> regions;
		//pointCloud->SegmentPointCloud(regions);
		//pointCloud->ComputeNormalsWithPCA();
		//pointCloud->SmoothPointCloud();
		//pointCloud->ComputeNormalsWithPCA();
		//pointCloud->SmoothPointCloudCGALBilateral();
		spointCloud = spointCloud->sampleUniformly();
	}	
}

void select_points_for_sor_creation()
{
	if (spointCloud)
	{
		std::set<int> selectionIndexes;
		spPointCloud selectedPointCloud = pointCloud->pointCloudFromSelection(selectionIndexes);
		spointCloud->computePrincipalCurvatures();
		if (selectedPointCloud->GetSize() == 1)
		{
			std::set<int> indexes;
			auto patch = pointCloud->GetPointsWithinRadiusFromPoint((*selectedPointCloud)[0].first, 7.5, indexes, *selectionIndexes.begin());
			CPointEx3D max, min;
			auto curvatures = CPointCloud::GetCurvaturesOfLocalPatch((*selectedPointCloud)[0].first, (*selectedPointCloud)[0].second, *patch, max, min);
			double k1 = curvatures.first;
			double k2 = curvatures.second;
			double c = sqrt(0.5 * (k1 * k1 + k2 * k2));
			FILE* file = fopen("curvatures.txt", "w");
			fprintf(file, "%lf %lf %lf\n", k1, k2, c);
			fclose(file);
			//auto slippableMatrix = selectedPointCloud->getSlippableVectors(pointCloud->FindBaryCenter());
			/*auto slippableMatrix = selectedPointCloud->getSlippableVectors(selectedPointCloud->FindBaryCenter());
			//auto slippableMatrix = selectedPointCloud->getSlippableVectors(CPointEx3D(0, 0, 0));
			if (slippableMatrix)
			{
				FILE* file = fopen("slippableMatrix.txt", "w");
				for (int i = 0; i < slippableMatrix->GetCols(); i++)
				{
					for (int j = 0; j < slippableMatrix->GetRows(); j++)
					{
						fprintf(file, "%lf ", (*slippableMatrix)(j, i));
					}
					fprintf(file, "\n");
				}
				fclose(file);
			}*/
		}
	}
}

void computerNormalsWithPCA()
{
	if (pointCloud) {
		pointCloud->ComputeNormalsWithPCA();
		//pointCloud->SmoothPointCloudCGALBilateral();
		//pointCloud->savePointCloud("BilateralSmooth.xyz");
	}
	
}

void denoise_points()
{
	if (spointCloud)
	{
		//pointCloud->computeCGALPrincipalCurvatures();
		pointCloud->SmoothPointCloudCGALBilateral();
		//pointCloud->bilateral_iterative(15, 0.05, 0.05, 0.65);
		pointCloud->savePointCloud("FeatureSensitiveBilateral.xyz");
		//spointCloud = spointCloud->sampleUniformlyByUpsampling();
		//spointCloud->SmoothPointCloud();
		//pointCloud = spointCloud.get();
	}
}

void wlop_denoise_points()
{
	if (spointCloud)
	{
		spointCloud = pointCloud->wlop();
		pointCloud = spointCloud.get();
		pointCloud->ComputeNormalsWithPCA();
		pointCloud->savePointCloud("WLOP.xyz");
	}
}

void jet_denoise_points()
{
	if (spointCloud)
	{
		spointCloud = pointCloud->SmoothPointCloud();
		pointCloud = spointCloud.get();
		pointCloud->ComputeNormalsWithPCA();
		pointCloud->savePointCloud("JetSmooth.xyz");
	}
}

void compute_curvatures()
{
	if (spointCloud && spointCloud->hasNormals()) {
		spointCloud->computeCGALPrincipalCurvatures();
		//spointCloud->GetTaubinCurvatures();
		//spointCloud->CreateMesh();
		//spointCloud->createMeshTriangular();
		//spointCloud->smooth_iterative(4, 0.7);
		//spointCloud->save_smoothed_tringulation();
		//spointCloud->savePointCloud("smoothedPointCloud.xyz");
	}
}

void save_point_cloud()
{
	if (spointCloud)
	{
		if (!spointCloud->hasNormals())
		{
			spointCloud->ComputeNormalsWithPCA();
		}
		spointCloud->savePointCloud("savedModel.xyz", true);
		spointCloud->savePointCloud("savedModel.pts", false);
	}
}
void uniform_sample()
{
	if (spointCloud)
	{
		 spointCloud = spointCloud->sampleUniformly();
	}
}
int main()
{
	
}
int user_initialize(
	int		argc,			 /* Inp: Pro/E arg count */
	char* argv[],			 /* Inp: Pro/E args	 */
	char* version,			 /* Inp: Pro/E version	 */
	char* build, 			 /* Inp: Pro/E build date code */
	wchar_t errbuf[80])		 /* Out: error message (opt)   */
{
	/* Declare external functions */
	ProError status;
	uiCmdCmdId cmd_id_load_point_cloud;
	uiCmdCmdId cmd_id_create_sor;
	uiCmdCmdId cmd_id_normals_pca;
	uiCmdCmdId cmd_id_smooth_points;
	uiCmdCmdId cmd_id_wlop_smooth_points;
	uiCmdCmdId cmd_id_jet_smooth_points;
	uiCmdCmdId cmd_id_ransac_primitives;
	uiCmdCmdId cmd_id_section;
	uiCmdCmdId cmd_bpf_id;
	uiCmdCmdId cmd_cpt_curv;
	uiCmdCmdId cmd_save_point_cloud;
	uiCmdCmdId cmd_taubin_smooth;
	gui = new GuiInterface();
	sectionGUI = new SectionGUI();
	bpfGUI = new BestPlaneFitInterface();

	ProMenubarmenuMenuAdd((char*)"Applications", const_cast<char*>("-PointCloud"), const_cast<char*>("-PointCloud"), NULL,
		PRO_B_TRUE, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("LoadPointCloud"), (uiCmdCmdActFn)(on_point_cloud_load),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_load_point_cloud);

	status = ProMenubarmenuPushbuttonAdd( 
		const_cast<char*>("-PointCloud"), const_cast<char*>("-LoadPoints"),
		const_cast<char*>("-LoadPoints"), const_cast<char*>("-LoadPointsHelp"),
		nullptr, PRO_B_TRUE, cmd_id_load_point_cloud, const_cast<wchar_t*>(MSGFIL));

	ProMenubarmenuMenuAdd(const_cast<char*>("-PointCloud"), const_cast<char*>("-CreatePrimitives"),
		const_cast<char*>("-CreatePrimitives"), const_cast<char*>("-LoadPoints"),
		PRO_B_TRUE, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("CreateSOR"), (uiCmdCmdActFn)(on_sor_create),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_create_sor);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-CreatePrimitives"), const_cast<char*>("-CreateSOR"),
		const_cast<char*>("-CreateSOR"), const_cast<char*>("-CreateSORHelp"),
		nullptr, PRO_B_TRUE, cmd_id_create_sor, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("RansacPrimitives"), (uiCmdCmdActFn)(GuiInterface::GuiActivate),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_ransac_primitives);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-CreatePrimitives"), const_cast<char*>("-CreateRansacPrimitives"),
		const_cast<char*>("-CreateRansacPrimitives"), const_cast<char*>("-CreateRansacPrimitivesHelp"),
		const_cast<char*>("-CreateSOR"), PRO_B_TRUE, cmd_id_ransac_primitives, const_cast<wchar_t*>(MSGFIL));
	
	status = ProCmdActionAdd(const_cast<char*>("NormalsPCA"), (uiCmdCmdActFn)(computerNormalsWithPCA),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_normals_pca);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-NormalsPCA"),
		const_cast<char*>("-NormalsPCA"), const_cast<char*>("-NormalsPCAHelp"),
		const_cast<char*>("-CreatePrimitives"), PRO_B_TRUE, cmd_id_normals_pca, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("DeNoisePoints"), (uiCmdCmdActFn)(denoise_points),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_smooth_points);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-SmoothPoints"),
		const_cast<char*>("-SmoothPoints"), const_cast<char*>("-SmoothPointsHelp"),
		const_cast<char*>("-NormalsPCA"), PRO_B_TRUE, cmd_id_smooth_points, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("WLOPDeNoisePoints"), (uiCmdCmdActFn)(wlop_denoise_points),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_wlop_smooth_points);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-WLOPSmoothPoints"),
		const_cast<char*>("-WLOPSmoothPoints"), const_cast<char*>("-WLOPSmoothPointsHelp"),
		const_cast<char*>("-SmoothPoints"), PRO_B_TRUE, cmd_id_wlop_smooth_points, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("JETDeNoisePoints"), (uiCmdCmdActFn)(jet_denoise_points),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_jet_smooth_points);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-JETSmoothPoints"),
		const_cast<char*>("-JETSmoothPoints"), const_cast<char*>("-JETSmoothPointsHelp"),
		const_cast<char*>("-WLOPSmoothPoints"), PRO_B_TRUE, cmd_id_jet_smooth_points, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("ChooseSection"), (uiCmdCmdActFn)(SectionGUI::Activate),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_id_section);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-SectionUI"),
		const_cast<char*>("-SectionUI"), const_cast<char*>("-SectionUIHelp"),
		const_cast<char*>("-JETSmoothPoints"), PRO_B_TRUE, cmd_id_section, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("CreateBestFitPlane"), (uiCmdCmdActFn)(BestPlaneFitInterface::Activate),
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_bpf_id);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-BestFitPlane"),
		const_cast<char*>("-BestFitPlane"), const_cast<char*>("-BestFitPlaneHelp"),
		const_cast<char*>("-SectionUI"), PRO_B_TRUE, cmd_bpf_id, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("CalculateCurvatures"), (uiCmdCmdActFn)compute_curvatures,
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_cpt_curv);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-ComputeCurvatures"),
		const_cast<char*>("-ComputeCurvatures"), const_cast<char*>("-ComputeCurvaturesHelp"),
		const_cast<char*>("-BestFitPlane"), PRO_B_TRUE, cmd_cpt_curv, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("SavePointCloud"), (uiCmdCmdActFn)save_point_cloud,
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_save_point_cloud);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-SavePointCloud"),
		const_cast<char*>("-SavePointCloud"), const_cast<char*>("-SavePointCloudHelp"),
		const_cast<char*>("-ComputeCurvatures"), PRO_B_TRUE, cmd_save_point_cloud, const_cast<wchar_t*>(MSGFIL));

	status = ProCmdActionAdd(const_cast<char*>("TaubinSmooth"), (uiCmdCmdActFn)TaubinDialog::Activate,
		uiProe2ndImmediate, accessFunction, PRO_B_TRUE, PRO_B_TRUE, &cmd_taubin_smooth);

	status = ProMenubarmenuPushbuttonAdd(
		const_cast<char*>("-PointCloud"), const_cast<char*>("-TaubinSmooth"),
		const_cast<char*>("-TaubinSmooth"), const_cast<char*>("-TaubinSmoothHelp"),
		const_cast<char*>("-SavePointCloud"), PRO_B_TRUE, cmd_taubin_smooth, const_cast<wchar_t*>(MSGFIL));

	return (0);
}

/*=============================================================*\
  Function: 	user_terminate
  Purpose:	To handle any termination actions
\*=============================================================*/
void user_terminate()
{
	delete gui;
	delete bpfGUI;
	delete sectionGUI;
}
