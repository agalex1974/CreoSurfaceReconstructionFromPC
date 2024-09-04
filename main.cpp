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

#include "GuiInterface.h"

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
	uiCmdCmdId cmd_id_ransac_primitives;

	gui = new GuiInterface();

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
	return (0);
}

/*=============================================================*\
  Function: 	user_terminate
  Purpose:	To handle any termination actions
\*=============================================================*/
void user_terminate()
{
	delete gui;
}
