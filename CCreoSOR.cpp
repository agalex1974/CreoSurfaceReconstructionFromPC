#include "CCreoSOR.h"
#include <ProSecdim.h>
#include "UtilMatrix.h"
#
#define SIZEOFARR(a) (sizeof(a)/sizeof(a[0]))

CCreoSOR::CCreoSOR(const CPointCloud& pnts, const char* name) :
	CSOR(pnts)
{
	// Create a full revolution
	tree[19].data.v.d = 0.;
	std::string sorName(name);
	auto bc = pnts.FindBaryCenter();
	CVector<CPointEx3D> generatixPnts;
	CVector<CPointEx3D> symGeneratrixPnts;
	CPointEx3D axisOfRevolution;
	ExtractAxisAndGeneratrix(generatixPnts, symGeneratrixPnts, axisOfRevolution);
	CMatrix<double>	transformation = GetTransormationMatrix();
	auto normalOfPlane = Mult(transformation, CPointCloud::PointToVector(CPointEx3D(0, 1, 0), false));
	ProPoint3d normal;
	normal[0] = normalOfPlane[0]; normal[1] = normalOfPlane[1]; normal[2] = normalOfPlane[2];
	ProPoint3d pnt;
	pnt[0] = bc[0]; pnt[1] = bc[1]; pnt[2] = bc[2];
	std::string sketchPlaneName = sorName + "_GenPlane";
	generatrixSketchPlane = std::make_shared<CCreoPlane>(normal, pnt, sketchPlaneName.c_str());

	ProPoint3d axisOfRevolutionVector;
	axisOfRevolutionVector[0] = axisOfRevolution[0];
	axisOfRevolutionVector[1] = axisOfRevolution[1];
	axisOfRevolutionVector[2] = axisOfRevolution[2];

	ProPoint3d pntOnNormal;
	pntOnNormal[0] = bc[0] + 0.1 * axisOfRevolutionVector[0];
	pntOnNormal[1] = bc[1] + 0.1 * axisOfRevolutionVector[1];
	pntOnNormal[2] = bc[2] + 0.1 * axisOfRevolutionVector[2];
	std::string axisOfSymmetryName = sorName + "_sym_axis";

	axisOfSymmetry = std::make_shared<CDatumAxis>(pnt, pntOnNormal, axisOfSymmetryName.c_str());
	Pro3dPnt orthogonalToAxisOfSymmetryVector;
	crossProduct(axisOfRevolutionVector, normal, orthogonalToAxisOfSymmetryVector);
	pntOnNormal[0] = bc[0] + 0.1 * orthogonalToAxisOfSymmetryVector[0];
	pntOnNormal[1] = bc[1] + 0.1 * orthogonalToAxisOfSymmetryVector[1];
	pntOnNormal[2] = bc[2] + 0.1 * orthogonalToAxisOfSymmetryVector[2];
	std::string orthToAxisOfSymmetryName = sorName + "_orth_sym_axis";
	orthogonalToAxisOfSymmetry = std::make_shared<CDatumAxis>(pnt, pntOnNormal, orthToAxisOfSymmetryName.c_str());
	std::string orthToAxisPlaneOfSymmetryName = sorName + "_orth_sym_plane";
	orthogonalPlaneToAxisOfSymmetry = std::make_shared<CCreoPlane>(orthogonalToAxisOfSymmetryVector, pnt, orthToAxisPlaneOfSymmetryName.c_str());;
	createSOR(generatixPnts, symGeneratrixPnts, normal, axisOfRevolution, name);
}

void CCreoSOR::NormalizeVector(Pro3dPnt& normal)
{
	double norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
	normal[0] /= norm; normal[1] /= norm; normal[2] /= norm;
}

void CCreoSOR::crossProduct(const Pro3dPnt& vect_A, const Pro3dPnt& vect_B, Pro3dPnt& cross_P)
{
	cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
	cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
	cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
	NormalizeVector(cross_P);
}


bool CCreoSOR::createSOR(ProMdl model, Parameter* params, int* sketchId, const char* featureName, int* side_id, int* bot_id, int* center_line_id)
{
	ProElement				elem_tree;
	ProElement				sketch_element;

	ProError				err;

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
		ProFeature feature;
		ProError status = ProFeatureInit(static_cast<ProSolid>(model), sorId, &feature);
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
		err = sketchSection(section, params, sketchId, side_id, bot_id, center_line_id);
		if (err != PRO_TK_NO_ERROR)
			break;
		/*------------------------------------------------------------*\
		Set section-dependent element values
		\*------------------------------------------------------------*/

		UtilityHelperClass::ProUtilFeatureSetSectionDependentValues(elem_tree, tree, SIZEOFARR(tree));

		opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;
		err = ProFeatureRedefine(NULL, &feature, elem_tree, opts, 1, &errs);

		ProSelection selection;
		ProModelitem modelitem;
		ProSelectionAlloc(NULL, &feature, &selection);
		err = ProSelectionModelitemGet(selection, &modelitem);
		sorInternalId = modelitem.id;
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

void CCreoSOR::CreateElementTree(ProMdl model)
{
	ProModelitem			model_item;
	ProSelection			model_sel;
	ProFeatureCreateOptions opts[1];
	ProElement				elem_tree;
	ProErrorlist			errs;
	ProFeature				feature;

	ProError err = UtilityHelperClass::ProUtilElemtreeCreate(tree, SIZEOFARR(tree), NULL, &elem_tree);
	/*----------------------------------------------------------*\
	Create the incomplete protrusion in the current model
	\*----------------------------------------------------------*/
	err = ProMdlToModelitem(model, &model_item);
	err = ProSelectionAlloc(NULL, &model_item, &model_sel);

	opts[0] = PRO_FEAT_CR_INCOMPLETE_FEAT;

	err = ProFeatureCreate(model_sel, elem_tree, opts, 1, &feature, &errs);
	sorId = feature.id;

	err = ProElementFree(&elem_tree);
	err = ProSelectionFree(&model_sel);
}

ProError CCreoSOR::sketchSection(
	ProSection section,		/*In : the section */
	Parameter* params,
	int* extrusionId,
	int* orientation_plane_projection_id,
	int* cutting_plane_projection_id,
	int* center_line_id)

{
	Pro2dSplinedef		spline;
	Pro2dEntdef* ent1, * ent2;
	Pro2dLinedef* line1;
	ProError			err;
	ProWSecerror		errors;
	ProMatrix location_matrix;
	ProMatrix inv;

	/*----------------------------------------------------------*\
	Create entities
	\*----------------------------------------------------------*/

	err = ProSectionEntityFromProjection(section, params[1].r, orientation_plane_projection_id);
	err = ProSectionEntityFromProjection(section, params[2].r, cutting_plane_projection_id);

	err = ProSectionEntityGet(section, *orientation_plane_projection_id, &ent1);
	line1 = (Pro2dLinedef*)ent1;
	line1->type = PRO_2D_LINE;
	err = ProSectionEntityGet(section, *cutting_plane_projection_id, &ent2);

	// Get the transformation matrix from Sketch -> World
	err = ProSectionLocationGet(section, location_matrix);

	// Get the transformation matrix from World -> Sketch
	ProUtilMatrixInvert(location_matrix, inv);
	CPointEx3D* axis = (CPointEx3D*)params[3].p;

	Pro2dPnt projectedNormal;
	projectedNormal[0] = (*axis)[0] * inv[0][0] + (*axis)[1] * inv[1][0] + (*axis)[2] * inv[2][0];
	projectedNormal[1] = (*axis)[0] * inv[0][1] + (*axis)[1] * inv[1][1] + (*axis)[2] * inv[2][1];
	double z = (*axis)[0] * location_matrix[2][0] + (*axis)[1] * location_matrix[2][1] + (*axis)[2] * location_matrix[2][2];

	std::vector<CPointEx> poly_line;
	std::vector<CPointEx> poly_line_sym;
	CVector<CPointEx3D>* pp = (CVector<CPointEx3D>*)params[0].p;
	CVector<CPointEx3D>* ppsym = (CVector<CPointEx3D>*)params[4].p;
	// transform from world -> sketch 
	for (int i = 0; i < pp->GetSize(); i += 100)
	{
		Pro2dPnt projectedPointOnSketch;
		projectedPointOnSketch[0] = (*pp)[i][0] * inv[0][0] + (*pp)[i][1] * inv[1][0] + (*pp)[i][2] * inv[2][0] + inv[3][0];
		projectedPointOnSketch[1] = (*pp)[i][0] * inv[0][1] + (*pp)[i][1] * inv[1][1] + (*pp)[i][2] * inv[2][1] + inv[3][1];
		//projectedPointOnSketch[2] = pp.x * inv[0][2] + pp.y * inv[1][2] + pp.z * inv[2][2] + inv[3][2];
		poly_line.push_back({ projectedPointOnSketch[0], projectedPointOnSketch[1] });
	}

	Pro2dClinedef cline;
	cline.type = PRO_2D_CENTER_LINE;
	cline.end1[0] = line1->end1[0];
	cline.end1[1] = line1->end1[1];
	cline.end2[0] = line1->end1[0] + projectedNormal[0];
	cline.end2[1] = line1->end1[1] + projectedNormal[1];
	axisInSketchStart.x = line1->end1[0];
	axisInSketchStart.y = line1->end1[1];
	axisInSketchEnd.x = line1->end1[0] + projectedNormal[0];
	axisInSketchEnd.y = line1->end1[1] + projectedNormal[1];
	err = ProSectionEntityAdd(section, (Pro2dEntdef*)&cline, center_line_id);

	int n = (int)poly_line.size();
	spline.n_points = n;
	spline.tangency_type = PRO_2D_SPLINE_TAN_NONE;
	spline.point_arr = (Pro2dPnt*)malloc((spline.n_points) * sizeof(Pro2dPnt));
	spline.type = PRO_2D_SPLINE;
	Pro2dPnt d;
	for (int i = 0; i < n; i++)
	{
		d[0] = poly_line[i].x - line1->end1[0];
		d[1] = poly_line[i].y - line1->end1[1];
		spline.point_arr[i][0] = line1->end1[0] + d[0];
		spline.point_arr[i][1] = line1->end1[1] + d[1];
	}
	err = ProSectionEntityAdd(section, (Pro2dEntdef*)&spline, extrusionId);

	/*----------------------------------------------------------*\
	Solve section
	\*----------------------------------------------------------*/
	err = ProSecerrorAlloc(&errors);
	err = ProSectionAutodim(section, &errors);
	err = ProSecerrorFree(&errors);

	return (err);
}

void CCreoSOR::setTheDrawingPlaneReference(ProMdl model, int surfaceId)
{
	ProError status;
	status = UtilityHelperClass::ProUtilSelectionFromSurfaceId(model, surfaceId, &tree[7].data.v.r);
}

void CCreoSOR::setTheDrawingPlaneOrientation(ProMdl model, int surfaceId)
{
	ProError status;
	status = UtilityHelperClass::ProUtilSelectionFromSurfaceId(model, surfaceId, &tree[10].data.v.r);
}

void CCreoSOR::createSOR(const CVector<CPointEx3D>& pnts, const CVector<CPointEx3D>& symPnts, Pro3dPnt normalCreo, CPointEx3D axis, const char* name)
{
	ProMdl model;
	ProError status = ProMdlCurrentGet(&model);
	setTheDrawingPlaneReference(model, generatrixSketchPlane->GetId());
	//CPointEx3D normal{ normalCreo[0], normalCreo[1], normalCreo[2] };
	//setTheAxisReference(model, axisOfSymmetry->GetAxisId());
	setTheDrawingPlaneOrientation(model, orthogonalPlaneToAxisOfSymmetry->GetId());
	Parameter params[5];
	params[0].p = (void*)&pnts;

	ProAxis axis1;
	ProModelitem modelitem1;
	ProError err = ProAxisInit((ProSolid)model, axisOfSymmetry->GetAxisId(), &axis1);
	err = ProAxisToGeomitem((ProSolid)model, axis1, (ProGeomitem*)&modelitem1);
	err = ProSelectionAlloc(NULL, &modelitem1, &params[1].r);

	ProAxis axis2;
	ProModelitem modelitem2;
	err = ProAxisInit((ProSolid)model, orthogonalToAxisOfSymmetry->GetAxisId(), &axis2);
	err = ProAxisToGeomitem((ProSolid)model, axis2, (ProGeomitem*)&modelitem2);
	err = ProSelectionAlloc(NULL, &modelitem2, &params[2].r);
	params[3].p = &axis;
	params[4].p = (void*)&symPnts;
	createSOR(model, params, &sketch_id, name, &orientation_id, &cutting_id, &center_line_id);
	//status = ProSelectionFree(&params[2].r);
}

CCreoSOR::~CCreoSOR()
{
	ProMdl part;
	ProFeatureDeleteOptions opt[] = { PRO_FEAT_DELETE_CLIP };
	ProError status = ProMdlCurrentGet(&part);
	status = ProFeatureDelete((ProSolid)part, &sorInternalId, 1, opt, 1);
}

void CCreoSOR::setTheAxisReference(ProMdl model, int axisId)
{
	ProAxis axis;
	ProModelitem modelitem;

	ProError err = ProAxisInit((ProSolid)model, axisId, &axis);
	err = ProAxisToGeomitem((ProSolid)model, axis, (ProGeomitem*)&modelitem);
	err = ProSelectionAlloc(NULL, &modelitem, &tree[14].data.v.r);
}

CMatrix<double> CCreoSOR::getTransformationMatrix(CPointEx3D vecFrom, CPointEx3D vecTo)
{
	vecFrom.normalize();
	vecTo.normalize();
	auto x = cross(vecFrom, vecTo);
	x.normalize();
	vecFrom.normalize();
	vecTo.normalize();
	double theta = acos(dot(vecFrom, vecTo));
	CMatrix<double> A(3, 3);
	A(0, 0) = 0.0;
	A(0, 1) = -x[2];
	A(0, 2) = x[1];
	A(1, 0) = x[2];
	A(1, 1) = 0.0;
	A(1, 2) = -x[0];
	A(2, 0) = -x[3];
	A(2, 1) = x[0];
	A(2, 2) = 0.0;
	CMatrix<double> Term2(A);
	Mult(Term2, sin(theta));
	CMatrix<double> Term3(A);
	Term3 = Mult(Term3, Term3);
	Mult(Term3, 1 - cos(theta));
	CMatrix<double> Term(3, 3);
	CMatrix<double> R(3, 3);
	R.Identity();
	Add(R, Term2, Term);
	R = Term;
	Add(R, Term3, Term);
	R = Term;
	return R;
}

void CCreoSOR::AugmentPointCloud(std::shared_ptr<const CCreoPointCloud>& creoPointCloud, spPointCloud pointCloudToAugment, std::set<int>& indexSet)
{
	auto bc = creoPointCloud->FindBaryCenter();
	auto slippableMatrixSelection = pointCloudToAugment->getSlippableVectors(bc);
	if (slippableMatrixSelection)
	{
		for (int i = 0; pointCloudToAugment->GetSize(); i++)
		{
			//creoPointCloud->GetKNearestNeighborIndex()
		}
	}
}
