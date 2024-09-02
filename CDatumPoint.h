#ifndef DATUM_POINT_H
#define DATUM_POINT_H

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
#include "HelperStructures.h"

class CDatumPoint
{
public:
	/**
	 * Constructor of the datum point
	 *
	 * @param pnt The 3D point
	 * @param name The name of the datum point
	 */
	CDatumPoint(const Pro3dPnt& pnt, const char* name);
	/**
	 * Get the point Id that is internal in Creo's UI
	 *
	 * @return the point intern Id
	 */
	int GetId() const;
	/**
	 * Get the point Id that is internal in Creo's UI
	 *
	 * @return the point intern Id
	 */
	int GetInternalId() const;
	/**
	 * Set the name of the datum point as displayed in the UI
	 *
	 * @param name The name of the datum point as displayed in the UI
	 */
	void SetName(const char* name);
	ProError getDatumPointCoordinates(ProPoint3d& pnt) const;
	~CDatumPoint();
private:
	
	ElemTreeData point_feature_tree[11] = {
		/* 0 */ {0, PRO_E_FEATURE_TREE, {(ProValueDataType)-1}},
		/* 1 */ {1, PRO_E_FEATURE_TYPE, {PRO_VALUE_TYPE_INT, PRO_FEAT_DATUM_POINT}},
		/* 2 */ {1, PRO_E_DPOINT_TYPE,  {PRO_VALUE_TYPE_INT, PRO_DPOINT_TYPE_OFFSET_CSYS}},
		/* 3 */ //{1, PRO_E_STD_FEATURE_NAME,  {(PRO_VALUE_TYPE_WSTRING)}},
		/* 3 */ {1, PRO_E_DPOINT_OFST_CSYS_TYPE,  {PRO_VALUE_TYPE_INT, PRO_DTMPNT_OFFCSYS_CARTESIAN}},
		/* 4 */	{1, PRO_E_DPOINT_OFST_CSYS_REF, {PRO_VALUE_TYPE_SELECTION}},
		/* 5 */ {1, PRO_E_DPOINT_OFST_CSYS_WITH_DIMS,  {PRO_VALUE_TYPE_INT, PRO_B_TRUE}},
		/* 6 */ {1, PRO_E_DPOINT_OFST_CSYS_PNTS_ARRAY,  {(ProValueDataType)-1}},
		/* 7 */ {2, PRO_E_DPOINT_OFST_CSYS_PNT,  {(ProValueDataType)-1}},
		/* 9 */ //{3, PRO_E_DPOINT_OFST_CSYS_PNT_NAME,  {(PRO_VALUE_TYPE_WSTRING)}},
		/* 8 */ {3, PRO_E_DPOINT_OFST_CSYS_DIR1_VAL,  {(PRO_VALUE_TYPE_DOUBLE)}},
		/* 9 */ {3, PRO_E_DPOINT_OFST_CSYS_DIR2_VAL,  {(PRO_VALUE_TYPE_DOUBLE)}},
		/*10 */ {3, PRO_E_DPOINT_OFST_CSYS_DIR3_VAL,  {(PRO_VALUE_TYPE_DOUBLE)}}
	};
	/**
	 * Set as coordinate system the coordinate system of creo  
	 */
	void setCsys();

	//void setName(const char* name);

	/**
	 * Set the coordinates in the feature tree
	 *
	 * @param x The x-coordinate
	 * @param y The y-coordinate
	 * @param z The z-coordinate
	 */
	void setCoordinates(double x, double y, double z);
	int point_id;			/**< The point feature Id */
	int point_internal_id;  /**< The point's Creo internal Id */
	ProPoint3d pntData;
};

#endif

