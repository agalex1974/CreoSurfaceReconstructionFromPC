#ifndef AXIS_THROUGH_TWO_POINTS_POINT_H
#define AXIS_THROUGH_TWO_POINTS_POINT_H
#include <memory>
#include "CDatumPoint.h"

class CDatumAxis
{
public:
	/**
	 * The constructor of the Creo axis
	 * This implementation creates the axis that passes through two datum points
	 *
	 * @param pnt1 the first Creo datum point through which the axis pass
	 * @param pnt2 the second Creo datum point through which the axis pass
	 * @param name the name of the axis displayed in Creo UI
	 */
	CDatumAxis(const CDatumPoint& pnt1, const CDatumPoint& pnt2, const char* name);
	CDatumAxis(const ProPoint3d& pnt1, const ProPoint3d& pnt2, const char* name);
	/**
	 * Get the feature Id
	 *
	 * @return the axis feature Id
	 */
	int GetAxisId() const;
	/**
	 * Get the axis Id that is internal in Creo's UI
	 *
	 * @return the axis intern Id
	 */
	int GetAxisInternalId() const;
	/**
	 * Set the name of the axis as displayed in the UI
	 *
	 * @param name The name of the axis as displayed in the UI
	 */
	void SetName(const char* name);
	~CDatumAxis();
private:
	ElemTreeData axis_tree[9] = {
		/* 0 */ {0, PRO_E_FEATURE_TREE, {(ProValueDataType)-1}},
		/* 1 */ {1, PRO_E_FEATURE_TYPE, {PRO_VALUE_TYPE_INT, PRO_FEAT_DATUM_AXIS}},
		/* 2 */ {1, PRO_E_DTMAXIS_CONSTRAINTS, {(ProValueDataType)-1}},
		/* 3 */ {2, PRO_E_DTMAXIS_CONSTRAINT, {(ProValueDataType)-1}},
		/* 4 */ {3, PRO_E_DTMAXIS_CONSTR_TYPE, {PRO_VALUE_TYPE_INT, PRO_DTMAXIS_CONSTR_TYPE_THRU}},
		/* 5 */ {3, PRO_E_DTMAXIS_CONSTR_REF, {PRO_VALUE_TYPE_SELECTION}},
		/* 6 */ {2, PRO_E_DTMAXIS_CONSTRAINT, {(ProValueDataType)-1}},
		/* 7 */ {3, PRO_E_DTMAXIS_CONSTR_TYPE, {PRO_VALUE_TYPE_INT, PRO_DTMAXIS_CONSTR_TYPE_THRU}},
		/* 8 */ {3, PRO_E_DTMAXIS_CONSTR_REF, {PRO_VALUE_TYPE_SELECTION}}
	};
	/**
	 * Set the point constraints in the tree
	 *
	 * @param pnt1 the first point
	 * @param pnt2 the second point
	 */
	void definePointContraints(const CDatumPoint& pnt1, const CDatumPoint& pnt2);
	/**
	 * Create the axis method
	 *
	 * @param name The name of the axis
	 */
	int createAxis(const char* name);
	int axis_id;						/**< The feature Id */
	int axis_internal_id;				/**< The axis Creo internal Id */
	std::shared_ptr<CDatumPoint> pntStart;
	std::shared_ptr<CDatumPoint> pntEnd;
};

#endif
