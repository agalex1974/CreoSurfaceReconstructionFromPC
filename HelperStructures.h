#ifndef HELPER_STRUCTURES_H
#define HELPER_STRUCTURES_H

#include <ProValue.h>

/*typedef struct my_point
{
	double x;
	double y;
	double z;
} my_point;

typedef struct my_point_2
{
	double x;
	double y;
} my_point_2;*/

typedef struct elem_tree_data
{
	int				level;
	int				elem_id;
	ProValueData	data;
} ElemTreeData;

typedef struct feat_by_name
{
	char *name;
	int   id;
} FeatByName;

typedef struct feat_by_name_find
{
	FeatByName *feats;
	int num_feats;
}  FeatByNameFind;

typedef union
{
	int    i;
	double d;
	void  *p;
	ProSelection r;
} Parameter;

#endif
