#include <iostream>
#include <fstream>
#include <cassert>
#include "bcrust.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>


// typedefs for defining the adaptor
// typedef for the result type of the point location

typedef CGAL::Exact_predicates_inexact_constructions_kernel						K;
typedef CGAL::Delaunay_triangulation_2<K>										DT;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, K>				Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>								Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>									Delaunay;
typedef Delaunay::Point															Point;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>					AT;
typedef AT::Site_2                    Site_2;
typedef AT::Point_2                   Point_2;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT>	AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP>										VD;




void calculate_b_crust(const std::vector<CPointEx>& pointsin, std::vector<std::vector<CPointEx>>& pointsoutPolygons)
{
	std::vector< Point > points;
	Site_2 t;

	for (auto const& p : pointsin)
	{
		points.push_back(Point(p.x, p.y));
	}
	
	int countOriginal = (int)points.size();
	
	VD vd;
	vd.insert(points.begin(), points.end());
	for (auto v = vd.vertices_begin(); v != vd.vertices_end(); v++)
	{
		points.push_back(Point(v->point().x(), v->point().y()));
	}
	//FILE* file = fopen("Voronoi_passed.txt", "w");
	//fclose(file);
	std::vector< std::pair<Point, unsigned> > points_with_index;
	std::vector< std::pair<int, int> > edges;
	int i = 0;
	for (auto const& point : points)
	{
		points_with_index.push_back(std::make_pair(Point(point.x(), point.y()), i++));
	}

	Delaunay dt;
	dt.insert(points_with_index.begin(), points_with_index.end());

	//file = fopen("Delaunay_passed.txt", "w");
	//fclose(file);
	
	auto edge = dt.edges_begin();

	while (edge != dt.edges_end())
	{
		auto e = *edge;
		int i1 = e.first->vertex((e.second + 1) % 3)->info();
		int i2 = e.first->vertex((e.second + 2) % 3)->info();
		if (i1 < countOriginal && i2 < countOriginal)
			edges.push_back(std::make_pair(i1, i2));
		edge++;
	}
	//file = fopen("got_edges.txt", "w");
	//fclose(file);

	
	int edges_count = (int)edges.size();
	std::vector<int> valid_points(countOriginal, 1);
	

	//std::ofstream ofs("approximation.poly");
	//ofs << points_with_index[edges[0].first].first.x() << " " << points_with_index[edges[0].first].first.y() << " " << 0.0 << "\n";
	//ofs << points_with_index[edges[0].second].first.x() << " " << points_with_index[edges[0].second].first.y() << " " << 0.0 << "\n";
	bool processTerminate = false;
	std::vector<bool> validEdges(edges.size(), true);
	int counter = 0;
	while (counter < 1) {

		//grab the firs valid index;
		int idxValid = 0;
		for (; idxValid < validEdges.size(); idxValid++)
		{
			if (validEdges[idxValid])
				break;
		}
		if (idxValid == validEdges.size()) break;
		std::vector<std::pair<int, int>> chain;
		chain.push_back(std::make_pair(edges[idxValid].first, edges[idxValid].second));
		valid_points[edges[idxValid].first] = 0;
		valid_points[edges[idxValid].second] = 0;
		std::vector<CPointEx> pointsout;
		pointsout.push_back({ points_with_index[edges[idxValid].first].first.x(),  points_with_index[edges[idxValid].first].first.y() });
		pointsout.push_back({ points_with_index[edges[idxValid].second].first.x(),  points_with_index[edges[idxValid].second].first.y() });
		int current_end = edges[idxValid].second;
		bool terminate = false;
		validEdges[idxValid] = false;
		int counterValid = 0;
		while (!terminate)
		{
			if (counterValid == edges_count) break;
			counterValid = 0;
			for (int i = 0; i < edges_count; i++)
			{
				if (!validEdges[i])
				{
					counterValid++;
					continue;
				} 

				if (edges[i].first == current_end)
				{
					if (valid_points[edges[i].second])
					{
						chain.push_back(std::make_pair(edges[i].first, edges[i].second));
						//std::cout << "(" << edges[i].first << ", " << edges[i].second << ")" << std::endl;
						valid_points[edges[i].second] = 0;
						current_end = edges[i].second;
						//ofs << points_with_index[edges[i].second].first.x() << " " << points_with_index[edges[i].second].first.y() << " " << 0.0 << "\n";
						pointsout.push_back({ points_with_index[edges[i].second].first.x(),  points_with_index[edges[i].second].first.y() });
						validEdges[i] = false;
						continue;
					}
					std::cout << "circle detected!" << std::endl;
					terminate = true;
					continue;
				}
				if (edges[i].second == current_end)
				{
					if (valid_points[edges[i].first])
					{
						chain.push_back(std::make_pair(edges[i].second, edges[i].first));
						//std::cout << "(" << edges[i].second << ", " << edges[i].first << ")" << std::endl;
						valid_points[edges[i].first] = 0;
						current_end = edges[i].first;
						//ofs << points_with_index[edges[i].first].first.x() << " " << points_with_index[edges[i].first].first.y() << " " << 0.0 << "\n";
						pointsout.push_back({ points_with_index[edges[i].first].first.x(),  points_with_index[edges[i].first].first.y() });
						validEdges[i] = false;
						continue;
					}

					//std::cout << "circle detected!" << std::endl;
					terminate = true;
					continue;
				}
			}
		}
		pointsoutPolygons.push_back(pointsout);
		counter++;
	}
	//ofs << points_with_index[edges[0].first].first.x() << " " << points_with_index[edges[0].first].first.y() << " " << 0.0 << "\n";
	//ofs.close();
}
