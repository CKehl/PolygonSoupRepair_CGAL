/*
 * main.cpp
 *
 *  Created on: 7 jan. 2018
 *      Author: christian
 */
#include "main.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/OFF_reader.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
//#include <CGAL/Polygon_mesh_processing/refine.h>
//#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;

int main(int argc, char* argv[]) {
	const char* filename = (argc > 1) ? argv[1] : "data/tet-shuffled.off";
	const char* out_filename = (argc > 2) ? argv[2] : "data/tet-oriented1.off";
	if(argc<3) {
		return 1;
	}
	std::ifstream input(filename);
	if (!input) {
		std::cerr << "Cannot open file " << std::endl;
		return 1;
	}
	std::vector<K::Point_3> points;
	std::vector< std::vector<std::size_t> > polygons;
	if (!CGAL::read_OFF(input, points, polygons)) {
		std::cerr << "Error parsing the OFF file " << std::endl;
		return 1;
	}
	CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
	Polyhedron mesh;
	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, mesh);
	if (CGAL::is_closed(mesh) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(mesh)))
		CGAL::Polygon_mesh_processing::reverse_face_orientations(mesh);

	// Incrementally fill the holes
	unsigned int nb_holes = 0, nb_holes_filled = 0;
	unsigned int N_halfedges = mesh.size_of_halfedges(); unsigned int he_index = 0;
	unsigned int N_facets = 0;
	BOOST_FOREACH(Halfedge_handle h, halfedges(mesh))
	{
		N_facets = mesh.size_of_facets();
		if(h->is_border()) {
			std::vector<Facet_handle>  patch_facets;
			std::vector<Vertex_handle> patch_vertices;
			bool success=false;
			try {
#ifdef COMPILE_WITH_EIGEN
				CGAL::cpp11::tuple<bool, std::back_insert_iterator< std::vector<Facet_handle> >, std::back_insert_iterator< std::vector<Vertex_handle> > > fill_results = CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(mesh,h,std::back_inserter(patch_facets),std::back_inserter(patch_vertices),CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)).geom_traits(K()));
				success = (CGAL::cpp11::get<0>(fill_results) && (mesh.size_of_facets()!=N_facets));
				std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
#else
				CGAL::Polygon_mesh_processing::triangulate_hole(mesh,h,std::back_inserter(patch_facets));
#endif
				std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
				std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
			} catch(...) {
				success=false;
			}
			++nb_holes;
			if(success)
				++nb_holes_filled;
		}
		if((he_index%100)==0)
			std::cout << "processed " << he_index << " of " << N_halfedges << " (" << (double(he_index)/double(N_halfedges))*100.0  << "%)." << std::endl;
		he_index++;
	}
	std::cout << std::endl;
	std::cout << nb_holes_filled << "of" << nb_holes <<  " holes have been filled" << std::endl;

	std::ofstream out(out_filename);
	out << mesh;
	out.close();
	//CGAL::Polygon_mesh_processing::reverse_face_orientations(mesh);
	//std::ofstream out2("tet-oriented2.off");
	//out2 << mesh;
	//out2.close();
	return 0;
}




