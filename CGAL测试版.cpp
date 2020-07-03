#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/array.h>
#include <CGAL/disable_warnings.h>


#include <list>
//#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <boost/iterator/iterator_adaptor.hpp>

typedef std::array<std::size_t, 3> Facet;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3  Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;




//typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
//typedef K::Triangle_3 Triangle;

typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

typedef CGAL::Polyhedron_3<K> Polyhedron;
//typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive1;
//typedef CGAL::AABB_traits<Kernel, Primitive1> Traits;
//typedef CGAL::AABB_tree<Traits> Tree1;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;
using namespace std;


struct Construct
{
	Mesh& mesh;
	template < typename PointIterator>
	Construct(Mesh& mesh, PointIterator b, PointIterator e)
		: mesh(mesh)
	{
		for (; b != e; ++b) {
			boost::graph_traits<Mesh>::vertex_descriptor v;
			v = add_vertex(mesh);
			mesh.point(v) = *b;
		}
	}
	Construct& operator=(const Facet f)
	{
		typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
		typedef boost::graph_traits<Mesh>::vertices_size_type size_type;
		mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
			vertex_descriptor(static_cast<size_type>(f[1])),
			vertex_descriptor(static_cast<size_type>(f[2])));


		return *this;
	}
	Construct&
		operator*() { return *this; }
	Construct&
		operator++() { return *this; }
	Construct
		operator++(int) { return *this; }
};
int main(int argc, char* argv[])
{
	std::ifstream in("球面200.xyz");
	std::vector<Point_3> points_original;
	std::vector<Point_3> points;
	std::vector<Facet> facets;
	Mesh m;



	std::vector<Point_3>face(3);
	//std::vector<float> verts;
	//std::vector<uint32_t> indices;

	std::copy(std::istream_iterator<Point_3>(in),
		std::istream_iterator<Point_3>(),
		//std::back_inserter(points));
	   std::back_inserter(points_original));
//降采样
//for (int i = 0; i < points_original.size(); i=i+1)
//{
//	points.push_back(points_original[i]);
//
//}
	Construct construct(m, points_original.begin(), points_original.end());
	CGAL::advancing_front_surface_reconstruction(points_original.begin(),
		points_original.end(),
		construct);

	std::cout << m << std::endl;



	std::list<Triangle> triangles;
	for (Mesh::Face_index face_index : m.faces())
	{
		for (vertex_descriptor vd : vertices_around_face(m.halfedge(face_index), m))
		{
			//std::cout << vd << std::endl;
			auto location = m.points();
			std::cout << location[vd] << std::endl;
			auto pt = location[vd];
			/*verts.push_back((float)pt.x());
			verts.push_back((float)pt.y());
			verts.push_back((float)pt.z());*/
			face.push_back(pt);

		}
		/*	Point a = face[0];
			Point b = face[1];
			Point c = face[2];*/

		triangles.push_back(Triangle(face[face.size() - 3], face[face.size() - 2], face[face.size() - 1]));

	}
	Tree tree(triangles.begin(), triangles.end());
	tree.accelerate_distance_queries();

 
    std::ifstream Tcloud("球面2001.xyz");
	std::vector<Point_3> Tpoints;
	std::copy(std::istream_iterator<Point_3>(Tcloud),
		std::istream_iterator<Point_3>(),
		std::back_inserter(Tpoints));
	
    vector<FT>distances;
    Point hint(40,40,40);
   
 for (Point point_query : Tpoints)
        {
	     FT  sqd = tree.squared_distance(point_query, hint);
	//FT  sqd = tree.squared_distance(point_query);
	      std::cout << "squared distance: " << sqd << std::endl;
	// computes closest point
	    /*  Point closest = tree.closest_point(point_query);
	      std::cout << "closest point: " << closest << std::endl;*/
	      distances.push_back(sqd);
        }

   auto max = *max_element(distances.begin(), distances.end());
   auto min = *min_element(distances.begin(), distances.end());
   double sum = 0;
   for (int i = 0; i < distances.size(); i++)
   {
	 sum += distances[i];
   }
  // return sum / distances.size();
   std::cout << "max: " << max << std::endl;
   std::cout << "min: " << min << std::endl;
   std::cout << "ave: " << sum / distances.size() << std::endl;
	
 return EXIT_SUCCESS;
}

	