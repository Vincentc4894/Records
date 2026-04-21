#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <string>
#include <iomanip>
#include <cmath>
#include <vector>
#include <math.h>

// This code is designed to utilize topic /scan data from the Rosmaster x3 lidar system
// and convert the scan into a CGAL surface mesh, then open an interactive window for visualizing the mesh.

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Simple_cartesian<double>                      K;
typedef K::Point_3                                          Point;
typedef CGAL::Search_traits_3<K>                            TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits>      Neighbor_search;
typedef CGAL::Kd_tree<TreeTraits>                           Tree;
typedef CGAL::Surface_mesh<Point>                           Mesh;


void ReadReal(Mesh &f, std::string s){
    std::ifstream in1(s);
    double theta, r, iter;
    theta = 0.0;
    std::string buff, g2;
    bool tr = false;
    while(in1 >> buff){
        double x, y;
        if(tr && buff == std::string("-")){
            in1 >> g2; 
            if(g2 == std::string(".inf") || g2 == std::string("'...'")){
                std::cout << "--Encountered a " << g2 << " token\n";
            }
            else{
            r = stod(g2);
            x = r * cos(theta);
            y = r * sin(theta);
            Point g(x, y, 0.0);
            f.add_vertex(g);
            }
            theta += iter;
        }
        else if(buff == std::string("angle_increment:") && tr == false){
            in1 >> iter;
            std::cout << "--Here: " << iter << std::endl;
        }
        else if(buff == std::string("ranges:") && tr == false) tr = true;
        else if(buff == std::string("intensities:")) break;

    }
    
}
int main(int argc, char * argv[]){
    if(argc < 2){
        std::cerr << "--ERROR: No point file directory given. Please give the name of a point file.\n";
        exit(1);
    }
    bool gen = false;
    bool loud = false;
    std::cout << "--Reading points from " << argv[1] << std::endl;
    Mesh t;
    std::string fle = std::string(argv[1]);
    ReadReal(t, fle);

    CGAL::Graphics_scene gs;
    CGAL::add_to_graphics_scene(t, gs);
    CGAL::Qt::QApplication_and_basic_viewer app(gs, "Detect Point Cloud Surface");
    app.basic_viewer().draw_vertices(true);
    app.basic_viewer().draw_edges(false);
    app.run();

}
