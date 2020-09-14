#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include <vector>

#include <iostream>

using namespace std;

int main() {
    double wx [25] = {132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
                   104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
                   92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
                   92.39,  92.39,  92.39,  92.39};
    double wy [25] = {195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
                   195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
                   181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
                   153.32, 149.32, 145.32, 141.84}; // These are the way-points
    double o_llx[1] = {92.89};   // lower_left_x
    double o_lly[1] = {191.75}; // lower_left_y
    double o_urx[1] = {92.89}; // upper_right_x
    double o_ury[1] = {191.75}; // upper_right_y

    // set up experiment
    FrenetInitialConditions fot_ic = {
        34.6, // initial s (frenet coordinates)
        7.10964962, // initial speed (frenet coordinates)
        -1.35277168, // initial d (frenet coordinates)
        -1.86, // initial d_d (frenet coordinates)
        0.0, // initial d_dd (frenet coordinates)
        10, // target speed (frenet coordinates)
        wx, // waypoints
        wy, // waypoints
        25,
        o_llx,
        o_lly,
        o_urx,
        o_ury,
        1
    };
    FrenetHyperparameters fot_hp = {
        25.0,//max_speed
        6.0, // max_accel
        10.0, // max_curvature
        5.0, // max_road_width_l
        1.0, // max_road_width_r
        0.25, // d_road_w
        0.25, // dt
        6.0, // max t
        2.0, // min t
        0.25, // d_t_s
        2.0, // n_s_sample
        1.0, // obstacle_clearance
        1.0, // k_d
        0.1, // k_v
        0.1, // k_a
        0.1, // k_j
        0.1, // k_t
        1.0, // k_o
        1.0, // k_lat
        1.0 // k_lon
    };

    // run experiment
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (best_frenet_path) {
        cout << "Success\n";
    }
    else{
        cout << "Failure\n";
        return 1;
    }
    // Euclidean attributes
    vector<double> x    = best_frenet_path->x;          // x position
    vector<double> y    = best_frenet_path->y;          // y position
    vector<double> yaw  = best_frenet_path->yaw;        // yaw in rad
    vector<double> ds   = best_frenet_path->ds;         // speed
    vector<double> c    = best_frenet_path->c;          // curvature
    return 0;
    
}
