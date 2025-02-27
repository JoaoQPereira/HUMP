#ifndef HUMPConfig_HPP
#define HUMPConfig_HPP

#include <string>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <list>
#include <vector>
#include <cmath>
#include <boost/smart_ptr.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <tuple>

#include "../config/config.hpp"
#include "object.hpp"

/** version of the library */
#define HUMP_VERSION_MAJOR 2
#define HUMP_VERSION_MINOR 0


//definition of the macro ASSERT
#ifndef DEBUG
#define ASSERT(x)
#else
#define ASSERT(x) \
    if (! (x)) \
{ \
    cout << "ERROR!! Assert " << #x << " failed\n"; \
    cout << " on line " << __LINE__  << "\n"; \
    cout << " in file " << __FILE__ << "\n";  \
    }
#endif


using namespace std;
using namespace Eigen;

/** This is the main namespace of the library */
namespace HUMotion{

typedef boost::shared_ptr<Object> objectPtr;/**< shared pointer to an object in the scenario */

//waypoints time problem ipopt options
const double WP_TOL = 1e-6; /**< desired convergence tolerance */
const double WP_ACC_TOL = 1e-2; /**< acceptable convergence tolerance */
const double WP_CONSTR_VIOL_TOL = 0.0001; /**< constraints violation tolerance */


// Final posture selection problem ipopt options
const double FINAL_TOL = 1e-6; /**< desired convergence tolerance */
const double FINAL_ACC_TOL = 1e-2; /**< acceptable convergence tolerance */
const double FINAL_CONSTR_VIOL_TOL = 0.0001; /**< constraints violation tolerance */

// Bounce posture selection problem ipopt options
const double BOUNCE_TOL = 1e-6; /**< desired convergence tolerance*/
const double BOUNCE_ACC_TOL = 1e-6; /**< acceptable convergence tolerance */
const double BOUNCE_CONSTR_VIOL_TOL = 0.0001; /**< constraints violation tolerance */

const double SPACER = 1.0*static_cast<double>(M_PI)/180; /**< degree used to space the joint limits [deg]: IPOPT sometimes does not fully respect all the constraints,
                                                              but the joint limits has to be respected
                                                              This parameter helps to stay within the joint range */
const double SPACER_PRISMATIC = 0.5; //[mm]

const double PHI = (-log(2.0)/log(TB));/**< parameter to control when the bounce posture is reached */
const double AP = 20.0 * static_cast<double>(M_PI) / 180; /**< aperture of the fingers when approaching to pick [rad] */
const double AP_PRISMATIC = -10; /**< aperture of the fingers when approaching to pick [mm] */

const double BLANK_PERCENTAGE_TAR = 0.10; /**< percentage of steps to eliminate when either reaching to grasp an object [0,1]*/
const double BLANK_PERCENTAGE_OBS = 0.20;/**< move at the beginning of a move movement [0,1] */

const int N_STEP_MIN = 10; /**< minimum number of steps for revolute joints */
const int N_STEP_MAX = 100; /**< maximum number of steps for revolute joints */

const int N_STEP_MIN_PRISMATIC = 5; /**< minimum number of steps for the prismatic joints */
const int N_STEP_MAX_PRISMATIC = 20; /**< maximum number of steps for the prismatic joints */

const double MU_INIT = 1e-1; /**< initial value of the barrier parameter */

const double W_RED_MIN = 1; /**< minimum value of angular velocity reduction when approaching and retreating */
//const double W_RED_APP_MAX = 5; /**< maximum value of angular velocity reduction when approaching */
//const double W_RED_RET_MAX = 5; /**< maximum value of angular velocity reduction when retreating */

const bool HAS_JOINT_ACCELEARATION_MAX_LIMIT = true; /**< true to check acceleration maximum limit, false otherwise */

const int D_LENGHT_TOL = 350; /** If length of the link is bigger than 350mm is create a point of arm*/

static bool abs_compare(double a, double b)
{
    return (std::abs(a) < std::abs(b));
}

/** this struct defines the Denavit-Hartenberg kinematic parameters */
typedef struct{
    vector<double> d; /**< distances between consecutive frames along the y axes in [mm] */
    vector<double> a; /**< distances between concecutive frames along the z axes in [mm] */
    vector<double> alpha; /**< angle around the x axes between consecutive z axes in [rad] */
    vector<double> theta; /**< angle around the z axes between consecutive x axes in [rad] */
} DHparameters;

/** this struct defines the barrett hand */
typedef struct{
    double maxAperture; /**< [mm] max aperture of the hand in [mm] */
    double Aw; /**< smallest distance between F1 and F2 in [mm] */
    double A1; /**< length of the 1st part of the finger in [mm] */
    double A2; /**< length of the first phalax in [mm] */
    double A3; /**< length of the second phalax in [mm] */
    double D3; /**< depth of the fingertip in [mm] */
    double phi2; /**< angular displacement between the 1st part of the finger and the 1st phalax in [rad] */
    double phi3; /**< angular displacement between the 1st and the 2nd phalax in [rad] */
    vector<int> rk; /**< r parameters of the barrett hand */
    vector<int> jk; /**< j parameters of the barrett hand */
} BarrettHand;

typedef struct{
    double maxAperture; /**< [mm] max aperture of the hand in [mm] */
    double minAperture; /**< [mm] in aperture of the hand in [mm] */
    double A1; /**< length of the finger in [mm] */
    double D3; /**< depth of the fingertip in [mm] */
    vector<int> rk; /**< r parameters of the electric parallel gripper */
} ElectricGripper;

/** this struct defines a human finger */
typedef struct{
    double ux; /**<  position of the finger with respect to the center of the palm along the x axis in [mm] */
    double uy; /**<  position of the finger with respect to the center of the palm along the y axis in [mm] */
    double uz; /**<  position of the finger with respect to the center of the palm along the z axis in [mm] */
    DHparameters finger_specs; /**< the Denavit-Hartenberg parameters of the finger */
} HumanFinger;

/** this struct defines a generic part of a robot body */
typedef struct{
    double Xpos; /**< position of the part along the x axis in [mm] */
    double Ypos; /**< position of the part along the y axis in [mm] */
    double Zpos; /**< position of the part along the z axis in [mm] */
    double Roll; /**< orientation of the part around the z axis in [rad] */
    double Pitch; /**< orientation of the part around the y axis in [rad] */
    double Yaw; /**< orientation of the part around the x axis in [rad] */
    double Xsize; /**< size of the part along the x axis in [mm] */
    double Ysize; /**< size of the part along the y axis in [mm] */
    double Zsize; /**< size of the part along the z axis in [mm] */
} RobotPart;

/** this struct defines a human thumb */
typedef struct{
    double uTx; /**<  position of the thumb with respect to the center of the palm along the x axis in [mm] */
    double uTy; /**<  position of the thumb with respect to the center of the palm along the y axis in [mm] */
    double uTz; /**<  position of the thumb with respect to the center of the palm along the z axis in [mm] */
    DHparameters thumb_specs; /**< the Denavit-Hartenberg parameters of the thumb */
} HumanThumb;

/** this struct defines a human hand */
typedef struct{
    vector<HumanFinger> fingers; /**< fingers of the hand */
    HumanThumb thumb; /**<  thumb of the hand */
    double maxAperture; /**< max aperture of the hand in [mm] */
} HumanHand;

/** this struct defines the  waypoint in Operational Space */
typedef struct wp_operat{
    double Xpos; /**< position of the part along the x axis in [mm] */
    double Ypos; /**< position of the part along the y axis in [mm] */
    double Zpos; /**< position of the part along the z axis in [mm] */
    double Roll; /**< orientation of the part around the z axis in [rad] */
    double Pitch; /**< orientation of the part around the y axis in [rad] */
    double Yaw; /**< orientation of the part around the x axis in [rad] */
    vector <double> velocity; /**< vector of angular velocity of the end effector in the waypoint   */
    vector <double> accelaration; /**< vector of angular accelaration of the end effector in the waypoint */
}OperatSpace;

/** this struct defines the  waypoint in Joint Space */
typedef struct wp_joint{
    vector <double> PosJoints; /** vector of Joints Positions in Joint Space*/
    vector <double> velocity; /**< vector of angular velocity of each joint in the waypoint   */
    vector <double> accelaration; /**< vector of angular accelaration of each joint in the waypoint */
}JointSpace;


/** this struct defines the waypoint in Joint Space and Operational Space*/
typedef struct wp_det{
    string name;
    JointSpace JntSpace;
    OperatSpace OpSpace;
}wp_specs;

/** this struct defines the waypoints in the necessary order for planning the trajectory through waypoints */
typedef struct wp_order{
    vector<double> xf_dof; /** final waypoint for each joint  */
    vector<double> x0_dof; /** initial waypoint for each joint  */
    vector<vector<double>> x_wp_dof; /** vector that defines the waypoints that each joint must pass */
}wp_traj;

/** this struct defines the parameters of the movement */
typedef struct{
    int arm_code; /**< the code of the arm: 0 = both arms, 1 = right arm, 2 = left arm */
    int hand_code;/**< the code of the hand: 0 = human hand, 1 = barrett hand */
    int head_code; /**< the code of the head: 0 = wobot without head, 1 = robot with head */
    string mov_infoline; /**< description of the movement */
    double dHO;/**< distanche hand-target*/
    std::vector<double> finalHand;/**< final posture of the hand */
    std::vector<double> target;/**< target / location to reach: target(0)=x, target(1)=y, target(2)=z, target(3)=roll, target(4)=pitch, target(6)=yaw,*/
    objectPtr obj; /**< object involved in the movement. The info of the object have to be updated according to the selected movement */
    std::string support_obj; /**< name of the object that is a support surface in place movements */
    bool approach;/**< true to use the approach options, false otherwise  */
    bool retreat;/**< true to use the retreat options, false otherwise */
    bool straight_line; /**< true to use the straight line option of the approach/retreat stage */
    double w_red_app_max;/**< maximum velocity reduction factor when approaching */
    double w_red_ret_max;/**< maximum velocity reduction factor when retreating */
    std::vector<double> pre_grasp_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> post_grasp_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> pre_place_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> post_place_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    bool rand_init; /**< true to select randon initialization in "plan" stages */
    bool coll; /**< true to enable collisions with the environment (body of the robot included) */
    bool use_move_plane; /**< true to constrain the end-effector to move on a plane in move movements, false otherwise*/
    std::vector<double> plane_params; /**< plane cartesian parameters in move movements: a*x+b*y+c*z+d=0. a=plane_params(0), b=plane_params(1), c=plane_params(2), d=plane_params(3) */
}mov_params;

/** this struct defines the boundary conditions of the movement*/
typedef struct{
    vector<double> vel_0; /**< initial velocity of the joints in [rad/s] */
    vector<double> vel_f; /**< final velocity of the joints in [rad/s] */
    vector<double> acc_0; /**< initial acceleration of the joints in [rad/s²] */
    vector<double> acc_f; /**< final acceleration of the joints in [rad/s²] */
} boundaryConditions;

/** this struct defines the tolerances that have to be set before planning the trajectory*/
typedef struct{
    mov_params mov_specs; /**< specifications of the movement */
    vector<double> tolsArm; /**< radius of the spheres along the arm in [mm] */
    MatrixXd tolsHand; /**< radius of the spheres along the fingers in [mm] */
    MatrixXd final_tolsObstacles; /**< tolerances of the final posture against the obstacles in [mm] */
    vector< MatrixXd > singleArm_tolsTarget; /**< tolerances of the trajectory against the target in [mm] */
    vector< MatrixXd > singleArm_tolsObstacles; /**< tolerances of the trajectory against the obstacles in [mm] */
    double tolTarPos; /**< tolerance of the final position of the hand against the target in [mm] */
    double tolTarOr; /**< tolerance of the final orientation of the hand against the target in [mm] */
    boundaryConditions bounds; /**< boundary condistions of the bounce problem */
    vector<double> vel_approach; /**< velocity of the joints in [rad/s] at the beginning of the approach stage */
    vector<double> acc_approach; /**< acceleration of the joints in [rad/s²] at the beginning of the approach stage */
    vector<double> lambda_final; /**< weights for the final posture optimization */
    vector<double> lambda_bounce; /**< weights for the bounce posture optimization */
    vector<double> lambda_wp; /**< weights of the joints for waypoints mode */
    vector<double> w_max; /**< maximum angular velocity for each joint [rad/s] */
    vector<double> alpha_max; /**< maximum angular acceleration for each joint [rad/s²] */
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
}hump_params;

/** This struct defines the result of the planned trajectory */
typedef struct{
    int mov_type;/**< type of the planned movement */
    int status;/**< status code of the planning */
    string status_msg;/**< status message of the planning */
    string object_id;/**< identity of the object involved in the movement */
    vector<MatrixXd> trajectory_stages;/**< sequence of the trajectories */
    vector<MatrixXd> velocity_stages;/**< sequence of the velocities */
    vector<MatrixXd> acceleration_stages;/**< sequence of the accelerations */
    vector<double> time_steps; /**< sequence of each time steps for each trajectory */
    vector<string> trajectory_descriptions;/**< description of the trajectories */

    MatrixXd calcWP; /** waypoints computed - used to get the error*/
    vector <double> pos_wp; /** position of the waypoints in the trajectory vector*/
}planning_result;



} // namespace HUMotion




#endif // HUMPConfig_HPP
