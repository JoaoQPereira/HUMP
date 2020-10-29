#ifndef HUMPLANNER_H
#define HUMPLANNER_H

#include "HUMPconfig.hpp"
#include "amplinterface.hpp"
#include "IpIpoptApplication.hpp"
#include "waypoint.hpp"

using namespace Ipopt;
using namespace std;

namespace HUMotion{

typedef boost::shared_ptr<planning_result> planning_result_ptr; /**< pointer to the results of the planning process*/
typedef boost::shared_ptr<Waypoint> wpPtr; /**< pointer to the waypoints */



//! The Object class
/**
 * @brief This class defines the concept of the Human-like Motion planner
 */
class HUMPlanner
{
public:

    static unsigned hand_fingers; /**< number of fingers per hand */
    static unsigned joints_arm; /**< number of joints per arm */
    static unsigned joints_hand; /**< number of joints per hand */
    static unsigned n_phalange; /**< number of phalanges per finger */

    std::vector<double> arange(double start, double stop, double step);

    /**
     * @brief HUMPlanner, a constructor
     * @param name
     */
    explicit HUMPlanner(string name);


    /**
     * @brief HUMPlanner, a copy constructor
     * @param hp
     */
    HUMPlanner(const HUMPlanner& hp);

    /**
     * @brief ~HUMPlanner, a destructor
     */
    ~HUMPlanner();

    /**
     * @brief setName
     * @param name
     */
    void setName(string name);

    /**
     * @brief getName
     * @return
     */
    string getName();
    /**
     * @brief addWaypoint
     * @param wp_ptr
     */
    void addWaypoint(wpPtr wp_ptr);
    /**
     * @brief getWaypoints
     * @param wps
     * @return
     */
    void getWaypoints(wpPtr &wps);

    /**
     * @brief addObstacle
     * @param obs
     */
    void addObstacle(objectPtr obs);

    /**
     * @brief setObstacleStagiare sviluppatore, informatico presso la sede principale
     * @param obs
     * @param pos
     * @return
     */
    bool setObstacle(objectPtr obs, unsigned pos);

    /**
     * @brief getObstacles
     * @param obs
     * @return
     */
    bool getObstacles(std::vector<objectPtr> &obs);

    /**
     * @brief getObstacle
     * @param pos
     * @return
     */
    objectPtr getObstacle(unsigned pos);

    /**
     * @brief getObstacle
     * @param name
     * @return
     */
    objectPtr getObstacle(std::string name);

    /**
     * @brief setObjTarget
     * @param obj
     */
    void setObjTarget(objectPtr obj);

    /**
     * @brief getObjTarget
     * @return
     */
    objectPtr getObjTarget();

    /**
     * @brief clearScenario
     */
    void clearScenario();

    /**
     * @brief setMatRightArm
     * @param m
     */
    void setMatRightArm(Matrix4d &m);

    /**
     * @brief getMatRightArm
     * @param m
     */
    void getMatRightArm(Matrix4d &m);

    /**
     * @brief setMatRightHand
     * @param m
     */
    void setMatRightHand(Matrix4d &m);

    /**
     * @brief getMatRightHand
     * @param m
     */
    void getMatRightHand(Matrix4d &m);

    /**
     * @brief setMatLeftArm
     * @param m
     */
    void setMatLeftArm(Matrix4d &m);

    /**
     * @brief getMatLeftArm
     * @param m
     */
    void getMatLeftArm(Matrix4d &m);

    /**
     * @brief setMatLeftHand
     * @param m
     */
    void setMatLeftHand(Matrix4d &m);

    /**
     * @brief getMatLeftHand
     * @param m
     */
    void getMatLeftHand(Matrix4d &m);

    /**
     * @brief setRightMinLimits
     * @param min_rl
     */
    void setRightMinLimits(vector<double> &min_rl);

    /**
     * @brief getRightMinLimits
     * @param min_rl
     */
    void getRightMinLimits(vector<double> &min_rl);

    /**
     * @brief setRightMaxLimits
     * @param max_rl
     */
    void setRightMaxLimits(vector<double> &max_rl);

    /**
     * @brief getRightMaxLimits
     * @param max_rl
     */
    void getRightMaxLimits(vector<double> &max_rl);

    /**
     * @brief setLeftMinLimits
     * @param min_ll
     */
    void setLeftMinLimits(vector<double> &min_ll);

    /**
     * @brief getLeftMinLimits
     * @param min_ll
     */
    void getLeftMinLimits(vector<double> &min_ll);

    /**
     * @brief setLeftMaxLimits
     * @param max_ll
     */
    void setLeftMaxLimits(vector<double> &max_ll);

    /**
     * @brief getLeftMaxLimits
     * @param max_ll
     */
    void getLeftMaxLimits(vector<double> &max_ll);

    /**
     * @brief setTorso
     * @param htorso
     */
    void setTorso(RobotPart& htorso);

    /**
     * @brief getTorso
     * @return
     */
    RobotPart getTorso();

    /**
     * @brief setDH_rightArm
     * @param p
     */
    void setDH_rightArm(DHparameters& p);

    /**
     * @brief getDH_rightArm
     */
    DHparameters getDH_rightArm();

    /**
     * @brief setDH_leftArm
     * @param p
     */
    void setDH_leftArm(DHparameters& p);

    /**
     * @brief getDH_leftArm
     */
    DHparameters getDH_leftArm();

    /**
     * @brief setBarrettHand
     * @param bhand
     */
    void setBarrettHand(BarrettHand& bhand);

    /**
     * @brief getBarrettHand
     * @return
     */
    BarrettHand getBarrettHand();

    /**
     * @brief setHead
     * @param hhead
     */
    void setHead(RobotPart& hhead);

    /**
     * @brief getHead
     * @return
     */
    RobotPart getHead();

    /**
     * @brief setElectricGripper
     * @param egripper
     */
    void setElectricGripper(ElectricGripper& egripper);

    /**
     * @brief getElectricGripper
     * @return
     */
    ElectricGripper getElectricGripper();

    /**
     * @brief setHumanHand
     * @param hhand
     */
    void setHumanHand(HumanHand& hhand);

    /**
     * @brief getHumanHand
     * @return
     */
    HumanHand getHumanHand();


    /**
     * @brief plan_pick
     * @param params
     * @param initPosture
     * @return
     */
    planning_result_ptr plan_pick(hump_params& params, std::vector<double> initPosture);

    /**
     * @brief plan_place
     * @param params
     * @param initPosture
     * @return
     */
    planning_result_ptr plan_place(hump_params& params, std::vector<double> initPosture);

    /**
     * @brief plan_move
     * @param params
     * @param initPosture
     * @return
     */
    planning_result_ptr plan_move(hump_params& params, std::vector<double> initPosture);

    /**
     * @brief plan_move
     * @param params
     * @param initPosture
     * @param finalPosture
     * @return
     */
    planning_result_ptr plan_move(hump_params& params, std::vector<double> initPosture, std::vector<double> finalPosture);

    /**
     * @brief plan_waypoints
     * @param wp
     *
     * @return
     */
    planning_result_ptr plan_waypoints(hump_params &params,vector <wp_specs> wp);


private:

    string name;/**< name of the planner */
    // scenario info
    wpPtr waypoints;

    vector<objectPtr> obstacles; /**< obstacles in the scenario */
    // Robot info
    std::vector<double> shPose; /**< pose of the shoulder of the Robot: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw */
    std::vector<double> elPose; /**< pose of the elbow of the Robot: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw  */
    std::vector<double> wrPose; /**< pose of the wrist of the Robot: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw  */
    std::vector<double> haPose; /**< pose of the hand of the Robot: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw  */
    Matrix4d matWorldToRightArm; /**< transformation matrix from the fixed world frame and the reference frame of the right arm (positions are in [mm]) */
    Matrix4d matRightHand;/**< trabsformation matrix from the last joint of the right arm and the palm of the right hand (positions are in [mm]) */
    std::vector<double> minRightLimits; /**< minimum right limits */
    std::vector<double> maxRightLimits; /**< maximum right limits */
    Matrix4d matWorldToLeftArm; /**< transformation matrix from the fixed world frame and the reference frame of the left arm (positions are in [mm]) */
    Matrix4d matLeftHand; /**< trabsformation matrix from the last joint of the left arm and the palm of the left hand (positions are in [mm]) */
    std::vector<double> minLeftLimits; /**< minimum left limits */
    std::vector<double> maxLeftLimits; /**< maximum left limits */
    DHparameters DH_rightArm; /**< current D-H parameters of the right arm */
    DHparameters DH_leftArm; /**< current D-H parameters of the left arm */
    BarrettHand bhand; /**< parameters of the barrett hand */
    RobotPart head; /**< parameters of the robot' head */
    RobotPart torso; /**< parameters of the robot' torso*/
    HumanHand hhand; /**< parameters of the human hand */
    ElectricGripper egripper; /**< parameters of the electric parallel gripper */

    /**
     * @brief getDerivative
     * @param function
     * @param step_values
     * @param derFunction
     */
    void getDerivative(std::vector<double> &function, std::vector<double> &step_values, std::vector<double> &derFunction);

    /**
     * @brief getTimeStep
     * @param tols
     * @param jointTraj
     * @param mod
     * @return
     */
    double getTimeStep(hump_params& tols, MatrixXd& jointTraj, int mod);

    double getTimeStepWP(hump_params &tols, MatrixXd &jointTraj,int mod);

    /**
     * @brief setBoundaryConditions
     * @param mov_type
     * @param params
     * @param steps
     * @param initPosture
     * @param finalPosture
     * @param mod
     * @return
     */
    bool setBoundaryConditions(int mov_type, hump_params& params, int steps, std::vector<double>& initPosture, std::vector<double>& finalPosture, int mod=0);

    /**
     * @brief directTrajectory
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Traj
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directTrajectory(int mov_type, int steps, hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Traj, MatrixXd &vel_app_ret, int mod);

    /**
     * @brief directTrajectoryNoBound
     * @param steps
     * @param initPosture
     * @param finalPosture
     * @param Traj
     */
    void directTrajectoryNoBound(int steps,std::vector<double>& initPosture, std::vector<double>& finalPosture, MatrixXd &Traj);

    /**
     * @brief directVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Vel
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directVelocity(int steps, hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Vel, MatrixXd &vel_app_ret, int mod);

    /**
     * @brief directAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Acc
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directAcceleration(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Acc,MatrixXd &vel_app_ret,int mod);

    /**
     * @brief backForthTrajectory
     * @param steps
     * @param initPosture
     * @param bouncePosture
     * @param Traj
     */
    void backForthTrajectory(int steps, std::vector<double>& initPosture, std::vector<double>& bouncePosture, MatrixXd& Traj);

    /**
     * @brief backForthVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param bouncePosture
     * @param timestep
     * @param Vel
     */
    void backForthVelocity(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& bouncePosture, double timestep, MatrixXd& Vel);

    /**
     * @brief backForthAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param bouncePosture
     * @param timestep
     * @param Acc
     */
    void backForthAcceleration(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& bouncePosture, double timestep, MatrixXd& Acc);

    /**
     * @brief computeMovement
     * @param direct
     * @param back
     * @param tot
     */
    void computeMovement(const MatrixXd& direct, const MatrixXd& back, MatrixXd& tot);

    /**
     * @brief getTrajectory
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getTrajectory(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                         MatrixXd &traj,MatrixXd &vel_app_ret,bool &success,int mod);

    /**
     * @brief getTrajectory
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getTrajectory(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture,
                         MatrixXd &traj, MatrixXd &vel_app_ret,bool &success, int mod);

    /**
     * @brief getVelocity
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getVelocity(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                       MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret, bool &success, int mod);
    /**
     * @brief getVelocity
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getVelocity(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret,bool &success, int mod);

    /**
     * @brief getAcceleration
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param acc
     * @param success
     * @param mod
     * @return
     */
    double getAcceleration(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod);

    /**
     * @brief getAcceleration
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param acc
     * @param success
     * @param mod
     * @return
     */
    double getAcceleration(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod);


    /**
     * @brief This method writes down the D-H parameters of the arms of the robot
     * @param dh
     * @param stream
     * @param k
     */
    void writeArmDHParams(DHparameters dh, std::ofstream& stream, int k);

    /**
     * @brief This method writes down the distance between the object and the hand
     * @param stream
     * @param dHO
     */
    void write_dHO(std::ofstream& stream, double dHO);

    /**
     * @brief This method writes down the joint limits of the arm
     * @param stream
     * @param minArmLimits
     * @param maxArmLimits
     */
    void writeArmLimits(std::ofstream& stream, std::vector<double>& minArmLimits,std::vector<double>& maxArmLimits, int hand_code);

    /**
     * @brief This method writes down the initial posture of the arm
     * @param stream
     * @param initArmPosture
     */
    void writeArmInitPose(std::ofstream& stream,std::vector<double>& initArmPosture);

    /**
     * @brief This method writes down the final posture of the fingers
     * @param stream
     * @param finalHand
     */
    void writeFingerFinalPose(std::ofstream& stream,std::vector<double>& finalHand);

    /**
     * @brief This method writes down the lambda of the objective function
     * @param stream
     * @param lambda
     */
    void writeLambda(std::ofstream& stream,std::vector<double>& lambda);


    /**
     * @brief writeHumanHandParams
     * This method writes down the parameter of the human hand
     * @param hhand
     * @param stream
     * @param k
     */
    void writeHumanHandParams(HumanHand& hhand, std::ofstream& stream, int k);

    /**
     * @brief This method writes down the declaration of the parameters of the human hand
     * @param stream
     */
    void writeHumanHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the human hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeHumanHandDirKin(std::ofstream& stream,MatrixXd& tolsHand, bool final, bool transport);

    /**
     * @brief writeBarrettHandParams
     * Thi method writes down the parameters of the Barrett Hand
     * @param bhand
     * @param stream
     */
    void writeBarrettHandParams(BarrettHand& bhand, std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the parameters of the Barrett hand
     * @param stream
     */
    void writeBarrettHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the Barrett hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeBarrettHandDirKin(std::ofstream& stream, MatrixXd& tolsHand, bool final, bool transport);

    /**
     * @brief writeBarrettHandParams
     * Thi method writes down the parameters of the Barrett Hand
     * @param bhand
     * @param stream
     */
    void writeElectricGripperParams(ElectricGripper& egripper, std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the parameters of the Barrett hand
     * @param stream
     */
    void writeElectricGripperParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the Barrett hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeElectricGripperDirKin(std::ofstream& stream, MatrixXd& tolsHand, bool final, bool transport);

    /**
     * @brief writeInfoTarget
     * This method writes down the info of the target
     * @param stream
     * @param tar: tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(5)=yaw
     */
    void writeInfoTarget(ofstream &stream, std::vector<double> tar);

    /**
     * @brief writeInfoObjects
     * @param stream
     * @param obstacles
     * @param head_code
     * @param torso
     */
    void writeInfoObjects(ofstream &stream, std::vector<objectPtr> &obstacles, int head_code, RobotPart &torso);

    /**
     * @brief writeInfoObjectTarget
     * @param stream
     * @param obj
     */
    void writeInfoObjectTarget(ofstream &stream, objectPtr obj);

    /**
     * @brief writePI
     * @param stream
     */
    void writePI(std::ofstream& stream);

    /**
     * @brief writeBodyInfoMod
     * @param stream
     * @param body_pos
     */
    void writeBodyInfoMod(ofstream &stream, int body_pos);

    /**
     * @brief writeArmDHParamsMod
     * @param stream
     */
    void writeArmDHParamsMod(ofstream &stream);

    /**
     * @brief write_dHOMod
     * @param stream
     */
    void write_dHOMod(ofstream &stream);
    /**
     * @brief writeInfoObjectsMod
     * @param stream
     * @param vec
     * @param nobjects
     */
    void writeInfoObjectsMod(ofstream &stream,bool vec, int nobjects);

    /**
     * @brief writeInfoObjectsMod_place
     * @param stream
     * @param vec
     * @param nobjects
     */
    void writeInfoObjectsMod_place(ofstream &stream,bool vec, int nobjects);

    /**
     * @brief writeRotMatObjects
     * @param stream
     * @param objects_size
     */
    void writeRotMatObjects(ofstream &stream, int objects_size);

    /**
     * @brief writeRotMatObjTar
     * @param stream
     */
    void writeRotMatObjTar(ofstream &stream);

    /**
     * @brief writeArmDirKin
     * @param stream
     * @param matWorldToArm
     * @param matHand
     * @param tolsArm
     * @param dh_arm_d
     * @param final
     */
    void writeArmDirKin(ofstream &stream, Matrix4d &matWorldToArm, Matrix4d &matHand, std::vector<double>& tolsArm, vector<double> dh_arm_d, bool final);

    /**
     * @brief writeObjective
     * @param stream
     * @param final
     */
    void writeObjective(ofstream &stream, bool final);

    /**
     * @brief writeBodyConstraints
     * @param stream
     * @param final
     * @param npoints
     * @param body_pos
     * @param nsphere
     */
    void writeBodyConstraints(ofstream &stream, bool final, int npoints, int body_pos, std::vector<int> nsphere);

    /**
     * @brief RPY_matrix
     * @param rpy
     * @param Rot
     */
    void RPY_matrix(std::vector<double>rpy, Matrix3d &Rot);

    /**
     * @brief Trans_matrix
     * @param xyz
     * @param rpy
     * @param Trans
     */
    void Trans_matrix(std::vector<double>xyz,std::vector<double>rpy,Matrix4d& Trans);

    /**
     * @brief getRotAxis
     * @param xt
     * @param id
     * @param rpy
     */
    void getRotAxis(vector<double>& xt, int id,std::vector<double>rpy);

    /**
     * @brief getRand
     * @param min
     * @param max
     * @return
     */
    double getRand(double min, double max);


    //std::string exec(const char* cmd);

    /**
     * @brief amplRead
     * @param datFile
     * @param modFile
     * @param nlFile
     * @return
     */
    bool amplRead(string &datFile, string &modFile, string &nlFile);

    /**
     * @brief optimize
     * @param nlfile
     * @param x
     * @param tol
     * @param acc_tol
     * @param constr_viol_tol
     * @return
     */
    bool optimize(string &nlfile, std::vector<Number>& x, double tol, double acc_tol, double constr_viol_tol);

    /**
     * @brief getObstaclesSingleArm
     * @param center
     * @param radius
     * @param obsts
     * @param hand_code
     */
    void getObstaclesSingleArm(std::vector<double> center, double radius, std::vector<objectPtr>& obsts, int hand_code);

    /**
     * @brief writeInfoApproachRetreat
     * @param stream
     * @param tar
     * @param approach_retreat
     */
    void writeInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat);

    /**
     * @brief writeInfoApproachRetreat_place
     * @param stream
     * @param tar
     * @param approach
     * @param retreat
     */
    void writeInfoApproachRetreat_place(ofstream &stream, std::vector<double> tar, std::vector<double> approach, std::vector<double> retreat);

    /**
     * @brief singleArmFinalPosture
     * @param mov_type
     * @param pre_post
     * @param params
     * @param initPosture
     * @param finalPosture
     * @return
     */
    bool singleArmFinalPosture(int mov_type,int pre_post,hump_params& params, std::vector<double> initPosture, std::vector<double>& finalPosture);

    /**
     * @brief writeFilesFinalPosture
     * @param params
     * @param mov_type
     * @param initArmPosture
     * @param initialGuess
     * @param obsts
     * @param pre_post
     * @return
     */
    bool writeFilesFinalPosture(hump_params& params,int mov_type, int pre_post,std::vector<double> initArmPosture, std::vector<double> initialGuess,std::vector<objectPtr> obsts);


    /**
     * @brief singleArmBouncePosture
     * @param steps
     * @param mov_type
     * @param pre_post
     * @param params
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @return
     */
    bool singleArmBouncePosture(int steps,int mov_type,int pre_post,hump_params& params,std::vector<double> initPosture,std::vector<double> finalPosture,std::vector<double>& bouncePosture);


    /**
     * @brief writeFilesBouncePosture
     * @param steps
     * @param params
     * @param mov_type
     * @param pre_post
     * @param minAuxLimits
     * @param maxAuxLimits
     * @param initAuxPosture
     * @param finalAuxPosture
     * @param initialGuess
     * @param objs
     * @param bAux
     * @return
     */
    bool writeFilesBouncePosture(int steps,hump_params& params,int mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                             std::vector<double> initialGuess, std::vector<objectPtr> objs,boundaryConditions bAux);


    /**
     * @brief getSteps
     * @param maxLimits
     * @param minLimits
     * @param initPosture
     * @param finalPosture
     * @return
     */
    int getSteps(std::vector<double>& maxLimits,std::vector<double>& minLimits,std::vector<double>& initPosture,std::vector<double>& finalPosture, int hand_code);

    int getStepsWP(std::vector<double> &maxLimits, std::vector<double> &minLimits, vector <wp_specs> wp);


    /**
     * @brief model_spheres
     * @param stream_dat
     * @param stream_model
     * @param obj_tar_size
     * @param final
     * @return
     */
    int model_spheres(ofstream &stream_dat,ofstream &stream_model,std::vector<double>& obj_tar_size,bool final);

    /**
     * @brief compare_sizes
     * @param pair_1
     * @param pair_2
     * @return
     */
    bool static compare_sizes (std::pair<std::string,double> pair_1, std::pair<std::string,double> pair_2);

    //double getAlpha(int arm, std::vector<double> &posture);


    //int invKinematics(int arm, std::vector<double>& pose, double alpha, std::vector<double> &init_posture, std::vector<double>& posture);

    /**
     * @brief RotMatrix
     * @param theta
     * @param alpha
     * @param Rot
     */
    void RotMatrix(double theta, double alpha, Matrix3d& Rot);

    /**
     * @brief transfMatrix
     * @param alpha
     * @param a
     * @param d
     * @param theta
     * @param T
     */
    void transfMatrix(double alpha, double a, double d, double theta, Matrix4d &T);

    /**
     * @brief getRPY
     * @param rpy
     * @param Rot
     * @return
     */
    bool getRPY(std::vector<double>& rpy, Matrix3d& Rot);

    /**
     * @brief directKinematicsSingleArm
     * @param arm
     * @param posture
     */
    void directKinematicsSingleArm(int arm, std::vector<double>& posture);


    /**
     * @brief getShoulderPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getShoulderPos(int arm,vector<double> &posture,vector<double> &pos);

    /**
     * @brief getShoulderOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getShoulderOr(int arm, vector<double> &posture,vector<double> &orient);


    /**
     * @brief getElbowPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getElbowPos(int arm,vector<double> &posture,vector<double> &pos);


    /**
     * @brief getElbowOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getElbowOr(int arm, vector<double> &posture,vector<double> &orient);

    /**
     * @brief getWristPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getWristPos(int arm,vector<double> &posture,vector<double> &pos);

    /**
     * @brief getWristOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getWristOr(int arm, vector<double> &posture,vector<double> &orient);

    /**
     * @brief getHandPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getHandPos(int arm,vector<double> &posture,vector<double> &pos);

    /**
     * @brief getHandOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getHandOr(int arm, vector<double> &posture,vector<double> &orient);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// trajectory through waypoints */
    ///
    /////////////////////////////////////////////////////////////////
    /**
     * @brief get_eq
     * @param tau_wp_or
     * @param tau_wp1
     * @param tau_wp2
     * @param a
     * @param b
     * @return
     */
    double get_eq(vector <double> tau_wp_or, vector <double> tau_wp1, vector <double> tau_wp2, int a , int b);
    /**
     * @brief den1_wp
     * @param tau_wp_or
     * @param tau_wp1
     * @param tau_wp2
     * @param n
     * @return
     */
    vector<double> den1_wp(vector <double> tau_wp_or, vector <double> tau_wp1, vector <double> tau_wp2, int n);
    /**
     * @brief num1_wp
     * @param tau_wp_or
     * @param tau_wp1
     * @param tau_wp2
     * @param n
     * @return
     */
    vector<double> num1_wp(vector <double> tau_wp_or, vector <double> tau_wp1, vector <double> tau_wp2, int n);
    /**
     * @brief get_eq_den
     * @param tau_wp_or
     * @param tau_wp_bk
     * @param tau_wp
     * @param res
     * @return
     */
    double get_eq_den(vector <double> tau_wp_or, vector <double> tau_wp_bk, vector <double> tau_wp, double res=0);
    /**
     * @brief get_eq_num
     * @param tau_wp_or
     * @param tau_wp_bk
     * @param tau_wp
     * @param x_wp
     * @param xf
     * @param x0
     * @param res
     * @return
     */
    double get_eq_num(vector <double> tau_wp_or, vector <double> tau_wp_bk, vector <double> tau_wp, vector<double> x_wp,double xf,double x0, double res=0);
    /**
     * @brief get_pi_num
     * @param tau_wp_or
     * @param tau_wp
     * @param x_wp
     * @param xf
     * @param x0
     * @return
     */
    vector<double> get_pi_num(vector <double> tau_wp_or, vector <double> tau_wp, vector <double> x_wp,double xf,double x0);
    /**
     * @brief get_pi_eq
     * @param tau_wp
     * @param x_wp
     * @param xf
     * @param x0
     * @return
     */
    vector<double> get_pi_eq(double tf,vector <double> tau_wp, vector <double> x_wp, double xf, double x0);
    /**
     * @brief get_equations
     * @param tau_wp
     * @param x_wp_dof
     * @param xf_dof
     * @param x0_dof
     * @return
     */
    vector<double> get_equations(vector<double> tau_wp,vector<vector<double>> x_wp_dof,vector<double> xf_dof,vector<double> x0_dof, double tf);
    /**
     * @brief get_eq_minus
     * @param tau_wp
     * @param pi
     * @param xf_dof
     * @param x0_dof
     * @param x_minus
     * @param v_minus
     * @param acc_minus
     */
    void get_eq_minus(vector<double> tau,vector <double> tau_wp, vector <double> pi, double xf_dof, double x0_dof, vector <double> &x_minus,vector <double> &v_minus,vector <double> &acc_minus, double tf);
    /**
     * @brief get_eq_plus
     * @param tau_wp
     * @param pi
     * @param x_minus
     * @param v_minus
     * @param acc_minus
     * @param x_plus
     * @param v_plus
     * @param acc_plus
     */
    void get_eq_plus(vector<double> tau,vector <double> tau_wp, vector <double> pi, vector <double> x_minus, vector <double> v_minus,  vector <double> acc_minus, vector <vector<double>> &x_plus, vector <vector<double>> &v_plus, vector <vector<double>> &acc_plus, double tf);
    /**
     * @brief compose
     * @param tau_wp
     * @param pi
     * @param xf_dof
     * @param x0_dof
     * @param pos
     * @param vel
     * @param acc
     * @param wp_pos_calc
     * @param wp_vel_calc
     * @param wp_acc_calc
     */
    void compose(double tf,int steps, vector<double>tau_wp,vector<double>pi, double xf_dof, double x0_dof,int joint, MatrixXd &pos, MatrixXd &vel, MatrixXd &acc, vector <double> &wp_pos_calc, vector <double> &wp_vel_calc, vector <double> &wp_acc_calc);
    /**
     * @brief get_init_guess
     * @param x_wp
     * @param x0
     * @param xf
     * @return
     */
    vector<double> get_init_guess(vector <double> x_wp, double x0, double xf);
    /**
     * @brief get_init_guess_dof
     * @param x_wp_dof
     * @param xf_dof
     * @param x0_dof
     * @return
     */
    vector <double> get_init_guess_dof(vector <vector<double>> x_wp_dof,vector <double> xf_dof,vector <double> x0_dof);
    /**
     * @brief more_init_guess
     * @param init_guess
     * @return
     */
    vector <vector<double>> more_init_guess(vector <double> init_guess);

    /**
     * @brief get_time_wp
     * @param x_wp_dof
     * @param xf_dof
     * @param x0_dof
     * @return
     */
    //function that returns the time of each waypoint
    vector <double> get_time_wp(vector<vector<double>> x_wp_dof, vector<double> xf_dof, vector<double> x0_dof);
    /**
     * @brief waypoint_solver
     * @param wp
     * @param wp_time
     * @return
     */
    bool waypoint_time_solver(wp_traj wp_traj_spec,std::vector<double>& wp_time);

    bool wp_time_py_solver();


    bool waypoint_time_solver_py(double tf, wp_traj wp_traj_spec , std::vector<double>& wp_time);

    bool writeFilesWaypoints_py(double tf, wp_traj wp_traj_spec);

    /**
     * @brief writeFilesWaypoints
     * @param wp
     * @param xf_dof
     * @param x0_dof
     * @param init_guess
     * @return
     */
    bool writeFilesWaypoints(wp_traj wp_traj_spec, std::vector <double> init_guess);
    /**
     * @brief writeWaypoints
     * @param wp
     * @param stream
     */
    void writeWaypoints(wp_traj wp_traj_spec, ofstream &stream);
    /**
     * @brief writeFirstWaypoint
     * @param x0_dof
     * @param stream
     */
    void writeFirstWaypoint( vector<double> x0_dof,ofstream &stream);
    /**
     * @brief writeLastWaypoint
     * @param xf_dof
     * @param stream
     */
    void writeLastWaypoint( vector<double> xf_dof,ofstream &stream);

    /**
    * @brief get_eq
    * @param tau_wp_or
    * @param tau_wp1
    * @param tau_wp2
    * @param a
    * @param b
    * @return
    */
   string get_eq_print(vector <double> tau_wp_or, vector <double> tau_wp1, vector <double> tau_wp2, int a , int b);
   /**
    * @brief den1_wp
    * @param tau_wp_or
    * @param tau_wp1
    * @param tau_wp2
    * @param n
    * @return
    */
   vector<string> den1_wp_print(vector <double> tau_wp_or, vector <double> tau_wp1, vector <double> tau_wp2, int n);
   /**
    * @brief num1_wp
    * @param tau_wp_or
    * @param tau_wp1
    * @param tau_wp2
    * @param n
    * @return
    */
   vector<string> num1_wp_print(vector <double> tau_wp_or, vector <double> tau_wp1, vector <double> tau_wp2, int n);
   /**
    * @brief get_eq_den
    * @param tau_wp_or
    * @param tau_wp_bk
    * @param tau_wp
    * @param res
    * @return
    */
   string get_eq_den_print(vector <double> tau_wp_or, vector <double> tau_wp_bk, vector <double> tau_wp, string res = "");
   /**
    * @brief get_eq_num
    * @param tau_wp_or
    * @param tau_wp_bk
    * @param tau_wp
    * @param x_wp
    * @param xf
    * @param x0
    * @param res
    * @return
    */
   string get_eq_num_print(vector <double> tau_wp_or, vector <double> tau_wp_bk, vector <double> tau_wp, vector<double> x_wp,double xf,double x0, string res = "");
   /**
    * @brief get_pi_num
    * @param tau_wp_or
    * @param tau_wp
    * @param x_wp
    * @param xf
    * @param x0
    * @return
    */
   vector<string> get_pi_num_print(vector <double> tau_wp_or, vector <double> tau_wp, vector <double> x_wp,double xf,double x0);
   /**
    * @brief get_pi_eq
    * @param tau_wp
    * @param x_wp
    * @param xf
    * @param x0
    * @return
    */
   vector <string> get_pi_eq_print(vector <double> tau_wp, vector <double> x_wp, double xf, double x0);
   /**
    * @brief get_equations
    * @param tau_wp
    * @param x_wp_dof
    * @param xf_dof
    * @param x0_dof
    * @return
    */
   void objective_wp_time(ofstream &stream,vector<double> tau_wp,vector<vector<double>> x_wp_dof,vector<double> xf_dof,vector<double> x0_dof);


   void objective_wp_constraints(ofstream &stream);

   void organize_waypoints(vector <wp_specs>wp, wp_traj &wp_traj_spec);
};

} // namespace HUMotion

#endif // HUMPLANNER_H
