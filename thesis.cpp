#include <dart/dart.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <Eigen/Eigen>

#include "config.h"
#include "mywindow.h"
#include "util.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <sstream>
#include <string>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ds = dart::simulation;
namespace dd = dart::dynamics;
namespace dc = dart::collision;
namespace du = dart::utils;

constexpr double jointMax1 = 3.1416;
constexpr double jointMax2 = 2.5744;
constexpr double jointMax3 = 2.5307;
constexpr double jointMax4 = 4.7124;
constexpr double jointMax5 = 2.4435;
constexpr double jointMax6 = 4.7124;

constexpr double jointMin1 = -3.1416;
constexpr double jointMin2 = -2.2689;
constexpr double jointMin3 = -2.5307;
constexpr double jointMin4 = -4.7124;
constexpr double jointMin5 = -2.0071;
constexpr double jointMin6 = -4.7124;

#define DIM 6

class Simple3DEnvironment {
    public:
        Simple3DEnvironment() {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            space->addDimension(jointMin1, jointMax1);
            space->addDimension(jointMin2, jointMax2);
            space->addDimension(jointMin3, jointMax3);
            space->addDimension(jointMin4, jointMax4);
            space->addDimension(jointMin5, jointMax5);
            space->addDimension(jointMin6, jointMax6);

            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

            // set state validity checking for this space
            ss_->setStateValidityChecker(std::bind(&Simple3DEnvironment::isStateValid,
                        this, std::placeholders::_1));
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
                    1.0 / space->getMaximumExtent());
            ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));
        }

        bool plan(const Eigen::VectorXd &init, const Eigen::VectorXd & final) {
            if (!ss_) return false;

            ob::ScopedState<> start(ss_->getStateSpace());
            for (std::size_t i = 0; i < DIM; ++i) start[i] = init[i] * M_PI / 180.0;

            ob::ScopedState<> goal(ss_->getStateSpace());
            for (std::size_t i = 0; i < DIM; ++i) goal[i] = final[i] * M_PI / 180.0;

            ss_->setStartAndGoalStates(start, goal);

            // this will run the algorithm for one second
            ss_->solve(60 * 1);

            // ss_->solve(1000); // it will run for 1000 seconds

            const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
            OMPL_INFORM("Found %d solutions", (int)ns);
            if (ss_->haveSolutionPath()) {
                ss_->simplifySolution();
                og::PathGeometric &p = ss_->getSolutionPath();
                ss_->getPathSimplifier()->simplifyMax(p);
                ss_->getPathSimplifier()->smoothBSpline(p); return true;
            } else
                return false;
        }

        void recordSolution() {
            if (!ss_ || !ss_->haveSolutionPath()) return;
            og::PathGeometric &p = ss_->getSolutionPath();
            p.interpolate(1000);
            std::ofstream resultfile;
            resultfile.open("result.txt", std::ios::trunc);

            std::ofstream endeffectorfile;
            endeffectorfile.open("endeffector.txt", std::ios::trunc);

            for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
            {
                const double j1 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];
                const double j2 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];
                const double j3 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2];
                const double j4 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[3];
                const double j5 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[4];
                const double j6 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[5];

                dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

                staubli->getDof(2)->setPosition(j1); 
                staubli->getDof(3)->setPosition(j2); 
                staubli->getDof(4)->setPosition(j3); 
                staubli->getDof(5)->setPosition(j4); 
                staubli->getDof(6)->setPosition(j5); 
                staubli->getDof(7)->setPosition(j6); 

                Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
                Eigen::Quaterniond quat(transform.rotation());
                Eigen::Vector3d tr = transform.translation();

                resultfile << j1 << " " << j2 << " " << j3 << " " << j4 << " " << j5 << " " << j6<< std::endl; 
                endeffectorfile << tr(0) << " " << tr(1) << " " << tr(2)
                    << " " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() 
                    << std::endl;
            }
            resultfile.close();

            //
            // ADD CODE HERE
            //
        }

        void setWorld(const ds::WorldPtr &world) { world_ = world; }

    private:
        bool isStateValid(const ob::State *state) const {
            double j1 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]; 
            double j2 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]; 
            double j3 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]; 
            double j4 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3]; 
            double j5 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4]; 
            double j6 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5]; 
            dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

            staubli->getDof(2)->setPosition(j1); 
            staubli->getDof(3)->setPosition(j2); 
            staubli->getDof(4)->setPosition(j3); 
            staubli->getDof(5)->setPosition(j4); 
            staubli->getDof(6)->setPosition(j5); 
            staubli->getDof(7)->setPosition(j6); 

            return !world_->checkCollision();

        }

        og::SimpleSetupPtr ss_;
        ds::WorldPtr world_;
};

std::istream& ignoreline(std::ifstream& in, std::ifstream::pos_type& pos) {
    pos = in.tellg();
    return in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

std::string getLastLine(std::ifstream& in) {
    std::ifstream::pos_type pos = in.tellg();

    std::ifstream::pos_type lastPos;
    while (in >> std::ws && ignoreline(in, lastPos)) pos = lastPos;

    in.clear();
    in.seekg(pos);

    std::string line;
    std::getline(in, line);
    return line;
}

std::string getWorkingDirectory() {
    char buff[1024] = {0};

    getcwd(buff, 1024);

    return std::string(buff);
}


Eigen::VectorXd joints (const Eigen::VectorXd &init, const Eigen::Isometry3d &final_point) 
{
    Eigen::VectorXd final_joint(6);
    final_joint << 0, 0, 0, 0, 0, 0;
    double alpha[7] = {0, -M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0};
    double a[7] = {0, 50, 650, 0, 0, 0, 0};
    double d[7] = {0, 0, 0, 50, 650, 0, 100};
    double theta[7] = {0, 0, 0, 0, 0, 0, 0};

    Eigen::Matrix3d rot_final_point = final_point.rotation();
    Eigen::Vector3d trans_final_point = final_point.translation();

    double a_x = rot_final_point(2,0);
    double a_y = rot_final_point(2,1);
    double a_z = rot_final_point(2,2);

    double p_ax = trans_final_point(0) - d[6] * a_x;
    double p_ay = trans_final_point(1) - d[6] * a_y;
    double p_az = trans_final_point(2) - d[6] * a_z;
    for (int i = 0 ; i < 8; i++) { 
        theta[1] = atan2(p_ay, p_ax) - atan2(d[3], pow(-1, (i & 2) >> 1)*sqrt(p_ax*p_ax + p_ay*p_ay - d[3]));

        double t_1 = (pow(p_ax*cos(theta[1]) + p_ay*sin(theta[1]) - a[1],2) + a[2]*a[2] + p_az*p_az - d[4]*d[4]) / (2*a[2]); 
        theta[2] = atan2((p_ax*cos(theta[1]) + p_ay*sin(theta[1]) - a[1]), p_az) 
            -atan2(t_1, pow(-1, (i & 4) >> 2) * sqrt(pow(p_ax*cos(theta[1]) + p_ay*sin(theta[1]) - a[1], 2) + p_az*p_az - t_1*t_1));// + M_PI / 2;

        theta[3] = atan2(p_ax*cos(theta[1]) + p_ay*sin(theta[1]) - a[1] - a[2]*cos(theta[2]), p_az + a[2]*sin(theta[2])) - theta[2];// - M_PI / 2;



        Eigen::Matrix3d rot_R3_0;
        rot_R3_0 = Eigen::Quaterniond((Eigen::AngleAxisd(theta[1], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(alpha[1], Eigen::Vector3d::UnitX())) * 
                (Eigen::AngleAxisd(theta[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(alpha[2], Eigen::Vector3d::UnitX())) *
                (Eigen::AngleAxisd(theta[3], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(alpha[3], Eigen::Vector3d::UnitX())));
        rot_R3_0 = rot_R3_0.transpose() * rot_final_point; 

        double a_wx = rot_R3_0(2,0);
        double a_wy = rot_R3_0(2,1);
        double a_wz = rot_R3_0(2,2);
        double n_wz = rot_R3_0(0,2);
        double o_wz = rot_R3_0(1,2);

        theta[5] = atan2(pow(-1, (i & 1)) * sqrt(a_wx*a_wx + a_wy*a_wy),a_wz);    

        theta[6] = -atan2(-a_wy / sin(theta[5]), -a_wx / sin(theta[5]));    

        theta[4] = ((i == 1) ? 1:(-1)) * atan2(-o_wz / sin(theta[5]), n_wz / sin(theta[5]));    

        final_joint << theta[1], theta[2] + M_PI / 2, theta[3] - M_PI/2 , theta[4] , theta[5] , theta[6]; 
        final_joint = final_joint * 180.0 / M_PI;
        //std::cout << final_joint << std::endl;
        //std::cout << std::endl;
        std::cout << theta[1]  << " " << theta[2] + M_PI / 2  << " " << theta[3] - M_PI/2 << " " << theta[4] << " " << theta[5] << " " << theta[6] << std::endl;
    }
    //std::cout << theta[1]  << " " << theta[2] + M_PI / 2  << " " << theta[3] - M_PI/2 << " " << theta[4] << " " << theta[5] << " " << theta[6] << std::endl;

    return final_joint;
}


int main(int argc, char* argv[]) 
{
    ds::WorldPtr world = std::make_shared<ds::World>();
    world->getConstraintSolver()->setCollisionDetector(
            new dc::FCLCollisionDetector());
    //    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

    std::string prefix = getWorkingDirectory();

    dd::SkeletonPtr staubli = du::SdfParser::readSkeleton(prefix + std::string("/model.sdf"));
    staubli->setName("staubli");
    /*
       dd::SkeletonPtr tensegrity = du::SdfParser::readSkeleton(prefix + std::string("/tensegrity.sdf"));
       tensegrity->setName("tensegrity");
     */
    setAllColors(staubli, Eigen::Vector3d(0.9, 0.9, 0.9));
    staubli->enableSelfCollision();
    /*
       dd::SkeletonPtr ball = dd::Skeleton::create("ball");
       Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

       tf.translation() = Eigen::Vector3d(-0.25, 0,1.75);
       createBall(ball, Eigen::Vector3d(0.15,0.15,0.15), tf);

       world->addSkeleton(ball);
     */
    world->addSkeleton(staubli);
    //   world->addSkeleton(tensegrity);

    //    staubli->getDof(3)->setPosition(290 * M_PI / 180.0); 
    //    staubli->getDof(4)->setPosition(290 * M_PI / 180.0); 
    ////    staubli->getDof(6)->setPosition(290 * M_PI / 180.0); 

    Eigen::VectorXd start(6);
    start << 0,0,0,0,0,0;

    Eigen::VectorXd finish(6);
    finish << 0, -67, -144, 0, 0, 0;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(334.59, 589.47, 419.04);
    //tf.translation() = Eigen::Vector3d(685.32, 392.71, 592.72);
    tf.rotate(Eigen::AngleAxisd(-167.76 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(9.18 * M_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(12.24 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
    //tf.rotate(Eigen::AngleAxisd(-180 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.001 * M_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.005 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));

    joints (start, tf) ;

    if (argc < 2) {


        Simple3DEnvironment env;
        env.setWorld(world);

        if(env.plan(start,finish))
        {
            env.recordSolution();
        }
    }

    MyWindow window(world);
    double j1, j2, j3, j4, j5, j6 = 0;

    std::thread t([&]()
            {
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            //while (true) 
            //{
            std::ifstream fin("result.txt");

            while (!fin.eof()) 
            {
            //float rx, ry, rz, rw;
            fin >> j1 >> j2 >> j3 >> j4 >> j5 >> j6;

            staubli->getDof(2)->setPosition(j1); 
            staubli->getDof(3)->setPosition(j2); 
            staubli->getDof(4)->setPosition(j3); 
            staubli->getDof(5)->setPosition(j4); 
            staubli->getDof(6)->setPosition(j5); 
            staubli->getDof(7)->setPosition(j6); 

            window.setViewTrack(j1,j2,j3,j4, j5, j6);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
    //}
            });


    glutInit(&argc, argv);
    window.initWindow(640 * 2, 480 * 2, "SDF");

    glutMainLoop();

    t.join();

    return 0;
}
