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
#include "ompl/geometric/planners/rrt/RRTstarQ.h"

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

constexpr double jointMax[6] = {3.1416, 2.5744, 2.5307, 4.7124, 2.4435, 4.7124};
constexpr double jointMin[6] = {-3.1416, -2.2689, -2.5307, -4.7124, -2.0071, -4.7124};
//constexpr double jointMin[6] = {-3.1416, -2.2689, -2.5307, -4.7124, -2.0071, -4.7124};
//constexpr double jointMax[6] = {3.1416, 2.5744, 2.5307, 4.7124, 2.4435, 4.7124};


class Simple3DEnvironment {
    public:
        Simple3DEnvironment(std::size_t jointNum) {
            jointNumber = jointNum; 
            space = new ob::RealVectorStateSpace();
            space->addDimension(jointMin[0], jointMax[0]);
            space->addDimension(jointMin[1], jointMax[1]);
            space->addDimension(jointMin[2], jointMax[2]);

            if (jointNumber > 3){
                space->addDimension(jointMin[3], jointMax[3]);
                space->addDimension(jointMin[4], jointMax[4]);
                space->addDimension(jointMin[5], jointMax[5]);
            }
            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

            // set state validity checking for this space
            ss_->setStateValidityChecker(std::bind(&Simple3DEnvironment::isStateValid,
                        this, std::placeholders::_1));
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
                    0.01 );
            std::cout << "Get MAximum EXtent***: " << space->getMaximumExtent() << std::endl;
            ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));
            //std::cout <<ss_->getPlanner()->as<og::RRTstar>()->setRange(10.0 * M_PI / 180.0) << std::endl;
        }

        bool plan(const Eigen::VectorXd &init, const Eigen::VectorXd & final) {
            if (!ss_) return false;

            ob::ScopedState<> start(ss_->getStateSpace());
            for (std::size_t i = 0; i < jointNumber; ++i) start[i] = init[i] * M_PI / 180.0;

            ob::ScopedState<> goal(ss_->getStateSpace());
            for (std::size_t i = 0; i < jointNumber; ++i) goal[i] = final[i] * M_PI / 180.0;

            ss_->setStartAndGoalStates(start, goal, 0.05);

            // this will run the algorithm for one second
            if (jointNumber > 3){
                ss_->solve(60 * 1 * 10);
            }
            else{
                ss_->solve(60 * 1 * 10);
            }

            // ss_->solve(1000); // it will run for 1000 seconds

            const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
            OMPL_INFORM("Found %d solutions", (int)ns);
            if (ss_->haveSolutionPath()) {
                ss_->simplifySolution();
                og::PathGeometric &p = ss_->getSolutionPath();
                ss_->getPathSimplifier()->simplifyMax(p);
                ss_->getPathSimplifier()->smoothBSpline(p); 
                return true;
            } else
                return false;
        }

        void recordSolution(const Eigen::VectorXd &start) {
            if (!ss_ || !ss_->haveSolutionPath()) return;
            og::PathGeometric &p = ss_->getSolutionPath();
            p.interpolate(1000);
            std::ofstream resultfile;
            resultfile.open("result1.txt", std::ios::app);

            std::ofstream endeffectorfile;
            endeffectorfile.open("endeffector1.txt", std::ios::app);

            for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
            {
                const double j1 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];
                const double j2 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];
                const double j3 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2];
                double j4 = start[3]; double j5 = start[4]; double j6 = start[5];
                if (jointNumber > 3)
                {
                    j4 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[3];
                    j5 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[4];
                    j6 = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[5];
                }

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
/*
            ob::PlannerData pdat(ss_->getSpaceInformation());
            ss_->getPlannerData(pdat);
            std::ofstream ofs_e("edges.txt");
            std::vector<unsigned int> edge_list;
            std::vector<double> reals;
            std::vector<double> realsOld;
            //bool isMajorTree = false;
            ob::State* s3 = space->allocState();
            for (unsigned int i(0); i < pdat.numVertices(); ++i) {
                unsigned int n_edge = pdat.getEdges(i, edge_list);
                const ob::State* s1 = pdat.getVertex(i).getState();
                //isMajorTree = pdat.getVertex(i).getTag();
                for (unsigned int i2(0); i2 < n_edge; ++i2) {
                    const ob::State* s2 = pdat.getVertex(edge_list[i2]).getState();
                    double step = 0.05;
                    if (space->distance(s1, s2) < 0.03) {
                        step = 0.2;
                    }
                    space->copyToReals(realsOld, s1);
                    for (double t = step; t <= 1.01; t += step) {
                        space->interpolate(s1, s2, t, s3);
                        space->copyToReals(reals, s3);

                        dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

                        //for (const auto& r : realsOld)

                        //    ofs_e << r << " ";
                        //

                        staubli->getDof(2)->setPosition(reals[0]); 
                        staubli->getDof(3)->setPosition(reals[1]); 
                        staubli->getDof(4)->setPosition(reals[2]); 

                        Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
                        Eigen::Vector3d tr = transform.translation();

                        ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;

                        // for (const auto& r : reals)
                        //    ofs_e << r << " ";
                        //
                        //ofs_e << "0x" << std::hex << (isMajorTree ? 0x4488AA : 0xDD6060)
                        //      << std::endl;
                        //ofs_e << std::endl;
                    }
                }
            }
*/
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
            double j4 = 0; double j5 = 0; double j6 = 0;
            if(jointNumber > 3)
            {
                j4 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3]; 
                j5 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4]; 
                j6 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5]; 
            }
            dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

            staubli->getDof(2)->setPosition(j1); 
            staubli->getDof(3)->setPosition(j2); 
            staubli->getDof(4)->setPosition(j3); 
            staubli->getDof(5)->setPosition(j4); 
            staubli->getDof(6)->setPosition(j5); 
            staubli->getDof(7)->setPosition(j6); 


            Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
            Eigen::Vector3d tr = transform.translation();

            return !world_->checkCollision();// && tr(0) <= 0.50;

        } 
        og::SimpleSetupPtr ss_;
        ds::WorldPtr world_;
        ob::RealVectorStateSpace *space;
        std::size_t jointNumber;
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


Eigen::VectorXd joints (const Eigen::VectorXd &init, const Eigen::Isometry3d &final_point, const ds::WorldPtr &world) 
{
    using Eigen::VectorXd;

    VectorXd final_joint(6);
    final_joint << 0, 0, 0, 0, 0, 0;
    double alpha[7] = {0, -M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0};
    double a[7] = {0, 50, 650, 0, 0, 0, 0};
    double d[7] = {0, 0, 0, 50, 650, 0, 100};
    double theta[7] = {0, 0, 0, 0, 0, 0, 0};

    Eigen::Matrix3d rot_final_point = final_point.rotation();
    Eigen::Vector3d trans_final_point = final_point.translation();

    double a_x = rot_final_point(0,2);
    double a_y = rot_final_point(1,2);
    double a_z = rot_final_point(2,2);


    double p_ax = trans_final_point(0) - d[6] * a_x;
    double p_ay = trans_final_point(1) - d[6] * a_y;
    double p_az = trans_final_point(2) - d[6] * a_z;

    double minCost = 10000;

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

        theta[4] = ((i == 8) ? 1:(-1)) * atan2(-o_wz / sin(theta[5]), n_wz / sin(theta[5]));    

        theta[2] = theta[2] + (M_PI / 2.0);
        theta[3] = theta[3] - (M_PI / 2.0);

        bool accept = true;
        for (int j = 1; j < 7; j++)
        {
            if(std::abs(theta[j]) > M_PI)
            {
                theta[j] = -1 * theta[j] / std::abs(theta[j]) * ( 2*M_PI - std::abs(theta[j]));
            }
            if (theta[j] > jointMax[j-1] || theta[j] < jointMin[j-1])
            {
                accept = false;
            }

        }

        theta[0] = sqrt(10*pow(theta[1] - init[0],2) +
                10*pow(theta[2] - init[1],2) +
                10*pow(theta[3] - init[2],2) +
                1*pow(theta[4] - init[3],2) +
                1*pow(theta[5] - init[4],2) +
                1*pow(theta[6] - init[5],2) );
        dd::SkeletonPtr staubli = world->getSkeleton("staubli");

        staubli->getDof(2)->setPosition(theta[1]); 
        staubli->getDof(3)->setPosition(theta[2]); 
        staubli->getDof(4)->setPosition(theta[3]); 
        staubli->getDof(5)->setPosition(theta[4]); 
        staubli->getDof(6)->setPosition(theta[5]); 
        staubli->getDof(7)->setPosition(theta[6]); 

        if (theta[0] < minCost && accept && !world->checkCollision())
        {
            final_joint << theta[1], theta[2], theta[3], theta[4] , theta[5] , theta[6]; 
            final_joint = final_joint * 180.0 / M_PI;
            minCost = theta[0];
        }   

        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

        std::cout << theta[0] << std::endl;
        std::cout << final_joint.format(CommaInitFmt) << std::endl;
        //std::cout << std::endl;
        std::cout << theta[1] * 180.0 / M_PI << " " << theta[2] * 180.0 / M_PI<< " " << theta[3] * 180.0 / M_PI<< " " << theta[4] * 180.0 / M_PI<< " " << theta[5] * 180.0 / M_PI<< " " << theta[6] * 180.0 / M_PI<< std::endl;
    }
    //std::cout << theta[1]  << " " << theta[2] + M_PI / 2  << " " << theta[3] - M_PI/2 << " " << theta[4] << " " << theta[5] << " " << theta[6] << std::endl;

    return final_joint;
}


Eigen::VectorXd getLastLineasVector()
{
    Eigen::VectorXd start(6);

    std::ifstream file("result1.txt");
    std::string line = getLastLine(file);
    std::cout << line << std::endl;
    file.close();

    std::string delimiter = " ";

    size_t pos = 0;
    std::string token;
    int i = 0;
    int arsize = 6;
    float linear[arsize];
    while ((pos = line.find(delimiter)) != std::string::npos && i < arsize) {
        token = line.substr(0, pos);
        linear[i] = std::stof(token);
        line.erase(0, pos + delimiter.length());
        i++;
    }

    start << linear[0] , linear[1] , linear[2] , linear[3] , linear[4] , linear[5] ;
    std::cout << start << std::endl;
    return start;
}       

void detachAttachStrings(Eigen::VectorXd strings, const ds::WorldPtr &world)
{
    
}

bool detachAttachCheck(int i, Eigen::VectorXd strings, const ds::WorldPtr &world)
{
   
    detachAttachStrings(strings, world); 
    Eigen::VectorXd zero_joints(6); 
    zero_joints << 0,0,0,0,0,0;
/*
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");

    Eigen::Isometry3d det_transform = tensegrity->getBodyNode("detach...")->getTransform();
    Eigen::Quaterniond det_quat(det_transform.rotation());
    Eigen::Vector3d det_tr = det_transform.translation();
    
    Eigen::Isometry3d at_transform = tensegrity->getBodyNode("attach...")->getTransform();
    Eigen::Quaterniond at_quat(at_transform.rotation());
    Eigen::Vector3d at_tr = at_transform.translation();
    // rotate stuff 

    Eigen::VectorXd det_joints(6);
    det_joints = joints(zero_joints, det_transform, world);

    if (det_joints == zero_joints)
    {
        return false;
    }

    Eigen::VectorXd at_joints(6);
    at_joints = joints(zero_joints, at_transform, world);

    if (at_joints == zero_joints)
    {
        return false;
    }
*/
    return true;    
}

void attachDetachCheckWithStrings(const ds::WorldPtr &world)
{
    std::ofstream sequenceFile;
    sequenceFile.open("sequence.txt", std::ios::trunc);
    Eigen::VectorXd strings(9);
    strings << 1, 1, 1, 1, 1, 1, 1, 1, 1;

    for(int st1 = 0; st1 < 9; st1++)
    {
        strings(st1) = 0;
        if (!detachAttachCheck(st1, strings, world))
        {
            strings(st1) = 1;
            break; 
        }
        for(int st2 = 0; st2 < 9; st2++)
        {
            if(strings(st2) == 0) 
            {
                continue;
            }
            strings(st2) = 0;
            if (!detachAttachCheck(st2, strings, world))
            {
                strings(st2) = 1;
                break; 
            }
            
            for(int st3 = 0; st3 < 9; st3++)
            {
                if(strings(st3) == 0) 
                {
                    continue;
                }
                strings(st3) = 0;
                if (!detachAttachCheck(st3, strings, world))
                {
                    strings(st3) = 1;
                    break; 
                }
            
                for(int st4 = 0; st4 < 9; st4++)
                {
                    if(strings(st4) == 0) 
                    {
                        continue;
                    }
                    strings(st4) = 0;
                    if (!detachAttachCheck(st4, strings, world))
                    {
                        strings(st4) = 1;
                        break; 
                    }
                    for(int st5 = 0; st5 < 9; st5++)
                    {
                        if(strings(st5) == 0) 
                        {
                            continue;
                        }
                        strings(st5) = 0;
                        if (!detachAttachCheck(st5, strings, world))
                        {
                            strings(st5) = 1;
                            break; 
                        }
                        for(int st6 = 0; st6 < 9; st6++)
                        {
                            if(strings(st6) == 0) 
                            {
                                continue;
                            }
                            strings(st6) = 0;
                            if (!detachAttachCheck(st6, strings, world))
                            {
                                strings(st6) = 1;
                                break; 
                            }
                            for(int st7 = 0; st7 < 9; st7++)
                            {
                                if(strings(st7) == 0) 
                                {
                                    continue;
                                }
                                strings(st7) = 0;
                                if (!detachAttachCheck(st7, strings, world))
                                {
                                    strings(st7) = 1;
                                    break; 
                                }
                                for(int st8 = 0; st8 < 9; st8++)
                                {
                                    if(strings(st8) == 0) 
                                    {
                                        continue;
                                    }
                                    strings(st8) = 0;
                                    if (!detachAttachCheck(st8, strings, world))
                                    {
                                        strings(st8) = 1;
                                        break; 
                                    }
                                    for(int st9 = 0; st9 < 9; st9++)
                                    {
                                        if(strings(st9) == 0) 
                                        {
                                            continue;
                                        }
                                        strings(st9) = 0;
                                        if (!detachAttachCheck(st9, strings, world))
                                        {
                                            strings(st9) = 1;
                                            break; 
                                        }
                                        sequenceFile << st9 << " " << st8 << " " << st7 << " " << st6 << " " << st5 << " " << st4 << " " << st3 << " " << st2 << " " << st1 <<std::endl;
                                        // srite to file in the reverse order of indeces
                                        
                                        strings(st9) = 1;
                                    }
                                    strings(st8) = 1;
                                }
                                strings(st7) = 1;
                            }
                            strings(st6) = 1;
                        }
                        strings(st5) = 1;
                    }
                    strings(st4) = 1;
                }
                strings(st3) = 1;
            }
            strings(st2) = 1;
        }
        strings(st1) = 1;
    }
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

    dd::SkeletonPtr tensegrity = du::SdfParser::readSkeleton(prefix + std::string("/tensegrity.sdf"));
    tensegrity->setName("tensegrity");


    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond tenRot;

    tenRot = Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitZ()) *Eigen::AngleAxisd(-45.0*M_PI/180.0, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(0.0*M_PI/180.0, Eigen::Vector3d::UnitX());

    tenMove.translation() << -0.75, -0.3, 1.4;
    tenMove.rotate(tenRot);
    moveSkeleton(tensegrity, tenMove);

    staubli->enableSelfCollision();

    /*
       dd::SkeletonPtr ball = dd::Skeleton::create("ball");
       Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

       tf.translation() = Eigen::Vector3d(-0.25, 0,1.75);
       createBall(ball, Eigen::Vector3d(0.15,0.15,0.15), tf);

       world->addSkeleton(ball);
     */
    world->addSkeleton(staubli);
    world->addSkeleton(tensegrity);

    //    staubli->getDof(3)->setPosition(290 * M_PI / 180.0); 
    //    staubli->getDof(4)->setPosition(290 * M_PI / 180.0); 
    ////    staubli->getDof(6)->setPosition(290 * M_PI / 180.0); 

    Eigen::VectorXd start(6);
    start << 0,0,0,0,0,0;

    Eigen::VectorXd finish(6);
    //finish << 0, -67, -144, 0, 0, 0;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    //tf.translation() = Eigen::Vector3d(685.32, 392.71, 592.72);

    double qw, qx, qy, qz = 0;
    double x, y, z = 0;

    std::ifstream initfile("init.txt");
    initfile >> x >> y >> z >> qw >> qx >> qy >> qz;

    /*qw = -0.171711;
      qx = 0.834576;
      qy = -0.402042;
      qz = 0.335203;
     */
    double len = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    std::cout << x <<" " << y << " " << z << " " << qw << " " << qx << " " << qy << " " << qz << std::endl;
    tf.rotate(Eigen::Quaterniond(qw/len, qx/len, qy/len, qz/len));
    tf.translation() = Eigen::Vector3d(x*1000, y*1000, z*1000-1278);

    //tf.rotate(Eigen::AngleAxisd(-167.76 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(9.18 * M_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(12.24 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
    //tf.rotate(Eigen::AngleAxisd(-180 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.001 * M_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.005 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));

    finish = joints (start, tf, world) ;
   /* 
       Eigen::VectorXd zero_joints(6);
       zero_joints << 0,0,0,0,0,0;
       for(int xx = 0; xx <= 1000; xx += 50)
       {
       for(int yy = -550; yy <= 550; yy += 50)
       {
       for(int aa_zz = -180; aa_zz < 180; aa_zz += 5)
       {

       tenMove = Eigen::Isometry3d::Identity();
       Eigen::Quaterniond tenRot;

       tenRot = Eigen::AngleAxisd(aa_zz*M_PI/180.0, Eigen::Vector3d::UnitZ());

       tenMove.rotate(tenRot);
       tenMove.translation() << xx, yy, 1.4;
       moveSkeleton(tensegrity, tenMove);

       if(!world->checkCollision())
       {
       int ii = 0;
       while(ii < 9)
       {

            if (!detachAttachCheck(ii, strings, world))
            {
                break;
            }

            ii++;
        }

    if (ii == 9)
    {
        // xx, yy, aa_zz -> save
    }
     */
    attachDetachCheckWithStrings(world);
    if (argc < 2) {

        std::ofstream resultfile;
        resultfile.open("result1.txt", std::ios::trunc);
        resultfile.close();

        std::ofstream endeffectorfile;
        endeffectorfile.open("endeffector1.txt", std::ios::trunc);
        endeffectorfile.close();

        //start = getLastLineasVector();
        Simple3DEnvironment env(3);
        env.setWorld(world);

        if(env.plan(start * 180.0 / M_PI,finish))
        {
            env.recordSolution(start);
        }
        /*
           start = getLastLineasVector();
           Simple3DEnvironment env1(6);
           env1.setWorld(world);
           if(env1.plan(start*180.0/M_PI,finish))
           {
           env1.recordSolution(start);
           }
         */       
    }
    for (int jj = 2; jj <=7; jj++){
        staubli->getDof(jj)->setPosition(0); 
    }
    MyWindow window(world);
    double j1, j2, j3, j4, j5, j6 = 0;
    //staubli->getBodyNode("table")->getVisualizationShape(0)->setHidden(true);
    //staubli->getBodyNode("table")->setCollidable(false);
    
    
    staubli->getBodyNode("gripper")->getVisualizationShape(0)->setColor(Eigen::Vector3d(0,1,0));
    std::thread t([&]()
            {
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            //while (true) 
            //{
            std::ifstream fin("result1.txt");

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
    window.initWindow(475 * 2, 300 * 2, "SDF");

    glutMainLoop();

    t.join();

    return 0;
}
