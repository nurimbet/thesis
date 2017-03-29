#include <Eigen/Eigen>
#include <dart/dart.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "config.h"
#include "mywindow.h"
#include "ompl/base/spaces/WeightedRealVectorStateSpace.h"
#include "ompl/geometric/planners/rrt/RRTstarQ.h"
#include "util.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ds = dart::simulation;
namespace dd = dart::dynamics;
namespace dc = dart::collision;
namespace du = dart::utils;

constexpr double jointMax[6] = { 3.1416, 2.5744, 2.5307, 4.7124, 2.4435, 4.7124 };
constexpr double jointMin[6] = { -3.1416, -2.2689, -2.5307, -4.7124, -2.0071, -4.7124 };

std::string resultFName = "result.txt";
std::string endeffectorFName = "endeffector.txt";
std::string edgesFName = "edges.txt";
std::string feasibleLocFName = "feasibleLocation.txt";
std::string plannableFName = "plannable.txt";
std::string sequenceFName = "sequence.txt";
std::string tightenerFName = "tightener.txt";
std::string tendonFName = "plannable.txt";

ds::WorldPtr world = std::make_shared<ds::World>();

class tensegrityEnvironment {
public:
    tensegrityEnvironment(std::size_t jointNum, std::size_t tendonNum, bool withStringConstraint)
    {
        jointNumber = jointNum;
        tendonNumber = tendonNum;
        withString = withStringConstraint;
        //space = new ob::WeightedRealVectorStateSpace();
        space = new ob::RealVectorStateSpace();
        space->addDimension(jointMin[0], jointMax[0]);
        space->addDimension(jointMin[1], jointMax[1]);
        space->addDimension(jointMin[2], jointMax[2]);

        if (jointNumber > 3) {
            space->addDimension(jointMin[3], jointMax[3]);
            space->addDimension(jointMin[4], jointMax[4]);
            space->addDimension(jointMin[5], jointMax[5]);
        }
        ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

        // set state validity checking for this space
        ss_->setStateValidityChecker(std::bind(
            &tensegrityEnvironment::isStateValid, this, std::placeholders::_1));
        space->setup();
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        ss_->setPlanner(
            ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));
    }

    bool plan(const Eigen::VectorXd& init, const Eigen::VectorXd& final)
    {
        if (!ss_)
            return false;

        ob::ScopedState<> start(ss_->getStateSpace());
        for (std::size_t i = 0; i < jointNumber; ++i)
            start[i] = init[i] * M_PI / 180.0;

        ob::ScopedState<> goal(ss_->getStateSpace());
        for (std::size_t i = 0; i < jointNumber; ++i)
            goal[i] = final[i] * M_PI / 180.0;

        ss_->setStartAndGoalStates(start, goal, 0.05);

        if (jointNumber > 3) {
            ss_->solve(60 * 1 * 1);
        } else {
            ss_->solve(60 * 1 * 1);
        }

        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()) {
            ss_->simplifySolution();
            og::PathGeometric& p = ss_->getSolutionPath();
            ss_->getPathSimplifier()->simplifyMax(p);
            ss_->getPathSimplifier()->smoothBSpline(p);
            return true;
        } else
            return false;
    }

    void recordSolution(const Eigen::VectorXd& start)
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric& p = ss_->getSolutionPath();
        p.interpolate(1000);
        std::ofstream resultfile;
        resultfile.open(resultFName, std::ios::app);

        std::ofstream endeffectorfile;
        endeffectorfile.open(endeffectorFName, std::ios::app);

        for (std::size_t i = 0; i < p.getStateCount(); ++i) {
            const double j1 = (double)p.getState(i)
                                  ->as<ob::RealVectorStateSpace::StateType>()
                                  ->values[0];
            const double j2 = (double)p.getState(i)
                                  ->as<ob::RealVectorStateSpace::StateType>()
                                  ->values[1];
            const double j3 = (double)p.getState(i)
                                  ->as<ob::RealVectorStateSpace::StateType>()
                                  ->values[2];
            double j4 = start[3] * M_PI / 180.0;
            double j5 = start[4] * M_PI / 180.0;
            double j6 = start[5] * M_PI / 180.0;
            dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

            staubli->getDof(2)->setPosition(j1);
            staubli->getDof(3)->setPosition(j2);
            staubli->getDof(4)->setPosition(j3);

            if (jointNumber > 3) {
                j4 = (double)p.getState(i)
                         ->as<ob::RealVectorStateSpace::StateType>()
                         ->values[3];
                j5 = (double)p.getState(i)
                         ->as<ob::RealVectorStateSpace::StateType>()
                         ->values[4];
                j6 = (double)p.getState(i)
                         ->as<ob::RealVectorStateSpace::StateType>()
                         ->values[5];
                staubli->getDof(5)->setPosition(j4);
                staubli->getDof(6)->setPosition(j5);
                staubli->getDof(7)->setPosition(j6);
            }

            Eigen::Isometry3d gripperTransform = staubli->getBodyNode("gripper")->getTransform();
            Eigen::Vector3d gripperTrans = gripperTransform.translation();
            Eigen::Matrix3d gripperRot = gripperTransform.rotation();
            Eigen::Quaterniond quat(gripperTransform.rotation());

            double xs = 30.0 / 1000.0;
            double ys = -60.0 / 1000.0;
            double zs = 95.0 / 1000.0;

            gripperTrans(0) += xs * gripperRot(0, 0) + ys * gripperRot(0, 1) + zs * gripperRot(0, 2);
            gripperTrans(1) += xs * gripperRot(1, 0) + ys * gripperRot(1, 1) + zs * gripperRot(1, 2);
            gripperTrans(2) += xs * gripperRot(2, 0) + ys * gripperRot(2, 1) + zs * gripperRot(2, 2);

            resultfile << j1 << " " << j2 << " " << j3 << " " << j4 << " " << j5
                       << " " << j6 << std::endl;

            endeffectorfile << gripperTrans(0) << " " << gripperTrans(1) << " " << gripperTrans(2) << " "
                            << quat.w() << " " << quat.x() << " " << quat.y()
                            << " " << quat.z() << std::endl;
        }
        resultfile.close();
        endeffectorfile.close();

        ob::PlannerData pdat(ss_->getSpaceInformation());
        ss_->getPlannerData(pdat);
        std::ofstream ofs_e(edgesFName);
        std::vector<unsigned int> edge_list;
        std::vector<double> reals;
        std::vector<double> realsOld;
        //ob::State* s3 = space->allocState();
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            const ob::State* s1 = pdat.getVertex(i).getState();
            //isMajorTree = pdat.getVertex(i).getTag();
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                const ob::State* s2 = pdat.getVertex(edge_list[i2]).getState();
                /*
                double step = 0.05;
                if (space->distance(s1, s2) < 0.03) {
                    step = 0.2;
                }
                */
                space->copyToReals(realsOld, s1);
                dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

                staubli->getDof(2)->setPosition(realsOld[0]);
                staubli->getDof(3)->setPosition(realsOld[1]);
                staubli->getDof(4)->setPosition(realsOld[2]);

                Eigen::Isometry3d gripperTransform = staubli->getBodyNode("gripper")->getTransform();
                Eigen::Vector3d tr = gripperTransform.translation();

                ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;
                /*
                for (double t = step; t <= 1.01; t += step) {
                    space->interpolate(s1, s2, t, s3);
                    space->copyToReals(reals, s3);

                    //for (const auto& r : realsOld)

                    //    ofs_e << r << " ";
                    //

                    staubli->getDof(2)->setPosition(reals[0]);
                    staubli->getDof(3)->setPosition(reals[1]);
                    staubli->getDof(4)->setPosition(reals[2]);

                    transform = staubli->getBodyNode("toolflange_link")->getTransform();
                    tr = transform.translation();

                    ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;
                    ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;

                }
*/
                space->copyToReals(reals, s2);

                staubli->getDof(2)->setPosition(reals[0]);
                staubli->getDof(3)->setPosition(reals[1]);
                staubli->getDof(4)->setPosition(reals[2]);

                gripperTransform = staubli->getBodyNode("gripper")->getTransform();
                tr = gripperTransform.translation();

                ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;
            }
        }
    }
    double getMinDistTreeAttach(const Eigen::Vector3d& finish, const Eigen::VectorXd& attach)
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return -1;

        ob::PlannerData pdat(ss_->getSpaceInformation());
        ss_->getPlannerData(pdat);

        std::vector<unsigned int> edge_list;
        std::vector<double> reals;
        std::vector<double> realsOld;

        double distMin = std::numeric_limits<float>::max();
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                const ob::State* s2 = pdat.getVertex(edge_list[i2]).getState();

                dd::SkeletonPtr staubli = world_->getSkeleton("staubli");
                space->copyToReals(reals, s2);

                staubli->getDof(2)->setPosition(reals[0]);
                staubli->getDof(3)->setPosition(reals[1]);
                staubli->getDof(4)->setPosition(reals[2]);
                staubli->getDof(5)->setPosition(attach[3]);
                staubli->getDof(6)->setPosition(attach[4]);
                staubli->getDof(7)->setPosition(attach[5]);

                Eigen::Isometry3d gripperTransform = staubli->getBodyNode("gripper")->getTransform();

                Eigen::Vector3d tr = gripperTransform.translation();

                Eigen::Matrix3d gripperRot = gripperTransform.rotation();

                double xs = 30.0 / 1000.0;
                double ys = -60.0 / 1000.0;
                double zs = 95.0 / 1000.0;

                tr(0) += xs * gripperRot(0, 0) + ys * gripperRot(0, 1) + zs * gripperRot(0, 2);
                tr(1) += xs * gripperRot(1, 0) + ys * gripperRot(1, 1) + zs * gripperRot(1, 2);
                tr(2) += xs * gripperRot(2, 0) + ys * gripperRot(2, 1) + zs * gripperRot(2, 2);

                double curMin = sqrt(pow(tr(0) - finish(0) / 1000, 2) + pow(tr(1) - finish(1) / 1000, 2) + pow(tr(2) - finish(2) / 1000, 2));

                if (curMin < distMin) {
                    distMin = curMin;
                }
            }
        }
        return distMin;
    }

    void setWorld(const ds::WorldPtr& world) { world_ = world; }

    bool isWithinReach() const
    {

        dd::SkeletonPtr staubli = world_->getSkeleton("staubli");
        dd::SkeletonPtr tensegrity = world_->getSkeleton("tensegrity");

        Eigen::Isometry3d attachTransform = tensegrity->getBodyNode("attach" + std::to_string(tendonNumber + 1))->getTransform();
        Eigen::Vector3d attachTrans = attachTransform.translation();

        Eigen::Isometry3d pulleyTransform = tensegrity->getBodyNode("pulley" + std::to_string(tendonNumber + 1))->getTransform();
        Eigen::Vector3d pulleyTrans = pulleyTransform.translation();

        Eigen::Isometry3d gripperTransform = staubli->getBodyNode("gripper")->getTransform();
        Eigen::Vector3d gripperTrans = gripperTransform.translation();
        Eigen::Matrix3d gripperRot = gripperTransform.rotation();

        double xs = 30.0 / 1000.0;
        double ys = -60.0 / 1000.0;
        double zs = 95.0 / 1000.0;

        gripperTrans(0) += xs * gripperRot(0, 0) + ys * gripperRot(0, 1) + zs * gripperRot(0, 2);
        gripperTrans(1) += xs * gripperRot(1, 0) + ys * gripperRot(1, 1) + zs * gripperRot(1, 2);
        gripperTrans(2) += xs * gripperRot(2, 0) + ys * gripperRot(2, 1) + zs * gripperRot(2, 2);

        //std::cout << (gripperTrans - pulleyTrans).squaredNorm() << std::endl;
        //std::cout << ((pulleyTrans - attachTrans).squaredNorm() + 0.05)<< std::endl;
        if ((gripperTrans - pulleyTrans).squaredNorm() > ((pulleyTrans - attachTrans).squaredNorm() + 0.05)) {
            //   std::cout << "dafaq?" << std::endl;
            return false;
        }

        return true;
    }

private:
    bool isStateValid(const ob::State* state) const
    {
        double j1 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
        double j2 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
        double j3 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
        double j4 = 0;
        double j5 = 0;
        double j6 = 0;

        dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

        staubli->getDof(2)->setPosition(j1);
        staubli->getDof(3)->setPosition(j2);
        staubli->getDof(4)->setPosition(j3);

        if (jointNumber > 3) {
            j4 = state->as<ompl::base::RealVectorStateSpace::StateType>()
                     ->values[3];
            j5 = state->as<ompl::base::RealVectorStateSpace::StateType>()
                     ->values[4];
            j6 = state->as<ompl::base::RealVectorStateSpace::StateType>()
                     ->values[5];

            staubli->getDof(5)->setPosition(j4);
            staubli->getDof(6)->setPosition(j5);
            staubli->getDof(7)->setPosition(j6);
        }

        if (withString) {
            return isWithinReach() && !world_->checkCollision();
        } else {
            return !world_->checkCollision();
        }
    }

    og::SimpleSetupPtr ss_;
    ds::WorldPtr world_;
    ob::RealVectorStateSpace* space;
    std::size_t jointNumber;
    std::size_t tendonNumber;
    bool withString;
};

std::istream& ignoreLine(std::ifstream& in, std::ifstream::pos_type& pos)
{
    pos = in.tellg();
    return in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

std::string getLastLine(std::ifstream& in)
{
    std::ifstream::pos_type pos = in.tellg();

    std::ifstream::pos_type lastPos;
    while (in >> std::ws && ignoreLine(in, lastPos))
        pos = lastPos;

    in.clear();
    in.seekg(pos);

    std::string line;
    std::getline(in, line);
    return line;
}

std::string getWorkingDirectory()
{
    char buff[1024] = { 0 };

    getcwd(buff, 1024);

    return std::string(buff);
}

Eigen::VectorXd getLastLineAsVector()
{
    Eigen::VectorXd start(6);

    std::ifstream file(resultFName);
    std::string line = getLastLine(file);

    file.close();

    std::string delimiter = " ";

    size_t pos = 0;
    std::string token;
    int i = 0;
    int arsize = 6;
    float linear[arsize];
    while (i < arsize) {
        pos = line.find(delimiter);
        token = line.substr(0, pos);
        linear[i] = std::stof(token);
        line.erase(0, pos + delimiter.length());
        i++;
    }

    start << linear[0], linear[1], linear[2], linear[3], linear[4], linear[5];
    return start;
}

std::vector<Eigen::VectorXd> getInverseKinematics(
    const Eigen::Isometry3d& final_point)
{
    using Eigen::VectorXd;
    using std::vector;
    //std::cout << final_point.translation();

    vector<VectorXd> final_joints;
    VectorXd final_joint(6);
    final_joint << 0, 0, 0, 0, 0, 0;
    dd::SkeletonPtr staubli = world->getSkeleton("staubli");

    double alpha[7] = { 0, -M_PI / 2, 0, M_PI / 2, -M_PI / 2, M_PI / 2, 0 };
    double a[7] = { 0, 50, 650, 0, 0, 0, 0 };
    double d[7] = { 0, 0, 0, 50, 650, 0, 100 };
    double theta[7] = { 0, 0, 0, 0, 0, 0, 0 };

    Eigen::Matrix3d rot_final_point = final_point.rotation();
    Eigen::Vector3d trans_final_point = final_point.translation();

    double a_x = rot_final_point(0, 2);
    double a_y = rot_final_point(1, 2);
    double a_z = rot_final_point(2, 2);

    double p_ax = trans_final_point(0) - d[6] * a_x;
    double p_ay = trans_final_point(1) - d[6] * a_y;
    double p_az = trans_final_point(2) - d[6] * a_z;

    for (int i = 0; i < 8; i++) {
        theta[1] = atan2(p_ay, p_ax) - atan2(d[3], pow(-1, (i & 2) >> 1) * sqrt(p_ax * p_ax + p_ay * p_ay - d[3]));

        double t_1 = (pow(p_ax * cos(theta[1]) + p_ay * sin(theta[1]) - a[1], 2) + a[2] * a[2] + p_az * p_az - d[4] * d[4]) / (2 * a[2]);
        theta[2] = atan2((p_ax * cos(theta[1]) + p_ay * sin(theta[1]) - a[1]), p_az) - atan2(t_1, pow(-1, (i & 4) >> 2) * sqrt(pow(p_ax * cos(theta[1]) + p_ay * sin(theta[1]) - a[1],
                                                                                                                                   2)
                                                                                                                              + p_az * p_az - t_1 * t_1)); // + M_PI / 2;

        theta[3] = atan2(p_ax * cos(theta[1]) + p_ay * sin(theta[1]) - a[1] - a[2] * cos(theta[2]),
                       p_az + a[2] * sin(theta[2]))
            - theta[2]; // - M_PI / 2;

        Eigen::Matrix3d rot_R3_0;
        rot_R3_0 = Eigen::Quaterniond(
            (Eigen::AngleAxisd(theta[1], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(alpha[1], Eigen::Vector3d::UnitX())) * (Eigen::AngleAxisd(theta[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(alpha[2], Eigen::Vector3d::UnitX())) * (Eigen::AngleAxisd(theta[3], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(alpha[3], Eigen::Vector3d::UnitX())));
        rot_R3_0 = rot_R3_0.transpose() * rot_final_point;

        double a_wx = rot_R3_0(0, 2);
        double a_wy = rot_R3_0(1, 2);
        double a_wz = rot_R3_0(2, 2);
        double n_wz = rot_R3_0(2, 0);
        double o_wz = rot_R3_0(2, 1);

        theta[5] = atan2(pow(-1, (i & 1)) * sqrt(a_wx * a_wx + a_wy * a_wy), a_wz);

        theta[4] = atan2(a_wy / sin(theta[5]), a_wx / sin(theta[5]));

        theta[6] = ((i == 8) ? 1 : (1)) * atan2(o_wz / sin(theta[5]), -n_wz / sin(theta[5]));

        theta[2] = theta[2] + (M_PI / 2.0);
        theta[3] = theta[3] - (M_PI / 2.0);

        bool withinJointRange = true;
        for (int j = 1; j < 7; j++) {
            if (std::abs(theta[j]) > M_PI) {
                theta[j] = -1 * theta[j] / std::abs(theta[j]) * (2 * M_PI - std::abs(theta[j]));
            }
        }
        for (int j = 1; j < 7; j++) {
            if (theta[j] > jointMax[j - 1] || theta[j] < jointMin[j - 1]) {
                withinJointRange = false;
            }
        }
        //std::cout << theta[1]*180/M_PI <<" " << theta[2]*180/M_PI <<" " <<theta[3]*180/M_PI <<" " <<theta[4]*180/M_PI <<" " <<theta[5]*180/M_PI <<" " <<theta[6]*180/M_PI  << std::endl;

        if (!(std::isnan(theta[1])) && !(std::isnan(theta[2])) && !(std::isnan(theta[3])) && !(std::isnan(theta[4])) && !(std::isnan(theta[5])) && !(std::isnan(theta[6]))) {

            staubli->getDof(2)->setPosition(theta[1]);
            staubli->getDof(3)->setPosition(theta[2]);
            staubli->getDof(4)->setPosition(theta[3]);
            staubli->getDof(5)->setPosition(theta[4]);
            staubli->getDof(6)->setPosition(theta[5]);
            staubli->getDof(7)->setPosition(theta[6]);

            if (withinJointRange && !world->checkCollision()) {
                //std::cout << std::endl;
                final_joint << theta[1], theta[2], theta[3], theta[4], theta[5],
                    theta[6];
                final_joint = final_joint * 180.0 / M_PI;
                final_joints.push_back(final_joint);
            }
        }
    }

    /*    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision,
        Eigen::DontAlignCols, " ", " ", "", "", "","");
    std::cout << final_joint.format(CommaInitFmt) << std::endl;
*/
    return final_joints;
}

void detachAllStrings()
{
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    for (int ii = 1; ii <= 9; ii++) {
        tensegrity->getBodyNode("tendon" + std::to_string(ii))
            ->getVisualizationShape(0)
            ->setHidden(true);
        tensegrity->getBodyNode("tendon" + std::to_string(ii))
            ->setCollidable(false);
    }
}

void attachAllStrings()
{
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    for (int ii = 1; ii <= 9; ii++) {
        tensegrity->getBodyNode("tendon" + std::to_string(ii))
            ->getVisualizationShape(0)
            ->setHidden(false);
        tensegrity->getBodyNode("tendon" + std::to_string(ii))
            ->setCollidable(true);
    }
}

void detachAttachStrings(Eigen::VectorXd strings)
{
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    for (int ii = 0; ii < 9; ii++) {
        if (strings(ii) == 0) {
            tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
                ->getVisualizationShape(0)
                ->setHidden(true);
            tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
                ->setCollidable(false);
        } else {
            tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
                ->getVisualizationShape(0)
                ->setHidden(false);
            tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
                ->setCollidable(true);
        }
    }
}

void detachStringAt(int ii)
{
    dd::SkeletonPtr staubli = world->getSkeleton("staubli");
    staubli->getBodyNode("claws")
        ->setCollidable(true);
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
        ->getVisualizationShape(0)
        ->setHidden(true);
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
        ->setCollidable(false);
}

void attachStringAt(int ii)
{
    dd::SkeletonPtr staubli = world->getSkeleton("staubli");
    staubli->getBodyNode("claws")
        ->setCollidable(false);

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
        ->getVisualizationShape(0)
        ->setHidden(false);
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
        ->setCollidable(true);
}

Eigen::Isometry3d getAttachPosition(int attNum,
    bool wristUp)
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("attach" + std::to_string(attNum + 1))
                                                ->getTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();

    double xs = 0;
    double ys = 70;
    //double zs = -80;
    double zs = -95;
    Eigen::Matrix3d rot_ten;
    if (wristUp) {
        rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d::UnitX());
    } else {
        rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(180 * M_PI / 180.0, Eigen::Vector3d::UnitY());
        zs = -100;
    }
    tenTrans *= 1000;

    tenTrans(0) += xs * rot_ten(0, 0) + ys * rot_ten(0, 1) + zs * rot_ten(0, 2);
    tenTrans(1) += xs * rot_ten(1, 0) + ys * rot_ten(1, 1) + zs * rot_ten(1, 2);
    tenTrans(2) += xs * rot_ten(2, 0) + ys * rot_ten(2, 1) + zs * rot_ten(2, 2);
    tenTrans(2) -= 1278;

    tf.linear() = rot_ten;
    tf.translation() = tenTrans;
    return tf;
}

Eigen::Isometry3d getDetachPosition(int detNum,
    bool wristUp)
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("detach" + std::to_string(detNum + 1))->getTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();

    double xs = 0;
    double ys = 80;

    double zs = -95;
    Eigen::Matrix3d rot_ten;
    if (wristUp) {
        rot_ten = tensegrityTransform.rotation();
    } else {
        rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(180 * M_PI / 180.0, Eigen::Vector3d::UnitY());
        xs = -25;
    }
    tenTrans *= 1000;

    tenTrans(0) += xs * rot_ten(0, 0) + ys * rot_ten(0, 1) + zs * rot_ten(0, 2);
    tenTrans(1) += xs * rot_ten(1, 0) + ys * rot_ten(1, 1) + zs * rot_ten(1, 2);
    tenTrans(2) += xs * rot_ten(2, 0) + ys * rot_ten(2, 1) + zs * rot_ten(2, 2);
    tenTrans(2) -= 1278;

    tf.linear() = rot_ten;
    tf.translation() = tenTrans;
    return tf;
}

Eigen::Isometry3d getTightenerPoint(int detNum)
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("tightener" + std::to_string(detNum + 1))->getTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();

    double xs = -9;
    double ys = 4.5;
    double zs = -190;

    Eigen::Matrix3d rot_ten;

    rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(90 * M_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(90 * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    tenTrans *= 1000;

    tenTrans(0) += xs * rot_ten(0, 0) + ys * rot_ten(0, 1) + zs * rot_ten(0, 2);
    tenTrans(1) += xs * rot_ten(1, 0) + ys * rot_ten(1, 1) + zs * rot_ten(1, 2);
    tenTrans(2) += xs * rot_ten(2, 0) + ys * rot_ten(2, 1) + zs * rot_ten(2, 2);
    tenTrans(2) -= 1278;

    tf.linear() = rot_ten;
    tf.translation() = tenTrans;
    return tf;
}

Eigen::Isometry3d getTendonPoint(int detNum)
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("pulley" + std::to_string(detNum + 1))->getTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();

    Eigen::Isometry3d tensegrityTransform1 = tensegrity->getBodyNode("attach" + std::to_string(detNum + 1))->getTransform();
    Eigen::Vector3d diff = tensegrityTransform1.translation() - tenTrans;
    tenTrans = tenTrans + 0.2 * diff / diff.squaredNorm();

    tensegrityTransform = tensegrity->getBodyNode("tendon" + std::to_string(detNum + 1))->getTransform();
    Eigen::Matrix3d rot_ten;
    rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(90 * M_PI / 180.0, Eigen::Vector3d::UnitY());

    double xs = 0.0;
    double ys = -50.0;
    double zs = -75.0;

    tenTrans *= 1000;

    tenTrans(0) += xs * rot_ten(0, 0) + ys * rot_ten(0, 1) + zs * rot_ten(0, 2);
    tenTrans(1) += xs * rot_ten(1, 0) + ys * rot_ten(1, 1) + zs * rot_ten(1, 2);
    tenTrans(2) += xs * rot_ten(2, 0) + ys * rot_ten(2, 1) + zs * rot_ten(2, 2);
    tenTrans(2) -= 1278;

    tf.linear() = rot_ten;
    tf.translation() = tenTrans;
    return tf;
}

bool isReachable(int i, Eigen::VectorXd strings)
{
    detachAttachStrings(strings);

    Eigen::Isometry3d tf_det(Eigen::Isometry3d::Identity());
    std::vector<Eigen::VectorXd> det_joints;

    tf_det = getDetachPosition(i, false);
    det_joints = getInverseKinematics(tf_det);

    if (det_joints.size() == 0) {
        tf_det = getDetachPosition(i, true);
        det_joints = getInverseKinematics(tf_det);

        if (det_joints.size() == 0) {
            return false;
        }
    }

    attachStringAt(i);

    Eigen::Isometry3d tf_att(Eigen::Isometry3d::Identity());
    std::vector<Eigen::VectorXd> at_joints;

    tf_att = getAttachPosition(i, false);
    at_joints = getInverseKinematics(tf_att);

    if (at_joints.size() == 0) {
        tf_att = getAttachPosition(i, true);
        at_joints = getInverseKinematics(tf_att);

        if (at_joints.size() == 0) {
            return false;
        }
    }
    detachStringAt(i);

    return true;
}

void printFeasibleTensegrityLocation()
{
    std::ofstream feasibleLocation;
    feasibleLocation.open(feasibleLocFName, std::ios::trunc);
    //std::ofstream nonFeasIdx;
    //nonFeasIdx.open("nonFeas.txt", std::ios::trunc);

    Eigen::VectorXd strings(9);
    strings << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");
    dd::SkeletonPtr staubli = world->getSkeleton("staubli");

    Eigen::Matrix3d rot_ten;
    for (int xx = -600; xx <= 600; xx += 50) {
        for (int yy = -600; yy <= 600; yy += 50) {
            if ((xx >= -400 && xx <= 400) && (yy >= -350 && yy <= 350)) {
                continue;
            }
            for (int aa_zz = -180; aa_zz < 180; aa_zz += 5) {
                //    for (int zz = -500; zz <= 500; zz += 10) {
                int ii = 0;
                rot_ten = Eigen::AngleAxisd((double)aa_zz * M_PI / 180.0,
                    Eigen::Vector3d::UnitZ());
                tf.linear() = rot_ten;
                tf.translation() << (double)xx / 1000.0, (double)yy / 1000.0,
                    (double)0.0 / 1000.0;
                moveSkeleton(tensegrity, tf);

                attachAllStrings();
                for (int kk = 2; kk <= 7; kk++) {
                    staubli->getDof(kk)->setPosition(0);
                }

                if (world->checkCollision()) {
                    continue;
                }
                detachAllStrings();

                while (ii < 9) {
                    if (!isReachable(ii, strings)) {
                        break;
                    }
                    ii++;
                }
                if (ii == 9) {
                    //feasibleLocation << xx << " " << yy << " " << zz << " " << aa_zz
                    feasibleLocation << xx << " " << yy << " " << aa_zz
                                     << std::endl;
                }
                /*
                    else
                    {
                        nonFeasIdx << ii << std::endl;
                    }
*/

                //}
            }
        }
    }
    feasibleLocation.close();
    //    nonFeasIdx.close();
}

void printAttachmentSequence()
{
    std::ofstream sequenceFile;
    sequenceFile.open(sequenceFName, std::ios::trunc);
    Eigen::VectorXd strings(9);
    strings << 1, 1, 1, 1, 1, 1, 1, 1, 1;

    for (int st1 = 0; st1 < 9; st1++) {
        strings(st1) = 0;
        if (!isReachable(st1, strings)) {
            strings(st1) = 1;
            continue;
        }
        for (int st2 = 0; st2 < 9; st2++) {
            if (strings(st2) == 0) {
                continue;
            }
            strings(st2) = 0;
            if (!isReachable(st2, strings)) {
                strings(st2) = 1;
                continue;
            }

            for (int st3 = 0; st3 < 9; st3++) {
                if (strings(st3) == 0) {
                    continue;
                }
                strings(st3) = 0;
                if (!isReachable(st3, strings)) {
                    strings(st3) = 1;
                    continue;
                }

                for (int st4 = 0; st4 < 9; st4++) {
                    if (strings(st4) == 0) {
                        continue;
                    }
                    strings(st4) = 0;
                    if (!isReachable(st4, strings)) {
                        strings(st4) = 1;
                        continue;
                    }
                    for (int st5 = 0; st5 < 9; st5++) {
                        if (strings(st5) == 0) {
                            continue;
                        }
                        strings(st5) = 0;
                        if (!isReachable(st5, strings)) {
                            strings(st5) = 1;
                            continue;
                        }
                        for (int st6 = 0; st6 < 9; st6++) {
                            if (strings(st6) == 0) {
                                continue;
                            }
                            strings(st6) = 0;
                            if (!isReachable(st6, strings)) {
                                strings(st6) = 1;
                                continue;
                            }
                            for (int st7 = 0; st7 < 9; st7++) {
                                if (strings(st7) == 0) {
                                    continue;
                                }
                                strings(st7) = 0;
                                if (!isReachable(st7, strings)) {
                                    strings(st7) = 1;
                                    continue;
                                }
                                for (int st8 = 0; st8 < 9; st8++) {
                                    if (strings(st8) == 0) {
                                        continue;
                                    }
                                    strings(st8) = 0;
                                    if (!isReachable(st8, strings)) {
                                        strings(st8) = 1;
                                        continue;
                                    }
                                    for (int st9 = 0; st9 < 9; st9++) {
                                        if (strings(st9) == 0) {
                                            continue;
                                        }
                                        strings(st9) = 0;
                                        if (!isReachable(st9, strings)) {
                                            strings(st9) = 1;
                                            continue;
                                        }
                                        sequenceFile << st9 << " " << st8 << " "
                                                     << st7 << " " << st6 << " "
                                                     << st5 << " " << st4 << " "
                                                     << st3 << " " << st2 << " "
                                                     << st1 << std::endl;

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

void printVector(Eigen::VectorXd final_joint)
{
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision,
        Eigen::DontAlignCols, " ", " ", "", "", "", "");
    std::cout << final_joint.format(CommaInitFmt) << std::endl;
}

Eigen::VectorXd collisionlessFinal(const Eigen::VectorXd& start, const Eigen::VectorXd& finish)
{
    Eigen::VectorXd cFinal(6);

    dd::SkeletonPtr staubli = world->getSkeleton("staubli");

    staubli->getDof(2)->setPosition(finish[0] * M_PI / 180.0);
    staubli->getDof(3)->setPosition(finish[1] * M_PI / 180.0);
    staubli->getDof(4)->setPosition(finish[2] * M_PI / 180.0);
    staubli->getDof(5)->setPosition(start[3] * M_PI / 180.0);
    staubli->getDof(6)->setPosition(start[4] * M_PI / 180.0);
    staubli->getDof(7)->setPosition(start[5] * M_PI / 180.0);

    if (!world->checkCollision()) {
        for (int ii = 0; ii < 6; ii++) {
            cFinal[ii] = staubli->getDof(ii + 2)->getPosition() * 180 / M_PI;
        }

        return cFinal;
    }
    int i = 2;
    while (true) {
        int inc = 0;
        if (i % 2 == 0) {
            inc = i / 2;
        } else {
            inc = -i / 2;
        }

        staubli->getDof(4)->setPosition((finish[2] + inc) * M_PI / 180.0);
        if (!world->checkCollision()) {
            break;
        }
        staubli->getDof(3)->setPosition((finish[1] + inc) * M_PI / 180.0);
        if (!world->checkCollision()) {
            break;
        }
        staubli->getDof(2)->setPosition((finish[0] + inc) * M_PI / 180.0);
        if (!world->checkCollision()) {
            break;
        }
        i++;
    }
    for (int ii = 0; ii < 6; ii++) {
        cFinal[ii] = staubli->getDof(ii + 2)->getPosition() * 180 / M_PI;
    }
    return cFinal;
}

double isPlannable(int kk)
{
    /*
    int kkk = kk;
    if (kk == 2) {
        kkk = 3;
    }
*/
    std::vector<Eigen::VectorXd> part1;
    std::vector<Eigen::VectorXd> part2;
    std::vector<Eigen::VectorXd> fullDetach;
    std::vector<Eigen::VectorXd> fullAttach;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf = getDetachPosition(kk, false);
    part1 = getInverseKinematics(tf);

    tf = getDetachPosition(kk, true);
    part2 = getInverseKinematics(tf);

    fullDetach.reserve(part1.size() + part2.size());
    fullDetach.insert(fullDetach.end(), part1.begin(), part1.end());
    fullDetach.insert(fullDetach.end(), part2.begin(), part2.end());

    attachStringAt(kk);

    tf = getAttachPosition(kk, false);
    part1 = getInverseKinematics(tf);

    tf = getAttachPosition(kk, true);
    part2 = getInverseKinematics(tf);

    fullAttach.reserve(part1.size() + part2.size());
    fullAttach.insert(fullAttach.end(), part1.begin(), part1.end());
    fullAttach.insert(fullAttach.end(), part2.begin(), part2.end());

    detachStringAt(kk);

    int minDetIdx = 0;
    int minAttIdx = 0;
    double minJointDist = std::numeric_limits<float>::max();
    for (size_t dd = 0; dd < fullDetach.size(); dd++) {
        for (size_t aa = 0; aa < fullAttach.size(); aa++) {
            double jointDist = (fullDetach[dd] - fullAttach[aa]).squaredNorm();
            if (jointDist < minJointDist) {
                minDetIdx = dd;
                minAttIdx = aa;
                minJointDist = jointDist;
            }
        }
    }

    tensegrityEnvironment env_t(3, kk, true);
    env_t.setWorld(world);

    Eigen::VectorXd finish_trans(6);
    finish_trans = collisionlessFinal(fullDetach[minDetIdx], fullAttach[minAttIdx]);

    if (env_t.plan(fullDetach[minDetIdx], finish_trans)) {
        Eigen::VectorXd attachLoc = tf.translation();
        attachLoc[2] += 1278;
        double minDist = env_t.getMinDistTreeAttach(attachLoc, fullAttach[minAttIdx]);
        return minDist;
    }

    return -1;
}

void printTightenerFeasibility()
{
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");

    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d tenRot;

    std::vector<Eigen::VectorXd> part1;
    detachAllStrings();
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    std::ifstream feas(feasibleLocFName);
    double xx, yy, aa_zz;
    std::ofstream tightener;
    tightener.open(tightenerFName, std::ios::trunc);
    while (!feas.eof()) {
        feas >> xx >> yy >> aa_zz;
        tightener << "Location: " << xx << " " << yy << " " << aa_zz << std::endl;

        tenRot = Eigen::AngleAxisd(aa_zz * M_PI / 180.0, Eigen::Vector3d::UnitZ());
        tenMove.linear() = tenRot;
        tenMove.translation() << (double)xx / 1000.0, (double)yy / 1000.0, 0.0;
        moveSkeleton(tensegrity, tenMove);

        int count = 0;
        for (int ll = 0; ll < 9; ll++) {
            tf = getTightenerPoint(ll);
            part1 = getInverseKinematics(tf);

            std::cout << ll << std::endl;
            for (size_t mm = 0; mm < part1.size(); mm++) {
                printVector(part1[mm]);
            }
            if (part1.size() > 0) {
                count += 1;
            }
        }
        tightener << count << std::endl;
    }
    tightener.close();
    feas.close();
}

void printTendonFeasibility()
{
    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");

    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d tenRot;

    std::vector<Eigen::VectorXd> part1;
    detachAllStrings();
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    //std::ifstream feas(feasibleLocFName);
    double xx, yy, aa_zz;
    std::ofstream tendon;
    tendon.open(tendonFName, std::ios::trunc);
    //while (!feas.eof()) {
    //    feas >> xx >> yy >> aa_zz;
    xx = 0.0;
    yy = -600.0;
    aa_zz = 45.0;
    tendon << "Location: " << xx << " " << yy << " " << aa_zz << std::endl;

    tenRot = Eigen::AngleAxisd(aa_zz * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    tenMove.linear() = tenRot;
    tenMove.translation() << (double)xx / 1000.0, (double)yy / 1000.0, 0.0;
    moveSkeleton(tensegrity, tenMove);

    int count = 0;
    for (int ll = 0; ll < 9; ll++) {
        tf = getTendonPoint(ll);
        part1 = getInverseKinematics(tf);

        std::cout << ll << std::endl;
        for (size_t mm = 0; mm < part1.size(); mm++) {
            printVector(part1[mm]);
        }
        if (part1.size() > 0) {
            count += 1;
        }
    }
    tendon << count << std::endl;
    //}
    tendon.close();
    //feas.close();
}

void printPlannable()
{

    dd::SkeletonPtr tensegrity = world->getSkeleton("tensegrity");

    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d tenRot;

    int kk = 0;
    std::vector<Eigen::VectorXd> part1;
    std::vector<Eigen::VectorXd> part2;
    std::vector<Eigen::VectorXd> fullDetach;
    std::vector<Eigen::VectorXd> fullAttach;

    std::ofstream plannable;
    plannable.open(plannableFName, std::ios::trunc);

    std::ifstream feas(feasibleLocFName);
    double xx, yy, aa_zz;
    double minDist;
    while (!feas.eof()) {
        feas >> xx >> yy >> aa_zz;
        plannable << "Location: " << xx << " " << yy << " " << aa_zz << std::endl;

        tenRot = Eigen::AngleAxisd(aa_zz * M_PI / 180.0, Eigen::Vector3d::UnitZ());
        tenMove.linear() = tenRot;
        tenMove.translation() << (double)xx / 1000.0, (double)yy / 1000.0, 0.0;
        moveSkeleton(tensegrity, tenMove);

        detachAllStrings();

        for (kk = 0; kk < 9; kk++) {
            minDist = isPlannable(kk);
            plannable << "kk = " << kk << " minimum distance = " << minDist << std::endl;
            if (minDist > 0.4) {
                break;
            }
        }
        plannable << "********************" << std::endl;
    }

    plannable.close();
}

void planAttachDirect(int kk)
{
    std::vector<Eigen::VectorXd> part1;
    std::vector<Eigen::VectorXd> part2;
    std::vector<Eigen::VectorXd> fullDetach;
    std::vector<Eigen::VectorXd> fullAttach;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf = getDetachPosition(kk, false);
    part1 = getInverseKinematics(tf);

    tf = getDetachPosition(kk, true);
    part2 = getInverseKinematics(tf);

    fullDetach.reserve(part1.size() + part2.size());
    fullDetach.insert(fullDetach.end(), part1.begin(), part1.end());
    fullDetach.insert(fullDetach.end(), part2.begin(), part2.end());

    attachStringAt(kk);

    tf = getAttachPosition(kk, false);
    part1 = getInverseKinematics(tf);

    tf = getAttachPosition(kk, true);
    part2 = getInverseKinematics(tf);

    fullAttach.reserve(part1.size() + part2.size());
    fullAttach.insert(fullAttach.end(), part1.begin(), part1.end());
    fullAttach.insert(fullAttach.end(), part2.begin(), part2.end());

    detachStringAt(kk);

    int minDetIdx = 0;
    int minAttIdx = 0;
    double minJointDist = std::numeric_limits<float>::max();
    for (size_t dd = 0; dd < fullDetach.size(); dd++) {
        for (size_t aa = 0; aa < fullAttach.size(); aa++) {
            double jointDist = (fullDetach[dd] - fullAttach[aa]).squaredNorm();
            if (jointDist < minJointDist) {
                minDetIdx = dd;
                minAttIdx = aa;
                minJointDist = jointDist;
            }
        }
    }

    Eigen::VectorXd start(6);
    Eigen::VectorXd finish(6);
    start = fullDetach[minDetIdx];
    finish = fullAttach[minAttIdx];

    std::ofstream resultfile;
    resultfile.open(resultFName, std::ios::trunc);
    resultfile.close();

    std::ofstream endeffectorfile;
    endeffectorfile.open(endeffectorFName, std::ios::trunc);
    endeffectorfile.close();

    tensegrityEnvironment env(3, kk, true);
    env.setWorld(world);

    Eigen::VectorXd finish_trans(6);
    finish_trans = collisionlessFinal(start, finish);

    std::cout << finish_trans << std::endl;

    if (env.plan(start, finish_trans)) {
        env.recordSolution(start);
    }

    start = getLastLineAsVector();
    tensegrityEnvironment env1(6, kk, true);
    env1.setWorld(world);
    if (env1.plan(start * 180.0 / M_PI, finish)) {
        env1.recordSolution(start * 180.0 / M_PI);
    }
}

void setUpStaubli()
{
    std::string prefix = getWorkingDirectory();

    dd::SkeletonPtr staubli = du::SdfParser::readSkeleton(prefix + std::string("/data/model.sdf"));
    staubli->setName("staubli");

    world->addSkeleton(staubli);

    staubli->getBodyNode("gripper")->getVisualizationShape(0)->setColor(Eigen::Vector3d(0, 1.0, 0));
    staubli->getBodyNode("table")->getVisualizationShape(0)->setColor(Eigen::Vector3d(0.6, 0.6, 0.6));
    
}

void setUpTensegrity()
{
    std::string prefix = getWorkingDirectory();

    du::DartLoader dl;

    dl.addPackageDirectory("tensegrity", prefix + std::string("/data/tensegrity/"));
    dd::SkeletonPtr tensegrity = dl.parseSkeleton(prefix + std::string("/data/tensegrity/robots/tensegrity.URDF"));
    tensegrity->setName("tensegrity");

    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d tenRot;

    tenRot = Eigen::AngleAxisd(80.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    tenMove.translation() << -0.6, -0.5, 0.0;
    tenMove.rotate(tenRot);
    moveSkeleton(tensegrity, tenMove);

    world->addSkeleton(tensegrity);
}

int main(int argc, char* argv[])
{
    world->getConstraintSolver()->setCollisionDetector(
        new dc::FCLCollisionDetector());

    setUpStaubli(); 
    setUpTensegrity();
    
    detachAllStrings(); 

    //printFeasibleTensegrityLocation();
    //printTightenerFeasibility();
    //printTendonFeasibility();
    //printPlannable();

    if (argc < 2) {
        int kk = 8;
        planAttachDirect(kk);
    }

    dd::SkeletonPtr staubli = world->getSkeleton("staubli");
    for (int jj = 2; jj <= 7; jj++) {
        staubli->getDof(jj)->setPosition(0);
    }

    MyWindow window(world);


    double jk1, jk2, jk3, jk4, jk5, jk6 = 0;
    std::thread t([&]() {
        while (true) {
            if (window.replay) {
                std::ifstream fin(resultFName);

                while (!fin.eof()) {
                    fin >> jk1 >> jk2 >> jk3 >> jk4 >> jk5 >> jk6;

                    staubli->getDof(2)->setPosition(jk1);
                    staubli->getDof(3)->setPosition(jk2);
                    staubli->getDof(4)->setPosition(jk3);
                    staubli->getDof(5)->setPosition(jk4);
                    staubli->getDof(6)->setPosition(jk5);
                    staubli->getDof(7)->setPosition(jk6);

                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    while (!window.replay) {
                    }
                }
                window.replay = false;
            }
        }
    });

    glutInit(&argc, argv);
    window.initWindow(600 * 2, 500 * 2, "SDF");

    glutMainLoop();
    t.join();

    return 0;
}
