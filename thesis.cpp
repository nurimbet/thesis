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

std::string resultFName = "data/results/paths/path";
std::string endeffectorFName = "data/results/endeffectors/endeffector";
std::string edgesFName = "data/results/edges.txt";
std::string feasibleLocFName = "data/results/feasibleLocation";
std::string plannableFName = "data/results/plannable";
std::string sequenceFName = "data/results/sequence.txt";
std::string tightenerFName = "data/results/tightener.txt";
std::string tendonFName = "data/results/tendon.txt";
std::string robotName = "staubli";
std::string tensegrityName = "tensegrity";
int seqArray[9] = { 5, 9, 3, 4, 6, 1, 2, 8, 7 };
int glob_ii = 0;
Eigen::VectorXd lastFinish(6);

ds::WorldPtr world = std::make_shared<ds::World>();

class tensegrityEnvironment {
public:
    tensegrityEnvironment(std::size_t jointNum, std::size_t tendonNum, bool withStringConstraint)
    {
        jointNumber = jointNum;
        tendonNumber = tendonNum;
        withString = withStringConstraint;
        /*
        dd::SkeletonPtr robot = world->getSkeleton(robotName);

        space = new ob::WeightedRealVectorStateSpace();

        static_cast<ob::WeightedRealVectorStateSpace*>(space)->setSkeleton(robot);
*/
        space = new ob::RealVectorStateSpace();

        for (size_t ii = 0; ii < jointNumber; ++ii) {
            space->addDimension(jointMin[ii], jointMax[ii]);
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
            ss_->solve(1 * 1 * 1);
        } else {
            ss_->solve(1 * 1 * 1);
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

    void recordSolution(const Eigen::VectorXd& start, int fileSequence)
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric& p = ss_->getSolutionPath();
        p.interpolate(1000);
        std::ofstream resultfile;
        resultfile.open(resultFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(0), std::ios::app);
        std::ofstream resultfile_sequence;
        resultfile_sequence.open(resultFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(fileSequence), std::ios::app);

        std::ofstream endeffectorfile;
        std::ofstream endeffectorfile_sequence;
        endeffectorfile.open(endeffectorFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(0), std::ios::app);
        endeffectorfile_sequence.open(endeffectorFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(fileSequence), std::ios::app);

        for (std::size_t i = 0; i < p.getStateCount(); ++i) {
            double j[6];

            for (size_t ii = 0; ii < jointNumber; ++ii) {
                j[ii] = (double)p.getState(i)
                            ->as<ob::RealVectorStateSpace::StateType>()
                            ->values[ii];
            }
            if (jointNumber == 3) {
                for (size_t ii = 3; ii < 6; ++ii) {
                    j[ii] = start[ii] * M_PI / 180.0;
                }
            }
            dd::SkeletonPtr robot = world_->getSkeleton(robotName);

            for (size_t ii = 0; ii < 6; ++ii) {
                robot->getDof(ii + 2)->setPosition(j[ii]);
            }

            Eigen::Isometry3d gripperTransform = robot->getBodyNode("gripper")->getTransform();
            Eigen::Vector3d gripperTrans = gripperTransform.translation();
            Eigen::Matrix3d gripperRot = gripperTransform.rotation();
            Eigen::Quaterniond quat(gripperTransform.rotation());

            double xs = 30.0 / 1000.0;
            double ys = -60.0 / 1000.0;
            double zs = 95.0 / 1000.0;

            gripperTrans(0) += xs * gripperRot(0, 0) + ys * gripperRot(0, 1) + zs * gripperRot(0, 2);
            gripperTrans(1) += xs * gripperRot(1, 0) + ys * gripperRot(1, 1) + zs * gripperRot(1, 2);
            gripperTrans(2) += xs * gripperRot(2, 0) + ys * gripperRot(2, 1) + zs * gripperRot(2, 2);

            for (size_t ii = 0; ii < 6; ++ii) {
                if (ii < 5) {
                    resultfile << j[ii] << " ";
                    resultfile_sequence << j[ii] << " ";
                } else {
                    resultfile << j[5] << std::endl;
                    resultfile_sequence << j[5] << std::endl;
                }
            }

            endeffectorfile << gripperTrans(0) << " " << gripperTrans(1) << " " << gripperTrans(2) << " "
                            << quat.w() << " " << quat.x() << " " << quat.y()
                            << " " << quat.z() << std::endl;
            endeffectorfile_sequence << gripperTrans(0) << " " << gripperTrans(1) << " " << gripperTrans(2) << " "
                                     << quat.w() << " " << quat.x() << " " << quat.y()
                                     << " " << quat.z() << std::endl;
        }
        resultfile.close();
        resultfile_sequence.close();
        endeffectorfile.close();
        endeffectorfile_sequence.close();

        if (jointNumber == 3) {
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
                    dd::SkeletonPtr robot = world_->getSkeleton(robotName);

                    robot->getDof(2)->setPosition(realsOld[0]);
                    robot->getDof(3)->setPosition(realsOld[1]);
                    robot->getDof(4)->setPosition(realsOld[2]);

                    Eigen::Isometry3d gripperTransform = robot->getBodyNode("gripper")->getTransform();
                    Eigen::Vector3d tr = gripperTransform.translation();

                    ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;
                    /*
                for (double t = step; t <= 1.01; t += step) {
                    space->interpolate(s1, s2, t, s3);
                    space->copyToReals(reals, s3);

                    //for (const auto& r : realsOld)

                    //    ofs_e << r << " ";
                    //

                    robot->getDof(2)->setPosition(reals[0]);
                    robot->getDof(3)->setPosition(reals[1]);
                    robot->getDof(4)->setPosition(reals[2]);

                    transform = robot->getBodyNode("toolflange_link")->getTransform();
                    tr = transform.translation();

                    ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;
                    ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;

                }
*/
                    space->copyToReals(reals, s2);

                    robot->getDof(2)->setPosition(reals[0]);
                    robot->getDof(3)->setPosition(reals[1]);
                    robot->getDof(4)->setPosition(reals[2]);

                    gripperTransform = robot->getBodyNode("gripper")->getTransform();
                    tr = gripperTransform.translation();

                    ofs_e << tr(0) << " " << tr(1) << " " << tr(2) << std::endl;
                }
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

        double distMin = std::numeric_limits<double>::max();
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                const ob::State* s2 = pdat.getVertex(edge_list[i2]).getState();

                dd::SkeletonPtr robot = world_->getSkeleton(robotName);
                space->copyToReals(reals, s2);

                for (size_t ii = 0; ii < 6; ++ii) {
                    robot->getDof(ii + 2)->setPosition(reals[ii]);
                }

                Eigen::Isometry3d gripperTransform = robot->getBodyNode("gripper")->getTransform();

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

        dd::SkeletonPtr robot = world_->getSkeleton(robotName);
        dd::SkeletonPtr tensegrity = world_->getSkeleton(tensegrityName);

        Eigen::Isometry3d attachTransform = tensegrity->getBodyNode("attach" + std::to_string(tendonNumber + 1))->getTransform();
        Eigen::Vector3d attachTrans = attachTransform.translation();

        Eigen::Isometry3d pulleyTransform = tensegrity->getBodyNode("pulley" + std::to_string(tendonNumber + 1))->getTransform();
        Eigen::Vector3d pulleyTrans = pulleyTransform.translation();

        Eigen::Isometry3d gripperTransform = robot->getBodyNode("gripper")->getTransform();
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
        double j[6];

        dd::SkeletonPtr robot = world_->getSkeleton(robotName);
        for (size_t ii = 0; ii < jointNumber; ++ii) {
            j[ii] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[ii];
            robot->getDof(ii + 2)->setPosition(j[ii]);
        }

        if (withString) {
            return isWithinReach() && !world_->checkCollision();
            //return !world_->checkCollision();
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

    std::ifstream file(resultFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(0));
    std::string line = getLastLine(file);

    file.close();

    std::string delimiter = " ";

    size_t pos = 0;
    std::string token;
    int i = 0;
    int arsize = 6;
    double linear[arsize];
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
    dd::SkeletonPtr robot = world->getSkeleton(robotName);

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

            robot->getDof(2)->setPosition(theta[1]);
            robot->getDof(3)->setPosition(theta[2]);
            robot->getDof(4)->setPosition(theta[3]);
            robot->getDof(5)->setPosition(theta[4]);
            robot->getDof(6)->setPosition(theta[5]);
            robot->getDof(7)->setPosition(theta[6]);

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
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
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
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
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
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
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
    // dd::SkeletonPtr robot = world->getSkeleton(robotName);
    // robot->getBodyNode("claws")
    //     ->setCollidable(true);
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
        ->getVisualizationShape(0)
        ->setHidden(true);
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))
        ->setCollidable(false);
}

void attachStringAt(int ii)
{
    //dd::SkeletonPtr robot = world->getSkeleton(robotName);
    //robot->getBodyNode("claws")
    //    ->setCollidable(false);

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
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

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("attach" + std::to_string(attNum + 1))
                                                ->getTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();

    double xs = -50;
    double ys = 70;
    //double zs = -80;
    double zs = -95;
    Eigen::Matrix3d rot_ten;
    if (wristUp) {
        rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d::UnitX());
    } else {
        rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(-90 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(180 * M_PI / 180.0, Eigen::Vector3d::UnitY());
        zs = -100;
        xs = -5;
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

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("detach" + std::to_string(detNum + 1))->getTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();

    double xs = 0;
    double ys = 80;

    double zs = -95;
    Eigen::Matrix3d rot_ten;
    if (wristUp) {
        rot_ten = tensegrityTransform.rotation();
        //rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(90 * M_PI / 180.0, Eigen::Vector3d::UnitX());
    } else {
        rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(180 * M_PI / 180.0, Eigen::Vector3d::UnitY());
        //rot_ten = tensegrityTransform.rotation() * Eigen::AngleAxisd(90 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(180 * M_PI / 180.0, Eigen::Vector3d::UnitY());
        xs = -(25);
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

Eigen::Isometry3d getMidPosition(int midNum,
    bool wristUp)
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
    Eigen::Isometry3d tensegrityTransform = tensegrity->getBodyNode("mid" + std::to_string(midNum + 1))->getTransform();
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
    //tenTrans(2) -= 1278;
    tenTrans(2) -= 1278;

    tf.linear() = rot_ten;
    tf.translation() = tenTrans;
    return tf;
}

Eigen::Isometry3d getTightenerPoint(int detNum)
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
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

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
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
    feasibleLocation.open(feasibleLocFName + std::to_string(glob_ii), std::ios::trunc);
    //std::ofstream nonFeasIdx;
    //nonFeasIdx.open("nonFeas.txt", std::ios::trunc);

    Eigen::VectorXd strings(9);
    strings << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
    dd::SkeletonPtr robot = world->getSkeleton(robotName);

    Eigen::Matrix3d rot_ten;
    for (int xx = -2200 + glob_ii * 440; xx <= -2200 + (glob_ii + 1) * 440; xx += 50) {
        for (int yy = -2200; yy <= 2200; yy += 50) {
            //if ((xx >= -400 && xx <= 400) && (yy >= -350 && yy <= 350)) {
            //    continue;
            //}
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
                    robot->getDof(kk)->setPosition(0);
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
                    //std::cout << ii << std::endl;
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
            std::cout << xx << " " << yy << std::endl;
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
    //int attDistSorted[9];
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
bool isWithinReach(int tendonNumber)
{

    dd::SkeletonPtr robot = world->getSkeleton(robotName);
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);

    Eigen::Isometry3d attachTransform = tensegrity->getBodyNode("attach" + std::to_string(tendonNumber + 1))->getTransform();
    Eigen::Vector3d attachTrans = attachTransform.translation();

    Eigen::Isometry3d pulleyTransform = tensegrity->getBodyNode("pulley" + std::to_string(tendonNumber + 1))->getTransform();
    Eigen::Vector3d pulleyTrans = pulleyTransform.translation();

    Eigen::Isometry3d gripperTransform = robot->getBodyNode("gripper")->getTransform();
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
    if ((gripperTrans - pulleyTrans).squaredNorm() > ((pulleyTrans - attachTrans).squaredNorm())) {
        //   std::cout << "dafaq?" << std::endl;
        return false;
    }

    return true;
}

std::vector<int> moveAround(Eigen::VectorXd& initPos, int kk)
{
    dd::SkeletonPtr robot = world->getSkeleton(robotName);

    int distMin = std::numeric_limits<int>::max();
    std::vector<int> saveInds;
    saveInds.push_back(-20);
    saveInds.push_back(-2);
    saveInds.push_back(5);
    int inc0 = 0;
    int inc1 = 0;
    int inc2 = 0;
    for (int ii = 0; ii < 8; ii++) {
        int sign[3] = { (int)((ii & 4) >> 2) * 2 - 1, (int)((ii & 2) >> 1) * 2 - 1, (int)(ii & 1) * 2 - 1 };
        for (int jj = 0; jj < 30; jj++) {
            inc0 = sign[0] * jj;
            if ((initPos[0] + inc0) * M_PI / 180.0 > jointMin[0] && (initPos[0] + inc0) * M_PI / 180.0 < jointMax[0]) {
                robot->getDof(2)->setPosition((initPos[0] + inc0) * M_PI / 180.0);
                for (int mm = 0; mm < 10; mm++) {
                    inc1 = sign[1] * mm;
                    if ((initPos[1] + inc1) * M_PI / 180.0 > jointMin[1] && (initPos[1] + inc1) * M_PI / 180.0 < jointMax[1]) {
                        robot->getDof(3)->setPosition((initPos[1] + inc1) * M_PI / 180.0);
                        for (int ll = 0; ll < 10; ll++) {
                            inc2 = sign[2] * ll;
                            if (initPos[2] + inc2 * M_PI / 180.0 > jointMin[2] && initPos[2] + inc2 * M_PI / 180.0 < jointMax[2]) {
                                robot->getDof(4)->setPosition((initPos[2] + inc2) * M_PI / 180.0);
                            }
                            if (!world->checkCollision() && isWithinReach(kk)) {
                                if (distMin > (inc0 * inc0 + inc1 * inc1 + inc2 * inc2)) {
                                    distMin = (inc0 * inc0 + inc1 * inc1 + inc2 * inc2);
                                    saveInds[0] = inc0;
                                    saveInds[1] = inc1;
                                    saveInds[2] = inc2;
                                }
                                if (distMin <= 25 || jj >=10) {
                                    return saveInds;
                                }
                                //std::cout << inc0 << " " << inc1 << " " << inc2 << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
    return saveInds;
}

Eigen::VectorXd collisionlessFinal(const Eigen::VectorXd& start, const Eigen::VectorXd& finish, int kk)
{
    Eigen::VectorXd cFinal(6);

    dd::SkeletonPtr robot = world->getSkeleton(robotName);
    Eigen::VectorXd initPos(6);
    initPos << finish[0],
        finish[1],
        finish[2],
        start[3],
        start[4],
        start[5];

    for (int jj = 0; jj < 6; jj++) {
        robot->getDof(jj + 2)->setPosition(initPos[jj] * M_PI / 180.0);
    }

    if (!world->checkCollision() && isWithinReach(kk)) {
        for (int ii = 0; ii < 6; ii++) {
            cFinal[ii] = robot->getDof(ii + 2)->getPosition() * 180 / M_PI;
        }

        return cFinal;
    }

    std::vector<int> saveInds = moveAround(initPos, kk);
    std::cout << saveInds[0] << " " << saveInds[1] << " " << saveInds[2] << std::endl;
    robot->getDof(2)->setPosition((initPos[0] + saveInds[0]) * M_PI / 180.0);
    robot->getDof(3)->setPosition((initPos[1] + saveInds[1]) * M_PI / 180.0);
    robot->getDof(4)->setPosition((initPos[2] + saveInds[2]) * M_PI / 180.0);

    for (int ii = 0; ii < 6; ii++) {
        cFinal[ii] = robot->getDof(ii + 2)->getPosition() * 180 / M_PI;
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
    double minJointDist = std::numeric_limits<double>::max();
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
    finish_trans = collisionlessFinal(fullDetach[minDetIdx], fullAttach[minAttIdx], kk);

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
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);

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
    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);

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

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);

    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d tenRot;

    int kk = 0;
    std::vector<Eigen::VectorXd> part1;
    std::vector<Eigen::VectorXd> part2;
    std::vector<Eigen::VectorXd> fullDetach;
    std::vector<Eigen::VectorXd> fullAttach;

    std::ofstream plannable;
    plannable.open(plannableFName + std::to_string(glob_ii), std::ios::trunc);

    std::ifstream feas(feasibleLocFName + std::to_string(glob_ii));
    double xx, yy, aa_zz;
    double minDist;
    while (!feas.eof()) {
        feas >> xx >> yy >> aa_zz;
        std::cout << "Location: " << xx << " " << yy << " " << aa_zz << std::endl;

        tenRot = Eigen::AngleAxisd(aa_zz * M_PI / 180.0, Eigen::Vector3d::UnitZ());
        tenMove.linear() = tenRot;
        tenMove.translation() << (double)xx / 1000.0, (double)yy / 1000.0, 0.0;
        moveSkeleton(tensegrity, tenMove);

        detachAllStrings();

        int count = 0;
        for (kk = 0; kk < 9; kk++) {
            minDist = isPlannable(kk);
            //plannable << "kk = " << kk << " minimum distance = " << minDist << std::endl;
            if (minDist > 0.25) {
                break;
                //count -= 1;
            }
            count += 1;
        }
        //plannable << "count = " << count << std::endl;
        if (count == 9) {
            plannable << xx << " " << yy << " " << aa_zz << std::endl;
        }
        //plannable << "********************" << std::endl;
    }

    plannable.close();
}

double getWeightedJointDist(const Eigen::VectorXd& detach, const Eigen::VectorXd& attach)
{
    double jointDist = 0;
    double weight1 = 20.0;
    double weight23 = 20.0;
    double weight46 = 1.0;
    double weight = 0.0;

    for (int ii = 0; ii < 6; ii++) {
        if (ii == 0) {
            weight = weight1;
        } else if (ii < 3) {
            weight = weight23;
        } else {
            weight = weight46;
        }
        jointDist += weight * (pow(detach[ii] - attach[ii], 2));
    }
    return jointDist;
}

void planAttachDirect(int kk, int jj)
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
    for (size_t j = 0; j < fullDetach.size(); j++) {
        printVector(fullDetach[j]);
    }

    attachStringAt(kk);

    tf = getAttachPosition(kk, false);
    part1 = getInverseKinematics(tf);

    tf = getAttachPosition(kk, true);
    part2 = getInverseKinematics(tf);

    fullAttach.reserve(part1.size() + part2.size());
    fullAttach.insert(fullAttach.end(), part1.begin(), part1.end());
    fullAttach.insert(fullAttach.end(), part2.begin(), part2.end());
    for (size_t j = 0; j < fullAttach.size(); j++) {
        printVector(fullAttach[j]);
    }

    detachStringAt(kk);

    int minDetIdx = 0;
    int minAttIdx = 0;
    double minJointDist = std::numeric_limits<double>::max();
    for (size_t dd = 0; dd < fullDetach.size(); dd++) {
        for (size_t aa = 0; aa < fullAttach.size(); aa++) {
            double jointDist = getWeightedJointDist(fullDetach[dd], fullAttach[aa]);
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
    lastFinish = finish;

    tensegrityEnvironment env(3, kk, true);
    env.setWorld(world);

    Eigen::VectorXd finish_trans(6);
    finish_trans = collisionlessFinal(start, finish, kk);

    std::cout << finish_trans << std::endl;

    if (env.plan(start, finish_trans)) {
        env.recordSolution(start, jj);
    }

    std::cout << finish_trans << std::endl;
    start = finish_trans;
    tensegrityEnvironment env1(6, kk, true);
    env1.setWorld(world);
    if (env1.plan(start, finish)) {
        env1.recordSolution(start, jj);
    }
}

void planGoToDetach(int kk, int jj)
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
    for (size_t j = 0; j < fullDetach.size(); j++) {
        printVector(fullDetach[j]);
    }

    attachStringAt(kk);

    tf = getAttachPosition(kk, false);
    part1 = getInverseKinematics(tf);

    tf = getAttachPosition(kk, true);
    part2 = getInverseKinematics(tf);

    fullAttach.reserve(part1.size() + part2.size());
    fullAttach.insert(fullAttach.end(), part1.begin(), part1.end());
    fullAttach.insert(fullAttach.end(), part2.begin(), part2.end());
    for (size_t j = 0; j < fullAttach.size(); j++) {
        printVector(fullAttach[j]);
    }

    detachStringAt(kk);

    int minDetIdx = 0;
    double minJointDist = std::numeric_limits<double>::max();
    for (size_t dd = 0; dd < fullDetach.size(); dd++) {
        for (size_t aa = 0; aa < fullAttach.size(); aa++) {
            double jointDist = getWeightedJointDist(fullDetach[dd], fullAttach[aa]);
            if (jointDist < minJointDist) {
                minDetIdx = dd;
                minJointDist = jointDist;
            }
        }
    }

    Eigen::VectorXd start(6);
    start = lastFinish;

    Eigen::VectorXd finish(6);
    finish = fullDetach[minDetIdx];

    tensegrityEnvironment env(3, kk, false);
    env.setWorld(world);

    Eigen::VectorXd finish_trans(6);
    finish_trans = collisionlessFinal(start, finish, kk);

    std::cout << finish_trans << std::endl;

    if (env.plan(start, finish_trans)) {
        env.recordSolution(start, jj);
    }

    start = finish_trans;
    tensegrityEnvironment env1(6, kk, false);
    env1.setWorld(world);
    if (env1.plan(start, finish)) {
        env1.recordSolution(start, jj);
    }
}

void planAttachMidPoint(int kk)
{
    std::vector<Eigen::VectorXd> part1;
    std::vector<Eigen::VectorXd> part2;
    std::vector<Eigen::VectorXd> fullDetach;
    std::vector<Eigen::VectorXd> fullAttach;
    std::vector<std::vector<Eigen::VectorXd> > fullMid;

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

    for (int mm = 0; mm < 6; mm++) {
        tf = getMidPosition(mm, false);
        part1 = getInverseKinematics(tf);

        tf = getMidPosition(mm, true);
        part2 = getInverseKinematics(tf);

        std::vector<Eigen::VectorXd> oneMid;

        oneMid.reserve(part1.size() + part2.size());
        oneMid.insert(oneMid.end(), part1.begin(), part1.end());
        oneMid.insert(oneMid.end(), part2.begin(), part2.end());
        fullMid.push_back(oneMid);
    }

    for (size_t i = 0; i < fullMid.size(); i++) {
        for (size_t j = 0; j < fullMid[i].size(); j++) {
            printVector(fullMid[i][j]);
        }
        std::cout << std::endl;
    }

    /*
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
    resultfile.open(resultFName + std::to_string(0), std::ios::trunc);
    resultfile.close();

    std::ofstream endeffectorfile;
    endeffectorfile.open(endeffectorFName + std::to_string(0), std::ios::trunc);
    endeffectorfile.close();

    tensegrityEnvironment env(3, kk, true);
    env.setWorld(world);

    Eigen::VectorXd finish_trans(6);
    finish_trans = collisionlessFinal(start, finish,kk);

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
*/
}

void setUpRobot()
{
    std::string prefix = getWorkingDirectory();

    dd::SkeletonPtr robot = du::SdfParser::readSkeleton(prefix + std::string("/data/robot/model.sdf"));
    robot->setName(robotName);

    world->addSkeleton(robot);
    robot->enableSelfCollision();

    robot->getBodyNode("gripper")->getVisualizationShape(0)->setColor(Eigen::Vector3d(0, 1.0, 0));
    robot->getBodyNode("table")->getVisualizationShape(0)->setColor(Eigen::Vector3d(0.6, 0.6, 0.6));
}

void setUpTensegrity()
{
    std::string prefix = getWorkingDirectory();

    du::DartLoader dl;

    dl.addPackageDirectory(tensegrityName, prefix + std::string("/data/tensegrity/"));
    dd::SkeletonPtr tensegrity = dl.parseSkeleton(prefix + std::string("/data/tensegrity/robots/tensegrity.URDF"));
    tensegrity->setName(tensegrityName);

    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d tenRot;

    std::ifstream fin("data/results/tenSetUp.txt");
    double x, y, alpha;
    int i = -1;
    while (!fin.eof() && glob_ii > i) {
        fin >> x >> y >> alpha;
        i++;
        std::cout << i << std::endl;
    }
    fin.close();
    std::cout << x << " " << y << " " << alpha << std::endl;

    tenRot = Eigen::AngleAxisd(alpha * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    tenMove.translation() << x / 1000.0, y / 1000.0, 0.0;
    tenMove.rotate(tenRot);
    moveSkeleton(tensegrity, tenMove);

    world->addSkeleton(tensegrity);
}

void tendonColor(int ii, bool tenRed, bool atDetRed)
{
    Eigen::Vector3d tenColor;
    Eigen::Vector3d atDetColor;
    tenColor << 0.792156862745098, 0.819607843137255, 0.933333333333333;
    atDetColor << 0.792156862745098, 0.819607843137255, 0.933333333333333;
    if (tenRed) {
        tenColor << 1, 0, 0;
    }
    if (atDetRed) {
        atDetColor << 1, 0, 0;
    }

    dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);
    tensegrity->getBodyNode("tendon" + std::to_string(ii + 1))->getVisualizationShape(0)->setColor(tenColor);
    tensegrity->getBodyNode("attach" + std::to_string(ii + 1))->getVisualizationShape(0)->setColor(atDetColor);
    tensegrity->getBodyNode("detach" + std::to_string(ii + 1))->getVisualizationShape(0)->setColor(atDetColor);
}

void resultReplay(MyWindow& window)
{

    dd::SkeletonPtr robot = world->getSkeleton(robotName);
    for (int jj = 2; jj <= 7; jj++) {
        robot->getDof(jj)->setPosition(0);
    }
    double jk[6];
    while (true) {
        if (window.replay && !window.stop) {
            int startLoop = window.fileSequence;
            int endLoop = window.fileSequence;
            if (window.fileSequence == 0) {
                startLoop = 1;
                endLoop = 9;
            }

            for (int kk = startLoop; kk <= endLoop; kk++) {
                std::ifstream fin(resultFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(kk));
                tendonColor(seqArray[kk - 1] - 1, false, true);

                std::cout << kk << std::endl;

                while (!fin.eof()) {
                    for (int ii = 0; ii < 6; ii++) {
                        fin >> jk[ii];
                        robot->getDof(ii + 2)->setPosition(jk[ii]);
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(window.speed));
                    if (window.stop) {
                        break;
                    }
                    while (!window.replay) {
                    }
                }
                fin.close();
                if (kk > 1) {
                    tendonColor(seqArray[kk - 2] - 1, false, false);
                }
                tendonColor(seqArray[kk - 1] - 1, true, false);
                attachStringAt(seqArray[kk - 1] - 1);

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                if (window.fileSequence == 0 && window.stop) {
                    break;
                }
            }
            if (!window.stop) {
                window.replay = false;
            }
        }
    }
    std::cout << "dafaq?" << std::endl;
}

void initFiles()
{

    std::ofstream resultfile;
    resultfile.open(resultFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(0), std::ios::trunc);
    resultfile << "0 0 0 0 0 0" << std::endl;
    resultfile.close();

    std::ofstream endeffectorfile;
    endeffectorfile.open(endeffectorFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(0), std::ios::trunc);
    endeffectorfile << "0.08 -0.01 2.773 1 0 0 0" << std::endl;
    endeffectorfile.close();

    for (int ii = 0; ii < 9; ii++) {

        std::ofstream resultfile;
        resultfile.open(resultFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(ii + 1), std::ios::trunc);
        resultfile.close();

        std::ofstream endeffectorfile;
        endeffectorfile.open(endeffectorFName + "_" + std::to_string(glob_ii) + "_" + std::to_string(ii + 1), std::ios::trunc);
        endeffectorfile.close();
    }
}

int main(int argc, char* argv[])
{
    world->getConstraintSolver()->setCollisionDetector(
        new dc::FCLCollisionDetector());

    if (argc >= 2) {
        glob_ii = std::atoi(argv[1]);
        std::cout << glob_ii << std::endl;
    }

    setUpRobot();
    setUpTensegrity();

    detachAllStrings();

    //printFeasibleTensegrityLocation();
    //printAttachmentSequence();
    //printTightenerFeasibility();
    //printTendonFeasibility();
    //printPlannable();
    //planAttachMidPoint(0);

    //dd::SkeletonPtr tensegrity = world->getSkeleton(tensegrityName);

    if (argc < 3) {
        lastFinish << -100, 90, -90, 0, 0, 0;

        dd::SkeletonPtr robot = world->getSkeleton(robotName);
        robot->getBodyNode("claws")
            ->setCollidable(false);

        initFiles();
        for (size_t kk = 0; kk < 9; ++kk) {
            planGoToDetach(seqArray[kk] - 1, kk + 1);

            planAttachDirect(seqArray[kk] - 1, kk + 1);
            attachStringAt(seqArray[kk] - 1);
        }
    }
    detachAllStrings();

    MyWindow window(world);
    glutInit(&argc, argv);
    window.initWindow(600 * 2, 500 * 2, "SDF");
    window.glob_jj = glob_ii;

    std::thread t(resultReplay, std::ref(window));

    glutMainLoop();
    t.join();
    return 0;
}
