// Default Projection est, pdst, KPIECE
#include <dart/dart.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
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
            ss_->setPlanner(ob::PlannerPtr(new og::PRM(ss_->getSpaceInformation())));
        }

        bool plan(const Eigen::Vector3d &init, const Eigen::Vector3d & final) {
            if (!ss_) return false;

            ob::ScopedState<> start(ss_->getStateSpace());
            for (std::size_t i = 0; i < 3; ++i) start[i] = init[i];

            ob::ScopedState<> goal(ss_->getStateSpace());
            for (std::size_t i = 0; i < 3; ++i) goal[i] = final[i];

            ss_->setStartAndGoalStates(start, goal);

            // this will run the algorithm for one second
            ss_->solve();

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
            for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
            {
                const double x = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];
                const double y = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];
                const double z = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2];
                resultfile << x << " " << y << " " << z << std::endl; 
            }
            resultfile.close();

            //
            // ADD CODE HERE
            //
        }

        void setWorld(const ds::WorldPtr &world) { world_ = world; }

    private:
        bool isStateValid(const ob::State *state) const {
            double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]; 
            double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]; 
            double z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]; 
            dd::SkeletonPtr staubli = world_->getSkeleton("staubli");

            staubli->getDof(2)->setPosition(x); 
            staubli->getDof(3)->setPosition(y); 
            staubli->getDof(4)->setPosition(z); 

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

int main(int argc, char* argv[]) 
{
    ds::WorldPtr world = std::make_shared<ds::World>();
    world->getConstraintSolver()->setCollisionDetector(
            new dc::BulletCollisionDetector());
    //    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

    std::string prefix = getWorkingDirectory();

    dd::SkeletonPtr staubli = du::SdfParser::readSkeleton(prefix + std::string("/model.sdf"));
    staubli->setName("staubli");
    setAllColors(staubli, Eigen::Vector3d(0.57, 0.6, 0.67));

    world->addSkeleton(staubli);
    
//    staubli->getDof(3)->setPosition(290 * M_PI / 180.0); 
//    staubli->getDof(4)->setPosition(290 * M_PI / 180.0); 
////    staubli->getDof(6)->setPosition(290 * M_PI / 180.0); 
    
    Eigen::Vector3d start(0, 0, 0);
    Eigen::Vector3d finish(90 * M_PI / 180.0, 90 * M_PI / 180.0, 90 * M_PI / 180.0);

    Simple3DEnvironment env;
    env.setWorld(world);

    if(env.plan(start,finish))
    {
        env.recordSolution();
    }

    std::thread t([&]()
    {
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        while (true) 
        {
            std::ifstream fin("result.txt");

            while (!fin.eof()) 
            {
                float x, y, z;
                //float rx, ry, rz, rw;
                fin >> x >> y >> z ;

                staubli->getDof(2)->setPosition(x); 
                staubli->getDof(3)->setPosition(y); 
                staubli->getDof(4)->setPosition(z); 
                 
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });

    MyWindow window(world);

    glutInit(&argc, argv);
    window.initWindow(640 * 2, 480 * 2, "SDF");

    glutMainLoop();

    t.join();

    return 0;
}
