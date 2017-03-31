#include "ompl/base/spaces/WeightedRealVectorStateSpace.h"
#include <Eigen/Eigen>

namespace ompl {
namespace base {
    
    
    void WeightedRealVectorStateSpace::setSkeleton(const dart::dynamics::SkeletonPtr &skl)
    {
       robot = skl;  
    }
    

    inline double ompl::base::WeightedRealVectorStateSpace::distance(const State* state1, const State* state2) const
    {
        //return localCost(state1, state2);

//        double dist = 0.0;
        const double* s1 = static_cast<const StateType*>(state1)->values;
        const double* s2 = static_cast<const StateType*>(state2)->values;

        for (size_t jj = 0; jj < dimension_; ++jj)
        {
            robot->getDof(jj+2)->setPosition(s1[jj]);
        }

        Eigen::Isometry3d gripperTransform = robot->getBodyNode("gripper")->getTransform();
        Eigen::Vector3d gripperTransS1 = gripperTransform.translation();

        for (size_t jj = 0; jj < dimension_; ++jj)
        {
            robot->getDof(jj+2)->setPosition(s2[jj]);
        }

        gripperTransform = robot->getBodyNode("gripper")->getTransform();
        Eigen::Vector3d gripperTransS2 = gripperTransform.translation();
        return (gripperTransS1 - gripperTransS2).squaredNorm(); 
/*
        double k = 0;
        for (unsigned int i = 0; i < dimension_; ++i) {
            double diff = (*s1++) - (*s2++);

            switch (i) {
            case 0:
            case 1:
            case 2:
                k = 1.00;
                break;
            case 3:
            case 4:
            case 5:
                k = 0.25;
                break;
            default:
                k = 0.0;
                break;
            }
            dist += (diff * diff) * k;
        }
        //return dist;
*/
        
        //return sqrt(dist);
    }

    inline ompl::base::StateSamplerPtr ompl::base::WeightedRealVectorStateSpace::allocDefaultStateSampler() const
    {
        return StateSamplerPtr(new WeightedRealVectorStateSampler(this));
    }
}
}
