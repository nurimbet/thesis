#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <Eigen/Eigen>
#include <dart/dart.h>
#include <iomanip>

namespace ds = dart::simulation;
namespace dg = dart::gui;

/**
 * @brief The MyWindow class renders the simulation, might be used to demonstrate the results
 */
class MyWindow : public dart::gui::SimWindow {
public:
    MyWindow(const ds::WorldPtr& world);

    void keyboard(unsigned char key, int x, int y) override;
    void drawAxes(const Eigen::Vector3d& tr, const Eigen::Matrix3d& rot);
    void drawSkels() override;
    void moveJoint(int jointNum, bool positive);
    void translateTensegrity(int axis, bool positive);
    void rotateTensegrity(int axis, bool positive);
    bool replay;
    bool stop;
    int speed;
    int currTree;
    int fileSequence;
    int glob_jj;
    int currPath;
private:
};

#endif // MYWINDOW_H
