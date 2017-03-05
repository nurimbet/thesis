#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <dart/dart.h>
#include <Eigen/Eigen>
#include <iomanip>

namespace ds = dart::simulation;
namespace dg = dart::gui;

/**
 * @brief The MyWindow class renders the simulation, might be used to demonstrate the results
 */
class MyWindow : public dart::gui::SimWindow {
 public:
  /// ctor
  MyWindow(const ds::WorldPtr& world);

  /// fixes rendering
  void keyboard(unsigned char key, int x, int y) override;
  void drawAxes(const Eigen::Vector3d &tr, const Eigen::Matrix3d &rot);  
  void drawSkels() override;
  void setViewTrack(double j1,double j2,double j3,double j4,double j5,double j6);
  void moveJoint(int jointNum, bool positive);
 private:
 std::mutex readMutex;
};

#endif  // MYWINDOW_H
