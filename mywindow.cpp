#include "mywindow.h"
namespace dd = dart::dynamics;

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    mZoom = 0.040;
    mTranslate = true;
}
void MyWindow::keyboard(unsigned char key, int x, int y)
{
        dd::SkeletonPtr staubli = mWorld->getSkeleton("staubli");
        double j  = 0; 
    switch(key)
    {
        case '1':
            j = staubli->getDof(2)->getPosition(); 
            staubli->getDof(2)->setPosition(j+M_PI/18.0); 
            break;
        case '2':
            j = staubli->getDof(3)->getPosition(); 
            staubli->getDof(3)->setPosition(j+M_PI/18.0); 
            break;
        case '3':
            j = staubli->getDof(4)->getPosition(); 
            staubli->getDof(4)->setPosition(j+M_PI/18.0); 
            break;
        case '4':
            j = staubli->getDof(5)->getPosition(); 
            staubli->getDof(5)->setPosition(j+M_PI/18.0); 
            break;
        case '5':
            j = staubli->getDof(6)->getPosition(); 
            staubli->getDof(6)->setPosition(j+M_PI/18.0); 
            break;
        case '6':
            j = staubli->getDof(7)->getPosition(); 
            staubli->getDof(7)->setPosition(j+M_PI/18.0); 
            break;
        case '!':
            j = staubli->getDof(2)->getPosition(); 
            staubli->getDof(2)->setPosition(j-M_PI/18.0); 
            break;
        case '@':
            j = staubli->getDof(3)->getPosition(); 
            staubli->getDof(3)->setPosition(j-M_PI/18.0); 
            break;
        case '#':
            j = staubli->getDof(4)->getPosition(); 
            staubli->getDof(4)->setPosition(j-M_PI/18.0); 
            break;
        case '$':
            j = staubli->getDof(5)->getPosition(); 
            staubli->getDof(5)->setPosition(j-M_PI/18.0); 
            break;
        case '%':
            j = staubli->getDof(6)->getPosition(); 
            staubli->getDof(6)->setPosition(j-M_PI/18.0); 
            break;
        case '^':
            j = staubli->getDof(7)->getPosition(); 
            staubli->getDof(7)->setPosition(j-M_PI/18.0); 
            break;
        default:
            SimWindow::keyboard(key, x, y);
    }
}

void MyWindow::drawSkels() {
    std::lock_guard<std::mutex> lock(readMutex);
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SimWindow::drawSkels();
}

void MyWindow::setViewTrack(const Eigen::Vector3d& v, const Eigen::Quaterniond& rot)
{
    std::lock_guard<std::mutex> lock(readMutex);
    mTrans = -v*1000.0;//-Eigen::Vector3d(20,20,20);
    //mTrans = -Eigen::Vector3d(25,25,25)*1000;
    Eigen::Quaterniond quat(Eigen::AngleAxisd(-3*M_PI/8.0, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())*rot);

    //    mTrackBall.setQuaternion(quat);
}
