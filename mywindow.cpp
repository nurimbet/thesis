#include "mywindow.h"
namespace dd = dart::dynamics;

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

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    mZoom = 0.20;
    mTranslate = true;
    mTrans = -Eigen::Vector3d(0.15,0.15,1.5)*1000;
    Eigen::Quaterniond quat( 0.854,  -0.354, 0.146,  0.354);

    mTrackBall.setQuaternion(quat);
}
void MyWindow::keyboard(unsigned char key, int x, int y)
{
    dd::SkeletonPtr staubli = mWorld->getSkeleton("staubli");
    double j  = 0; 
    double k = M_PI / 180.0;
    switch(key)
    {
        case '1':
            j = staubli->getDof(2)->getPosition(); 
            if (j + k <= jointMax1)
            {
                staubli->getDof(2)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
   //                 std::cout << "collision?" << std::endl;
                    staubli->getDof(2)->setPosition(j); 
                }
            }
            break;
        case '2':
            j = staubli->getDof(3)->getPosition(); 
            if (j + k <= jointMax2)
            {
                staubli->getDof(3)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(3)->setPosition(j); 
                }
            }
            break;
        case '3':
            j = staubli->getDof(4)->getPosition(); 
            if (j + k <= jointMax3)
            {
                staubli->getDof(4)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(4)->setPosition(j); 
                }
            }
            break;
        case '4':
            j = staubli->getDof(5)->getPosition(); 
            if (j + k <= jointMax4)
            {
                staubli->getDof(5)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(5)->setPosition(j); 
                }
            }
            break;
        case '5':
            j = staubli->getDof(6)->getPosition(); 
            if (j + k <= jointMax5)
            {
                staubli->getDof(6)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(6)->setPosition(j); 
                }
            }
            break;
        case '6':
            j = staubli->getDof(7)->getPosition(); 
            if (j + k <= jointMax6)
            {
                staubli->getDof(7)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(7)->setPosition(j); 
                }
            }
            break;
        case '!':
            j = staubli->getDof(2)->getPosition(); 
            if (j - k >= jointMin1)
            {
                staubli->getDof(2)->setPosition(j-k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(2)->setPosition(j); 
                }
            }
            break;
        case '@':
            j = staubli->getDof(3)->getPosition(); 
            if (j - k >= jointMin2)
            {
                staubli->getDof(3)->setPosition(j-k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(3)->setPosition(j); 
                }
            }
            break;
        case '#':
            j = staubli->getDof(4)->getPosition(); 
            if (j - k >= jointMin3)
            {
                staubli->getDof(4)->setPosition(j-k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(4)->setPosition(j); 
                }
            }
            break;
        case '$':
            j = staubli->getDof(5)->getPosition(); 
            if (j - k >= jointMin4)
            {
                staubli->getDof(5)->setPosition(j-k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(5)->setPosition(j); 
                }
            }
            break;
        case '%':
            j = staubli->getDof(6)->getPosition(); 
            if (j - k >= jointMin5)
            {
                staubli->getDof(6)->setPosition(j-k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(6)->setPosition(j); 
                }
            }
            break;
        case '^':
            j = staubli->getDof(7)->getPosition(); 
            if (j - k >= jointMin6)
            {
                staubli->getDof(7)->setPosition(j-k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(7)->setPosition(j); 
                }
            }
            break;
        default:
            SimWindow::keyboard(key, x, y);
    }
    
    double j1, j2, j3, j4, j5, j6;
    j1 = staubli->getDof(2)->getPosition() * 180 / M_PI; 
    j2 = staubli->getDof(3)->getPosition()* 180 / M_PI; 
    j3 = staubli->getDof(4)->getPosition()* 180 / M_PI; 
    j4 = staubli->getDof(5)->getPosition()* 180 / M_PI; 
    j5 = staubli->getDof(6)->getPosition()* 180 / M_PI; 
    j6 = staubli->getDof(7)->getPosition()* 180 / M_PI; 
    std::cout << "\r" <<
        std::setw(8) << std::setfill(' ') << j1 << " " 
        << std::setw(8) << j2 << " " 
        << std::setw(8) << j3 << " " 
        << std::setw(8) << j4 << " " 
        << std::setw(8) << j5 << " " 
        << std::setw(8) << j6 << " " << std::flush;
    

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
