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

double jj1, jj2, jj3, jj4, jj5, jj6 =0;

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

    double j1, j2, j3, j4, j5, j6 ;
    j1 = staubli->getDof(2)->getPosition() * 180 / M_PI; 
    j2 = staubli->getDof(3)->getPosition()* 180 / M_PI; 
    j3 = staubli->getDof(4)->getPosition()* 180 / M_PI; 
    j4 = staubli->getDof(5)->getPosition()* 180 / M_PI; 
    j5 = staubli->getDof(6)->getPosition()* 180 / M_PI; 
    j6 = staubli->getDof(7)->getPosition()* 180 / M_PI; 
    jj1 = j1; 
    jj2 = j2; 
    jj3 = j3; 
    jj4 = j4; 
    jj5 = j5; 
    jj6 = j6; 
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
    glColor3f(0.0, 0.0, 0.0);

    dg::drawStringOnScreen(0.85f, 0.65f, "Joint 1: ");
    dg::drawStringOnScreen(0.85f, 0.625f, "Joint 2: ");
    dg::drawStringOnScreen(0.85f, 0.6f, "Joint 3: ");
    dg::drawStringOnScreen(0.85f, 0.575f, "Joint 4: ");
    dg::drawStringOnScreen(0.85f, 0.55f, "Joint 5: ");
    dg::drawStringOnScreen(0.85f, 0.525f, "Joint 6: ");

    glColor3f(0.0, 1.0, 0.0);


    dg::drawStringOnScreen(0.9f, 0.65f, std::to_string(jj1));
    dg::drawStringOnScreen(0.9f, 0.625f,std::to_string(jj2));
    dg::drawStringOnScreen(0.9f, 0.6f,  std::to_string(jj3));
    dg::drawStringOnScreen(0.9f, 0.575f,std::to_string(jj4));
    dg::drawStringOnScreen(0.9f, 0.55f, std::to_string(jj5));
    dg::drawStringOnScreen(0.9f, 0.525f,std::to_string(jj6));
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SimWindow::drawSkels();

}

  void MyWindow::setViewTrack(double j1,double j2,double j3,double j4,double j5,double j6){
    
    jj1 = j1 * 180 / M_PI; 
    jj2 = j2 * 180 / M_PI; 
    jj3 = j3 * 180 / M_PI; 
    jj4 = j4 * 180 / M_PI; 
    jj5 = j5 * 180 / M_PI; 
    jj6 = j6 * 180 / M_PI; 

}
