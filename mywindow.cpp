#include "mywindow.h"
#include <fstream>
#include <iostream>
#include "util.h"

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


double joint1, joint2, joint3, joint4, joint5, joint6 = 0;
bool showPath = false;
bool showTree = false;
bool collisionEnabled = true;

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    mZoom = 0.28;
    mTranslate = true;
    mTrans = -Eigen::Vector3d(-0.301,-0.171,1.3)*1000;
    Eigen::Quaterniond quat( 0.764165,  -0.644268, -0.026487,  -0.030964);

    mTrackBall.setQuaternion(quat);
}
void MyWindow::keyboard(unsigned char key, int x, int y)
{


    dd::SkeletonPtr staubli = mWorld->getSkeleton("staubli");
    dd::SkeletonPtr tensegrity = mWorld->getSkeleton("tensegrity");
    
    Eigen::Isometry3d tensegrityTransform = tensegrity->getRootBodyNode()->getWorldTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();
    
    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond tenRot;

    
    
    double j  = 0; 
    double k = M_PI / 180.0;
    static int angX, angY, angZ = 0;
    double transStep = 0.05;
    double rotStep = 15.0; 
    switch(key)
    {
        case '1':
            j = staubli->getDof(2)->getPosition(); 
            if (j + k <= jointMax1)
            {
                staubli->getDof(2)->setPosition(j+k); 
                if(mWorld->checkCollision())
                {
                    staubli->getDof(2)->setPosition(j); 
                }
            }
            if (!collisionEnabled)
            staubli->getDof(2)->setPosition(j+k); 
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
            if (!collisionEnabled)
            staubli->getDof(3)->setPosition(j+k); 
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
            if (!collisionEnabled)
            staubli->getDof(4)->setPosition(j+k); 
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
            if (!collisionEnabled)
            staubli->getDof(5)->setPosition(j+k); 
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
            if (!collisionEnabled)
            staubli->getDof(6)->setPosition(j+k); 
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
            if (!collisionEnabled)
            staubli->getDof(7)->setPosition(j+k); 
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
            if (!collisionEnabled)
            staubli->getDof(2)->setPosition(j-k); 
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
            if (!collisionEnabled)
            staubli->getDof(3)->setPosition(j-k); 
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
            if (!collisionEnabled)
            staubli->getDof(4)->setPosition(j-k); 
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
            if (!collisionEnabled)
            staubli->getDof(5)->setPosition(j-k); 
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
            if (!collisionEnabled)
            staubli->getDof(6)->setPosition(j-k); 
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
            if (!collisionEnabled)
            staubli->getDof(7)->setPosition(j-k); 
            break;
        case 'p':
            showPath  = !showPath;
            break;
        case 't':
            showTree  = !showTree;
            break;
        case 'c':
            collisionEnabled = !collisionEnabled; 
            break;
        case 'a':
            tenMove.translation() << (tenTrans(0) + transStep), tenTrans(1), tenTrans(2);
            tenMove.rotate(tensegrityTransform.rotation());
            moveSkeleton(tensegrity, tenMove);
        
            break;
        case 'A':
            tenMove.translation() << (tenTrans(0) - transStep), tenTrans(1), tenTrans(2);
            tenMove.rotate(tensegrityTransform.rotation());
            moveSkeleton(tensegrity, tenMove);
        
            break;
        case 's':
            tenMove.translation() << tenTrans(0), (tenTrans(1) + transStep), tenTrans(2);
            tenMove.rotate(tensegrityTransform.rotation());
            moveSkeleton(tensegrity, tenMove);
        
            break;
        case 'S':
            tenMove.translation() << tenTrans(0), (tenTrans(1) - transStep), tenTrans(2);
            tenMove.rotate(tensegrityTransform.rotation());
            moveSkeleton(tensegrity, tenMove);
        
            break;
        case 'd':
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) + transStep);
            tenMove.rotate(tensegrityTransform.rotation());
            moveSkeleton(tensegrity, tenMove);
        
            break;
        case 'D':
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) - transStep);
            tenMove.rotate(tensegrityTransform.rotation());
            moveSkeleton(tensegrity, tenMove);
        
            break;
        case 'j':
            tenRot = Eigen::AngleAxisd(rotStep*M_PI/180.0, Eigen::Vector3d::UnitX())*tensegrityTransform.rotation();
            tenMove.rotate(tenRot);
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
            moveSkeleton(tensegrity, tenMove);
            angX = (int)(angX + rotStep)%360;
            break;
        case 'J':
            tenRot = Eigen::AngleAxisd(-rotStep*M_PI/180.0, Eigen::Vector3d::UnitX())*tensegrityTransform.rotation();
            tenMove.rotate(tenRot);
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
            moveSkeleton(tensegrity, tenMove);
            angX = (int)(angX - rotStep)%360;
            break;
        case 'k':
            tenRot = Eigen::AngleAxisd(rotStep*M_PI/180.0, Eigen::Vector3d::UnitY())*tensegrityTransform.rotation();
            tenMove.rotate(tenRot);
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
            moveSkeleton(tensegrity, tenMove);
            angY = (int)(angY + rotStep)%360;
            break;
        case 'K':
            tenRot = Eigen::AngleAxisd(-rotStep*M_PI/180.0, Eigen::Vector3d::UnitY())*tensegrityTransform.rotation();
            tenMove.rotate(tenRot);
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
            moveSkeleton(tensegrity, tenMove);
            angY = (int)(angY - rotStep)%360;
            break;
        case 'l':
            tenRot = Eigen::AngleAxisd(rotStep*M_PI/180.0, Eigen::Vector3d::UnitZ())*tensegrityTransform.rotation();
            tenMove.rotate(tenRot);
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
            moveSkeleton(tensegrity, tenMove);
            angZ = (int)(angZ + rotStep)%360;
            break;
        case 'L':
            tenRot = Eigen::AngleAxisd(-rotStep*M_PI/180.0, Eigen::Vector3d::UnitZ())*tensegrityTransform.rotation();
            tenMove.rotate(tenRot);
            tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
            moveSkeleton(tensegrity, tenMove);
            angZ = (int)(angZ - rotStep)%360;
            break;
        default:
            SimWindow::keyboard(key, x, y);
    }

    tenTrans = tensegrityTransform.translation();
     
    std::cout << tenTrans(0) << " " << tenTrans(1) << " " << tenTrans(2) << " " << angX << " " << angY << " " << angZ << std::endl; 

    joint1 = staubli->getDof(2)->getPosition()* 180 / M_PI; 
    joint2 = staubli->getDof(3)->getPosition()* 180 / M_PI; 
    joint3 = staubli->getDof(4)->getPosition()* 180 / M_PI; 
    joint4 = staubli->getDof(5)->getPosition()* 180 / M_PI; 
    joint5 = staubli->getDof(6)->getPosition()* 180 / M_PI; 
    joint6 = staubli->getDof(7)->getPosition()* 180 / M_PI; 

    /*
       std::cout << "\r" <<
       std::setw(8) << std::setfill(' ') << j1 << " " 
       << std::setw(8) << j2 << " " 
       << std::setw(8) << j3 << " " 
       << std::setw(8) << j4 << " " 
       << std::setw(8) << j5 << " " 
       << std::setw(8) << j6 << " " << std::flush;
     */


}

void MyWindow::drawSkels() {

    std::lock_guard<std::mutex> lock(readMutex);
    glColor3f(0.0, 0.0, 0.0);

    dg::drawStringOnScreen(0.85f, 0.65f, "J1: ");
    dg::drawStringOnScreen(0.85f, 0.625f,"J2: ");
    dg::drawStringOnScreen(0.85f, 0.6f,  "J3: ");
    dg::drawStringOnScreen(0.85f, 0.575f,"J4: ");
    dg::drawStringOnScreen(0.85f, 0.55f, "J5: ");
    dg::drawStringOnScreen(0.85f, 0.525f,"J6: ");

    glColor3f(0.0, 1.0, 0.0);


    dg::drawStringOnScreen(0.9f, 0.65f, std::to_string(joint1));
    dg::drawStringOnScreen(0.9f, 0.625f,std::to_string(joint2));
    dg::drawStringOnScreen(0.9f, 0.6f,  std::to_string(joint3));
    dg::drawStringOnScreen(0.9f, 0.575f,std::to_string(joint4));
    dg::drawStringOnScreen(0.9f, 0.55f, std::to_string(joint5));
    dg::drawStringOnScreen(0.9f, 0.525f,std::to_string(joint6));


    dd::SkeletonPtr staubli = mWorld->getSkeleton("staubli");
    Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
    Eigen::Vector3d tr = transform.translation();
    Eigen::Quaterniond quat1(transform.rotation());

    glColor3f(0.0, 0.0, 0.0);
    dg::drawStringOnScreen(0.85f, 0.475f, "X: ");
    dg::drawStringOnScreen(0.85f, 0.45f , "Y: ");
    dg::drawStringOnScreen(0.85f, 0.425f, "X: ");
    dg::drawStringOnScreen(0.85f, 0.4f  , "qw: ");
    dg::drawStringOnScreen(0.85f, 0.375f, "qx: ");
    dg::drawStringOnScreen(0.85f, 0.35f , "qy: ");
    dg::drawStringOnScreen(0.85f, 0.325f, "qx: ");

    glColor3f(0.0, 1.0, 0.0);
    dg::drawStringOnScreen(0.9f, 0.475f, std::to_string(tr(0)));
    dg::drawStringOnScreen(0.9f, 0.45f , std::to_string(tr(1)));
    dg::drawStringOnScreen(0.9f, 0.425f, std::to_string(tr(2)));
    dg::drawStringOnScreen(0.9f, 0.4f  , std::to_string(quat1.w()));
    dg::drawStringOnScreen(0.9f, 0.375f, std::to_string(quat1.x()));
    dg::drawStringOnScreen(0.9f, 0.35f , std::to_string(quat1.y()));
    dg::drawStringOnScreen(0.9f, 0.325f, std::to_string(quat1.z()));

    Eigen::Matrix3d rotTrackBall = mTrackBall.getRotationMatrix();
    Eigen::Quaterniond quatTrackBall(rotTrackBall);

    dg::drawStringOnScreen(0.05f, 0.65f, std::to_string(-mTrans(0)));
    dg::drawStringOnScreen(0.05f, 0.625f , std::to_string(-mTrans(1)));
    dg::drawStringOnScreen(0.05f, 0.6f, std::to_string(-mTrans(2)));
    dg::drawStringOnScreen(0.05f, 0.575f,   std::to_string(quatTrackBall.w()));
    dg::drawStringOnScreen(0.05f, 0.55f, std::to_string(quatTrackBall.x()));
    dg::drawStringOnScreen(0.05f, 0.525f,  std::to_string(quatTrackBall.y()));
    dg::drawStringOnScreen(0.05f, 0.5f, std::to_string(quatTrackBall.z()));

    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


    if (showPath)  
    {
        double x,y,z,ign;
        glLineWidth(3); 
        glBegin(GL_LINES);
        glColor3f(0, 0, 0);
        std::ifstream fin_to("endeffector1.txt");
        fin_to >> x >> y >> z >> ign >> ign >> ign >> ign;
        glVertex3f(x, y, z);
        while(!fin_to.eof()){
            fin_to >> x >> y >> z >> ign >> ign >> ign >> ign;

            glVertex3f(x, y, z);
            glVertex3f(x, y, z);

        }
        glVertex3f(x, y, z);
        glEnd();

/*
        glLineWidth(1); 
        glColor3f(0.33, 0.66, 0.99);
        glBegin(GL_LINES);

        int xmax = 2;
        int ymax = 2;

        for (int ii = -xmax*10; ii <= xmax*10; ii+=1) 
        {
            glVertex3f(ii * 1.0 / 10.0, transStep, -ymax);

            glVertex3f(ii * 1.0 / 10.0, transStep, ymax);
        }
        for (int ii = -ymax*0; ii <= ymax*10; ii+=1) 
        {
            glVertex3f(-xmax, transStep,ii * 1.0 / 10.0);
            glVertex3f(xmax,transStep, ii * 1.0 / 10.0);
        }
        //float rx, ry, rz, rw;


        glEnd();
*/
    }
    if (showTree)
    {
        
        using std::vector;
        using std::string;
        using std::size_t;
        using std::ifstream;
        double x1,y1,z1;
        
        static vector <vector <double>> solution_path;
        if (solution_path.size() == 0)  
        {
            string fname{"edges.txt"};
            ifstream fin(fname);
            while(fin >> x1 >> y1 >> z1)
            {
                solution_path.push_back(vector<double>{x1, y1, z1});
            }
        }
        
        glLineWidth(1); 
        glBegin(GL_LINES);
        glColor3f(1, 1, 0);
        for(size_t i = 0; i < solution_path.size() - 1; i++)
        {
            glVertex3d(solution_path[i][0], solution_path[i][1], solution_path[i][2]);
            glVertex3d(solution_path[i+1][0], solution_path[i+1][1], solution_path[i+1][2]);

        }
        glEnd();
    }
    SimWindow::drawSkels();

}

void MyWindow::setViewTrack(double j1,double j2,double j3,double j4,double j5,double j6){

    joint1 = j1 * 180 / M_PI; 
    joint2 = j2 * 180 / M_PI; 
    joint3 = j3 * 180 / M_PI; 
    joint4 = j4 * 180 / M_PI; 
    joint5 = j5 * 180 / M_PI; 
    joint6 = j6 * 180 / M_PI; 

}
