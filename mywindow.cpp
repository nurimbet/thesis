#include "mywindow.h"
    #include <fstream>
#include <iostream>
#include "util.h"

namespace dd = dart::dynamics;

constexpr double jointMax[6] = {3.1416, 2.5744, 2.5307, 4.7124, 2.4435, 4.7124};
constexpr double jointMin[6] = {-3.1416, -2.2689, -2.5307, -4.7124, -2.0071, -4.7124};

double joint1, joint2, joint3, joint4, joint5, joint6 = 0;
bool showPath = false;
bool showTree = false;
bool collisionEnabled = true;
bool showAxes = true;
int idx = 0;

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
    double gojoint[6]; 
    static std::ifstream jointfile("joints.txt");

    switch(key)
    {
        case '0':
            
            for (int ii = 1; ii <=6; ii++)
            {
                staubli->getDof(ii + 1)->setPosition(0); 
            }
            break;
        case '9':
            if (jointfile.eof())
            {
                std::cout << "lol" << std::endl;
                jointfile.close();
                jointfile.open("joints.txt");
            }
            

            jointfile >> gojoint[0] >> gojoint[1]>> gojoint[2]>> gojoint[3]>> gojoint[4]>> gojoint[5];
            for (int ii = 1; ii <=6; ii++)
            {
                staubli->getDof(ii + 1)->setPosition(gojoint[ii-1]*M_PI/180.0); 
            }

            break;
        case '8':
            idx = idx + 1;
            if (idx > 8)
            {
                idx = 0;
            }
            std::cout << idx + 1 << std::endl;
            break;
        case '7':
            idx = idx - 1;
            if(idx < 0)
            {
                idx = 8;
            }
            std::cout << idx + 1 << std::endl;
            break;
        case '1':
            moveJoint(1,true);
            break;
        case '2':
            moveJoint(2,true);
            break;
        case '3':
            moveJoint(3,true);
            break;
        case '4':
            moveJoint(4,true);
            break;
        case '5':
            moveJoint(5,true);
            break;
        case '6':
            moveJoint(6,true);
            break;
        case '!':
            moveJoint(1,false);
            break;
        case '@':
            moveJoint(2,false);
            break;
        case '#':
            moveJoint(3,false);
            break;
        case '$':
            moveJoint(4,false);
            break;
        case '%':
            moveJoint(5,false);
            break;
        case '^':
            moveJoint(6,false);
            break;
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
            translateTensegrity(0, true);
            break;
        case 'A':
            translateTensegrity(0, false);
            break;
        case 's':
            translateTensegrity(1, true);
            break;
        case 'S':
            translateTensegrity(1, false);
            break;
        case 'd':
            translateTensegrity(2, true);
            break;
        case 'D':
            translateTensegrity(2, false);
            break;
        case 'j':
            rotateTensegrity(0, true);
            break;
        case 'J':
            rotateTensegrity(0, false);
            break;
        case 'k':
            rotateTensegrity(1, true);
            break;
        case 'K':
            rotateTensegrity(1, false);
            break;
        case 'l':
            rotateTensegrity(2, true);
            break;
        case 'L':
            rotateTensegrity(2, false);
            break;
        case '+':
            showAxes = !showAxes;
            break;
        default:
            SimWindow::keyboard(key, x, y);
    }


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
        glColor3f(0, 1, 0);
        for(size_t i = 0; i < solution_path.size() - 1; i++)
        {
            glVertex3d(solution_path[i][0], solution_path[i][1], solution_path[i][2]);
            glVertex3d(solution_path[i+1][0], solution_path[i+1][1], solution_path[i+1][2]);

        }
        glEnd();
    }

    if (showAxes)
    {
        
        Eigen::Matrix3d rot_tr = transform.rotation();
        drawAxes(tr, rot_tr);

        double xs = 30.0 / 1000.0;
        double ys = -60.0 / 1000.0;
        double zs = 95.0 / 1000.0;
        Eigen::Isometry3d transform1 = staubli->getBodyNode("gripper")->getTransform();
        Eigen::Vector3d tr1 = transform1.translation();
        
        Eigen::Matrix3d rot_tr1 = transform1.rotation();//*Eigen::AngleAxisd(90*M_PI/180.0, Eigen::Vector3d::UnitZ());
        tr1(0) += xs * rot_tr1(0, 0) + ys * rot_tr1(0, 1) + zs * rot_tr1(0, 2);
        tr1(1) += xs * rot_tr1(1, 0) + ys * rot_tr1(1, 1) + zs * rot_tr1(1, 2);
        tr1(2) += xs * rot_tr1(2, 0) + ys * rot_tr1(2, 1) + zs * rot_tr1(2, 2);
        
        drawAxes(tr1, rot_tr1);


        dd::SkeletonPtr tensegrity = mWorld->getSkeleton("tensegrity");
        
        //Eigen::Isometry3d tensegrityTransform = tensegrity->getRootBodyNode()->getWorldTransform();
        Eigen::Isometry3d tensegrityTransform =tensegrity->getBodyNode("attach" + std::to_string(idx+1))->getTransform();
        Eigen::Vector3d tenTrans = tensegrityTransform.translation();
        Eigen::Matrix3d rot_ten = tensegrityTransform.rotation();
        drawAxes(tenTrans, rot_ten);

        tensegrityTransform =tensegrity->getBodyNode("detach" + std::to_string(idx+1))->getTransform();
        tenTrans = tensegrityTransform.translation();
        rot_ten = tensegrityTransform.rotation();
        drawAxes(tenTrans, rot_ten);
        
        
        tensegrityTransform =tensegrity->getBodyNode("tightener" + std::to_string(idx+1))->getTransform();
        tenTrans = tensegrityTransform.translation();
        rot_ten = tensegrityTransform.rotation();
        drawAxes(tenTrans, rot_ten);


        tensegrityTransform =tensegrity->getBodyNode("tightener" + std::to_string(idx+1))->getTransform();
        tenTrans = tensegrityTransform.translation();
        rot_ten = tensegrityTransform.rotation();
        drawAxes(tenTrans, rot_ten);


        tensegrityTransform =tensegrity->getBodyNode("pulley" + std::to_string(idx+1))->getTransform();
        tenTrans = tensegrityTransform.translation();
        rot_ten = tensegrityTransform.rotation();
        drawAxes(tenTrans, rot_ten);


        tensegrityTransform =tensegrity->getBodyNode("tendon" + std::to_string(idx+1))->getTransform();
        tenTrans = tensegrityTransform.translation();
        rot_ten = tensegrityTransform.rotation();
        //drawAxes(tenTrans, rot_ten);
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
void MyWindow::drawAxes(const Eigen::Vector3d &tr, const Eigen::Matrix3d &rot)
{

        glLineWidth(2); 
        glBegin(GL_LINES);
        glColor3f(0, 0, 1);
        

        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tr(0) + 0.25*rot(0,2), tr(1) + 0.25*rot(1,2), tr(2)+0.25*rot(2,2));
        glColor3f(0, 1, 0);
        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tr(0) + 0.25*rot(0,1), tr(1) + 0.25*rot(1,1), tr(2)+0.25*rot(2,1));
        glColor3f(1, 0, 0);
        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tr(0) + 0.25*rot(0,0), tr(1) + 0.25*rot(1,0), tr(2)+0.25*rot(2,0));

        glEnd();
}

void MyWindow::moveJoint(int numJoint, bool positive)
{

    dd::SkeletonPtr staubli = mWorld->getSkeleton("staubli");
    double j = staubli->getDof(numJoint+1)->getPosition(); 
    double k = M_PI / 180.0;
    double l = 0.0;
    if(positive)
    {
        l = j + k;
    }
    else
    {
        l = j - k; 
    }
    if (l <= jointMax[numJoint - 1])
    {
        staubli->getDof(numJoint+1)->setPosition(l); 
        if(mWorld->checkCollision())
        {
            staubli->getDof(numJoint+1)->setPosition(j); 
        }
    }
    if (!collisionEnabled)
        staubli->getDof(numJoint + 1)->setPosition(l); 
}

void MyWindow::translateTensegrity(int axis, bool positive)
{
    dd::SkeletonPtr tensegrity = mWorld->getSkeleton("tensegrity");
    
    Eigen::Isometry3d tensegrityTransform = tensegrity->getRootBodyNode()->getWorldTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();
    
    Eigen::Isometry3d tenMove;
    
    tenMove = Eigen::Isometry3d::Identity();
    double transStep = 0.05;
    double trans[3] = {0.0,0.0,0.0};
    trans[axis] = transStep;
   
    if(positive)
    { 
        tenMove.translation() << (tenTrans(0) + trans[0]), tenTrans(1) + trans[1], tenTrans(2) + trans[2];
    }
    else
    {
        tenMove.translation() << (tenTrans(0) - trans[0]), tenTrans(1) - trans[1], tenTrans(2) - trans[2];
    }
    tenMove.rotate(tensegrityTransform.rotation());
    moveSkeleton(tensegrity, tenMove);
    
}

void MyWindow::rotateTensegrity(int axis, bool positive)
{
    dd::SkeletonPtr tensegrity = mWorld->getSkeleton("tensegrity");
    
    Eigen::Isometry3d tensegrityTransform = tensegrity->getRootBodyNode()->getWorldTransform();
    Eigen::Vector3d tenTrans = tensegrityTransform.translation();
    
    Eigen::Isometry3d tenMove;
    tenMove = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond tenRot;

    static int angX, angY, angZ = 0;
    double rotStep = 15.0; 

    if(!positive)
    {
        rotStep = -rotStep;
    }

    if(axis == 0)
    {
        tenRot = Eigen::AngleAxisd(rotStep*M_PI/180.0, Eigen::Vector3d::UnitX())*tensegrityTransform.rotation();
        angX = (int)(angX + rotStep)%360;
    }
    else if(axis == 1)
    {
        tenRot = Eigen::AngleAxisd(rotStep*M_PI/180.0, Eigen::Vector3d::UnitY())*tensegrityTransform.rotation();
        angY = (int)(angY + rotStep)%360;
    }
    else if(axis == 2)
    {
        tenRot = Eigen::AngleAxisd(rotStep*M_PI/180.0, Eigen::Vector3d::UnitZ())*tensegrityTransform.rotation();
        angZ = (int)(angZ + rotStep)%360;
    }

    tenMove.rotate(tenRot);
    tenMove.translation() << tenTrans(0), tenTrans(1), (tenTrans(2) );
    moveSkeleton(tensegrity, tenMove);
    
    tenTrans = tensegrityTransform.translation();

    std::cout << tenTrans(0) << " " << tenTrans(1) << " " << tenTrans(2) << " " << angX << " " << angY << " " << angZ << std::endl; 
}
