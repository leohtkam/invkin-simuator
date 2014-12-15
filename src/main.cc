#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <time.h>
#include <math.h>

//Libraries I added
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>

//Header files
#include "invkin.h"

using namespace std;

class BoundingSphere;
float clampAngle(float, float);
float degToRad(float);
float radToDeg(float);

/* Global variables */
char title[] = "Inverse Kinematic Simulator";
BoundingSphere* bsphere;
System sys;
float currZoom = 0.0f;
float amountZoom = 0.1f;
float xTrans = 0.0f;
float yTrans = 0.0f;
float amountTrans = 0.05f;
float yRot = 0.0f;
float xRot = 0.0f;
float amountRot = 5;  //Degree
GLfloat aspect;


class BoundingSphere {
public:
    Eigen::Vector3f pmin, pmax;
    float radius;

    BoundingSphere() {
        pmin[0] = numeric_limits<float>::infinity();
        pmin[1] = numeric_limits<float>::infinity();
        pmin[2] = numeric_limits<float>::infinity();
        pmax[0] = -1.0f * numeric_limits<float>::infinity();
        pmax[1] = -1.0f * numeric_limits<float>::infinity();
        pmax[2] = -1.0f * numeric_limits<float>::infinity();
        radius = 0.0f;
    }
};

void
BallJoint::move() {
    glRotatef(radToDeg(currRot.x()), 1.0f, 0.0f, 0.0f);
    glRotatef(radToDeg(currRot.y()), 0.0f, 1.0f, 0.0f);
    glRotatef(radToDeg(currRot.z()), 0.0f, 0.0f, 1.0f);
}

void
BallJoint::render() {
    glColor3f(0.1, 0.3, 1.0);
    glutSolidCone(1.0f, length, 20, 20);        
}

void
System::render() {
    for (int i = 0; i < joints.size(); i++) {
        joints[i]->move();
        joints[i]->render();
        glTranslatef(0.0f, 0.0f, joints[i]->length);
    }

    glColor3f(1.0f, 0.0f, 0.0f);
    glutSolidSphere(3.0f, 20, 20);
}

void updateSystem() {

}

void renderSystem() {
    sys.render();
}

/* Initialize OpenGL Graphics */
void initGL() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClearDepth(1.0f);                   // Set background depth to farthest
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);    //Double sided lighting.
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    //Enable positional light at (1, 1, 1)
    GLfloat light_ambient[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_position[] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}
 
/* Handler for window-repaint event. Called back when the window first appears and
   whenever the window needs to be re-painted. */
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, currZoom * bsphere->radius); // user input zoom.
    glTranslatef(0.0f, 0.0f, -1.0f * 1.5 * (2 * bsphere->radius / tan(PI / 4.0f)));    // initial zoom.
    glTranslatef(xTrans, yTrans, 0.0f);          // user input translation.
    glRotatef(xRot, 1.0f, 0.0f, 0.0f);           // user input rotation.
    glRotatef(yRot, 0.0f, 1.0f, 0.0f);           // user input rotation.

    updateSystem();
    renderSystem();

    glFlush();
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}
 
/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
    // Compute aspect ratio of the new window
    if (height == 0) height = 1;                // To prevent divide by 0
    aspect = (GLfloat)width / (GLfloat)height;

    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping volume to match the viewport
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset
    // Enable perspective projection with fovy, aspect, zNear and zFar
    gluPerspective(45.0f, aspect, 0.1f, 99999999.0f);
}
 


/* Parse commandline arguments. */
/*void parseCommandLineOptions(int argc, char *argv[]) {
    string s;
    if (argc < 3) {
        cerr << "Not enough arguemnts, need at least .bez file name and parameter." << endl;
        exit(1);
    }
    filename = argv[1];
    s = string(argv[2]);
    stringstream ss(s);
    ss >> parameter;
    if (ss.fail()) {
        cerr << "Invalid value for parameter, should be float: " << s << endl;
        exit(1);
    }
    if (argc > 3) {
        for (int i = 3; i < argc; i++) {
            s = string(argv[i]);
            if (s == "-a")
                isUniform = false;
            else if (s == "-d")
                debug = true;
            else 
                cout << "Ignored unknown option " << s << endl;
        }
    }
}*/


/* Handler for keystroke */
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case '+':
        case '=':
            currZoom += amountZoom;
            break;
        case '-':
            currZoom -= amountZoom;
            break;
        default:
            return;
    }
    glutPostRedisplay();
}

void SpecialInput(int key, int x, int y)
{
    int mod = glutGetModifiers();

    if (mod == GLUT_ACTIVE_SHIFT) {
        switch(key)
        {
        case GLUT_KEY_UP:
            yTrans += amountTrans;
            break;
        case GLUT_KEY_DOWN:
            yTrans -= amountTrans;
            break;
        case GLUT_KEY_LEFT:
            xTrans -= amountTrans;
            break;
        case GLUT_KEY_RIGHT:
            xTrans += amountTrans;
            break;
        }
    } else {
        switch(key)
        {
        case GLUT_KEY_UP:
            xRot -= amountRot;
            break;
        case GLUT_KEY_DOWN:
            xRot += amountRot;
            break;
        case GLUT_KEY_LEFT:
            yRot -= amountRot;
            break;
        case GLUT_KEY_RIGHT:
            yRot += amountRot;
            break;
        }

        if (yRot >= 360)
            yRot = yRot - 360;
        if (xRot >= 360)
            xRot = xRot - 360;
    }
    glutPostRedisplay();
}

void setBoundingSphere(BoundingSphere *bsphere, System sys) {
    bsphere->radius = 0.0f;
    for (int i = 0; i < sys.joints.size(); i++)
        bsphere->radius += sys.joints[i]->length;
}

void printout() {
    cout << "Number of joints = " << sys.joints.size() << endl;
    cout << "Radius of bounding sphere = " << bsphere->radius << endl;
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {
    glutInit(&argc, argv);            // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB); // Enable double buffered and depth mode
    glutInitWindowSize(500, 500);   // Set the window's initial width & height
    aspect = (GLfloat)500 / (GLfloat)500;
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutCreateWindow(title);          // Create window with the given title
    glutDisplayFunc(display);       // Register callback handler for window re-paint event
    //glutIdleFunc(idle);
    glutReshapeFunc(reshape);       // Register callback handler for window re-size event
    glutKeyboardFunc(keyboard);     // Register callback handler for keystroke
    glutSpecialFunc(SpecialInput);  // Register callback handler for special keystroke
    initGL();                       // Our own OpenGL initialization
 
    vector<Joint*> joints;   
    //parseCommandLineOptions(argc, argv);
    joints.push_back(new BallJoint(5.0f));
    joints.push_back(new BallJoint(10.0f));
    joints.push_back(new BallJoint(5.0f));
    joints.push_back(new BallJoint(15.0f));
    ((BallJoint*) joints[0])->currRot<< 0, 0.5 * PI, 0;

    sys.joints = joints;

    bsphere = new BoundingSphere();
    setBoundingSphere(bsphere, sys);
    printout();

    glutMainLoop();                 // Enter the infinite event-processing loop
    return 0;
}