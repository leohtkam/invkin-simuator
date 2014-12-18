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
string filename = "testCircle.invkin";
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
bool paused = false;
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
    if (!sys.path.isEmpty()) {
        //Drawing the path.
        glColor3f(1.0f, 0.0f, 0.0f);
        sys.path.render();
        //Drawing the current point on the path.
        if (sys.goalTooFarAway) {
            //Red color if goal is unreachable by this system.
            glColor3f(1.0f, 0.0f, 0.0f);
        } else {
            //Else color it green.
            glColor3f(0.0f, 1.0f, 0.0f);
        }
        glTranslatef(sys.currGoal->x(), sys.currGoal->y(), sys.currGoal->z());
        glutSolidSphere(0.5f, 20, 20);
        glTranslatef(-sys.currGoal->x(), -sys.currGoal->y(), -sys.currGoal->z());
    }

    //Draw the arms
    for (int i = 0; i < joints.size(); i++) {
        joints[i]->move();
        joints[i]->render();
        glTranslatef(0.0f, 0.0f, joints[i]->length);
    }

}

void
Path::render() {
    if (this->isEmpty())
        return;
    Eigen::Vector3f* v;
    glBegin(GL_LINES);
        for (int i = 0; i < this->points.size(); i++) {
            v = this->points[i];
            glVertex3f(v->x(), v->y(), v->z());
        }
        v = this->points[0];
        glVertex3f(v->x(), v->y(), v->z());
    glEnd();
}

bool pressed = false;
void updateSystem() {
    /*if (!pressed)
        return;
    pressed = false;*/
    sys.currGoal = sys.path.curr();
    sys.update(*sys.currGoal);
    sys.path.next();
    cout << endl;
    if (debug > 1) {
        cout << "Current goal =" << endl << *sys.currGoal << endl;
        cout << "Current endpoint =" << endl << sys.endpoint << endl;
        cout << "Current rotation =" << endl << ((BallJoint*)sys.joints[0])->currRot << endl;
    }
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

    //Do not update system if the simulation is paused.
    if (!paused)
        updateSystem();
    renderSystem();

    glFlush();
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
    glutPostRedisplay();
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
void parseCommandLineOptions(int argc, char *argv[]) {
    if (argc > 1) {
        string s;
        for (int i = 1; i < argc; i++) {
            s = string(argv[i]);
            if (s == "-p") {
                paused = true;
            } else if (s == "--debug" || s == "-d") {
                string s2(argv[i+1]);
                stringstream ss(s2);
                i++;
                if (ss.fail()) {
                    cerr << "Invalid format for '-d/--debug' option." << endl;
                } else
                    ss >> debug;
            }else {
                filename = argv[i];
            }
        }
    }
}

Path parsePathFile(const char* inputfile) {
    fstream f(inputfile);
    string line;

    if (!f.is_open()) {
        cerr << "Failed to open input file '" << inputfile << "'." << endl;
        exit(1);
    }
    Path path;
    float x, y, z;
    int linecount = 1;
    while(getline(f, line)) {
        stringstream ss(line);
        if (line.at(0) == '#') {
            //Do nothing
        } else {
            ss >> x >> y >> z;
            if (ss.fail()) {
                cerr << "Error parsing path file '" << inputfile << "' on line " << linecount << endl;
                exit(1);
            }
            path.points.push_back(new Eigen::Vector3f(x, y, z));
        }
        linecount++;
    }
    f.close();
    cout << path << endl;
    return path;
}

vector<Joint*> parseJointFile(const char* inputfile) {
    fstream f(inputfile);
    string line;

    if (!f.is_open()) {
        cerr << "Failed to open input file '" << inputfile << "'." << endl;
        exit(1);
    }
    vector<Joint*> joints;
    float length, rotx, roty, rotz;
    string s;
    int linecount = 1;
    while(getline(f, line)) {
        stringstream ss(line);
        ss >> s;
        if (line.at(0) == '#') {
            //Do nothing
        }
        //Ball joint
        else if (s == "bj") {
            ss >> length >> rotx >> roty >> rotz;
            if (ss.fail()) {
                cerr << "Error parsing Joint file '" << inputfile << "' on line " << linecount << endl;
                exit(1);
            }
            BallJoint* bj = new BallJoint(length);
            if (rotx != 0 || roty != 0 || rotz != 0)
                bj->update(rotx, roty, rotz);
            joints.push_back(bj);
            cout << *bj << endl;
        } else {
            cout << "Ignoring unknown joint type on line " << linecount << ": '" << line << "'." << endl;
        }
        linecount++;
    }
    f.close();
    return joints;
}

void parseConfigFile(const char* inputfile) {
    fstream f(inputfile);
    string line;

    if (!f.is_open()) {
        cerr << "Failed to open input file '" << inputfile << "'." << endl;
        exit(1);
    }
    string pathfile;
    string jointfile;

    getline(f, line);
    while(line.at(0) == '#') {
        getline(f, line);
    }
    stringstream ss(line);
    ss >> pathfile;
    getline(f, line);
    ss.str("");
    ss << line;
    ss >> jointfile;

    f.close();
    Path path = parsePathFile(pathfile.c_str());
    vector<Joint*> joints = parseJointFile(jointfile.c_str());
    sys.initialize(joints, path);
}

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
        case 'p':
        case 'P':
            paused = !paused;
            break;
        case 'n':
            pressed = true;
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
    cout << "Debug level = " << debug << endl;
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
 
    parseCommandLineOptions(argc, argv);
    parseConfigFile(filename.c_str());

    bsphere = new BoundingSphere();
    setBoundingSphere(bsphere, sys);
    printout();

    glutMainLoop();                 // Enter the infinite event-processing loop
    return 0;
}