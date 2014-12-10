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
#include "parse.h"
#include "data.h"

#define PI 3.14159265

using namespace std;

 
/* Global variables */
char title[] = "Inverse Kinematic Simulator";


/* Initialize OpenGL Graphics */
void initGL() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClearDepth(1.0f);                   // Set background depth to farthest
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);    //Double sided lighting.
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    //Enable positional light at (1, 1, 1)
    GLfloat light_ambient[] = {0.1, 0.3, 1.0, 1.0};
    GLfloat light_diffuse[] = {0.1, 0.3, 1.0, 1.0};
    GLfloat light_position[] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}
 
/* Handler for window-repaint event. Called back when the window first appears and
   whenever the window needs to be re-painted. */
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    // Render a color-cube consisting of 6 quads with different colors
    glLoadIdentity();                 // Reset the model-view matrix
    glTranslatef(0.0f, 0.0f, currZoom * radius); // user input zoom.
    glTranslatef(0.0f, 0.0f, -1.0f * 1.5 * (2 * radius / tan(PI / 4.0f)));    // initial zoom.
    glTranslatef(xTrans, yTrans, 0.0f);          // user input translation.
    glRotatef(yRot, 1.0f, 0.0f, 0.0f);           // user input rotation.
    glRotatef(xRot, 0.0f, 1.0f, 0.0f);           // user input rotation.
    glTranslatef(-1 * centerPos.x(), -1 * centerPos.y(), -1 * centerPos.z());  // Move object to origin.
    
    glColor3f(0.1f, 0.3f, 0.8f);
    if (isSmoothShading)
        glShadeModel(GL_SMOOTH);
    else
        glShadeModel(GL_FLAT);
    if (isWireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (isHiddenLine && isWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(0.1f, 0.3f, 0.8f);
        glCallList(1);

        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        glColor3f(0.0f, 0.0f, 0.0f);
        glCallList(1);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glEnable(GL_LIGHTING);
    } else {
        glCallList(1);                // Render the surface.
    }
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
 
/*void idle(){
    currDeg += amount;
   if (currDeg >= 360)
        currDeg -= 360;
    glutPostRedisplay();
}*/


/* Parse commandline arguments. */
void parseCommandLineOptions(int argc, char *argv[]) {
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
}


/* Generating a bezier surface and pass the info to OpenGL for rendering. */
void generateSurface() {
    if (debug) {
        cout << "Parsing file " << filename << endl;
        cout << "isUniform = " << isUniform << endl;
    }
    vector<Patch*> patches;
    parse(patches, filename);
    glNewList(1, GL_COMPILE);
    if (isUniform) {
        if (debug)
            cout << "Uniform subdivision." << endl;
        glBegin(GL_QUADS);
    } else {
        if (debug)
            cout << "Adaptive subdivision." << endl;
        glBegin(GL_TRIANGLES);
    }
    Eigen::Vector3f minP, maxP;
    minP << numeric_limits<float>::infinity(), numeric_limits<float>::infinity(), 
        numeric_limits<float>::infinity();
    maxP << -1.0 * numeric_limits<float>::infinity(), -1.0 * numeric_limits<float>::infinity(),
        -1.0 * numeric_limits<float>::infinity();
    for (int i = 0; i < patches.size(); i++) {
        if (isUniform)
            subdivisionPatch(*patches[i], parameter);
        else
            adaptiveTessellation(*patches[i], parameter);
        minP[0] = std::min(minP[0], patches[i]->minP[0]);
        minP[1] = std::min(minP[1], patches[i]->minP[1]);
        minP[2] = std::min(minP[2], patches[i]->minP[2]);
        maxP[0] = std::max(maxP[0], patches[i]->maxP[0]);
        maxP[1] = std::max(maxP[1], patches[i]->maxP[1]);
        maxP[2] = std::max(maxP[2], patches[i]->maxP[2]);
    }
    glEnd();
    glEndList();
    centerPos = (minP + maxP) / 2.0f;
    radius = (maxP - centerPos).norm();

    if (isUniform) {
        cout << "Uniform subdivision: ";
    } else {
        cout << "Adaptive subdivision: ";
    }
    cout << numDone << " faces." << endl;
    cout << "Radius of bounding sphere = " << radius << endl;

    if (debug) {
        cout << "minP = (" << minP.x() << ", " << minP.y() << ", " << minP.z() << ")" << endl;
        cout << "maxP = (" << maxP.x() << ", " << maxP.y() << ", " << maxP.z() << ")" << endl;
        cout << "centerPos = (" << centerPos.x() << ", " << centerPos.y() << ", " << centerPos.z() << ")" << endl;
        cout << "radius = " << radius << endl;
    }
}

/* Handler for keystroke */
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 's':
            isSmoothShading = !isSmoothShading;
            break;
        case 'w':
            isWireframe = !isWireframe;
            break;
        case 'h':
            isHiddenLine = !isHiddenLine;
            break;
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
            yRot -= amountRot;
            break;
        case GLUT_KEY_DOWN:
            yRot += amountRot;
            break;
        case GLUT_KEY_LEFT:
            xRot -= amountRot;
            break;
        case GLUT_KEY_RIGHT:
            xRot += amountRot;
            break;
        }

        if (xRot >= 360)
            xRot = xRot - 360;
        if (yRot >= 360)
            yRot = yRot - 360;
    }
    glutPostRedisplay();
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
    generateSurface();
    glutMainLoop();                 // Enter the infinite event-processing loop
    return 0;
}