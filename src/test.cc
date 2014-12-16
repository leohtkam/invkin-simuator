#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <string>
#include <sstream>
#include <limits>

#include "invkin.h"

using namespace std;

void
BallJoint::move() {}

void
BallJoint::render() {}

void
System::render() {}

/* Test the jacobian of a system with only one arm. */
bool testJacobian1() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 3);
    solution << 0, -6.0f, 0,
                6.0f, 0, 0,
                0, 0, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 3);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian1 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian1 ..... PASS" << endl;
    return true;
}


/* Test the jacobian of a system with only one arm at some angle. */
bool testJacobian2() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));
    joints[0]->update(0, 0.5 * PI, 0);

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 3);
    solution << 0, 0, 0,
                0, 0, -6.0f,
                0, 6.0f, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 3);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian2 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian2 ..... PASS" << endl;
    return true;
}

/* Test the jacobian of a system with two arms. */
bool testJacobian3() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));
    joints.push_back(new BallJoint(5.0f));

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 6);
    solution << 0, -11.0f, 0, 0, -5.0f, 0,
                11.0f, 0, 0, 5.0f, 0, 0,
                0, 0, 0, 0, 0, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 6);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian3 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian3 ..... PASS" << endl;
    return true;
}

/* Test the jacobian of a system with two arms with first arm at some angle. */
bool testJacobian4() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));
    joints.push_back(new BallJoint(5.0f));
    joints[0]->update(0, 0.5 * PI, 0);

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 6);
    solution << 0, 0, 0, 0, 0, 0,
                0, 0, -11.0f, 5.0f, 0, 0,
                0, 11.0f, 0, 0, 5.0f, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 6);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian4 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian4 ..... PASS" << endl;
    return true;
}

/* Test the jacobian of a system with two arms with second arm at some angle. */
bool testJacobian5() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));
    joints.push_back(new BallJoint(5.0f));
    joints[1]->update(0, 0.5 * PI, 0);

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 6);
    solution << 0, -6.0f, 0, 0, 0, 0,
                6.0f, 0, -5.0f, 0, 0, -5.0f,
                0, 5.0f, 0, 0, 5.0f, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 6);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian5 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian5 ..... PASS" << endl;
    return true;
}

/* Test the jacobian of a system with two arms with both arms at some angle. */
bool testJacobian6() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));
    joints.push_back(new BallJoint(5.0f));
    joints[0]->update(0, 0.5 * PI, 0);
    joints[1]->update(0, 0.5 * PI, 0);

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 6);
    solution << 0, 5.0f, 0, 0, 5.0f, 0,
                -5.0f, 0, -6.0f, 0, 0, -5.0f,
                0, 6.0f, 0, 0, 0, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 6);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian6 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian6 ..... PASS" << endl;
    return true;
}

/* Test the jacobian of a system with two arms with both arms at some angle. */
bool testJacobian7() {
    vector<Joint*> joints;
    joints.push_back(new BallJoint(6.0f));
    joints.push_back(new BallJoint(5.0f));
    joints[0]->update(0, 0.5 * PI, 0);
    joints[1]->update(0, -0.5 * PI, 0);

    System sys;
    sys.addJoints(joints);

    Eigen::MatrixXf solution(3, 6);
    solution << 0, -5.0f, 0, 0, -5.0f, 0,
                5.0f, 0, -6.0f, 0, 0, 5.0f,
                0, 6.0f, 0, 0, 0, 0;
    if (debug)
        cout << "Expected jacobian:\n" << solution << endl; 

    Eigen::MatrixXf J(3, 6);
    sys.getJacobian(J);
    if (debug)
        cout << "Actual jacobian:\n" << J << endl; 

    if (!solution.isApprox(J)) {
        cout << "testJacobian7 ..... FAIL" << endl;
        return false;
    }

    cout << "testJacobian7 ..... PASS" << endl;
    return true;
}

int main(int argc, char *argv[]) {
    if (argc > 1) {
        string s(argv[1]);
        stringstream ss(s);
        ss >> debug;
    }

    bool allPassed = true;
    allPassed &= testJacobian1();
    allPassed &= testJacobian2();
    allPassed &= testJacobian3();
    allPassed &= testJacobian4();
    allPassed &= testJacobian5();
    allPassed &= testJacobian6();
    allPassed &= testJacobian7();

    /*Eigen::Transform<float, 3, Eigen::Affine> t;
    t.setIdentity();
    t.rotate(Eigen::AngleAxis<float>(0.5*PI, Eigen::Vector3f(0, 1, 0)));
    Eigen::MatrixXf m(3, 3);
    m << 0, -6, 0,
        6, 0, 0,
        0, 0, 0;
    cout << t.matrix().block(0,0,3,3) * m << endl;*/

    cout << endl;
    if (allPassed)
        cout << "PASS all tests!" << endl;
    else
        cout << "FAIL one or more tests!" << endl;
}