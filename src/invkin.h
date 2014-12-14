#ifndef INVKIN_H
#define INVKIN_H

#include <vector>
#include <Eigen/Eigen>

#define PI 3.14159265

//Forward method declaration
float clampAngle(float, float);
float degToRad(float);
float radToDeg(float);

class Joint {
public:
    float length;
    Eigen::Transform<float, 3, Eigen::Affine> bodyToWorld;
    Eigen::Transform<float, 3, Eigen::Affine> worldToBody;
    int numColumns;

    Joint(): length(0), numColumns(0)
    {
        bodyToWorld.setIdentity();
        worldToBody.setIdentity();
    }

    Joint(float length): length(length), numColumns(0)
    {
        bodyToWorld.setIdentity();
        worldToBody.setIdentity();
    }

    virtual void move() {}
    virtual void render() {}
    virtual void getJacobian(Eigen::MatrixXf &jacobian) {}
    virtual void update(float dx, float dy, float dz) {}
};

class BallJoint: public Joint{
public:
    Eigen::Vector3f currRot; //in radians

    BallJoint(): Joint(), currRot(0, 0, 0)
    {
        numColumns = 3;
    }

    BallJoint(float length): Joint(length), currRot(0, 0, 0)
    {
        numColumns = 3;
    }

    void move() {
        glRotatef(currRot.x(), 1.0f, 0.0f, 0.0f);
        glRotatef(currRot.y(), 0.0f, 1.0f, 0.0f);
        glRotatef(currRot.z(), 0.0f, 0.0f, 1.0f);
    }

    void render() {
        glColor3f(0.1, 0.3, 1.0);
        glutSolidCone(1.0f, length, 20, 20);        
    }

    void update(float dx, float dy, float dz) {
        currRot[0] += dx;
        currRot[1] += dy;
        currRot[2] += dz;

        currRot[0] = clampAngle(currRot[0], 2*PI);
        currRot[1] = clampAngle(currRot[1], 2*PI);
        currRot[2] = clampAngle(currRot[2], 2*PI);

        //The transformation for calculating the p vector for this joint.
        worldToBody.setIdentity();
        worldToBody.rotate(Eigen::AngleAxis<float>(-1 * currRot.x(), Eigen::Vector3f(1, 0, 0)));
        worldToBody.rotate(Eigen::AngleAxis<float>(-1 * currRot.y(), Eigen::Vector3f(0, 1, 0)));
        worldToBody.rotate(Eigen::AngleAxis<float>(-1 * currRot.z(), Eigen::Vector3f(0, 0, 1)));

        //The transformation for changing Jacobian back to global Jacobian.
        bodyToWorld.setIdentity();
        bodyToWorld.rotate(Eigen::AngleAxis<float>(currRot.x(), Eigen::Vector3f(1, 0, 0)));
        bodyToWorld.rotate(Eigen::AngleAxis<float>(currRot.y(), Eigen::Vector3f(0, 1, 0)));
        bodyToWorld.rotate(Eigen::AngleAxis<float>(currRot.z(), Eigen::Vector3f(0, 0, 1)));
    }

    void getJacobian(Eigen::MatrixXf &jacobian) {

    }
};


class System {
public:
    std::vector<Joint*> joints;
    Eigen::Vector3f basepoint;      //Base point of the system of arms in world space.
    Eigen::Vector3f endpoint;       //End point (End effector) of the system of arms in world space.
    Eigen::MatrixXf jacobian;
    float eps;                      //Tolerance of the distance between end point and goal.
    int numColumns;
    float length;

    void addJoints(std::vector<Joint*> joints) {
        this->joints = joints;
        basepoint << 0, 0, 0;
        endpoint << 0, 0, 0;
        numColumns = 0;
        length = 0;
        eps = 0.01;
        for (int i = 0; i < joints.size(); i++) {
            endpoint[2] += joints[i]->length;
            length += joints[i]->length;
            numColumns += joints[i]->numColumns;
        }
        jacobian = Eigen::MatrixXf(3, numColumns);
    }

    void updateEndPoint() {

    }

    void updateAngles(Eigen::VectorXf dtheta) {

    }

    void getJacobian(Eigen::MatrixXf &jacobian) {

    }

    bool update(Eigen::Vector3f g) {
        Eigen::Vector3f g_sys = g - basepoint;

        //If the goal is too far away, use a possible goal instead.
        if (g_sys.norm() > length) {
            g = g_sys.normalized() * length;
        }

        Eigen::Vector3f dp = g - endpoint;
        if (dp.norm() > eps) {
            getJacobian(jacobian);
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(jacobian);
            Eigen::VectorXf dtheta = svd.solve(dp);
            
            updateAngles(dtheta);
            updateEndPoint();
            return false;
        }
        return true;
    }

    void render() {
        for (int i = 0; i < joints.size(); i++) {
            joints[i]->move();
            joints[i]->render();
            glTranslatef(0.0f, 0.0f, joints[i]->length);
        }

        glColor3f(1.0f, 0.0f, 0.0f);
        glutSolidSphere(3.0f, 20, 20);
    }

};

float clampAngle(float angle, float range) {
    if (angle >= range)
        angle = angle - range;
    if (angle <= 0)
        angle = range + angle;
    return angle;
}

float radToDeg(float rad) {
    return rad * 180.0f / PI;
}

float degToRad(float deg) {
    return deg * PI / 180.0f;
}


#endif /* INVKIN_H */