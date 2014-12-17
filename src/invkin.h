#ifndef INVKIN_H
#define INVKIN_H

#include <vector>
#include <Eigen/Eigen>

#define PI 3.14159265

int debug = 0;

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
    virtual void getJacobian(Eigen::MatrixXf &J, Eigen::Vector3f p) {}
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

        if (debug > 2) {
            std::cout << "BallJoint update:" << std::endl;
            std::cout << "currRot = " << std::endl << currRot << std::endl;
            std::cout << "worldToBody = " << std::endl << worldToBody.matrix() << std::endl;
            std::cout << "bodyToWorld = " << std::endl << bodyToWorld.matrix() << std::endl;
            std::cout << std::endl;
        }
    }

    void getJacobian(Eigen::MatrixXf &J, Eigen::Vector3f p) {
        J(0, 0) = 0;
        J(0, 1) = -p.z();
        J(0, 2) = p.y();
        J(1, 0) = p.z();
        J(1, 1) = 0;
        J(1, 2) = -p.x();
        J(2, 0) = -p.y();
        J(2, 1) = p.x();
        J(2, 2) = 0;
    }

    void move();
    void render();
};


class System {
public:
    std::vector<Joint*> joints;
    Eigen::Vector3f basepoint;      //Base point of the system of arms in world space.
    Eigen::Vector3f endpoint;       //End point (End effector) of the system of arms in world space.
    Eigen::MatrixXf J;
    float eps;                      //Tolerance of the distance between end point and goal.
    int numColumns;
    float length;

    void initialize(std::vector<Joint*> joints) {
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
        J = Eigen::MatrixXf(3, numColumns);
    }

    void updateEndPoint() {
        endpoint << 0, 0, 0;
        Eigen::Transform<float, 3, Eigen::Affine> t;
        Joint* joint;
        for (int i = joints.size() - 1; i >= 0; i--) {
            joint = joints[i];
            t.setIdentity();
            t.translate(Eigen::Vector3f(0, 0, joint->length));
            endpoint = joint->bodyToWorld * t * endpoint;
        }
    }

    void updateAngles(Eigen::VectorXf dtheta) {
        int x = 0;
        for (int i = 0; i < joints.size(); i++) {
            if (joints[i]->numColumns == 3)
                joints[i]->update(dtheta[x], dtheta[x+1], dtheta[x+2]);
            else
                joints[i]->update(dtheta[x], 0, 0);
            x += joints[i]->numColumns;
        }
    }

    void getJacobian(Eigen::MatrixXf &J) {
        Eigen::Vector3f p(0, 0, 0);
        int col;
        Eigen::MatrixXf jac(3, 3);
        Eigen::Transform<float, 3, Eigen::Affine> t;

        Joint* joint;
        col = J.cols();

        if (debug > 1) {
            std::cout << "Calculating local jacobian:" << std::endl;
        }
        //Start from the last arm, iterating backward, to calculate the jacobian local to each arm.
        for (int i = joints.size()-1; i >= 0; i--) {
            joint = joints[i];
            //Get the p vector.
            t.setIdentity();
            t.translate(Eigen::Vector3f(0, 0, joint->length));
            //p = joint->worldToBody * t * p;
            p = joint->bodyToWorld * t * p;
            //Get the local jacobian.
            col -= joint->numColumns;
            jac = Eigen::MatrixXf(3, joint->numColumns);
            joint->getJacobian(jac, p);
            if (debug > 1) {
                std::cout << "Joint #" << i << ":" << std::endl;
                std::cout << "  length = " << joint->length << std::endl;
                std::cout << "  numColumns = " << joint->numColumns << std::endl;
                std::cout << "size of system jacobian = (" << J.rows() << ", " << J.cols() << ")" << std::endl;
                std::cout << "p = " << std::endl << p << std::endl;
                std::cout << "size of jac = (" << jac.rows() << ", " << jac.cols() << ")" << std::endl;
                std::cout << "col = " << col << std::endl;
                std::cout << "jac = " << std::endl << jac << std::endl;
                std::cout << "block = " << std::endl << J.block(0, col, 3, joint->numColumns) << std::endl;
                std::cout << std::endl;
            }
            J.block(0, col, 3, joint->numColumns) = jac.block(0, 0, 3, joint->numColumns);
        }

        if (debug > 1) {
            std::cout << "Transforming jacobian to global:" << std::endl;
        }

        t.setIdentity();
        col = 0;
        //Start from the first arm, to transform the local jacobian into world space.
        for (int i = 0; i < joints.size(); i++) {
            joint = joints[i];
            if (debug > 1) {
                std::cout << "Joint #" << i << std::endl;
                std::cout << "Jacobian before = " << std::endl << J.block(0, col, 3, joint->numColumns) << std::endl;
            }
            J.block(0, col, 3, joint->numColumns) = t.matrix().block(0, 0, 3, 3) * J.block(0, col, 3, joint->numColumns);
            if (debug >> 1) {
                std::cout << "Jacobian after = " << std::endl << J.block(0, col, 3, joint->numColumns) << std::endl;

            }
            col += joint->numColumns;
            t = t * joint->bodyToWorld;
        }
    }

    bool update(Eigen::Vector3f g) {
        Eigen::Vector3f g_sys = g - basepoint;

        //If the goal is too far away, use a possible goal instead.
        if (g_sys.norm() > length) {
            g = g_sys.normalized() * length;
        }

        Eigen::Vector3f dp = g - endpoint;
        if (dp.norm() > eps) {
            getJacobian(J);
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(J);
            Eigen::VectorXf dtheta = svd.solve(dp);
            
            updateAngles(dtheta);
            updateEndPoint();
            return false;
        }
        return true;
    }

    void render();

};

float clampAngle(float angle, float range) {
    if (angle >= range)
        angle = angle - range;
    if (angle < 0)
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