#ifndef INVKIN_H
#define INVKIN_H

#include <vector>
#include <Eigen/Eigen>

#define PI 3.14159265

//Forward method declaration
float clampAngle(float, float);
float degToRad(float);
float radToDeg(float);

//Variables
int debug = 0;
float updateCap = degToRad(10.0f);         //The maximum update value allowed for each time step.

class Joint {
public:
    float length;
    Eigen::Transform<float, 3, Eigen::Affine> bodyToWorld;
    int numColumns;

    Joint(): length(0), numColumns(0)
    {
        bodyToWorld.setIdentity();
    }

    Joint(float length): length(length), numColumns(0)
    {
        bodyToWorld.setIdentity();
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
        currRot[0] -= dx;
        currRot[1] -= dy;
        currRot[2] -= dz;

        currRot[0] = clampAngle(currRot[0], 2*PI);
        currRot[1] = clampAngle(currRot[1], 2*PI);
        currRot[2] = clampAngle(currRot[2], 2*PI);

        //The transformation for changing Jacobian back to global Jacobian.
        bodyToWorld.setIdentity();
        bodyToWorld.rotate(Eigen::AngleAxis<float>(currRot.x(), Eigen::Vector3f(1, 0, 0)));
        bodyToWorld.rotate(Eigen::AngleAxis<float>(currRot.y(), Eigen::Vector3f(0, 1, 0)));
        bodyToWorld.rotate(Eigen::AngleAxis<float>(currRot.z(), Eigen::Vector3f(0, 0, 1)));

        if (debug > 2) {
            std::cout << "BallJoint update:" << std::endl;
            std::cout << "currRot = " << std::endl << currRot << std::endl;
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

class Path {
public:
    std::vector<Eigen::Vector3f*> points;         //The points that made up the path.
    int index;

    Path(): points(), index(0)
    {
    }

    bool isEmpty() {
        return points.size() == 0;
    }

    Eigen::Vector3f* curr() {
        return points[index];
    }

    Eigen::Vector3f* next(int step) {
        Eigen::Vector3f* p = curr();
        index += step;
        if (index >= points.size())
            index = 0;
        return p;
    }

    void render();
};

class System {
public:
    std::vector<Joint*> joints;
    Eigen::Vector3f basepoint;      //Base point of the system of arms in world space.
    Eigen::Vector3f endpoint;       //End point (End effector) of the system of arms in world space.
    Eigen::MatrixXf J;
    float eps;                      //Tolerance of the distance between end point and goal.
    bool goalTooFarAway;            //Whether the current is too far away for the system to reach or not.
    Path path;                      //The path this system is moving along.
    int numColumns;
    float length;
    float maxDtheta;
    bool overThreshold;
    Eigen::Vector3f* currGoal;


    void initialize(std::vector<Joint*> joints, Path path) {
        this->path = path;
        currGoal = path.curr();
        this->joints = joints;
        basepoint << 0, 0, 0;
        updateEndPoint(endpoint);
        numColumns = 0;
        length = 0;
        eps = 0.01;
        maxDtheta = 0.0f;
        goalTooFarAway = false;
        for (int i = 0; i < joints.size(); i++) {
            length += joints[i]->length;
            numColumns += joints[i]->numColumns;
        }
        J = Eigen::MatrixXf(3, numColumns);
    }

    void updateEndPoint(Eigen::Vector3f &ep) {
        ep << 0, 0, 0;
        Eigen::Transform<float, 3, Eigen::Affine> t;
        Joint* joint;
        for (int i = joints.size() - 1; i >= 0; i--) {
            joint = joints[i];
            t.setIdentity();
            t.translate(Eigen::Vector3f(0, 0, joint->length));
            ep = joint->bodyToWorld * t * ep;
        }
    }

    void updateAngles(Eigen::VectorXf dtheta) {
        int x = 0;
        for (int i = 0; i < joints.size(); i++) {
            joints[i]->update(dtheta[x], dtheta[x+1], dtheta[x+2]);
            /*if (joints[i]->numColumns == 3)
                joints[i]->update(dtheta[x], dtheta[x+1], dtheta[x+2]);
            else
                joints[i]->update(dtheta[x], 0, 0);*/
            x += joints[i]->numColumns;
        }
    }

    // Do some preprocessing of the dtheta, like storing the max (in magn) and clamping at some value.
    void processDtheta(Eigen::VectorXf &dtheta) {
        overThreshold = false;
        for (int i = 0; i < dtheta.size(); i++) {
            maxDtheta = std::max(maxDtheta, (float)fabs(dtheta[i]));
            if (fabs(dtheta[i]) > updateCap) {
                overThreshold = true;
                if (dtheta[i] < 0)
                    dtheta[i] = -updateCap;
                else
                    dtheta[i] = updateCap;
            }
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

    void hackhack(Eigen::VectorXf &dtheta, Eigen::Vector3f &g) {
        processDtheta(dtheta);
        updateAngles(dtheta);
        updateEndPoint(endpoint);

        /*Eigen::Vector3f ep;
        updateEndPoint(ep);

        if ((g - ep).norm() <= (g - endpoint).norm()) {
            //Nothing wrong
            endpoint = ep;
        } else {
            std::cout << "=====================================" << std::endl;
            std::cout << "Moving away from goal!!!!" << std::endl;
            std::cout << "Goal = " << std::endl << g << std::endl;
            std::cout << "Current endpoint = " << std::endl << endpoint << std::endl;
            std::cout << "New endpoint = " << std::endl << ep << std::endl;
            for (int i = 0; i < joints.size(); i++) {
                std::cout << "Current rotation for joint #" << i << ":" << std::endl;
                std::cout << ((BallJoint*)joints[i])->currRot << std::endl;
            }
            std::cout << "=====================================" << std::endl;
            updateAngles(-2 * dtheta);
            updateEndPoint(endpoint);
        }*/
    }

    bool update(Eigen::Vector3f g) {
        Eigen::Vector3f g_sys = g - basepoint;

        goalTooFarAway = false;
        //If the goal is too far away, use a possible goal instead.
        if (g_sys.norm() > length + eps) {
            goalTooFarAway = true;
            g = g_sys.normalized() * (length - eps);
        }

        Eigen::Vector3f dp = g - endpoint;
        if (dp.norm() > eps) {
            getJacobian(J);
            
            if (debug > 1) {
                std::cout << "Solving pseudoinverse:" << std::endl;
                std::cout << "Jacobian =" << std::endl << J << std::endl;
                std::cout << "dp =" << std::endl << dp << std::endl;
            }

            Eigen::VectorXf dtheta = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dp);

            if (debug > 0) {
                std::cout << "Goal = " << std::endl << g << std::endl;
                std::cout << "End point = " << std::endl << endpoint << std::endl;
                std::cout << "Goal beyond reach = " << goalTooFarAway << std::endl;
                std::cout << "dp = " << std::endl << dp << std::endl;
                std::cout << "dtheta =" << std::endl << dtheta << std::endl;
            }


            hackhack(dtheta, g);
            return false;
        }
        return true;
    }

    void render();

};

float clampAngle(float angle, float range) {
    if (angle >= 0.5 * range)
        angle = angle - 0.5 * range;
    if (angle < -0.5 * range)
        angle = 0.5 * range + angle;
    return angle;
}

float radToDeg(float rad) {
    return rad * 180.0f / PI;
}

float degToRad(float deg) {
    return deg * PI / 180.0f;
}

std::ostream& operator<<(std::ostream &strm, const BallJoint &bj) {
    strm << "Ball Joint: length(" << bj.length << "), rotation(" << bj.currRot.x() << ", " << bj.currRot.y() << ", " << bj.currRot.z() << ")" << std::endl;
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const Path &p) {
    strm << "Path: " << p.points.size() << " points" << std::endl;
    return strm;
}

#endif /* INVKIN_H */