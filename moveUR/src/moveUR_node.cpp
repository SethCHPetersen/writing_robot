#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <iostream>
#include <cmath>
#include "std_msgs/String.h"

#include "../../find_writing_surface/include/quanternionMath/quanternionMath.h"
class Link
{
public:
    float d;
    float theta;
    float alpha;
    float a;
    Link()
    {
    }
    Link(float theta, float a, float d, float alpha)
    {
        this->d = d;
        this->theta = theta;
        this->alpha = alpha;
        this->a = a;
    }

public:
    float getD()
    {
        return this->d;
    }

public:
    float getA()
    {
        return this->a;
    }

public:
    float getAlpha()
    {
        return this->alpha;
    }

public:
    float getTheta()
    {
        return this->theta;
    }
};
class moveUR
{

    bool connected = false;
    ros::Publisher MovePub;

public:
    Link link1;
    Link link2;
    Link link3;
    Link link4;
    Link link5;
    Link link6;
    vector<float> currentPosition;

    moveUR(Link link1, Link link2, Link link3, Link link4, Link link5, Link link6)
    {
        this->link1 = link1;
        this->link2 = link2;
        this->link3 = link3;
        this->link4 = link4;
        this->link5 = link5;
        this->link6 = link6;
    }
    moveUR(vector<float> currentPosition, Link link1, Link link2, Link link3, Link link4, Link link5, Link link6)
    {
        this->link1 = link1;
        this->link2 = link2;
        this->link3 = link3;
        this->link4 = link4;
        this->link5 = link5;
        this->link6 = link6;

        this->currentPosition.push_back(currentPosition.at(0));
        this->currentPosition.push_back(currentPosition.at(1));
        this->currentPosition.push_back(currentPosition.at(2));
        this->currentPosition.push_back(currentPosition.at(3));
        this->currentPosition.push_back(currentPosition.at(4));
        this->currentPosition.push_back(currentPosition.at(5));
    }

public:
    void setPosition(vector<float> positionVector) // sets current position relative to base cordinates
    {
        for (int i = 0; i < 6; i++)
        {
            this->currentPosition.at(i) = positionVector.at(i);
        }
        return;
    }

public:
    vector<float> getPosition() // sets current position relative to base cordinates
    {
        return this->currentPosition;
    }

public:
    static vector<float> rotationMatrixToRPY(vector<vector<float>> transformationMatrix)
    {
        float yaw = atan(transformationMatrix.at(1).at(0) / transformationMatrix.at(0).at(0));
        float pitch = atan(-1 * transformationMatrix.at(2).at(0) / (sqrt(pow(transformationMatrix.at(2).at(1), 2) + pow(transformationMatrix.at(2).at(2), 2))));
        float roll = atan(transformationMatrix.at(2).at(1) / transformationMatrix.at(2).at(2));
        vector<float> rpy;
        rpy.push_back(roll);
        rpy.push_back(pitch);
        rpy.push_back(yaw);
        return rpy;
    }

public:
    static vector<vector<float>> getTransformationDH(Link link, float thetaRotation)
    {
        vector<vector<float>> t;

        vector<float> temp;
        temp.push_back(cos(link.theta + thetaRotation));
        temp.push_back(-1 * sin(link.theta + thetaRotation) * cos(link.alpha));
        temp.push_back(sin(link.theta + thetaRotation) * sin(link.alpha));
        temp.push_back(link.a * cos(link.theta + thetaRotation));
        // QMath::printVector(temp, "temp ");
        t.push_back(temp);
        temp.clear();
        // QMath::printVector(t.at(0), "t2");

        temp.push_back(sin(link.theta + thetaRotation));
        temp.push_back(cos(link.theta + thetaRotation) * cos(link.alpha));
        temp.push_back(-1 * cos(link.theta + thetaRotation) * sin(link.alpha));
        temp.push_back(link.a * sin(link.theta + thetaRotation));
        t.push_back(temp);
        // QMath::printVector(temp, "temp");

        temp.clear();
        // QMath::printMatrix(t, "t3");

        temp.push_back(0);
        temp.push_back(sin(link.alpha));
        temp.push_back(cos(link.alpha));
        temp.push_back(link.d);
        t.push_back(temp);
        // QMath::printVector(temp, "temp");

        temp.clear();
        // QMath::printMatrix(t, "t4");

        temp.push_back(0);
        temp.push_back(0);
        temp.push_back(0);
        temp.push_back(1);
        t.push_back(temp);
        // QMath::printVector(temp, "temp");

        temp.clear();
        // QMath::printMatrix(t, "t5");
        return t;
    }

public:
    void forwardKinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
    {
        vector<vector<float>> t1;
        vector<vector<float>> t2;
        vector<vector<float>> t3;
        vector<vector<float>> t4;
        vector<vector<float>> t5;
        vector<vector<float>> t6;

        t1 = getTransformationDH(this->link1, theta1);
        t2 = getTransformationDH(this->link2, theta2);
        t3 = getTransformationDH(this->link3, theta3);
        t4 = getTransformationDH(this->link4, theta4);
        t5 = getTransformationDH(this->link5, theta5);
        t6 = getTransformationDH(this->link6, theta6);

        vector<vector<float>> m1 = QMath::matrixmultiplyer(t1, t2);
        vector<vector<float>> m2 = QMath::matrixmultiplyer(m1, t3);
        vector<vector<float>> m3 = QMath::matrixmultiplyer(m2, t4);
        vector<vector<float>> m4 = QMath::matrixmultiplyer(m3, t5);
        vector<vector<float>> m5 = QMath::matrixmultiplyer(m4, t6);

        vector<float> positionVector;
        positionVector.push_back(m5.at(0).at(3));
        positionVector.push_back(m5.at(1).at(3));
        positionVector.push_back(m5.at(2).at(3));

        vector<float> rpy = rotationMatrixToRPY(m5);
        positionVector.push_back(rpy.at(0));
        positionVector.push_back(rpy.at(1));
        positionVector.push_back(rpy.at(2));

        this->setPosition(positionVector);
        return;
    }

public:
    void connectToRobot()
    {
        int attempts = 0;
        ros::NodeHandle n;
        MovePub = n.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 1000);
        while (MovePub.getNumSubscribers() < 1)
        {
            attempts++;
            ros::WallDuration sleep_t(0.5);
            sleep_t.sleep();
            cout << "still waiting for subscriber" << endl;
            if (attempts == 10)
            {
                cout << "topic has zero subscribers" << endl;
                return;
            }
        }
        return;
    }

public:
    static std_msgs::String convertVectorToString(vector<float> v)
    {

        std_msgs::String returnString;
        returnString.data = to_string('[');
        for (int i = 0; i < v.size(); i++)
        {
            returnString.data += to_string(v.at(i));
        }
        returnString.data += to_string(']');

        return returnString;
    }

public:
    void moveRobot(vector<float> targetPos, float veloctity, float acceleration)
    {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "movel(" + convertVectorToString(targetPos).data + ", a=" + to_string(acceleration) + ", v=" + to_string(veloctity) + ", t=0, r=0)";

        msg.data = ss.str();
        MovePub.publish(msg);

        return;
    }

public:
    void zeroFTSensor()
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "zero_ftsensor()";
        msg.data = ss.str();
        MovePub.publish(msg);
        return;
    }
};

int main()
{
    Link link1(0, 0, .15185, 3.14159265358979 / 2); // got these from https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    Link link2(0, -.24355, 0, 0);
    Link link3(0, -.2132, 0, 0);
    Link link4(0, 0, .13102, 3.14159265358979 / 2);
    Link link5(0, 0, 0.08535, -3.14159265358979 / 2);
    Link link6(0, 0, 0.0921, 0);

    vector<float> currentPos;
    currentPos.push_back(0);
    currentPos.push_back(0);
    currentPos.push_back(0);
    currentPos.push_back(0);
    currentPos.push_back(0);
    currentPos.push_back(0);

    moveUR UR3(currentPos, link1, link2, link3, link4, link5, link6);
    //[0.5641759077655237, -2.3371530971922816, -4.702834431325094, -2.0123635731139125, -1.6021960417376917, -2.615519348775045]
    // should be .126, .189, .659, .851, -.431, .316
    //[-5.445096794758932, -0.5880602163127442, 0.5673182646380823, -1.6850735149779261, -1.6023038069354456, -2.615530792866842]
    
    UR3.forwardKinematics(-5.445096794758932, -0.5880602163127442, 0.5673182646380823, -1.6850735149779261, -1.6023038069354456, -2.615530792866842);
    QMath::printVector(UR3.getPosition(), "UR current position");

    return 1;
}