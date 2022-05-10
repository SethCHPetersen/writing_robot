#include <fstream>  // File IO
#include <iostream> // Terminal IO
#include <sstream>  // Stringstreams
#include <cmath>
#include <vector>

using namespace std;

class QMath // class used to do linear algebra, specifically dealing with quaternions
{

public:
    static void printArray(float array[], int numElements, string message) // print an array message
    {
        cout << message << endl;
        for (int i = 0; i < numElements; i++)
            cout << " " << array[i];
        cout << "" << endl;
        return;
    }

public:
    static void printVector(vector<float> v, string message) // print a vector with a message
    {
        cout << message << endl;
        for (int i = 0; i < v.size(); i++)
            cout << " " << v.at(i);
        cout << "" << endl;
        return;
    }

public:
    static void printMatrix(vector<vector<float>> m, string message) // print a matrix with a message
    {
        cout << message << endl;

        for (int i = 0; i < m.at(0).size(); i++)
        {
            for (int k = 0; k < m.size(); k++)
            {
                cout << m.at(i).at(k) << " ";
            }
            cout << "" << endl;
        }
        return;
    }

public:
    static vector<float> arrayToVector(float a[], int size) // convert an array to a vector
    {
        vector<float> returnVector;
        // cout << " " << endl;
        for (int i = 0; i < size; i++)
        {
            returnVector.push_back(a[i]);
            // cout << "adding this point " << a[i] << endl;
        }
        // cout << " " << endl;
        return returnVector;
    }

public:
    static vector<vector<float>> arrayToMatrix3d(float a[][3], int rows, int columns) // convert an array, 3x3, to a matrix
    {
        vector<vector<float>> returnMatrix;
        vector<float> tempVector;
        for (int i = 0; i < rows; i++)
        {

            for (int k = 0; k < columns; k++)
            {
                tempVector.push_back(a[i][k]);
            }
            returnMatrix.push_back(tempVector);
            tempVector.clear();
        }
        return returnMatrix;
    }

public:
    static vector<vector<float>> arrayToMatrix4d(float a[][4], int rows, int columns) // convert an array, 4x4, to a matrix
    {
        vector<vector<float>> returnMatrix;
        vector<float> tempVector;
        for (int i = 0; i < rows; i++)
        {

            for (int k = 0; k < columns; k++)
            {
                tempVector.push_back(a[i][k]);
            }
            returnMatrix.push_back(tempVector);
            tempVector.clear();
        }
        return returnMatrix;
    }

public:
    static vector<float> multiplyVectorByScalar(float a, vector<float> v1) // mulitply a vector by a scalar
    {
        vector<float> ReturnVector;
        for (int i = 0; i < v1.size(); i++)
        {
            ReturnVector.push_back(v1[i] * a);
        }

        return ReturnVector;
    }

public:
    static vector<float> addVectors(vector<float> v1, vector<float> v2) // add two vectors
    {
        vector<float> ReturnVector;
        for (int i = 0; i < v1.size(); i++)
        {
            ReturnVector.push_back(v1[i] + v2[i]);
        }

        return ReturnVector;
    }

public:
    static vector<float> subtractVectors(vector<float> v1, vector<float> v2) // subtract vector 2 from vector 1
    {
        vector<float> ReturnVector;
        for (int i = 0; i < v1.size(); i++)
        {
            ReturnVector.push_back(v1[i] - v2[i]);
        }

        return ReturnVector;
    }

public:
    static vector<float> crosproduct(vector<float> a, vector<float> b) // compute the cross product of two vectors, a X b
    {
        vector<float> ReturnVector;
        ReturnVector.push_back((a[2] * b[3]) - a[3] * b[2]);
        ReturnVector.push_back((a[3] * b[1]) - a[1] * b[3]);
        ReturnVector.push_back((a[1] * b[2]) - a[2] * b[1]);
        return ReturnVector;
    }

public:
    static vector<vector<float>> matrixmultiplyer(vector<vector<float>> m1, vector<vector<float>> m2) // multiply two matrices m1 * m2
    {
        vector<vector<float>> returnMatrix;

        int nColumns = m1.at(0).size();
        int mRows = m2.size();
        float element;
        int secondMatrixPlacement = 0;
        vector<float> tempVector;

        if (mRows != nColumns)
        {
            cout << "CANNOT MULTIPLY MATRICES OF WRONG SIZES" << endl;
        }
        for (int i = 0; i < mRows; i++)
        {
            for (int k = 0; k < mRows; k++)
            {
                for (int j = 0; j < mRows; j++)
                {
                    element += (m1.at(i).at(j) * m2.at(j).at(k));
                }
                tempVector.push_back(element);
                element = 0;

                if (tempVector.size() == mRows)
                {
                    returnMatrix.push_back(tempVector);
                    tempVector.clear();
                }
            }
        }
        return returnMatrix;
    }

    // public: // REMOVED FOR HAMILTON PRODUCT
    //     static vector<float> mulitplyQuanternions(vector<float> q1, vector<float> q2) //multiply two quatermions q1 * q2
    //     {
    //         vector<float> ReturnVector;
    //         float dot1 = q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    //         ReturnVector.push_back(q1[0] * q2[0] - dot1);
    //         vector<float> v1;
    //         vector<float> v2;
    //         v1.push_back(q1[1]);
    //         v1.push_back(q1[2]);
    //         v1.push_back(q1[3]);
    //         v2.push_back(q2[1]);
    //         v2.push_back(q2[2]);
    //         v2.push_back(q2[3]);
    //         vector<float> m1 = multiplyVectorByScalar(q1[0], v1);
    //         vector<float> m2 = multiplyVectorByScalar(q2[0], v2);
    //         vector<float> product = addVectors(m1, m2);
    //         vector<float> crossprod = crosproduct(v1, v2);
    //         vector<float> product2 = addVectors(product, crossprod);
    //         ReturnVector.push_back(product2[0]);
    //         ReturnVector.push_back(product2[1]);
    //         ReturnVector.push_back(product2[2]);
    //         return ReturnVector;
    //     }

public:
    static vector<float> hamiltonProduct(vector<float> v1, vector<float> v2) // compute the hamilton product of two quaternions, v1 * v2
    {
        vector<float> returnVecetor;

        float a = (v1.at(0) * v2.at(0)) - (v1.at(1) * v2.at(1)) - (v1.at(2) * v2.at(2)) - (v1.at(3) * v2.at(3));
        float b = (v1.at(0) * v2.at(1)) + (v1.at(1) * v2.at(0)) + (v1.at(2) * v2.at(3)) - (v1.at(3) * v2.at(2));
        float c = (v1.at(0) * v2.at(2)) - (v1.at(1) * v2.at(3)) + (v1.at(2) * v2.at(0)) + (v1.at(3) * v2.at(1));
        float d = (v1.at(0) * v2.at(3)) + (v1.at(1) * v2.at(2)) - (v1.at(2) * v2.at(1)) + (v1.at(3) * v2.at(0));

        returnVecetor.push_back(a);
        returnVecetor.push_back(b);
        returnVecetor.push_back(c);
        returnVecetor.push_back(d);
        return returnVecetor;
    }

public:
    static vector<float> getUnitQuanternion(vector<float> v1) // convert a quaternion to a unit quaternion
    {
        vector<float> returnVector;

        float magnitude = sqrt(pow(v1.at(0), 2) + pow(v1.at(1), 2) + pow(v1.at(2), 2) + pow(v1.at(3), 2));
        returnVector.push_back(v1.at(0) / magnitude);
        returnVector.push_back(v1.at(1) / magnitude);
        returnVector.push_back(v1.at(2) / magnitude);
        returnVector.push_back(v1.at(3) / magnitude);

        return returnVector;
    }

public:
    static float getQuaternNorm(vector<float> quaternion) // get the normal of a quaternion
    {
        vector<float> returnVector;
        float magnitude = sqrt(pow(quaternion.at(0), 2) + pow(quaternion.at(1), 2) + pow(quaternion.at(2), 2) + pow(quaternion.at(3), 2));
        return magnitude;
    }

public:
    static vector<float> getQuaternionConjugate(vector<float> quaternion) // get the conjugate of a quaternion
    {
        vector<float> returnVector;

        returnVector.push_back(quaternion.at(0));
        returnVector.push_back(quaternion.at(1) * -1);
        returnVector.push_back(quaternion.at(2) * -1);
        returnVector.push_back(quaternion.at(3) * -1);

        return returnVector;
    }

public:
    static vector<float> getQuaternionInverse(vector<float> quaternion) // get the inverse of a quaternion
    {
        float norm = getQuaternNorm(quaternion);
        vector<float> returnVector;
        vector<float> conjugate = getQuaternionConjugate(quaternion);

        float a = conjugate.at(0) / pow(norm, 2);
        float b = conjugate.at(1) / pow(norm, 2);
        float c = conjugate.at(2) / pow(norm, 2);
        float d = conjugate.at(3) / pow(norm, 2);
        returnVector.push_back(a);
        returnVector.push_back(b);
        returnVector.push_back(c);
        returnVector.push_back(d);
        // float a = cos(unitQ.at(0) / 2);
        // float b = -1 * unitQ.at(1) * sin(unitQ.at(0) / 2);
        // float c = -1 * unitQ.at(2) * sin(unitQ.at(0) / 2);
        // float d = -1 * unitQ.at(3) * sin(unitQ.at(0) / 2);
        // returnVector.push_back(a);
        // returnVector.push_back(b);
        // returnVector.push_back(c);
        // returnVector.push_back(d);
        return returnVector;
    }

public:
    static vector<float> transformPoint(float pointArray[3], float translation[3], float quanternion[4]) // transform a point from on cord system to another using a quaternion and translation vector
    {

        vector<float> pointVector = arrayToVector(pointArray, 3);
        pointVector.insert(pointVector.begin(), 0.0);
        vector<float> translationVector = arrayToVector(translation, 3);
        vector<float> unitQ = getUnitQuanternion(arrayToVector(quanternion, 4));
        // printVector(unitQ, "here is the unit quanternion");

        vector<float> q1 = unitQ;
        vector<float> q2 = getQuaternionInverse(unitQ);

        vector<float> h1 = hamiltonProduct(q1, pointVector);
        vector<float> h2 = hamiltonProduct(h1, q2);
        // printVector(h2, "here is the hamilton product");
        h2.erase(h2.begin());
        h2 = addVectors(h2, translationVector);

        //-------------- for testing purposes

        // printVector(h2, " here is the translated hamtilon product ");

        // cout << "testing Hamilton Product" << endl;
        // vector<float> testPoint = {0, 0.148412, -0.167296, 0.955};
        // vector<float> testQuanternion = {0.19093701021630496, -0.8941473997722545, 0.39227689972072916, -0.100808330785048};
        // vector<float> testTranslation = {-0.3634422254733067, -0.6520691528769079, 0.8675182456610416};
        // // QMath::printVector(QMath::getQInverseForRotation(testQuanternion), "testing getInverse");
        // cout << " " << endl;
        // cout << " " << endl;

        // QMath::printVector(QMath::getUnitQuanternion(testQuanternion), "testing unitQuanternion");
        // vector<float> unitQtest = QMath::getUnitQuanternion(testQuanternion);

        // QMath::printVector(QMath::getQuaternionConjugate(QMath::getUnitQuanternion(testQuanternion)), "testing getConjugate");
        // vector<float> Qconjugate = QMath::getQuaternionConjugate(QMath::getUnitQuanternion(testQuanternion));

        // QMath::printVector(QMath::getQuaternionInverse(QMath::getUnitQuanternion(testQuanternion)), "testing getInverse");
        // vector<float> QInverseTest = QMath::getQuaternionInverse(QMath::getUnitQuanternion(testQuanternion));

        // // q*p*q-1
        // vector<float> m1 = QMath::hamiltonProduct(unitQtest, testPoint);
        // vector<float> m2 = QMath::hamiltonProduct(m1, QInverseTest);
        // QMath::printVector(m2, "final hamilton prodcuct of test");

        // m2.erase(m2.begin());
        // QMath::printVector(QMath::addVectors(m2, testTranslation), "add translation");
        // cout << " " << endl;
        // cout << " " << endl;
        // // vector<float> testOutcome = QMath::hamiltonProduct(testQuanternion, testPoint);
        // // vector<float> finalVector = QMath::hamiltonProduct(testOutcome, testQuanternionInverse);
        // // QMath::printVector(finalVector, "test outcome ");

        // // vector<float> translation2 = {-0.3634422254733067, -0.6520691528769079, 0.8675182456610416};

        // // QMath::printVector(QMath::addVectors(finalVector, translation2), "minus translation");

        // // vector_pub.publish(paperVector);
        // cout << "done testing Hamilton Product" << endl;

        return h2;
    }
};
