#include <fstream>  // File IO
#include <iostream> // Terminal IO
#include <sstream>  // Stringstreams
#include <cmath>
#include <vector>

using namespace std;

class QMath
{

public:
    static void printArray(float array[], int numElements, string message)
    {
        cout << message << endl;
        for (int i = numElements - 1; i >= 0; i--)
            cout << " " << array[i];
        cout << "" << endl;
        return;
    }

public:
    static void printVector(vector<float> v, string message)
    {
        cout << message << endl;
        for (int i = 0; i < v.size(); i++)
            cout << " " << v.at(i);
        cout << "" << endl;
        return;
    }

public:
    static void printMatrix(vector<vector<float>> m, string message)
    {
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
    static vector<float> arrayToVector(float a[], int size)
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
    static vector<vector<float>> arrayToMatrix3d(float a[][3], int rows, int columns)
    {
        vector<vector<float>> returnMatrix;
        for (int i = 0; i < rows; i++)
        {

            for (int k = 0; k < columns; k++)
            {
                returnMatrix.at(i).at(k) = a[i][k];
            }
        }
        return returnMatrix;
    }

public:
    static vector<float> multiplyVectorByScalar(float a, vector<float> v1)
    {
        vector<float> ReturnVector;
        for (int i = 0; i < v1.size(); i++)
        {
            ReturnVector.push_back(v1[i] * a);
        }

        return ReturnVector;
    }

public:
    static vector<float> addVectors(vector<float> v1, vector<float> v2)
    {
        vector<float> ReturnVector;
        for (int i = 0; i < v1.size(); i++)
        {
            ReturnVector.push_back(v1[i] + v2[i]);
        }

        return ReturnVector;
    }

public:
    static vector<float> subtractVectors(vector<float> v1, vector<float> v2)
    {
        vector<float> ReturnVector;
        for (int i = 0; i < v1.size(); i++)
        {
            ReturnVector.push_back(v1[i] - v2[i]);
        }

        return ReturnVector;
    }

public:
    static vector<float> crosproduct(vector<float> a, vector<float> b)
    {
        vector<float> ReturnVector;
        ReturnVector.push_back((a[2] * b[3]) - a[3] * b[2]);
        ReturnVector.push_back((a[3] * b[1]) - a[1] * b[3]);
        ReturnVector.push_back((a[1] * b[2]) - a[2] * b[1]);
        return ReturnVector;
    }

public:
    static vector<vector<float>> matrixmultiplyer(vector<vector<float>> m1, vector<vector<float>> m2)
    {
        vector<vector<float>> returnMatrix;

        int nColumns = m1.at(0).size();
        int mRows = m2.size();
        float element;
        int secondMatrixPlacement = 0;

        if (mRows != nColumns)
        {
            cout << "CANNOT MULTIPLY MATRICES OF WRONG SIZES" << endl;
        }

        for (int i = 0; i < mRows; i++)
        {
            for (int k = 0; k < mRows; i++)
            {
                element += m1.at(i).at(k) * m2.at(k).at(i);
            }
            returnMatrix.at(i).at(secondMatrixPlacement) = element;
            secondMatrixPlacement++;
        }

        return returnMatrix;
    }

public:
    static vector<float> mulitplyQuanternions(vector<float> q1, vector<float> q2)
    {
        vector<float> ReturnVector;
        float dot1 = q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

        ReturnVector.push_back(q1[0] * q2[0] - dot1);
        vector<float> v1;
        vector<float> v2;
        v1.push_back(q1[1]);
        v1.push_back(q1[2]);
        v1.push_back(q1[3]);
        v2.push_back(q2[1]);
        v2.push_back(q2[2]);
        v2.push_back(q2[3]);
        vector<float> m1 = multiplyVectorByScalar(q1[0], v1);
        vector<float> m2 = multiplyVectorByScalar(q2[0], v2);
        vector<float> product = addVectors(m1, m2);
        vector<float> crossprod = crosproduct(v1, v2);
        vector<float> product2 = addVectors(product, crossprod);
        ReturnVector.push_back(product2[0]);
        ReturnVector.push_back(product2[1]);
        ReturnVector.push_back(product2[2]);
        return ReturnVector;
    }

public:
    static vector<float> hamiltonProduct(vector<float> v1, vector<float> v2)
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
    static vector<float> getUnitQuanternion(vector<float> v1)
    {
        vector<float> returnVector;

        float magnitude = pow(v1.at(1), 2) + pow(v1.at(2), 2) + pow(v1.at(3), 2);
        returnVector.push_back(v1.at(0));
        returnVector.push_back(v1.at(1) / magnitude);
        returnVector.push_back(v1.at(2) / magnitude);
        returnVector.push_back(v1.at(3) / magnitude);

        return returnVector;
    }

public:
    static vector<float> getQForRotation(vector<float> unitQ)
    {
        vector<float> returnVector;
        float a = cos(unitQ.at(0) / 2);
        float b = unitQ.at(1) * sin(unitQ.at(0) / 2);
        float c = unitQ.at(2) * sin(unitQ.at(0) / 2);
        float d = unitQ.at(3) * sin(unitQ.at(0) / 2);
        returnVector.push_back(a);
        returnVector.push_back(b);
        returnVector.push_back(c);
        returnVector.push_back(d);
        return returnVector;
    }

public:
    static vector<float> getQInverseForRotation(vector<float> unitQ)
    {
        vector<float> returnVector;
        float a = cos(unitQ.at(0) / 2);
        float b = -1 * unitQ.at(1) * sin(unitQ.at(0) / 2);
        float c = -1 * unitQ.at(2) * sin(unitQ.at(0) / 2);
        float d = -1 * unitQ.at(3) * sin(unitQ.at(0) / 2);
        returnVector.push_back(a);
        returnVector.push_back(b);
        returnVector.push_back(c);
        returnVector.push_back(d);
        return returnVector;
    }

public:
    static vector<float> transformPoint(float pointArray[3], float translation[3], float quanternion[4])
    {

        vector<float> pointVector = arrayToVector(pointArray, 3);
        pointVector.insert(pointVector.begin(), 0.0);
        vector<float> translationVector = arrayToVector(translation, 3);
        vector<float> unitQ = getUnitQuanternion(arrayToVector(quanternion, 4));
        printVector(unitQ, "here is the unit quanternion");

        vector<float> q1 = getQForRotation(unitQ);
        vector<float> q2 = getQInverseForRotation(unitQ);

        printVector(q1, "here is the q1");
        printVector(q2, "here is the q2");

        printVector(hamiltonProduct(q1, q2), "checking the q1 * q2 = 1 0 0 0 ");

        vector<float> h1 = hamiltonProduct(q2, pointVector);
        vector<float> h2 = hamiltonProduct(h1, q1);
        printVector(h2, "here is the hamilton product");
        h2.erase(h2.begin());
        h2 = subtractVectors(h2, translationVector);
        printVector(h2, " here is the translated hamtilon product ");

        return h2;
    }
};

int main()
{
    // float pointArray[3] = {-0.0291109, 0.142287, 0.536};
    // float translation[3] = {-0.15664590130365624, -0.47121726992138524, 0.47149692272069615};
    // float quanternion[4] = {0.6981244379642474, -0.2897346707473156, 0.5428195820255429, 0.36609150639377475};
    // transformPoint(pointArray, translation, quanternion);

    // float array[3][3] = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};
    // float array2[3][3] = {{5, 7, 8}, {4, 3, 3}, {1, 9, 3}};

    // vector<vector<float>> m1 = QMath::arrayToMatrix3d(array, 3, 3);
    // vector<vector<float>> m2 = QMath::arrayToMatrix3d(array2, 3, 3);

    // QMath::printMatrix(QMath::matrixmultiplyer(m1,m2), "hellow");

    return 1;
}