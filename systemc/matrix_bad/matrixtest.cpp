#include<iostream>
#include"matrix.cpp"

using namespace std;

int main(int argc, char const *argv[])
{
    Matrix<double> m(2,2);
    m << 1 << 0 << 0 << 1;
    cout << m << endl;
    Matrix<double> m2(2,2);
    m2 << 1 << 2 << 3 << 4;
    Matrix<double> m3 = m+m2;
    cout << m3 <<endl;
    Matrix<double> m4 = m*m2;
    cout << m4 << endl;
    Matrix<double> m5(2,3);
    m5 << 1 << 2 << 3 << 4 << 5 << 6;
    cout << m2 << endl;
    cout << m5 << endl;
    Matrix<double> m6 = m2*m5;
    cout << m6 << endl;
    cout << m6.transpose() << endl;
    Matrix<double> m7(3,3);
    m7 << 2 << 0 << 0 << 0 << 1 << 0 << 0 << 0 << 3;
    cout << m7 << endl;
    cout << m7.inverse() << endl;

    return 0;
}
