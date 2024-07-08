#include"matrix.cpp"
#include<iostream>
using namespace std;

#define DT double
int main(int argc, char const *argv[])
{
    Matrix<DT> m(2,2);
    m << 2 << 0
      << 0 << 3;
    m.print();
    Matrix<DT> m2 = m.inv();
    m2.print();
    m2.inv().print();
    return 0;
}
