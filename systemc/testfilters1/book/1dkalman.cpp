#include<iostream>
#include"example_hl.cpp"
using namespace std;

/**
 * Example 8 – Estimating the temperature of a heating liquid II
*/
class KalmanFilter {
    private:
    double x; // Current state estimate
    double p; // State estimate variance
    double r; // Measurement variance
    double k; // Kalman gain
    double q; // Process noise variance
    public:
    KalmanFilter(double x, double p, double q, double r) {
        this->x = x;
        this->p = p + q;
        this->q = q;
        this->r = r;
        cout << "KalmanFilter" << endl;
        cout << "x:" << this->x << endl;
        cout << "p:" << this->p << endl;
        cout << "q:" << this->q << endl;
        cout << "r:" << this->r << endl;
        cout << endl;
    }
    void filterProcess(double z/* measurement */) {
        // State update
        k = p/(p+r);
        x = x + k * (z - x);
        p = (1 - k) * p;
        // State predict
        p = p + q;
    }
    double getX() {
        return x;
    }
    double getP() {
        return p;
    }
    double getK() {
        return k;
    }

};

int main(int argc, char const *argv[])
{
    KalmanFilter filter(10.0, 10000.0, 0.15, 0.01);
    for (int i = 0; i < HeatingLiquid::count; i++)
    {
        filter.filterProcess(HeatingLiquid::measurements[i]);
        cout << "Iteration: " << i+1 << endl;
        cout << "k:" << filter.getK() << endl;
        cout << "x = " << filter.getX() << endl;
        cout << "truth = " << HeatingLiquid::truth[i] << endl;
        cout << "p = " << filter.getP() << endl;
        cout << endl;
    }
    return 0;
}
