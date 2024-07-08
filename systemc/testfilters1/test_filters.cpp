#define SC_INCLUDE_FX

#include"madgwick.cpp"
#include"mahony.cpp"
#include"imu_dataset.cpp"
#include"quaternion.cpp"
#include"vector3.cpp"
#include<systemc>
#include<vector>
#include<map>
#include<cmath>
#include <iomanip>
#include"basic_vqf_module.cpp"
using namespace sc_dt;
using namespace sc_core;
#define T2 sc_fixed<24,9,SC_RND,SC_SAT>
/**
 * @brief This method calculates the initial orientation of a sensor
 * This method is derived from BROAD paper
 *
 * @tparam T Type of data
 * @param a0 First accelerometer value of x axis
 * @param a1 First accelerometer value of y axis
 * @param a2 First accelerometer value of z axis
 * @param m0 First magnetometer value of x axis
 * @param m1 First magnetometer value of y axis
 * @param m2 First magnetometer value of z axis
 * @return Quaternion<T> Returns initial orientation as a quaternion
 */
template <typename T>
Quaternion<T> quatFromAccMag(T a0, T a1, T a2, T m0, T m1, T m2) {
    Vector3<T> acc(a0, a1, a2); //  Representing a vector in 3-dimensional space
    Vector3<T> mag(m0, m1, m2);
    Vector3<T> z = acc;
    Vector3<T> x = z.cross(-mag).cross(z); //cross product
    Vector3<T> y = z.cross(x);
    x = x.normalized();
    y = y.normalized();
    z = z.normalized();
    Vector3<T> R0 = Vector3<T>(x.getX(), y.getX(), z.getX());//x.normalized();
    Vector3<T> R1 = Vector3<T>(x.getY(), y.getY(), z.getY());//y.normalized();
    Vector3<T> R2 = Vector3<T>(x.getZ(), y.getZ(), z.getZ());//z.normalized();

    T w_sq = (1 + R0.getX() + R1.getY() + R2.getZ()) / 4;
    T x_sq = (1 + R0.getX() - R1.getY() - R2.getZ()) / 4;
    T y_sq = (1 - R0.getX() + R1.getY() - R2.getZ()) / 4;
    T z_sq = (1 - R0.getX() - R1.getY() + R2.getZ()) / 4;

    Quaternion<T> q(0, 0, 0, 0);
    q.w = sqrt(w_sq);
    q.x = copysign(sqrt(x_sq), R2.getY() - R1.getZ());
    q.y = copysign(sqrt(y_sq), R0.getZ() - R2.getX());
    q.z = copysign(sqrt(z_sq), R1.getX() - R0.getY());
    return q;
}



/**
 * @brief SignalGateway class is used to store the signals to
 * SystemC module input/output ports. The ports of the modules
 * must be bound before the simulation starts
 *
 * @tparam T Template datatype
 */
template <typename T>
class SignalGateway {
    AbstractFilter<T> *filter; // Pointer to the filter
    public:
    //double rmse;// Value to store rmse
    double trmse;
    double eachrmse;
    int dataset_count;
    sc_signal<T2> gx; // Gyroscope signals
    sc_signal<T2> gy;
    sc_signal<T2> gz;
    sc_signal<T2> ax; // Accelerometer signals
    sc_signal<T2> ay;
    sc_signal<T2> az;
    sc_signal<T2> mx; // Magnetometer signals
    sc_signal<T2> my;
    sc_signal<T2> mz;
    sc_signal<T> q0; // Quaternion outputs
    sc_signal<T> q1;
    sc_signal<T> q2;
    sc_signal<T> q3;
    /**
     * @brief Construct a new SignalGateway object
     *
     * @param filter Pointer to the filter
     */
    SignalGateway(AbstractFilter<T> *filter) {
        dataset_count = 0;
        //rmse = 0.0;
        eachrmse=0.0;
        trmse =0.0;
        this->filter = filter;
        // Bind signals
        filter->gxin(gx);
        filter->gyin(gy);
        filter->gzin(gz);
        filter->axin(ax);
        filter->ayin(ay);
        filter->azin(az);
        filter->mxin(mx);
        filter->myin(my);
        filter->mzin(mz);
        filter->q0out(q0);
        filter->q1out(q1);
        filter->q2out(q2);
        filter->q3out(q3);
    }
    /**
     * @brief Put sensor data into the module or filter ports through signals
     *
     */
    void putData(T2 _gx,T2 _gy,T2 _gz,T2 _ax,T2 _ay,T2 _az,T2 _mx,T2 _my,T2 _mz) {
        gx = _gx;
        gy = _gy;
        gz = _gz;
        ax = _ax;
        ay = _ay;
        az = _az;
        mx = _mx;
        my = _my;
        mz = _mz;
    }
    // Set initial orientation values works as setter
    void setValues(T q0, T q1, T q2, T q3) {
        filter->setValues(q0, q1, q2, q3);
    }
};

/**
 * @brief Performer class stores SignalGateway objects and performs
 * the calculations by reading inputs from the datasets and using
 * filter estimates
 *
 * @tparam T Template parameter
 */
template <typename T>
class Performer {
    // Vector object to store SignalGateway pointers
    vector<SignalGateway<T>*> gateways;
    // Dataset files
    string files[39] = {
        "../broad/01.dat",
        "../broad/02.dat",
        "../broad/03.dat",
        "../broad/04.dat",
        "../broad/05.dat",
        "../broad/06.dat",
        "../broad/07.dat",
        "../broad/08.dat",
        "../broad/09.dat",
        "../broad/10.dat",
        "../broad/11.dat",
        "../broad/12.dat",
        "../broad/13.dat",
        "../broad/14.dat",
        "../broad/15.dat",
        "../broad/16.dat",
        "../broad/17.dat",
        "../broad/18.dat",
        "../broad/19.dat",
        "../broad/20.dat",
        "../broad/21.dat",
        "../broad/22.dat",
        "../broad/23.dat",
        "../broad/24.dat",
        "../broad/25.dat",
        "../broad/26.dat",
        "../broad/27.dat",
        "../broad/28.dat",
        "../broad/29.dat",
        "../broad/30.dat",
        "../broad/31.dat",
        "../broad/32.dat",
        "../broad/33.dat",
        "../broad/34.dat",
        "../broad/35.dat",
        "../broad/36.dat",
        "../broad/37.dat",
        "../broad/38.dat",
        "../broad/39.dat"
    };
    public:
    Performer(){}
    // push back the filter gateway into vector
    void add_gateway(SignalGateway<T> *gateway) {
        gateways.push_back(gateway);
    }

    /**
     * @brief This method performs tests using the datasets between
     * test_start and test_end.
     *
     * @param test_start Start index of datasets(loop variable initial value)
     * @param test_end End index of datasets(loop variable end value)
     */
    void performTests(int test_start, int test_end) {
        test_end = test_end <= 0 ? 0 : test_end <= 39 ? test_end : 39;
        test_start = test_start <= 0 ? 0 : test_start <= 39 ? test_start : 39;
        // Store mse for each filter/gateway along with filter gateway
        // a container holding a key value pair in the form of filter gateway with its corresponding mse
        // when we have two filter values, we need to find a way to discrimate
        map<SignalGateway<T>*,double> mse;
        //double msed = 0.0;
        long long row_count = 0;
        // Set mses to zero
        for(auto gateway:gateways)
            mse[gateway] = 0.0;

        // Multiplication quaternion from BROAD paper source code to make sure earth frame is ENU
        Quaternion<T> mul(1 / sqrt(2), 0, 0, 1 / sqrt(2));
        // Loop to read datasets
        for (size_t i = test_start; i < test_end; i++)
        {
            // Load dataset
            IMUDataset *dataset = new IMUDataset(files[i]);
            cout << "Working on: " << dataset->filename << ":" << dataset->row_count << endl;
            // We load the initial orientation from mag and acc
            Quaternion<double> initial = quatFromAccMag(
                dataset->acc[0][0],
                dataset->acc[0][1],
                dataset->acc[0][2],
                dataset->mag[0][0],
                dataset->mag[0][1],
                dataset->mag[0][2]);
            for(auto gateway:gateways)
                gateway->setValues(initial.w, initial.x, initial.y, initial.z);
            // Calculate the starting quaternion(angle) and put it into the gateways
            row_count = 0;
            for(auto gateway : gateways) {
                mse[gateway] = 0.0;
            }
            //msed=0.0;
            //  Loop to use each row in the dataset
            for (size_t j = 0; j < dataset->row_count; j++)
            {
                // Put data for each gateway
                for(SignalGateway<T> *gateway:gateways) {
                    gateway->putData(
                        dataset->gyr[j][0],
                        dataset->gyr[j][1],
                        dataset->gyr[j][2],
                        dataset->acc[j][0],
                        dataset->acc[j][1],
                        dataset->acc[j][2],
                        dataset->mag[j][0],
                        dataset->mag[j][1],
                        dataset->mag[j][2]);
                }
                // Advance the simulation
                sc_start(1, SC_SEC);

                // Calculate error for each gateway if there is movement and the opt_quat row is valid(not nan)
                if (dataset->movement[j] && !isnan(dataset->quat[j][0])) {
                    for(SignalGateway<T> *gateway:gateways) {
                        Quaternion<T> qOMC(dataset->quat[j][0], dataset->quat[j][1], dataset->quat[j][2], dataset->quat[j][3]);
                        //cout << qOMC << endl;
                        Quaternion<T> q(gateway->q0.read(), gateway->q1.read(), gateway->q2.read(), gateway->q3.read());
                        Quaternion<T> quat = mul * q;
                        // cout << j << ":" << quat;
                        quat.normalize();
                        qOMC.normalize();
                        Quaternion<T> out = quat * (~qOMC);
                        out.normalize();

                        double e = abs(out.w);
                        e = e <= 0 ? 0 : e <= 1 ? e : 1;
                        e = 2 * acos(e);
                        // cout << row_count << ":" << e << ":" << mse[gateway] <<endl;
                        mse[gateway] += e * e;
                        // if(row_count%1000 == 0)
                            //cout << j <<":" << mse[gateway]<< endl;
                    }
                    row_count += 1;
                }
            }
            for(auto gateway : gateways) {
                //cout << std::fixed;
                //cout << std::setprecision(2);
                gateway->eachrmse=sqrt(mse[gateway]/row_count) / M_PI * 180.0;
                cout<< "rmse: " << gateway->eachrmse <<endl;
                //gateway->rmse += sqrt(mse[gateway]/row_count) / M_PI * 180.0;
                gateway->trmse+=gateway->eachrmse;
                gateway->dataset_count++;
            }
            // Free dataset object
            delete dataset;
        }
        // Calculate rmse for each gateway
        for(auto gateway : gateways) {
            //cout << std::fixed;
            //cout << std::setprecision(2);
            gateway->trmse /= gateway->dataset_count;
            cout << "Total rmse: " << gateway->trmse << endl;
        }
    }
};
#define SCDATATYPE sc_fixed<15,3,SC_RND,SC_SAT>
int sc_main(int argc, char *argv[])
{
    MadgwickFilter<SCDATATYPE> filter1("Madgwick");
    filter1.setParameters(0.12, 285.71428571);

    MahonyFilter<SCDATATYPE> filter2("Mahony");
    filter2.setParameters(0.74, 0.0012, 285.71428571);

    BasicVQF<SCDATATYPE> filter3("basic_vqf");
    filter3.setParameters(3.0,9.0,0.0035,0.0035,0.0035);

    SignalGateway<SCDATATYPE> sg1(&filter1);
    SignalGateway<SCDATATYPE> sg2(&filter2);
    SignalGateway<SCDATATYPE> sg3(&filter3);


    Performer<SCDATATYPE> p;
    p.add_gateway(&sg1);
    p.add_gateway(&sg2);
    //p.add_gateway(&sg3); // when running tests for VQF, remove mul from line number 262
    // Perform tests
    p.performTests(0, 39);
    return 0;
}
