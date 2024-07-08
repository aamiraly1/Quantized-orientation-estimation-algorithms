#include<systemc>
#include "abstract_filter.hpp"

#ifndef MADGWICK_FILTER_HPP
#define MADGWICK_FILTER_HPP
using namespace sc_core;
using namespace sc_dt;
#define T2 sc_fixed<24,9,SC_RND,SC_SAT>
#define T3 sc_fixed<32,14,SC_RND,SC_SAT>
template <typename T>
class MadgwickFilter: public AbstractFilter<T> {

    T beta;
    float sampleFreq;
    public:
    SC_HAS_PROCESS(MadgwickFilter);
    MadgwickFilter(const sc_module_name& name) : AbstractFilter<T>(name) {
        SC_METHOD(filter);
        this->sensitive << this->gxin << this->gyin << this->gzin << this->axin << this->ayin << this->azin << this->mxin << this->myin << this->mzin;
    }

    void setParameters(T beta, float sampleFreq) {
        this->beta = beta;
        this->sampleFreq = sampleFreq;
        reset();
    }

    void reset() override {
        //[ 0.7f2379941,  0.0f2145037,  0.0f0400592, -0.6f8966532] for 01.mfat
        // this->q0 = 0.7f2379941;
        // this->q1 = 0.0f2145037;
        // this->q2 = 0.0f0400592;
        // this->q3 = -0.6f8966532;
        this->q0 = 1.0f;
        this->q1 = 0.0f;
        this->q2 = 0.0f;
        this->q3 = 0.0f;
    }
    /**
     * @brief This method is used to normalize 4 values to avoid potential division by zero
     *
     * @param v1
     * @param v2
     * @param v3
     * @param v4
     */
    void normalize4(T &v1, T &v2, T &v3, T &v4) {
        T _v1 = v1;
        T _v2 = v2;
        T _v3 = v3;
        T _v4 = v4;
        T epsilon = T(1e-10);
        T3 total = _v1 * _v1 + _v2 * _v2 + _v3 * _v3 + _v4 * _v4;
        if (std::abs(total) <= epsilon)
        {
            return;
        }
        T3 norm = 1.0f/sqrtf(total);
        v1 *= norm;
        v2 *= norm;
        v3 *= norm;
        v4 *= norm;
    }
    void filter() override {
        T2 gx = this->gxin.read();
        T2 gy = this->gyin.read();
        T2 gz = this->gzin.read();
        T2 ax = this->axin.read();
        T2 ay = this->ayin.read();
        T2 az = this->azin.read();
        T2 mx = this->mxin.read();
        T2 my = this->myin.read();
        T2 mz = this->mzin.read();
        //std::cout << mz << std::endl;
        T recipNorm;
        T s0, s1, s2, s3;
        T qDot1, qDot2, qDot3, qDot4;
        T hx, hy;
        T _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            filterIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Rate of change of quaternion from gyroscope
        // computes the rate of change of the quaternion from gyroscope measurements
        // and applies feedback corrections based on accelerometer and magnetometer data
        qDot1 = 0.5f * (-this->q1 * gx - this->q2 * gy - this->q3 * gz);
        qDot2 = 0.5f * (this->q0 * gx + this->q2 * gz - this->q3 * gy);
        qDot3 = 0.5f * (this->q0 * gy - this->q1 * gz + this->q3 * gx);
        qDot4 = 0.5f * (this->q0 * gz + this->q1 * gy - this->q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {



            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * this->q0 * mx;
            _2q0my = 2.0f * this->q0 * my;
            _2q0mz = 2.0f * this->q0 * mz;
            _2q1mx = 2.0f * this->q1 * mx;
            _2q0 = 2.0f * this->q0;
            _2q1 = 2.0f * this->q1;
            _2q2 = 2.0f * this->q2;
            _2q3 = 2.0f * this->q3;
            _2q0q2 = 2.0f * this->q0 * this->q2;
            _2q2q3 = 2.0f * this->q2 * this->q3;
            q0q0 = this->q0 * this->q0;
            q0q1 = this->q0 * this->q1;
            q0q2 = this->q0 * this->q2;
            q0q3 = this->q0 * this->q3;
            q1q1 = this->q1 * this->q1;
            q1q2 = this->q1 * this->q2;
            q1q3 = this->q1 * this->q3;
            q2q2 = this->q2 * this->q2;
            q2q3 = this->q2 * this->q3;
            q3q3 = this->q3 * this->q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * this->q3 + _2q0mz * this->q2 + mx * q1q1 + _2q1 * my * this->q2 + _2q1 * mz * this->q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * this->q3 + my * q0q0 - _2q0mz * this->q1 + _2q1mx * this->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * this->q3 - my * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * this->q2 + _2q0my * this->q1 + mz * q0q0 + _2q1mx * this->q3 - mz * q1q1 + _2q2 * my * this->q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * this->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * this->q3 + _2bz * this->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * this->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * this->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * this->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * this->q2 + _2bz * this->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * this->q3 - _4bz * this->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * this->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * this->q2 - _2bz * this->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * this->q1 + _2bz * this->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * this->q0 - _4bz * this->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * this->q3 + _2bz * this->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * this->q0 + _2bz * this->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * this->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);


            //recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            //s0 *= recipNorm;
            //s1 *= recipNorm;
            //s2 *= recipNorm;
            //s3 *= recipNorm;
            normalize4(s0, s1, s2, s3);

            // Apply feedback step
            qDot1 = qDot1 - beta * s0;
            qDot2 = qDot2 - beta * s1;
            qDot3 = qDot3 - beta * s2;
            qDot4 = qDot4 - beta * s3;

        }

        // Integrate rate of change of quaternion to yield quaternion
        this->q0 = this->q0 + qDot1 * (1.0f / sampleFreq);
        this->q1 = this->q1 + qDot2 * (1.0f / sampleFreq);
        this->q2 = this->q2 + qDot3 * (1.0f / sampleFreq);
        this->q3 = this->q3 + qDot4 * (1.0f / sampleFreq);
        // Normalise quaternion
        normalize4(this->q0,this->q1,this->q2,this->q3);
        this->q0out.write(this->q0);
        this->q1out.write(this->q1);
        this->q2out.write(this->q2);
        this->q3out.write(this->q3);
    }
    void filterIMU(T gx, T gy, T gz, T ax, T ay, T az) {
        T recipNorm;
        T s0, s1, s2, s3;
        T qDot1, qDot2, qDot3, qDot4;
        T _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-this->q1 * gx - this->q2 * gy - this->q3 * gz);
        qDot2 = 0.5f * (this->q0 * gx + this->q2 * gz - this->q3 * gy);
        qDot3 = 0.5f * (this->q0 * gy - this->q1 * gz + this->q3 * gx);
        qDot4 = 0.5f * (this->q0 * gz + this->q1 * gy - this->q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * this->q0;
            _2q1 = 2.0f * this->q1;
            _2q2 = 2.0f * this->q2;
            _2q3 = 2.0f * this->q3;
            _4q0 = 4.0f * this->q0;
            _4q1 = 4.0f * this->q1;
            _4q2 = 4.0f * this->q2;
            _8q1 = 8.0f * this->q1;
            _8q2 = 8.0f * this->q2;
            q0q0 = this->q0 * this->q0;
            q1q1 = this->q1 * this->q1;
            q2q2 = this->q2 * this->q2;
            q3q3 = this->q3 * this->q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * this->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * this->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * this->q3 - _2q1 * ax + 4.0f * q2q2 * this->q3 - _2q2 * ay;
            //recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            //s0 *= recipNorm;
            //s1 *= recipNorm;
            //s2 *= recipNorm;
            //s3 *= recipNorm;
            normalize4(s0, s1, s2, s3);


            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        this->q0 += qDot1 * (1.0f / sampleFreq);
        this->q1 += qDot2 * (1.0f / sampleFreq);
        this->q2 += qDot3 * (1.0f / sampleFreq);
        this->q3 += qDot4 * (1.0f / sampleFreq);


        // Normalise quaternion
        normalize4(this->q0,this->q1,this->q2,this->q3);

        this->q0out.write(this->q0);
        this->q1out.write(this->q1);
        this->q2out.write(this->q2);
        this->q3out.write(this->q3);

    }

    T3 invSqrt(T3 x) {
        return 1.0f/sqrtf(x);
    }
};

#endif
