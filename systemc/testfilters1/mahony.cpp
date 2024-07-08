#include<systemc>
#include "abstract_filter.hpp"

#ifndef MAHONY_FILTER_HPP
#define MAHONY_FILTER_HPP
using namespace sc_core;
using namespace sc_dt;
template <typename T>
#define T2 sc_fixed<24,9,SC_RND,SC_SAT>
#define T3 sc_fixed<32,14,SC_RND,SC_SAT>
class MahonyFilter: public AbstractFilter<T> {
    T Kp, Ki;
    T twoKp;									// 2 * proportional gain (Kp)
    T twoKi;									// 2 * integral gain (Ki)
    //T q0, q1, q2, q3;					        // quaternion of sensor frame relative to auxiliary frame
    T integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki
    float sampleFreq;

    public:
    // Create constructor that derives from AbstractFilter
    SC_HAS_PROCESS(MahonyFilter);
    MahonyFilter(const sc_module_name& name) : AbstractFilter<T>(name) {
        SC_METHOD(filter);
        this->sensitive << this->gxin
                        << this->gyin
                        << this->gzin
                        << this->axin
                        << this->ayin
                        << this->azin
                        << this->mxin
                        << this->myin
                        << this->mzin;
    }

    // Set filter parameters 
    void setParameters(T Kp, T Ki, float sampleFreq) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->sampleFreq = sampleFreq;
        reset();
    }

    void reset() override {
        twoKp = 2 * Kp;
        twoKi = 2 * Ki;
        integralFBx = 0.0;
        integralFBy = 0.0;
        integralFBz = 0.0;
        this->q0 = 1.0;
        this->q1 = 0.0;
        this->q2 = 0.0;
        this->q3 = 0.0;
    }
     /**
     * @brief This method is used to normalize 4 values
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
        // Read the sensor value coming from signals
        T2 gx = this->gxin.read();
        T2 gy = this->gyin.read();
        T2 gz = this->gzin.read();
        T2 ax = this->axin.read();
        T2 ay = this->ayin.read();
        T2 az = this->azin.read();
        T2 mx = this->mxin.read();
        T2 my = this->myin.read();
        T2 mz = this->mzin.read();

        T recipNorm;
        T q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        T hx, hy, bx, bz;
        T halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        T halfex, halfey, halfez;
        T qa, qb, qc;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
            filterIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            //std::cout<<ax<<std::endl;
            //std::cout<<ay<<std::endl;
            //std::cout<<az<<std::endl;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
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
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5 + q3q3;
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute and apply integral feedback if enabled
            if(twoKi > 0.0) {
                integralFBx += twoKi * halfex * (1.0 / sampleFreq);	// integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0 / sampleFreq);
                integralFBz += twoKi * halfez * (1.0 / sampleFreq);
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            }
            else {
                integralFBx = 0.0;	// prevent integral windup
                integralFBy = 0.0;
                integralFBz = 0.0;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
            //std::cout<<gx<<std::endl;
            //std::cout<<gy<<std::endl;
            //std::cout<<gz<<std::endl;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5 * (1.0 / sampleFreq));		// pre-multiply common factors
        gy *= (0.5 * (1.0 / sampleFreq));
        gz *= (0.5 * (1.0 / sampleFreq));
        qa = this->q0;
        qb = this->q1;
        qc = this->q2;
        this->q0 += (-qb * gx - qc * gy - this->q3 * gz);
        this->q1 += (qa * gx + qc * gz - this->q3 * gy);
        this->q2 += (qa * gy - qb * gz + this->q3 * gx);
        this->q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        //recipNorm = invSqrt(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
        //this->q0 *= recipNorm;
        //this->q1 *= recipNorm;
        //this->q2 *= recipNorm;
        //this->q3 *= recipNorm;
        normalize4(this->q0,this->q1,this->q2,this->q3);
        // Write output port values
        this->q0out.write(this->q0);
        this->q1out.write(this->q1);
        this->q2out.write(this->q2);
        this->q3out.write(this->q3);
    }

    void filterIMU(T gx, T gy, T gz, T ax, T ay, T az) {
        T recipNorm;
        T halfvx, halfvy, halfvz;
        T halfex, halfey, halfez;
        T qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = this->q1 * this->q3 - this->q0 * this->q2;
            halfvy = this->q0 * this->q1 + this->q2 * this->q3;
            halfvz = this->q0 * this->q0 - 0.5 + this->q3 * this->q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute and apply integral feedback if enabled
            if(twoKi > 0.0) {
                integralFBx += twoKi * halfex * (1.0 / sampleFreq);	// integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0 / sampleFreq);
                integralFBz += twoKi * halfez * (1.0 / sampleFreq);
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            }
            else {
                integralFBx = 0.0;	// prevent integral windup
                integralFBy = 0.0;
                integralFBz = 0.0;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
            //std::cout<<gx<<std::endl;
            //std::cout<<gy<<std::endl;
            //std::cout<<gz<<std::endl;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5 * (1.0 / sampleFreq));		// pre-multiply common factors
        gy *= (0.5 * (1.0 / sampleFreq));
        gz *= (0.5 * (1.0 / sampleFreq));
        qa = this->q0;
        qb = this->q1;
        qc = this->q2;
        this->q0 += (-qb * gx - qc * gy - this->q3 * gz);
        this->q1 += (qa * gx + qc * gz - this->q3 * gy);
        this->q2 += (qa * gy - qb * gz + this->q3 * gx);
        this->q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        //recipNorm = invSqrt(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
        //this->q0 *= recipNorm;
        //this->q1 *= recipNorm;
        //this->q2 *= recipNorm;
        //this->q3 *= recipNorm;
        normalize4(this->q0,this->q1,this->q2,this->q3);

        this->q0out.write(this->q0);
        this->q1out.write(this->q1);
        this->q2out.write(this->q2);
        this->q3out.write(this->q3);
    }
    // Inverse sqrt
    T3 invSqrt(T3 x) {
        return 1.0/sqrt(x);
    }
};

#endif
