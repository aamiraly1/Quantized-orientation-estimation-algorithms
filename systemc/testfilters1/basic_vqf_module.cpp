#include<systemc>
#include "abstract_filter.hpp"
#include <iostream>



#ifndef BASIC_VQF_FILTER_HPP
#define BASIC_VQF_FILTER_HPP
using namespace sc_core;
//#define VQF_SC // comment to use double type
#ifndef VQF_SC
typedef double vqf_real_t;
#else
typedef sc_fixed<32,16,SC_TRN,SC_SAT> vqf_real_t;
#endif /* VQF_SC */
#define T2 sc_fixed<24,9,SC_RND,SC_SAT>
#define T3 sc_fixed<32,14,SC_RND,SC_SAT>
#define EPS std::numeric_limits<vqf_real_t>::epsilon()
#define NaN std::numeric_limits<vqf_real_t>::quiet_NaN()

template <typename T> 
class BasicVQFParams
{
    public:
        BasicVQFParams(){}
        T tauAcc;
        T tauMag;
};

template <typename T> 
class BasicVQFState {
    public:
        T gyrQuat[4];
        T accQuat[4];
        T delta;
        T2 lastAccLp[3];
        double accLpState[3*2]; // Internal low-pass filter state for lastAccLp.
        T kMagInit;
};

template <typename T> 
class BasicVQFCoefficients
{
    public:
        T gyrTs;
        T accTs;
        T magTs;
        double accLpB[3]; // Numerator coefficients of the acceleration low-pass filter
        double accLpA[2]; // Denominator coefficients of the acceleration low-pass filter
        T kMag;
};

//thanhnt

template <typename T>
class BasicVQF: public AbstractFilter<T> {


     BasicVQFParams<T> params;
     BasicVQFState<T> state;
     BasicVQFCoefficients<T> coeffs;
     size_t count = 0;

    public:
    SC_HAS_PROCESS(BasicVQF);
    BasicVQF(const sc_module_name& name) : AbstractFilter<T>(name) {
        SC_METHOD(filter);
        this->sensitive << this->gxin << this->gyin << this->gzin << this->axin << this->ayin << this->azin << this->mxin << this->myin << this->mzin;
    }

    void setParameters(T tauAcc, T tauMag, T gyrTs, T accTs, T magTs) {
        this->params.tauAcc = tauAcc; //Time constant for accelerometer low-pass filtering in seconds.
        this->params.tauMag = tauMag; //Time constant for magnetometer update in seconds
        this->coeffs.gyrTs = gyrTs; //Sampling time of the gyroscope measurements (in seconds)
        this->coeffs.accTs = accTs; //Sampling time of the accelerometer measurements (in seconds)
        this->coeffs.magTs = magTs; //Sampling time of the magnetometer measurements (in seconds)
        reset();
        setup();
    
    }

    void reset() override {
         this->q0 = 1.0f;
         this->q1 = 0.0f;
         this->q2 = 0.0f;
         this->q3 = 0.0f;
    }

    void updateGyr(const T2 gyr[3])
    {
        T gyrNorm = norm2(gyr, 3);
        T angle = gyrNorm * this->coeffs.gyrTs;
        if (gyrNorm > EPS) {
            T c = cos(angle / 2);
            T s = sin(angle / 2) / gyrNorm;
            T gyrStepQuat[4] = { c, s * gyr[0], s * gyr[1], s * gyr[2] };
            quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
            normalize(state.gyrQuat, 4);
        }
    }

    void updateAcc(const T2 acc[3])
    {
        if (acc[0] == T(0.0) && acc[1] == T(0.0) && acc[2] == T(0.0)) {
            return;
        }

        T accEarth[3];
        quatRotate(this->state.gyrQuat, acc, accEarth);
        filterVec(accEarth, 3, this->params.tauAcc, this->coeffs.accTs, this->coeffs.accLpB, this->coeffs.accLpA, this->state.accLpState, this->state.lastAccLp);
        quatRotate(this->state.accQuat, this->state.lastAccLp, accEarth);
        normalize(accEarth, 3);

        T accCorrQuat[4];
        T q_w = sqrt((accEarth[2] + 1) / 2);
        if (q_w > 1e-6) {
            accCorrQuat[0] = q_w;
            accCorrQuat[1] = 0.5 * accEarth[1] / q_w;
            accCorrQuat[2] = -0.5 * accEarth[0] / q_w;
            accCorrQuat[3] = 0;
        } else {
            accCorrQuat[0] = 0;
            accCorrQuat[1] = 1;
            accCorrQuat[2] = 0;
            accCorrQuat[3] = 0;
        }

        quatMultiply(accCorrQuat, state.accQuat, state.accQuat);
        normalize(state.accQuat, 4);
    }

    void updateMag(const T2 mag[3])
    {
        // ignore [0 0 0] samples
        if (mag[0] == T(0.0) && mag[1] == T(0.0) && mag[2] == T(0.0)) {
            return;
        }

        T magEarth[3];

        // bring magnetometer measurement into 6D earth frame
        T accGyrQuat[4];
        getQuat6D(accGyrQuat);
        quatRotate(accGyrQuat, mag, magEarth);

        // calculate disagreement angle based on current magnetometer measurement
        T magDisAngle = atan2(magEarth[0], magEarth[1]) - state.delta;

        // make sure the disagreement angle is in the range [-pi, pi]
        if (magDisAngle > T(M_PI)) {
            magDisAngle -= T(2*M_PI);
        } else if (magDisAngle < T(-M_PI)) {
            magDisAngle += T(2*M_PI);
        }

        T k = this->coeffs.kMag;

        // ensure fast initial convergence
        if (this->state.kMagInit != T(0.0)) {
            // make sure that the gain k is at least 1/N, N=1,2,3,... in the first few samples
            if (k < this->state.kMagInit) {
                k = this->state.kMagInit;
            }

            // iterative expression to calculate 1/N
            state.kMagInit = state.kMagInit/(state.kMagInit+1);

            // disable if t > tauMag
            if (this->state.kMagInit*this->params.tauMag < this->coeffs.magTs) {
                this->state.kMagInit = 0.0;
            }
        }

        this->state.delta += k*magDisAngle;
        if (this->state.delta > T(M_PI)) {
            this->state.delta -= T(2*M_PI);
        } else if (state.delta < T(-M_PI)) {
            this->state.delta += T(2*M_PI);
        }
    }

    void update(const T2 gyr[3], const T2 acc[3], const T2 mag[3])
    {
        updateGyr(gyr);
        updateAcc(acc);
        updateMag(mag);
    }

    void updateBatch(const T2 gyr[], const T2 acc[], const T2 mag[], size_t N,
                            T out6D[], T out9D[], T outDelta[])
    {
        for (size_t i = 0; i < N; i++) {
            if (mag) {
                update(gyr+3*i, acc+3*i, mag+3*i);
            } else {
                update(gyr+3*i, acc+3*i);
            }
            if (out6D) {
                getQuat6D(out6D+4*i);
            }
            if (out9D) {
                getQuat9D(out9D+4*i);
            }
            if (outDelta) {
                outDelta[i] = state.delta;
            }
        }
    }

    void getQuat3D(T out[4])
    {
        std::copy(this->state.gyrQuat, this->state.gyrQuat+4, out);
    }

    void getQuat6D(T out[4])
    {
        quatMultiply(this->state.accQuat, this->state.gyrQuat, out);
    }

    void getQuat9D(T out[4])
    {
        quatMultiply(this->state.accQuat, this->state.gyrQuat, out);
        quatApplyDelta(out, this->state.delta, out);
    }

    T getDelta()
    {
        return this->state.delta;
    }

    /*void setTauAcc(T tauAcc)
    {
        if (this->params.tauAcc == tauAcc) {
            return;
        }
        this->params.tauAcc = tauAcc;
        double newB[3];
        double newA[3];

        filterCoeffs(this->params.tauAcc, this->coeffs.accTs, newB, newA);
        filterAdaptStateForCoeffChange(this->state.lastAccLp, 3, this->coeffs.accLpB, this->coeffs.accLpA, newB, newA, this->state.accLpState);

        std::copy(newB, newB+3, this->coeffs.accLpB);
        std::copy(newA, newA+2, this->coeffs.accLpA);
    }

    void setTauMag(T tauMag)
    {
        this->params.tauMag = tauMag;
        this->coeffs.kMag = gainFromTau(this->params.tauMag, this->coeffs.magTs);
    }*/

    // const getParams() const // thanhnt may not need
    // {
    //     return this->params;
    // }

    // const getCoeffs() const
    // {
    //     return this->coeffs;
    // }

    // const getState() const
    // {
    //     return this->state;
    // }

    // void setState(const BasicVQFState& state)
    // {
    //     this->state = state;
    // }

    void resetState()
    {
        quatSetToIdentity(this->state.gyrQuat);
        quatSetToIdentity(this->state.accQuat);
        state.delta = 0.0;

        std::fill(this->state.lastAccLp, this->state.lastAccLp+3, 0);
        std::fill(this->state.accLpState, this->state.accLpState + 3*2, NaN);

        this->state.kMagInit = 1.0;
    }

    void quatMultiply(const T q1[4], const T q2[4], T out[4])
    {
        T w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
        T x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
        T y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
        T z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
        out[0] = w; out[1] = x; out[2] = y; out[3] = z;
    }

    void quatConj(const T q[4], T out[4])
    {
        T w = q[0];
        T x = -q[1];
        T y = -q[2];
        T z = -q[3];
        out[0] = w; out[1] = x; out[2] = y; out[3] = z;
    }

    void quatSetToIdentity(T out[4])
    {
        out[0] = 1;
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
    }

    void quatApplyDelta(T q[], T delta, T out[])
    {
        // out = quatMultiply([cos(delta/2), 0, 0, sin(delta/2)], q)
        T c = cos(delta/2);
        T s = sin(delta/2);
        T w = c * q[0] - s * q[3];
        T x = c * q[1] - s * q[2];
        T y = c * q[2] + s * q[1];
        T z = c * q[3] + s * q[0];
        out[0] = w; out[1] = x; out[2] = y; out[3] = z;
    }

    void quatRotate(const T q[4], const T2 v[3], T out[3])
    {
        T x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
        T y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
        T z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
        out[0] = x; out[1] = y; out[2] = z;
    }

    T norm(const T vec[], size_t N)
    {
        T3 s = 0;
        for(size_t i = 0; i < N; i++) {
            s += vec[i]*vec[i];
        }
        return sqrt(s);
    }

    void normalize(T vec[], size_t N) { // with VQF: void VQF::normalize(vqf_real_t vec[], size_t N = 4)
        T3 n = norm(vec, N);
        if (n < EPS) {
                return;
            }
        for(size_t i = 0; i < N; i++) {
            vec[i] /= n;
        }
    }
    T2 norm2(const T2 vec[], size_t N)
    {
        T3 s = 0;
        for(size_t i = 0; i < N; i++) {
            s += vec[i]*vec[i];
        }
        return sqrt(s);
    }

    void clip(T vec[], T N, T min, T max)
    {
        for(size_t i = 0; i < N; i++) {
            if (vec[i] < min) {
                vec[i] = min;
            } else if (vec[i] > max) {
                vec[i] = max;
            }
        }
    }

    T gainFromTau(T tau, T Ts)
    {
        //assert(Ts > 0);
        if (tau < T(0.0)) {
            return 0; // k=0 for negative tau (disable update)
        } else if (tau == T(0.0)) {
            return 1; // k=1 for tau=0
        } else {
            return 1 - exp(-Ts/tau);  // fc = 1/(2*pi*tau)
        }
    }

    void filterCoeffs(T tau, T Ts, double outB[], double outA[])
    {
        // assert(tau > 0);
        // assert(Ts > 0);
        // second order Butterworth filter based on https://stackoverflow.com/a/52764064
        double fc = (M_SQRT2 / (2.0*M_PI))/double(tau); // time constant of dampened, non-oscillating part of step response
        double C = tan(M_PI*fc*double(Ts));
        double D = C*C + sqrt(2)*C + 1;
        double b0 = C*C/D;
        outB[0] = b0;
        outB[1] = 2*b0;
        outB[2] = b0;
        // a0 = 1.0
        outA[0] = 2*(C*C-1)/D; // a1
        outA[1] = (1-sqrt(2)*C+C*C)/D; // a2
    }

    void filterInitialState(T x0, const double b[3], const double a[2], double out[])
    {
        out[0] = x0*(1 - b[0]);
        out[1] = x0*(b[2] - a[1]);
    }

    void filterAdaptStateForCoeffChange(T last_y[], size_t N, const double b_old[],
                                                const double a_old[], const double b_new[],
                                                const double a_new[], double state[])
    {
        if (isnan(state[0])) {
            return;
        }
        for (size_t i = 0; i < N; i++) {
            state[0+2*i] = state[0+2*i] + (b_old[0] - b_new[0])*last_y[i];
            state[1+2*i] = state[1+2*i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0])*last_y[i];
        }
    }

    T filterStep(T x, const double b[3], const double a[2], double state[2])
    {
        double y = b[0]*x + state[0];
        state[0] = b[1]*x - a[0]*y + state[1];
        state[1] = b[2]*x - a[1]*y;
        return y;
    }

    void filterVec(const T x[], size_t N, T tau, T Ts, const double b[3],
                            const double a[2], double state[], T2 out[])
    {
        assert(N>=2);
        if (isnan(state[0])) { // initialization phase
            if (isnan(state[1])) { // first sample
                state[1] = 0; // state[1] is used to store the sample count
                for(size_t i = 0; i < N; i++) {
                    state[2+i] = 0; // state[2+i] is used to store the sum
                }
            }
            state[1]++;
            for (size_t i = 0; i < N; i++) {
                state[2+i] += x[i];
                out[i] = state[2+i]/state[1];
            }
            if (state[1]*Ts >= tau) {
                for(size_t i = 0; i < N; i++) {
                filterInitialState(out[i], b, a, state+2*i);
                }
            }
            return;
        }

        for (size_t i = 0; i < N; i++) {
            out[i] = filterStep(x[i], b, a, state+2*i);
        }
    }

    void setup()
    {
#ifdef VQF_SC
        coeffs.gyrTs = standard_value(coeffs.gyrTs);
        coeffs.accTs = standard_value(coeffs.accTs);
        coeffs.magTs = standard_value(coeffs.magTs);
#endif
        //assert(coeffs.gyrTs > 0);
        //assert(coeffs.accTs > 0);
        //assert(coeffs.magTs > 0);

        filterCoeffs(this->params.tauAcc, this->coeffs.accTs, this->coeffs.accLpB, this->coeffs.accLpA);

        coeffs.kMag = gainFromTau(this->params.tauMag, this->coeffs.magTs);

        resetState();
    }
    
    void filter() override {
        T2 gyr[3], acc[3], mag[3];
        T recipNorm;

        gyr[0] = this->gxin.read();
        gyr[1] = this->gyin.read();
        gyr[2] = this->gzin.read();
        acc[0] = this->axin.read();
        acc[1] = this->ayin.read();
        acc[2] = this->azin.read();
        mag[0] = this->mxin.read();
        mag[1] = this->myin.read();
        mag[2] = this->mzin.read();


#ifdef VQF_SC
        // gyr[0] = standard_value(gyr[0]);
        // gyr[1] = standard_value(gyr[1]);
        // gyr[2] = standard_value(gyr[2]);

        // acc[0] = standard_value(acc[0]);
        // acc[1] = standard_value(acc[1]);
        // acc[2] = standard_value(acc[2]);

        // mag[0] = standard_value(mag[0]);
        // mag[1] = standard_value(mag[1]);
        // mag[2] = standard_value(mag[2]); 
#endif
    
        T out[4] = {0, 0, 0, 0};

        
        update(gyr, acc, mag);
        getQuat9D(out);
        

        this->q0 = out[0];
        this->q1 = out[1];
        this->q2 = out[2];
        this->q3 = out[3];

        // Normalise quaternion
        recipNorm = invSqrt(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
        this->q0 *= recipNorm;
        this->q1 *= recipNorm;
        this->q2 *= recipNorm;
        this->q3 *= recipNorm;

        // Write output port values
        this->q0out.write(this->q0);
        this->q1out.write(this->q1);
        this->q2out.write(this->q2);
        this->q3out.write(this->q3);
    }
    float invSqrt(float x) {
        return 1.0f/sqrtf(x);
    }
};

#endif
