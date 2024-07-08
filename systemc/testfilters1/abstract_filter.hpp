#include <systemc>
#ifndef ABSTRACT_FILTER_HPP
#define ABSTRACT_FILTER_HPP
using namespace sc_core;
using namespace sc_dt;
#define T2 sc_fixed<24,9,SC_RND,SC_SAT>
template <typename T>
class AbstractFilter : public sc_module
{
public:
    // Input ports
    sc_in<T2> gxin;
    sc_in<T2> gyin;
    sc_in<T2> gzin;
    sc_in<T2> axin;
    sc_in<T2> ayin;
    sc_in<T2> azin;
    sc_in<T2> mxin;
    sc_in<T2> myin;
    sc_in<T2> mzin;
    // Output ports
    sc_out<T> q0out;
    sc_out<T> q1out;
    sc_out<T> q2out;
    sc_out<T> q3out;
    // Pure virtual(abstract) filter method, implemented in filters
    virtual void filter() = 0;
    // Pure virtual reset method
    virtual void reset() = 0;
    // Constructor that calls the constructor of sc_module
    AbstractFilter(const sc_module_name& name) : sc_module(name) {}
    T q0, q1, q2, q3;
    void setValues(T q0, T q1, T q2, T q3) {
        reset();
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;
    }
};

#endif
