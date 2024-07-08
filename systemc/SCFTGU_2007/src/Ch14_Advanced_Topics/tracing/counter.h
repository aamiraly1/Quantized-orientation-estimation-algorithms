#ifndef counter_H
#define counter_H
//FILE: counter.h (systemc)
//# vim600:set sw=2 tw=0 fdm=marker:
// Module definition
#include <systemc>
SC_MODULE(counter) {
  // Ports
  sc_core::sc_in<sc_dt::sc_uint<2> >  transact_ip;
  sc_core::sc_in<unsigned>    addr_ip;
  sc_core::sc_in<int>         data_in_ip;
  sc_core::sc_out<int>        data_out_op;
  // Constructor declarations
  SC_CTOR(counter);
  // Processe declarations
  void counter_thread();
  enum transaction {IDLE, READ, WRITE};
};
#endif
//Portions COPYRIGHT (C) 2004 Eklectic Ally, Inc.------------------{{{//
// Permission granted for anybody to use this template provided this  //
// acknowledgement of Eklectic Ally, Inc. remains.                    //
//-----------------------------------------------------------------}}}//
//END $Id: counter.h 761 2010-02-24 16:47:11Z dcblack $
