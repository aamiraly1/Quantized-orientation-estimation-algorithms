#ifndef DEVICE_H
#define DEVICE_H
//BEGIN device.h
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// See varports.h
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <systemc.h>

SC_MODULE(device) {
  sc_fifo_in<int>  rcv_p;
  sc_fifo_out<int> xmt_p;
  SC_CTOR(device)
  {
    SC_THREAD(rcv_thread);
    SC_THREAD(xmt_thread);
  }
  void rcv_thread();
  void xmt_thread();
};

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: device.h,v 1.1 2004/02/21 21:04:35 dcblack Exp $
