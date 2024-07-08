#ifndef STIM_H
#define STIM_H
//BEGIN stim.h
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// See interrupt.h
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <systemc>
#include "ea_interrupt_gen_if.h"

SC_MODULE(stim) {
  sc_port<ea_interrupt_gen_if> irq_op;
  SC_CTOR(stim)
  {
    SC_THREAD(stim_thread);
  }
  void stim_thread(void);
};

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: stim.h,v 1.1 2004/02/10 22:47:00 dcblack Exp $
