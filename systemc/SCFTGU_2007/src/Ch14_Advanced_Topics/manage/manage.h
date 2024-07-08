#ifndef MANAGE_H
#define MANAGE_H
//BEGIN manage.h
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// DESCRIPTION
//   TO_BE_SUPPLIED
//
// DESIGN HIERARCHY
//   sc_main()
//   +- manage_i
//    +- manage_PROCESS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <systemc>
#include "mem.h"
#include "testbench.h"

SC_MODULE(manage) {
  mem* mem_i;
  testbench* testbench_i;
  SC_HAS_PROCESS(manage);
  manage(sc_module_name nm)
  : sc_module(nm)
  {
    testbench_i = new testbench("testbench_i",0,1000,100);
    mem_i = new mem("mem_i",0,1000);
    testbench_i->mem(mem_i->CPU_p);
  }
  void manage_PROCESS(void);
};

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: manage.h,v 1.1 2004/02/29 04:39:35 dcblack Exp $
