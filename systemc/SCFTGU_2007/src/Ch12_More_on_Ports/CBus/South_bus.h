//BEGIN: South_bus.h (systemc)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//See CBus.h for more information
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifndef SOUTH_BUS_H
#define SOUTH_BUS_H
#include "CBus_if.h"
struct South_Bus :public sc_interface, public CBus_South_if  {
  void write(unsigned long addr, long data);
  long read(unsigned long  addr);
  void irq(unsigned long addr);
};

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: CBus_if.h,v 1.1 2004/03/03 03:18:36 dcblack Exp $


