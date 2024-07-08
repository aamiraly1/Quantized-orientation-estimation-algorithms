#ifndef BUS_IF_H
#define BUS_IF_H
//BEGIN: bus_if.h (systemc)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//See processor.h for more information
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

struct bus_if: public sc_interface {
  virtual void write(unsigned addr, unsigned data)=0;
  virtual unsigned read(unsigned addr)=0;
};

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: bus_if.h,v 1.1 2004/02/22 19:50:50 dcblack Exp $
