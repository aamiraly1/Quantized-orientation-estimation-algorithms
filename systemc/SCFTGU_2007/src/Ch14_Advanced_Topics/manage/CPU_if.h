#ifndef CPU_IF_H
#define CPU_IF_H
//BEGIN: CPU_if.h (systemc)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//See hier_chan.h for more information
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

struct CPU_if: public sc_interface {
  virtual void write(unsigned long addr, long data)=0;
  virtual long read(unsigned long  addr)=0;
};

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: CPU_if.h,v 1.1 2004/02/29 04:39:35 dcblack Exp $
