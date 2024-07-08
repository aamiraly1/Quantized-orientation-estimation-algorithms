//BEGIN ea_pcix_trans.cpp
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <systemc.h>
#include "ea_pcix_trans.h"

// Print a PCIX transaction packet out to a stream (usually just the terminal
// window), in a nice-looking format
ostream& operator<<(ostream& os, const ea_pcix_trans& trans) {
  os << "{" << endl
     << "  cmnd: " << trans.cmnd   << ", "
     << "attr1:" << trans.attr1  << ", "
     << "attr2:" << trans.attr2  << "," << endl
     << "  devnum:"<< trans.devnum << ", "
     << "addr:"  << trans.addr   << "," << endl
     << "  data: " << trans.data[0] << "." << trans.data[1] << "." << trans.data[2] << "." << trans.data[3]
     <<      "." << trans.data[4] << "." << trans.data[5] << "." << trans.data[6] << "." << trans.data[7]
     << "," << endl
     << "  done:"  << (trans.done?"true":"false")  << endl
     << "}";                       
  return os;
}//end operator<<

// trace function, only required if actually used
void sc_trace(sc_trace_file*& tf, const ea_pcix_trans& trans, char& nm) {
  sc_trace(tf, trans.devnum,   strcat(&nm,".devnum"));
  sc_trace(tf, trans.addr,     strcat(&nm,".addr"));
  sc_trace(tf, trans.attr1,    strcat(&nm,".attr1"));
  sc_trace(tf, trans.attr2,    strcat(&nm,".attr2"));
  sc_trace(tf, trans.cmnd,     strcat(&nm,".cmnd"));
  sc_trace(tf, trans.data[0],  strcat(&nm,".data[0]"));
  sc_trace(tf, trans.data[1],  strcat(&nm,".data[1]"));
  sc_trace(tf, trans.data[2],  strcat(&nm,".data[2]"));
  sc_trace(tf, trans.data[3],  strcat(&nm,".data[3]"));
  sc_trace(tf, trans.data[4],  strcat(&nm,".data[4]"));
  sc_trace(tf, trans.data[5],  strcat(&nm,".data[5]"));
  sc_trace(tf, trans.data[6], strcat(&nm,".data[6]"));
  sc_trace(tf, trans.data[7], strcat(&nm,".data[7]"));
  sc_trace(tf, trans.done,     strcat(&nm,".done"));
}//end sc_trace

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: ea_pcix_trans.cpp,v 1.0 2004/29/11 21:29:45 dcblack Exp $
