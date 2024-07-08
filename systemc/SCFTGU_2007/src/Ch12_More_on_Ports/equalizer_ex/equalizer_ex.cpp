//BEGIN equalizer_ex.cpp
//# vim600:set sw=2 tw=0 fdm=marker:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//See equalizer_ex.h for more information
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// systemc
#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;
#include "equalizer_ex.h"

void equalizer_ex::equalizer_thread(void) {//{{{
  for(;;) { 
    double sample;
    double result;
    wait();
    raw_fifo_ip->nb_read(sample);
    // ...
    // process data
    // ...
    equalized_fifo_op->write(result);
  }//endforever
}//}}}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//END $Id: equalizer_ex.cpp,v 1.2 2004/02/02 12:33:37 dcblack Exp $
