#ifndef WAVE_H
#define WAVE_H
//FILE: wave.h (systemc)
//# vim600:sw=2:tw=0:fdm=marker
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// DESCRIPTION
//   This design creates two signals and provides a
//   VCD (Value Change Dump) wave-form output file.
//
// DESIGN HIERARCHY
//   sc_main()
//   +- wave_i
//    +- wave_thread
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Module definition
SC_MODULE(wave) {
  // Local channel & submodule declarations
  sc_signal<bool>    oscillate;
  sc_signal<double>  pressure;
  sc_trace_file*  tracefile;
  wave(sc_module_name nm); // Constructor
  ~wave();// Destructor
  // Process declarations
  void wave_thread();
private:
  // Local data members
  double temperature;
  unsigned cylinder;
};
#endif
//Portions COPYRIGHT (C) 2004,2007 Eklectic Ally, Inc. DBA ESLX Inc. ------------{{{//
// Permission granted for anybody to use this template provided this  //
// acknowledgement of Eklectic Ally, Inc. remains.                    //
//-----------------------------------------------------------------}}}//
//END $Id: wave.h,v 1.1 2004/02/29 04:41:14 dcblack Exp $
