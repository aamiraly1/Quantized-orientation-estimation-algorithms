#ifndef REPORT_H
#define REPORT_H
//FILE: report.h (systemc)
//# vim600:sw=2:tw=0:fdm=marker
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// DESCRIPTION
//   This example illustrates the error reporting facilities of SystemC
//   version 2.1.
//
// DESIGN HIERARCHY
//   sc_main()
//   +- report_i
//      +- report_thread
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Module definition
SC_MODULE(report) {
  // Constructor declarations
  report(sc_module_name nm);
  // Process declarations
  void report_thread();
};
#endif
//Portions COPYRIGHT (C) 2004,2007 Eklectic Ally, Inc. DBA ESLX Inc.-----------{{{//
// Permission granted for anybody to use this template provided this  //
// acknowledgement of Eklectic Ally, Inc. remains.                    //
//-----------------------------------------------------------------}}}//
//END $Id: report.h,v 1.1 2004/03/01 02:46:53 dcblack Exp $
