#ifndef SYSTEMC_INCLUDED
#define SYSTEMC_INCLUDED
// Just the data type includes from SYSTEMC (i.e. sc_dt:: namespace)
#  include "sysc/datatypes/bit/sc_bit.h"
#  include "sysc/datatypes/bit/sc_logic.h"
#  include "sysc/datatypes/bit/sc_bv.h"
#  include "sysc/datatypes/bit/sc_lv.h"
#  include "sysc/datatypes/int/sc_bigint.h"
#  include "sysc/datatypes/int/sc_biguint.h"
#  include "sysc/datatypes/int/sc_int.h"
#  include "sysc/datatypes/int/sc_uint.h"
#  include "sysc/datatypes/misc/sc_concatref.h" 
#  include "sysc/datatypes/fx/fx.h"
using sc_dt::sc_bit;
using sc_dt::sc_logic;
using sc_dt::sc_bv_base;
using sc_dt::sc_bv;
using sc_dt::sc_lv_base;
using sc_dt::sc_lv;
using sc_dt::int64;
using sc_dt::uint64;
using sc_dt::sc_numrep;
using sc_dt::SC_NOBASE;
using sc_dt::SC_BIN;
using sc_dt::SC_OCT;
using sc_dt::SC_DEC;
using sc_dt::SC_HEX;
using sc_dt::SC_BIN_US;
using sc_dt::SC_BIN_SM;
using sc_dt::SC_OCT_US;
using sc_dt::SC_OCT_SM;
using sc_dt::SC_HEX_US;
using sc_dt::SC_HEX_SM;
using sc_dt::SC_CSD;
using sc_dt::sc_io_show_base;
using sc_dt::SC_LOGIC_0;
using sc_dt::SC_LOGIC_1;
using sc_dt::SC_LOGIC_Z;
using sc_dt::SC_LOGIC_X;
using sc_dt::sc_length_param;
using sc_dt::sc_length_context;
using sc_dt::sc_signed;
using sc_dt::sc_bigint;
using sc_dt::sc_unsigned;
using sc_dt::sc_biguint;
using sc_dt::sc_int_base;
using sc_dt::sc_int;
using sc_dt::sc_uint_base;
using sc_dt::sc_uint;
using sc_dt::sc_fxnum;
using sc_dt::sc_fxnum_bitref;
using sc_dt::sc_fxnum_fast;
using sc_dt::sc_fix;
using sc_dt::sc_fix_fast;
using sc_dt::sc_ufix;
using sc_dt::sc_ufix_fast;
using sc_dt::sc_fixed;
using sc_dt::sc_fixed_fast;
using sc_dt::sc_ufixed;
using sc_dt::sc_ufixed_fast;
using sc_dt::sc_fxval;
using sc_dt::sc_fxval_fast;
using sc_dt::sc_fxcast_switch;
using sc_dt::sc_fxcast_context;
using sc_dt::sc_fxtype_params;
using sc_dt::sc_fxtype_context;
using sc_dt::sc_q_mode;
using sc_dt::SC_RND;
using sc_dt::SC_RND_ZERO;
using sc_dt::SC_RND_MIN_INF;
using sc_dt::SC_RND_INF;
using sc_dt::SC_RND_CONV;
using sc_dt::SC_TRN;
using sc_dt::SC_TRN_ZERO;
using sc_dt::sc_o_mode;
using sc_dt::SC_SAT;
using sc_dt::SC_SAT_ZERO;
using sc_dt::SC_SAT_SYM;
using sc_dt::SC_WRAP;
using sc_dt::SC_WRAP_SM;
using sc_dt::sc_switch;
using sc_dt::SC_OFF;
using sc_dt::SC_ON;
using sc_dt::sc_fmt;
using sc_dt::SC_F;
using sc_dt::SC_E;
using sc_dt::sc_context_begin;
using sc_dt::SC_NOW;
using sc_dt::SC_LATER;
#endif
