/* -*- c++ -*- */

#define PFB_CHANNELIZER_API
#define ETTUS_API

%include "gnuradio.i"/*			*/// the common stuff

//load generated python docstrings
%include "pfb_channelizer_swig_doc.i"
//Header from gr-ettus
%include "ettus/device3.h"
%include "ettus/rfnoc_block.h"
%include "ettus/rfnoc_block_impl.h"

%{
#include "ettus/device3.h"
#include "ettus/rfnoc_block_impl.h"
#include "pfb_channelizer/chanmux.h"
#include "pfb_channelizer/poly_channelizer.h"
%}

%include "pfb_channelizer/chanmux.h"
GR_SWIG_BLOCK_MAGIC2(pfb_channelizer, chanmux);
%include "pfb_channelizer/poly_channelizer.h"
GR_SWIG_BLOCK_MAGIC2(pfb_channelizer, poly_channelizer);
