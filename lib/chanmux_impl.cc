/* -*- c++ -*- */
/*
 * Copyright 2018 Ettus Research
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "chanmux_impl.h"
#include <gnuradio/block.h>

namespace gr {
  namespace pfb_channelizer {

    chanmux::sptr
    chanmux::make(
        const gr::ettus::device3::sptr &dev,
        const int block_select,
        const int device_select
    )
    {
      return gnuradio::get_initial_sptr(
        new chanmux_impl(
            dev,
            block_select,
            device_select
        )
      );
    }

    /*
     * The private constructor
     */
    chanmux_impl::chanmux_impl(
        const gr::ettus::device3::sptr &dev,
        const int block_select,
        const int device_select
    )
      : gr::ettus::rfnoc_block("chanmux"),
        gr::ettus::rfnoc_block_impl(
            dev,
            gr::ettus::rfnoc_block_impl::make_block_id("chanmux",  block_select, device_select),
            ::uhd::stream_args_t("fc32", "sc16"),
            ::uhd::stream_args_t("fc32", "sc16"))
    {
        d_fft_size = 128;  // 128 by default.
        gr::block::set_min_noutput_items(256);
    }

    /*
     * Our virtual destructor.
     */
    chanmux_impl::~chanmux_impl()
    {
    }

    void
    chanmux_impl::set_block_size(const int fft_size)
    {
        gr::thread::scoped_lock guard(d_mutex);
        get_block_ctrl_throw< ::uhd::rfnoc::chanmux_block_ctrl>()-> set_fft_size(fft_size);

        // read back to ensure HW and SW are in sync.
        d_fft_size = get_block_ctrl_throw< ::uhd::rfnoc::chanmux_block_ctrl >()->get_fft_size();
        // now always pulling blocks of 256 samples.
    }

    void
    chanmux_impl::get_block_size()
    {
        d_fft_size = get_block_ctrl_throw< ::uhd::rfnoc::chanmux_block_ctrl >()-> get_fft_size();
    }
  } /* namespace pfb_channelizer */
} /* namespace gr */
