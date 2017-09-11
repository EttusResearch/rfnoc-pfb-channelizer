/* -*- c++ -*- */
/*
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_PFB_CHANNELIZER_CHANMUX_IMPL_H
#define INCLUDED_PFB_CHANNELIZER_CHANMUX_IMPL_H

#include <pfb_channelizer/chanmux.h>
#include <pfb_channelizer/chanmux_block_ctrl.hpp>
#include <ettus/rfnoc_block_impl.h>

namespace gr {
  namespace pfb_channelizer {

    class chanmux_impl : public chanmux, public gr::ettus::rfnoc_block_impl
    {
     private:
         size_t d_fft_size;
         gr::thread::mutex d_mutex; // mutex to protect set/work access
         void handler(pmt::pmt_t msg);
         void set_rfnoc_taps(const std::vector<int> &taps);
        //  int nextpow2(unsigned int i);

     public:
      chanmux_impl(const gr::ettus::device3::sptr &dev, const int block_select, const int device_select);
      ~chanmux_impl();

      void set_block_size(const int fft_size);
      void get_block_size();
      // Where all the action really happens
      // void forecast (int noutput_items, gr_vector_int &ninput_items_required);
    };

  } // namespace pfb_channelizer
} // namespace gr

#endif /* INCLUDED_PFB_CHANNELIZER_CHANMUX_IMPL_H */
