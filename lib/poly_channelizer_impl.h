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

#ifndef INCLUDED_PFB_CHANNELIZER_POLY_CHANNELIZER_IMPL_H
#define INCLUDED_PFB_CHANNELIZER_POLY_CHANNELIZER_IMPL_H

#include <pfb_channelizer/poly_channelizer.h>

namespace gr {
  namespace pfb_channelizer {

    class poly_channelizer_impl : public poly_channelizer
    {
     private:
      // Nothing to declare in this block.
      size_t d_fft_size;

      int d_offset;
      std::vector<int> d_channel_map;
      gr::thread::mutex d_mutex; // mutex to protect set/work access

     public:
      poly_channelizer_impl(const int fft_size, const std::vector<int> &map);
      ~poly_channelizer_impl();

      void set_block_size(const int fft_size);
      int get_block_size();
      void set_channel_map(const std::vector<int> &map) ;
      std::vector<int> get_channel_map() const;

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace pfb_channelizer
} // namespace gr

#endif /* INCLUDED_PFB_CHANNELIZER_POLY_CHANNELIZER_IMPL_H */
