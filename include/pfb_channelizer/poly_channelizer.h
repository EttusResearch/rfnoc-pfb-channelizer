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


#ifndef INCLUDED_PFB_CHANNELIZER_POLY_CHANNELIZER_H
#define INCLUDED_PFB_CHANNELIZER_POLY_CHANNELIZER_H

#include <pfb_channelizer/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace pfb_channelizer {

    /*!
     * \brief <+description of block+>
     * \ingroup pfb_channelizer
     *
     */
    class PFB_CHANNELIZER_API poly_channelizer : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<poly_channelizer> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of pfb_channelizer::poly_channelizer.
       *
       * To avoid accidental use of raw pointers, pfb_channelizer::poly_channelizer's
       * constructor is in a private implementation
       * class. pfb_channelizer::poly_channelizer::make is the public interface for
       * creating new instances.
       */
      static sptr make(const int fft_size, const std::vector<int> &map);
      virtual void set_block_size(const int fft_size) = 0;
      virtual void set_channel_map(const std::vector<int> &map) = 0;
    };

  } // namespace pfb_channelizer
} // namespace gr

#endif /* INCLUDED_PFB_CHANNELIZER_POLY_CHANNELIZER_H */
