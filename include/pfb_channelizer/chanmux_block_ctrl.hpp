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


#ifndef INCLUDED_LIBUHD_RFNOC_PFB_CHANNELIZER_CHANMUX_HPP
#define INCLUDED_LIBUHD_RFNOC_PFB_CHANNELIZER_CHANMUX_HPP

#include <uhd/rfnoc/source_block_ctrl_base.hpp>
#include <uhd/rfnoc/sink_block_ctrl_base.hpp>

namespace uhd {
    namespace rfnoc {

/*! \brief Block controller for the standard copy RFNoC block.
 *
 */
class UHD_API chanmux_block_ctrl : public source_block_ctrl_base, public sink_block_ctrl_base
{
public:
    UHD_RFNOC_BLOCK_OBJECT(chanmux_block_ctrl)
    virtual void set_taps(const std::vector<int>& taps) = 0;
    //! Returns the number of filter taps in this block.
    virtual size_t get_n_taps() const = 0;
    virtual size_t get_fft_size() = 0;
    virtual void set_fft_size(const int fft_size) = 0;
    /*!
     * Your block configuration here
    */
}; /* class chanmux_block_ctrl*/

}} /* namespace uhd::rfnoc */

#endif /* INCLUDED_LIBUHD_RFNOC_PFB_CHANNELIZER_CHANMUX_BLOCK_CTRL_HPP */
