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


#include <pfb_channelizer/chanmux_block_ctrl.hpp>
#include <uhd/convert.hpp>
// #include <uhd/utils/msg.hpp>

using namespace uhd::rfnoc;

class chanmux_block_ctrl_impl : public chanmux_block_ctrl
{
public:

    static const boost::uint32_t RB_NUM_TAPS = 128;
    static const boost::uint32_t SR_FFT_SIZE = 129;
    static const boost::uint32_t SR_RELOAD = 130;
    static const boost::uint32_t SR_RELOAD_TLAST = 131;
    static const boost::uint32_t SR_CONFIG = 132;

    UHD_RFNOC_BLOCK_CONSTRUCTOR(chanmux_block_ctrl)
    {
        _n_taps = boost::uint32_t(user_reg_read64(RB_NUM_TAPS));
        //UHD_MSG(status) << "chanmux_block_ctrl::chanmux_block_ctrl() n_taps ==" << _n_taps << std::endl;
        UHD_ASSERT_THROW(_n_taps);

    }

    void set_fft_size(const int fft_size)
    {
        sr_write(SR_FFT_SIZE, boost::uint32_t(fft_size));
    }

    size_t get_fft_size()
    {
        _fft_size = boost::uint32_t(user_reg_read64(SR_FFT_SIZE));
        return _fft_size;
    }

    void set_taps(const std::vector<int>& taps_)
    {
        //UHD_MSG(status) << "chanmux_block_ctrl::chanmux_block_ctrl() setting taps" << std::endl;
        UHD_RFNOC_BLOCK_TRACE() << "chanmux_block_ctrl::set_taps()" << std::endl;
        if (taps_.size() > _n_taps) {
            throw uhd::value_error(str(
                boost::format("Too many filter coefficients! Provided %d, FIR allows %d.\n")
                % taps_.size() % _n_taps));
        }
        for (size_t i = 0; i < taps_.size(); i++) {
            if (taps_[i] > 16777215  || taps_[i] < -16777216) {
                throw uhd::value_error(str(
                    boost::format("PFB Channelizer: Coefficient %d out of range! Value %d, Allowed range [-16777216,16777215].\n")
                    % i % taps_[i]));
            }
        }
        std::vector<int> taps = taps_;
        UHD_VAR(taps.size()); // status) << "chanmux_block_ctrl::chanmux_block_ctrl() taps.size ==" << taps.size() << std::endl;
        // printf("inside set_taps\n");
        // for (size_t i; i<taps_.size(); i++)
        // {
        //     printf("Ctrl - %d, %d\n", i, taps[i]);
        // }
        if (taps.size() < _n_taps) {
            taps.resize(_n_taps, 0);
        }

        UHD_VAR(taps.size());
        // Write taps via the reload bus
        for (size_t i = 0; i < taps.size() - 1; i++) {
            sr_write(SR_RELOAD, boost::uint32_t(taps[i]));
        }
        UHD_HERE();
        sr_write(SR_RELOAD_TLAST, boost::uint32_t(taps.back()));
        // This follows the convention -- the taps have already been
        // written at this point -- not enough room for double buffering.
        // Note: This configuration bus does not require tlast
        // sr_write(SR_CONFIG, 0);
    }

    //! Returns the number of filter taps in this block.
    size_t get_n_taps() const
    {
        return _n_taps;
    }
private:
    const std::string _item_type;
    size_t _n_taps;
    size_t _fft_size;
};

UHD_RFNOC_BLOCK_REGISTER(chanmux_block_ctrl,"chanmux");
