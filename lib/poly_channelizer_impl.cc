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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "poly_channelizer_impl.h"
#include <gnuradio/block.h>
#include <stdio.h>


namespace gr {
  namespace pfb_channelizer {

    static const pmt::pmt_t EOB_KEY = pmt::string_to_symbol("rx_eob");

    poly_channelizer::sptr
    poly_channelizer::make(const int fft_size, const std::vector<int> &map)
    {
        return gnuradio::get_initial_sptr(new poly_channelizer_impl(fft_size, map));
    }

    /*
     * The private constructor
     */
    poly_channelizer_impl::poly_channelizer_impl(const int fft_size, const std::vector<int> &map)
      : gr::block("poly_channelizer",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 512, sizeof(gr_complex)))
    {
        d_fft_size = fft_size;
        gr::thread::scoped_lock guard(d_mutex);
        // gr::block::set_max_noutput_items(256);

        if (map.size() > 0) {
            unsigned int max = (unsigned int)*std::max_element(map.begin(), map.end());
            d_channel_map = map;
        }

        set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    poly_channelizer_impl::~poly_channelizer_impl()
    {
    }

    void
    poly_channelizer_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        ninput_items_required[0] = d_fft_size * noutput_items;
    }

    void
    poly_channelizer_impl::set_block_size(const int fft_size)
    {
        gr::thread::scoped_lock guard(d_mutex);
        d_fft_size = fft_size;
    }

    int
    poly_channelizer_impl::get_block_size()
    {
        return d_fft_size;
    }

    std::vector<int>
    poly_channelizer_impl::get_channel_map() const
    {
        return d_channel_map;
    }

    void
    poly_channelizer_impl::set_channel_map(const std::vector<int>& map)
    {
        gr::thread::scoped_lock guard(d_mutex);

        // std::cout << "map.size() =  "<< (int) map.size() << std::endl;
        //   //  * (int) d_fft_size);
        if (map.size() > 0) {
            unsigned int max = (unsigned int)*std::max_element(map.begin(), map.end());
            d_channel_map = map;
        }
        set_tag_propagation_policy(TPP_DONT);
    }

    int
    poly_channelizer_impl::general_work(int noutput_items,
                                   gr_vector_int& ninput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
    {
        gr::thread::scoped_lock guard(d_mutex);

        const gr_complex *in = (gr_complex *)input_items[0];
        gr_complex *out = (gr_complex *)output_items[0];
        std::vector<tag_t> tags;
        std::vector<tag_t>::iterator ti;
        int in_idx = 0;
        int num_consumed = 0;
        get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items[0], EOB_KEY);
        int oo = 0;
        size_t noutputs = output_items.size();
        for (ti = tags.begin(); ti != tags.end(); ti++) {
            tag_t tag = *ti;
            uint32_t burst_sz = (uint32_t)(tag.offset - nitems_read(0) - in_idx + 1);
            // std::cout << "burst_sz = " << (int) burst_sz << std::endl;
            if (oo == noutput_items || num_consumed >= ninput_items[0]) {
                break;
            }
            else
            {
                num_consumed += burst_sz;
                if (burst_sz == d_fft_size || burst_sz == 256) {
                    int num_ffts = burst_sz / d_fft_size;
                    // std::cout << "num_ffts = " << (int) num_ffts << std::endl;
                    int offset;
                    for (unsigned int mm = 0; mm < num_ffts; mm++) {
                        offset = mm * d_fft_size;
                        for (unsigned int nn = 0; nn < noutputs; nn++) {
                            out = (gr_complex *)output_items[nn];
                            out[oo] = in[in_idx + d_channel_map[nn] + offset];
                        }
                        oo++;
                    }

                }
                else {
                    std::cout << "burst_sz = " << (int) burst_sz << " Dropping Data" << std::endl;
                }
                in_idx += burst_sz;
            }
        }
        consume_each(num_consumed);
        for (unsigned int nn = 0; nn < noutputs; nn++) {
            produce(nn, oo);
        }
        return WORK_CALLED_PRODUCE;
    }

  } /* namespace pfb_channelizer */
} /* namespace gr */
