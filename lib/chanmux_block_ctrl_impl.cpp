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

#include <stdio.h>
#include <vector>
#include <gnuradio/blocks/api.h>
#include <gnuradio/types.h>
#include <gnuradio/fft/fft.h>
#include <uhd/convert.hpp>
#include <volk/volk.h>
#include <pfb_channelizer/chanmux_block_ctrl.hpp>
#include <gnuradio/fft/fft.h>
#include <gnuradio/sys_paths.h>
#include <gnuradio/gr_complex.h>

using namespace uhd::rfnoc;

static const float a1 = 0.254829592;
static const float a2 = -0.284496736;
static const float a3 = 1.421413741;
static const float a4 = -1.453152027;
static const float a5 = 1.061405429;
static const float p = 0.3275911;
static const float K = 10.519;

static const int taps_per_phase = 24;
static const int qvec_coef[2] = {25, 24};
static const int max_fft_size = 4;
static const int qvec[2] = {16, 15};

static const boost::uint32_t RB_NUM_TAPS = 128;
static const boost::uint32_t SR_FFT_SIZE = 129;
static const boost::uint32_t SR_RELOAD = 130;
static const boost::uint32_t SR_RELOAD_TLAST = 131;
static const boost::uint32_t SR_CONFIG = 132;

class chanmux_block_ctrl_impl : public chanmux_block_ctrl
{
public:

    UHD_RFNOC_BLOCK_CONSTRUCTOR(chanmux_block_ctrl)
    {
        _n_taps = uint32_t(user_reg_read64(RB_NUM_TAPS));
        UHD_ASSERT_THROW(_n_taps);
        set_taps(64);
    }

    void set_fft_size(const int fft_size)
    {
        sr_write(SR_FFT_SIZE, uint32_t(fft_size));
        set_taps(fft_size);
    }

    size_t get_fft_size()
    {
        _fft_size = uint32_t(user_reg_read64(SR_FFT_SIZE));
        return _fft_size;
    }

private:
    size_t _n_taps;
    size_t _fft_size;

    void set_taps(const int fft_size)
    {

        gr_vector_float taps;
        tap_equation(fft_size, taps);
        int desired_msb = 40;  //TODO: replace internal constant
        gr_vector_int taps_fi;
        gen_fixed_filter(taps, fft_size, desired_msb, taps_fi);

        if (taps_fi.size() > _n_taps) {
            throw uhd::value_error(str(
                boost::format("Too many filter coefficients! Provided %d, "
                              "FIR allows %d.\n")
                % taps_fi.size() % _n_taps));
        }

        printf("taps size = %d\n", (int) taps_fi.size());
        for (size_t i = 0; i < taps_fi.size(); i++) {
            if (taps_fi[i] > 16777215  || taps_fi[i] < -16777216) {
                throw uhd::value_error(str(
                    boost::format(
                        "Channelizer: Coefficient %d out of range! "
                        "Value %d, Allowed range [-16777216,16777215].\n")
                    % i % taps_fi[i]));
            }
        }

        UHD_VAR(taps_fi.size());
        if (taps_fi.size() < _n_taps) {
            taps_fi.resize(_n_taps, 0);
        }

        UHD_VAR(taps_fi.size());
        // sr_read(RB_NUM_TAPS)
        uint32_t num_taps_read;
        num_taps_read = user_reg_read32(RB_NUM_TAPS);
        printf("num_taps = %d\n", (int) num_taps_read);
        for (size_t i = 0; i < taps_fi.size() - 1; i++) {
            sr_write(SR_RELOAD, boost::uint32_t(taps_fi[i]));
            printf("tap[%d] = %d\n", (int) i, (int) boost::uint32_t(taps_fi[i]));
        }
        sr_write(SR_RELOAD_TLAST, boost::uint32_t(taps_fi.back())); 
        printf("final tap = %d\n", (int) boost::uint32_t(taps_fi.back()));
        printf("set_taps() done\n");
    }

    void
    erfc(const gr_vector_float& value, gr_vector_float& ret_val)
    {
        // save the sign of x
        gr_vector_int sign;
        int temp;
        gr_vector_float x_val;
        for (size_t i=0; i < value.size(); i++) {
            temp = ((value[i] >= 0) ? 1 : -1);
            sign.push_back(temp);
            x_val.push_back(std::abs(value[i]));
        }
        // sign = [1 if val >= 0 else -1 for val in x]
        // A&S formula 7.1.26
        for (size_t i=0; i < x_val.size(); i++) {
            float x = x_val[i];
            float t = 1.0/(1.0 + p*x);
            float y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*std::exp(-x*x);
            ret_val.push_back(1 - sign[i]*y);
        }
    }


    int
    nextpow2(const int value)
    {
        int n = 0;
        while ((1 << n) < value)
        {
            n++;
        }
        return n;
    }

    int
    ret_num_bitsU(const int value)
    {

        float val_new = (float) value;
        if (value == 0)
        {
            return 1;
        }

        float temp = std::ceil(std::log(std::abs(val_new + .5)) / std::log(2.));
        int ret_value = (int) temp;
        return ret_value;
    }

    int
    ret_num_bitsS(const int value)
    {
        int temp;
        if (value < 0)
        {
            temp = ret_num_bitsU(std::abs(value) - 1);
        }
        else
        {
            temp = ret_num_bitsU(value) + 1;
        }
        return temp;
    }

    void
    tap_equation(const int fft_size, gr_vector_float& ret_val)
    {
        int vec_len = taps_per_phase * fft_size;
        gr_vector_float x_vec;
        float F_val;
        float temp;
        for (int i=0; i < vec_len; i++)
        {
            F_val = (float) i / (float) vec_len;
            temp = K * ((float) fft_size * F_val - .5);
            x_vec.push_back(temp);
        }

        gr_vector_float erfc_vals;
        erfc(x_vec, erfc_vals);

        gr_vector_float A_vals;
        for (size_t i=0; i < erfc_vals.size(); i++)
        {
            A_vals.push_back(std::sqrt(.5 * erfc_vals[i]));
        }

        int N = A_vals.size();
        // idx = np.arange(N / 2)

        for (int i=0; i < N / 2; i++)
        {
            A_vals[N - i - 1] = A_vals[1 + i];
        }
        A_vals[N / 2] = 0;

        float six_db = 10 * std::log10(.25);
        float db_diff = six_db - 10. * std::log10(.5);
        float exponent = std::pow(10., -db_diff / 10.);
        for (size_t i=0; i < A_vals.size(); i++)
        {
            A_vals[i] = std::pow(A_vals[i], exponent);
        }
        // cast A as complex.
        // gr_complex *A_complex = new gr_complex;
        // volk_32f_x2_interleave_32fc(A_complex, &A_vals[0], q_a, A_vals.size());

        int fil_size = (int) A_vals.size();
        gr::fft::fft_complex *d_fwdfft = new gr::fft::fft_complex(fil_size, false, 1);

        // copy real vector into complex vector
        // gr_vector_compex
        std::vector<gr_complex> A_comp;

        gr_complex temp_grc;
        for (int j=0; j<fil_size; j++)
        {
            temp_grc = (gr_complex) A_vals[j];
            A_comp.push_back(temp_grc);
            // printf("A_comp[%d] = %f + j%f;\n", j, temp_grc.real(), temp_grc.imag());
        }

        // copy A values into fft input buffer
        memcpy(d_fwdfft->get_inbuf(), &A_comp[0], fil_size * sizeof(gr_complex));

        d_fwdfft->execute(); // compute fwd xform

        gr_complex *b = d_fwdfft->get_outbuf();
        gr_complex *b_copy;

        // memcpy(b_copy, b, fft_size);
        // perform shift
        int shift_pt = fil_size >> 1;
        float sum_b = 0.;

        for (int i=0; i < fil_size; i++)
        {
            temp = b[(i + shift_pt) % fil_size].real();
            sum_b += temp;
            ret_val.push_back(temp);
        }

        // normalize b.
        for (int i=0; i < fil_size; i++)
        {
            ret_val[i] = ret_val[i] / sum_b;
        }
        // take real part of b.
        //

        // b = np.fft.ifft(A)
        // b = (np.fft.fftshift(b)).real
        // b /= np.sum(b)
        delete d_fwdfft;
    }

    void
    gen_fixed_filter(const gr_vector_float& taps, const int fft_size, const int desired_msb, gr_vector_int& output_vector)
    {
        // float max_coeff_val = (2. **(qvec_coef[0] - 1) - 1) * (2. ** -self.qvec_coef[1])
        float max_coeff_val = (std::pow(2., (float) qvec_coef[0] - 1.) - 1.) * std::pow(2., (float)-qvec_coef[1]);
        int max_input = (int) std::pow(2., (float) qvec[0] - 1.) - 1;

        float max_value = 0;
        float temp;
        for (size_t i = 0; i < taps.size(); i++)
        {
            temp = std::fabs(taps[i]);
            if (temp > max_value)
            {
                max_value = temp;
            }
        }

        gr_vector_int taps_fi;
        float taps_gain = max_coeff_val / max_value;
        float temp_tap;
        int fixed_tap;
        for (size_t i=0; i<taps.size(); i++)
        {
            temp_tap = taps[i] * taps_gain;
            // convert to fixed point
            fixed_tap = (int) (temp_tap * std::pow(2., qvec_coef[1]));
            taps_fi.push_back(fixed_tap);
        }
        std::vector< gr_vector_int > poly_fi;
        gr_vector_int path_gain;
        // create polyphase implementation of taps.
        for (int i=0; i < fft_size; i++)
        {
            int tap_offset = 0;
            gr_vector_int temp_row;
            int temp_gain = 0;
            for (int j=0; j < taps_per_phase; j++)
            {
                int index = i + tap_offset;
                temp_gain += taps_fi[index];
                temp_row.push_back(taps_fi[index]);
                tap_offset += fft_size;
            }
            poly_fi.push_back(temp_row);
            path_gain.push_back(std::abs(temp_gain));
        }

        //
        std::vector<int>::const_iterator s_gain;
        s_gain = std::max_element(path_gain.begin(), path_gain.end());
        // // compute noise and signal gain.
        int gain_msb = nextpow2(*s_gain);
        //
        // gain_msb = self.nextpow2(s_gain)
        int max_coef_val = (int) std::pow(2.,  (float) gain_msb - 1.);
        float in_use = (float) *s_gain / (float) max_coef_val;
        // // print(np.max(s_gain), np.max(max_input))
        int int_max_value = *s_gain * max_input;
        int num_bits = ret_num_bitsS(int_max_value);
        // // print(num_bits)
        int msb = num_bits - 1;
        if (in_use <= .9)
        {
            // note we are scaling down here hence the - 1
            msb = msb - 1;
            float delta_gain = .5 * ((float) max_coef_val / (float) *s_gain);
            // scale polyphase filter.
            for (int i=0; i < fft_size; i++)
            {
                for (int j=0; j < taps_per_phase; j++)
                {
                    poly_fi[i][j] = (int) ((float) poly_fi[i][j] * delta_gain);
                }
            }
        }
        //
        if (desired_msb < msb)
        {
            int diff = msb - desired_msb;
            for (int i=0; i < fft_size; i++)
            {
                for (int j=0; j < taps_per_phase; j++)
                {
                    poly_fi[i][j] = poly_fi[i][j] >> diff;
                }
            }
        }

        // reshape poly_fi into single vector with padding.
        int pad = max_fft_size - fft_size;
        // // print("msb = {}".format(msb))
        // // taps_fi = np.reshape(poly_fil, (1, -1), order='F')
        // poly_fil = poly_fil.astype(np.int32)
        //
        for (int j=0; j < taps_per_phase; j++)
        {
            for (int i=0; i < fft_size; i++)
            {
                output_vector.push_back(poly_fi[i][j]);
            }
            for (int m=0; m < pad; m++)
            {
                output_vector.push_back(0);
            }
        }
        for (size_t i=0; i<output_vector.size(); i++){
            printf("output_vector[i] = %d\n", output_vector[i]);
        }
    }

};

UHD_RFNOC_BLOCK_REGISTER(chanmux_block_ctrl,"chanmux");
