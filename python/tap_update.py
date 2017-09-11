#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2017 <+YOU OR YOUR COMPANY+>.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import numpy as np
import copy
import pmt
import sys
import time
import threading
from gnuradio import gr

six_db = 10 * np.log10(.25)

class tap_update(gr.sync_block):
    """
    docstring for block tap_update
    """
    def __init__(self):
        gr.sync_block.__init__(self, name="tap_update", in_sig=None, out_sig=None)

        self.last_sent = 0
        self.finished = False
        self.K = 10.519
        self.message_port_register_out(pmt.intern('to_rfnoc'))
        self.max_fft_size = 512
        self.qvec_coef = (25, 24)
        self.qvec = (18, 17)
        self.taps_per_phase = 24
        self.old_size = 0
        self.fft_size = 0
        self.taps_set = False

        self.thread = threading.Thread(target=self.settap_thread)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.thread.join()
        self.finished = True
        return gr.sync_block.stop(self)

    def erfc(self, x):
        # save the sign of x
        sign = [1 if val >= 0 else -1 for val in x]
        x = abs(x)

        # constants
        a1 = 0.254829592
        a2 = -0.284496736
        a3 = 1.421413741
        a4 = -1.453152027
        a5 = 1.061405429
        p = 0.3275911

        # A&S formula 7.1.26
        t = 1.0/(1.0 + p*x)
        y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*np.exp(-x*x)
        ret_val = 1 - sign*y
        return ret_val

    def nextpow2(self, i):
        """
            Find 2^n that is equal to or greater than.
        """
        n = 0
        while (2**n) < i:
            n += 1
        return n

    def ret_num_bitsU(self, value):
        '''
            Function returns required number of bits for unsigned binary
            representation.
        '''
        val_new = np.floor(value)

        if value == 0:
            return 1

        temp = np.ceil(np.log2(np.abs(val_new + .5)))
        return temp.astype(np.int)

    def ret_num_bitsS(self, value):
        '''
            Function returns required number of bits for 2's
            complement representation.
        '''
        if value < 0:
            temp = self.ret_num_bitsU(np.abs(value) - 1)
        else:
            temp = self.ret_num_bitsU(value) + 1
        return temp

    def tap_equation(self, fft_size):
        F = np.arange(self.taps_per_phase * fft_size)
        F = np.double(F) / len(F)

        x = self.K * (np.double(fft_size) * F - .5)
        A = np.sqrt(0.5 * self.erfc(x))

        N = len(A)

        idx = np.arange(N / 2)
        A[N - idx - 1] = np.conj(A[1 + idx])
        A[N // 2] = 0

        db_diff = six_db - 10 * np.log10(.5)
        exponent = 10. ** (-db_diff / 10.)

        A = A ** exponent

        b = np.fft.ifft(A)
        b = (np.fft.fftshift(b)).real

        b /= np.sum(b)

        return b

    def gen_fixed_filter(self, taps, fft_size, desired_msb=40):

        max_coeff_val = (2. **(self.qvec_coef[0] - 1) - 1) * (2. ** -self.qvec_coef[1])

        taps_gain = max_coeff_val / np.max(np.abs(taps))
        taps *= taps_gain
        taps_fi = (taps * (2. ** self.qvec_coef[1])).astype(np.int)
        poly_fil = np.reshape(taps_fi, (fft_size, -1), order='F')

        max_input = int(2.**(self.qvec[0] - 1)) - 1
        # compute noise and signal gain.
        s_gain = np.abs(np.max(np.sum(poly_fil, axis=1)))

        path_gain = np.max(np.abs(np.sum(poly_fil, axis=1)))
        bit_gain = self.nextpow2(np.max(s_gain))

        gain_msb = self.nextpow2(s_gain)
        max_coef_val = 2. ** gain_msb - 1
        in_use = s_gain / max_coef_val
        # print(np.max(s_gain), np.max(max_input))
        max_value = np.max(s_gain).astype(np.int64) * np.max(max_input).astype(np.int64)
        num_bits = self.ret_num_bitsS(max_value)
        # print(num_bits)
        msb = num_bits - 1
        if in_use > .9:
            new_b = poly_fil
            delta_gain = 1
        else:
            # note we are scaling down here hence the - 1
            msb = msb - 1
            delta_gain = .5 * (max_coef_val / s_gain)
            new_b = np.floor(poly_fil * delta_gain).astype(int)

        poly_fil = new_b
        if desired_msb is not None:
            if msb > desired_msb:
                diff = msb - desired_msb
                poly_fil = poly_fil >> diff
                msb = desired_msb

        # print("msb = {}".format(msb))
        # taps_fi = np.reshape(poly_fil, (1, -1), order='F')
        poly_fil = poly_fil.astype(np.int32)

        return poly_fil

    def gen_tap_vec(self, poly_fil, fft_size):
        """
            Helper function that generates a single file used for
            programming the interanal ram
        """
        pfb_fil = copy.deepcopy(poly_fil)
        pfb_fil = pfb_fil.T
        # qvec = (self.qvec_coef[0], 0)
        vec = np.array([])
        pad = np.array([0] * (self.max_fft_size - fft_size))
        for col in pfb_fil:
            col_vec = np.concatenate((col, pad))
            vec = np.concatenate((vec, col_vec))

        return vec.astype(np.int)

    def initialize(self, fft_size):
        self.fft_size = fft_size

    def send_taps(self, fft_size):

        self.fft_size = fft_size
        self.taps_set = True
        taps = self.tap_equation(fft_size)
        taps_fi = self.gen_fixed_filter(taps, fft_size)
        taps_vec = self.gen_tap_vec(taps_fi, fft_size)
        data_rfnoc = pmt.init_s32vector(len(taps_vec), taps_vec.tolist())
        msg = pmt.cons(pmt.get_PMT_NIL(), data_rfnoc)
        # print(msg)
        self.message_port_pub(
            pmt.intern('to_rfnoc'), msg)
        print("done sending")

    def settap_thread(self):
        time.sleep(3)
        while(self.fft_size == 0):
            time.sleep(.1)
        if self.taps_set is False:
            self.send_taps(self.fft_size)

        return 0
