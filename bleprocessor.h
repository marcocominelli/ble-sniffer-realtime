//
// Copyright 2020 Marco Cominelli
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef BLE_PROCESSOR
#define BLE_PROCESSOR

#define LE1M_SRATE 2
#define LE2M_SRATE 1

#include <vector>
#include <complex>

typedef std::complex<double> iqsamp_t;

const std::vector<double> usrp_tune_freqs = {2420e6, 2462e6};

// Allocate and de-allocate decoder data.
void init_decoder(size_t databuf_size);
void free_decoder();

void ble_processing(iqsamp_t* samplebuf, uint8_t* binbuf, size_t nsamps, size_t num_mboards);
void iq_processing(iqsamp_t* samplebuf, uint8_t* binbuf, size_t nsamps, size_t num_mboards);
void chan_processing(iqsamp_t* samplebuf, uint8_t* binbuf, size_t nsamps, size_t num_mboards);
//void old_proc(iqsamp_t* samplebuf, uint8_t* binbuf, size_t nsamps, size_t num_mboards);

// Useful auxiliary functions.
__host__ __device__ void swap_byte(uint8_t* data);
__host__ __device__ void swap_nbytes(uint8_t* data, size_t length);
__host__ __device__ void ble_dewhiten(uint8_t* data, uint32_t ble_channel, uint32_t length);
uint32_t freq2chan(uint32_t freq);

void offlineproc(void);

/*
FIR filter designed with http://t-filter.appspot.com
sampling frequency: 40 MHz
0 Hz - 1.1 MHz: gain = 1, ripple < 5 dB
1.8 MHz - 20 MHz: gain = 0, attenuation = -40 dB
*/
#define DECFACTOR 20
#define POLY_LEN 5
#define FILTER_TAP_NUM 81
static double filter_taps[FILTER_TAP_NUM] = {
    0.003989802934893269, 0.00116263219316108, 0.0011703374498432562,
    0.0010575602093139385, 0.0007904292201485264, 0.0003445822832488563,
    -0.0002912585172182479, -0.00112240569967483, -0.002149586684204096,
    -0.0033629943422702636, -0.0047376765392051915, -0.006234764567221647,
    -0.007804569055051714, -0.009387176269585316, -0.010911824889796546,
    -0.01229840649650942, -0.013461103961554147, -0.014311866387691807,
    -0.014763220530113879, -0.014731867554766416, -0.014143372660971522,
    -0.012936454876455132, -0.011066018535722264, -0.008505752416612545,
    -0.0052509708989958125, -0.00132093629137646, 0.003240262111946999,
    0.008364028825701473, 0.013958181063568975, 0.019908736331785467,
    0.026083156593932136, 0.032334757557726, 0.038507447619447605,
    0.04444061393143045, 0.049974562618593114, 0.0549564134512078,
    0.05924571381235369, 0.06271938833181666, 0.06527626590868955,
    0.06684117505481149, 0.06736798398456653, 0.06684117505481149,
    0.06527626590868955, 0.06271938833181666, 0.05924571381235369,
    0.0549564134512078, 0.049974562618593114, 0.04444061393143045,
    0.038507447619447605, 0.032334757557726, 0.026083156593932136,
    0.019908736331785467, 0.013958181063568975, 0.008364028825701473,
    0.003240262111946999, -0.00132093629137646, -0.0052509708989958125,
    -0.008505752416612545, -0.011066018535722264, -0.012936454876455132,
    -0.014143372660971522, -0.014731867554766416, -0.014763220530113879,
    -0.014311866387691807, -0.013461103961554147, -0.01229840649650942,
    -0.010911824889796546, -0.009387176269585316, -0.007804569055051714,
    -0.006234764567221647, -0.0047376765392051915, -0.0033629943422702636,
    -0.002149586684204096, -0.00112240569967483, -0.0002912585172182479,
    0.0003445822832488563, 0.0007904292201485264, 0.0010575602093139385,
    0.0011703374498432562, 0.00116263219316108, 0.003989802934893269
};

#endif //BLE_PROCESSOR
