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

#include <iostream>
#include <complex>
#include <cstdio>
#include <cmath>

#include <boost/format.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuComplex.h>

#include "bleprocessor.h"
#include "pcapsaver.h"

typedef std::complex<double> iqsamp_t;

// TODO: rm global vars and create a class for the BLE processor.

// Store polyphase filter in constant memory for faster access
__constant__ double poly[DECFACTOR][POLY_LEN];
__constant__ cuDoubleComplex twiddle[DECFACTOR][DECFACTOR];

#define MAX_NUM_CONN 128
uint32_t* conn_list;

static const std::complex<double> i_unit(0.0, 1.0);

static cuDoubleComplex* sigbuf;
static cuDoubleComplex* chanbuf;

__device__ __host__
size_t get_aa_idx(size_t idx)
{
    return idx;
}

__device__ __host__
size_t get_crc_idx(size_t idx)
{
    return (idx + MAX_NUM_CONN);
}     

__global__
void init_conn_list(uint32_t* conn_list)
{
    conn_list[0] = 2;
    conn_list[get_aa_idx(1)] = 0x8e89bed6;
    conn_list[get_crc_idx(1)] = 0x555555;
}


__host__ __device__
void swap_byte(uint8_t* data)
{
    uint8_t tmp = 0;
    size_t shift = 0;
    for (int i = 0x80; i; i >>= 1) {
        if (*data & i) tmp |= (0x1 << shift);
        shift++; 
    }
    *data = tmp;
}


__host__ __device__
void swap_nbytes(uint8_t* data, size_t nbytes)
{
    while (nbytes--) swap_byte(data++);
}


__host__ __device__
void ble_dewhiten(uint8_t* data, uint32_t ble_channel, uint32_t nbytes)
{
    uint32_t lfsr = (ble_channel & 0x3F) | 0x40;
    while (nbytes--) {
        #pragma unroll
        for (int i = 0x80; i; i >>= 1) {
            if (lfsr & 0x01) {
                *data ^= i;
                lfsr ^= 0x88;
            }
            lfsr >>= 1;
        }
        ++data;
    }
}


__host__ __device__
bool is_adv_chan(uint32_t blechan)
{
    switch(blechan) {
        case 37: case 38: case 39:
            return true;
        default:
            return false;
    }
}


__device__
uint32_t compute_crc(const uint8_t* data, size_t nbytes, uint32_t init_val)
{
    uint32_t crc = 0, tmp = 0;
    size_t shift;
    
    // Reverse bits of init val
    shift = 0;
    #pragma unroll
    for (int i = 0x800000; i; i >>= 1) {
        if (init_val & i) tmp |= 0x1 << shift;
        shift++;
    }

    while (nbytes) {
        for (int i = 0x01; i <= 0x80; i <<= 1) {
            if ( ((*data & i) && !(tmp & 0x1)) || (!(*data & i) && (tmp & 0x1)) ) {
                tmp ^= 0x01B4C000;
            }
            tmp >>= 1;
        }
        ++data;
        --nbytes;
    }

    // Reverse bits for final crc value
    shift = 0;
    #pragma unroll
    for (int i = 0x800000; i; i >>= 1) {
        if (tmp & i) crc |= 0x1 << shift;
        shift++;
    }

    return crc;
}


__global__
void demod(cuDoubleComplex* samplebuf, uint8_t* binbuf, size_t nsamps)
{
    unsigned int i = (blockIdx.x * blockDim.x) + threadIdx.x;
    if (i == 0) {
        binbuf[i] = 0;
    } else if (i < nsamps) {
        double phasediff_sign = cuCimag(samplebuf[i]) * cuCreal(samplebuf[i-1])
                              - cuCreal(samplebuf[i]) * cuCimag(samplebuf[i-1]);
        binbuf[i] = (phasediff_sign > 0.0) ? 0x1 : 0x0;
    }
}


__device__
bool aa_is_known(uint32_t aa, uint32_t* crc, uint32_t* conn_list)
{
    #pragma unroll
    for (size_t k = 1; k < MAX_NUM_CONN; k++)
        if (aa == conn_list[get_aa_idx(k)]) {
            *crc = conn_list[get_crc_idx(k)];
            return true;
        }

    return false;
}

__global__
void bfscan_channel(uint8_t* bits, size_t buflen, size_t blechan, uint32_t* conn_list, size_t srate)
{
    unsigned int i = (blockIdx.x * blockDim.x) + threadIdx.x;
    if (i < buflen - 128*8) {
        uint8_t transitions = 0;
        for (size_t c = 0; c < 8; ++c) {
            if ((bits[i+(c+1)*srate] & 0x1) != (bits[i+c*srate] & 0x1))
                transitions++;
        }
        if (transitions != 8) return;

        uint32_t aa = 0x00;
        size_t offset = i + 8*srate;
        for (size_t c = 0; c < 32; ++c) {
            aa |= (bits[offset+c*srate] & 0x1) << c;
        }

        uint32_t init_val = 0;
        if (aa_is_known(aa, &init_val, conn_list)) return;
        if (aa == 0 || aa == 0xffffffff) return;

        offset += (32 + (5 * 8) + 150 + 8) * srate;

        offset -= 2;
        for (int kk = 0; kk < 4; kk++) {
            uint32_t aanew = 0x00;
            for (size_t c = 0; c < 32; ++c) aanew |= (bits[offset+c*srate+kk] & 0x1) << c;

            if (aanew == aa) {
                bits[i] |= 0x10;
                break;
            }
        }
    }
}


__global__
void detect_ble_packets(uint8_t* bits, size_t buflen, size_t blechan, uint32_t* conn_list, size_t srate)
{
    unsigned int i = (blockIdx.x * blockDim.x) + threadIdx.x;
    if (i < buflen - 128*8) {
        uint8_t transitions = 0;
        for (size_t c = 0; c < 8; ++c) {
            if ((bits[i+(c+1)*srate] & 0x1) != (bits[i+c*srate] & 0x1))
                transitions++;
        }
        if (transitions != 8) return;

        uint32_t aa = 0x00;
        size_t offset = i + 8*srate;
        for (size_t c = 0; c < 32; ++c) {
            aa |= (bits[offset+c*srate] & 0x1) << c;
        }

        uint32_t init_val = 0;
        if (aa_is_known(aa, &init_val, conn_list) == false) return;

        offset += 32*srate;
        const size_t header_size = 2;
        const size_t crc_size = 3;
        uint8_t header[2] = {0};
        uint8_t pdu[260] = {0};
        uint32_t payload_size = 0;
        uint32_t crc_computed = 0;
        uint32_t crc_packet = 0;

        // Extract packet length.
        for (int byte = 0; byte < header_size; ++byte) {
            #pragma unroll
            for (int b = 0; b < 8; ++b)
                header[byte] |= (bits[offset + (byte*8+b)*srate] & 0x1) << (7-b);
        }
        ble_dewhiten(header, blechan, header_size);
        swap_nbytes(header, 2);
        payload_size = header[1];

        // Extract BLE frame.
        size_t pdu_size = header_size + payload_size;
        size_t frame_size = pdu_size + crc_size;
        for (int byte = 0; byte < frame_size; ++byte) {
            #pragma unroll
            for (int b = 0; b < 8; ++b)
                pdu[byte] |= (bits[offset + (byte*8+b)*srate] & 0x1) << (7-b);
        }
        ble_dewhiten(pdu, blechan, frame_size);
        swap_nbytes(pdu, pdu_size);

        // Extract CRC from BLE frame.
        int shift = 23;
        for (int byte = 0; byte < crc_size; ++byte) {
            for (int b = 0x80; b; b >>= 1) {
                if (pdu[pdu_size + byte] & b) crc_packet |= 0x1 << shift;
                shift--;
            }
        }

        // Verify CRC matching.
        crc_computed = compute_crc(pdu, pdu_size, init_val);
        if (crc_computed == crc_packet) {
            switch (srate) {
                case LE1M_SRATE: bits[i] |= 0x80; break;
                case LE2M_SRATE: bits[i] |= 0x40; break;
                default: break;
            }
            if (is_adv_chan(blechan) && ((header[0] & 0xf) == 0x5)) {

                uint32_t new_aa = 0x00;
                uint32_t new_crc = 0x00;
                offset = header_size + 12;

                for (size_t c = 0; c < 4; ++c) new_aa |= pdu[offset+c] << c*8;
                offset += 4;
                for (size_t c = 0; c < 3; ++c) new_crc |= pdu[offset+c] << c*8;

                int idx = atomicAdd(&conn_list[0], 1);
                conn_list[get_aa_idx(idx)] = new_aa;
                conn_list[get_crc_idx(idx)] = new_crc;
            }
        }
    }
}


__global__
void filter(cuDoubleComplex* x, cuDoubleComplex* s, size_t nsamps)
{
    unsigned int i = (blockIdx.x * blockDim.x) + threadIdx.x;
    if (i < nsamps/DECFACTOR - FILTER_TAP_NUM+1) {
        size_t j = (FILTER_TAP_NUM - 1) + i * DECFACTOR;
        for (unsigned int k = 0; k < POLY_LEN; ++k) {
            size_t idx = j/DECFACTOR;
            // First filter component
            cuDoubleComplex tap = make_cuDoubleComplex(poly[0][k]/(double)POLY_LEN, 0);
            cuDoubleComplex inc = cuCmul(x[j-k*DECFACTOR], tap);
            s[idx-1] = cuCadd(s[idx-1], inc);
            // Other filter components
            for (unsigned int c = 1; c < DECFACTOR; ++c) {
                tap = make_cuDoubleComplex(poly[c][k]/((double)POLY_LEN-1), 0);
                inc = cuCmul(x[j-k*DECFACTOR+(DECFACTOR-c)], tap);
                size_t newidx = c*nsamps/DECFACTOR + idx;
                s[newidx] = cuCadd(s[newidx], inc);
            }
        }
    }
}


__global__
void dft(cuDoubleComplex* s, cuDoubleComplex* y, size_t nsamps)
{
    unsigned int i = (blockIdx.x * blockDim.x) + threadIdx.x;
    size_t samps_per_chan = nsamps/DECFACTOR;
    if (i < samps_per_chan) {
        for (unsigned int ch = 0; ch < DECFACTOR; ++ch)
            for (unsigned int k = 0; k < DECFACTOR; ++k) 
                y[ch*samps_per_chan+i] =
                    cuCadd(y[ch*samps_per_chan+i],
                        cuCmul(s[k*samps_per_chan+i], twiddle[ch][k]));
    }
}


void init_decoder(size_t data_buffer_size)
{
    // Initialize polyphase filter and copy to Constant Memory
    double h_filter[DECFACTOR][POLY_LEN];
    for (size_t i = 0; i < DECFACTOR * POLY_LEN; ++i) {
        size_t r = i%DECFACTOR;
        size_t c = i/DECFACTOR;
        h_filter[r][c] = (i < FILTER_TAP_NUM) ? filter_taps[i] : 0.0;
    }
    cudaMemcpyToSymbol(poly, h_filter, DECFACTOR * POLY_LEN * sizeof(double));

    // Initalize twiddle matrix
    std::complex<double> h_twiddle[DECFACTOR][DECFACTOR];
    for (size_t row = 0; row < DECFACTOR; ++row) {
        for (size_t col = 0; col < DECFACTOR; ++col) {
            std::complex<double> tmp =
                exp(i_unit * 2.0 * M_PI * ((double) col*row) / (double)DECFACTOR);
            h_twiddle[row][col] = tmp;
        }
    }
    cudaMemcpyToSymbol(twiddle, h_twiddle, DECFACTOR * DECFACTOR * sizeof(cuDoubleComplex));

    // Allocate auxiliary data buffers
    cudaMalloc((void**)&sigbuf,  data_buffer_size * sizeof(cuDoubleComplex));
    cudaMalloc((void**)&chanbuf, data_buffer_size * sizeof(cuDoubleComplex));

    // Allocate connection list: first half is aa, second half is crc + 1 element idx
    cudaMalloc((void**)&conn_list, (1 + 2 * MAX_NUM_CONN) * sizeof(uint32_t));
    init_conn_list<<<1,1>>>(conn_list);
}


void free_decoder()
{
    // Deallocate auxiliary data buffers
    cudaFree(sigbuf);
    cudaFree(chanbuf);

    cudaFree(conn_list);
}


uint32_t freq2chan(uint32_t freq)
{

    if (freq == 2402) return 37;
    if (freq == 2426) return 38;
    if (freq == 2480) return 39;
    if (freq < 2426) return (freq - 2404)/2;
    if (freq == 2482) return 18; // <- Trick to get channel 18
    return ((freq - 2428)/2 + 11);
}

__global__
void clean_aa_table(uint32_t* conn_list)
{
    for (size_t k = 1; k < conn_list[0]; k++) {
        for (size_t sk = k+1; sk < conn_list[0]; sk++) {
            if (conn_list[k] == conn_list[sk]) {
                for (size_t ssk = sk; ssk < conn_list[0]; ssk++) {
                    conn_list[get_aa_idx(ssk)] = conn_list[get_aa_idx(ssk)+1];
                    conn_list[get_crc_idx(ssk)] = conn_list[get_crc_idx(ssk)+1];
                }
                conn_list[0]--;
                break;
            }
        }
    }
}


void iq_processing(iqsamp_t* samplebuf, uint8_t* binbuf, size_t num_samps, size_t num_mboards)
{
    // Cast IQ samples to 'cuDoubleComplex' array for CUDA C compatibility
    cuDoubleComplex* cbuf = (cuDoubleComplex*) samplebuf;

    // Reset buffers
    cudaMemset(sigbuf, 0, num_mboards * num_samps * sizeof(cuDoubleComplex));
    cudaMemset(chanbuf, 0, num_mboards * num_samps * sizeof(cuDoubleComplex));

    size_t nthreads = 512;
    size_t nblocks = (num_samps + nthreads - 1) / nthreads;

    for (size_t b = 0; b < num_mboards; ++b) {
        const uint32_t central_freq = (uint32_t) (usrp_tune_freqs[b]/1e6);

        // Filtering
        filter<<<nblocks, nthreads>>>(cbuf+b*num_samps, sigbuf+b*num_samps, num_samps);

        // DFT 
        dft<<<nblocks, nthreads>>>(sigbuf+b*num_samps, chanbuf+b*num_samps, num_samps);
    }
}
  
void chan_processing(iqsamp_t* samplebuf, uint8_t* binbuf, size_t num_samps, size_t num_mboards)
{
    // Reset buffers
    cudaMemset(binbuf, 0, num_mboards * num_samps * sizeof(uint8_t));

    const size_t samps_per_chan = num_samps/DECFACTOR;

    size_t nthreads = 512;
    size_t nblocks = (num_samps + nthreads - 1) / nthreads;

    clean_aa_table<<<1,1>>>(conn_list);

    for (size_t b = 0; b < num_mboards; ++b) {
        const uint32_t central_freq = (uint32_t) (usrp_tune_freqs[b]/1e6);

        // Demodulate bit sequences
        for (size_t ch = 0; ch < DECFACTOR; ch++) {
            demod<<<nblocks, nthreads>>>(chanbuf + b*num_samps + ch*samps_per_chan,
                                         binbuf + b*num_samps + ch*samps_per_chan,
                                         samps_per_chan);
        }

        // First process Adv Channels (discover new connections)
        for (int ch = 0; ch < DECFACTOR; ch++) {
            int32_t delta_freq = (ch <= DECFACTOR/2) ? ch*2 : (-DECFACTOR + ch)*2;
            uint32_t blechan = freq2chan(central_freq + delta_freq);

            if (is_adv_chan(blechan))
                detect_ble_packets<<<nblocks, nthreads>>>(
                    binbuf + b*num_samps + ch*samps_per_chan,
                    samps_per_chan, blechan, conn_list, LE1M_SRATE);
        }

        // Detect packets on all the other channels
        for (int ch = 0; ch < DECFACTOR; ch++) {
            int32_t delta_freq = (ch <= DECFACTOR/2) ? ch*2 : (-DECFACTOR + ch)*2;
            uint32_t blechan = freq2chan(central_freq + delta_freq);

            if (not is_adv_chan(blechan)) {
                // LE 1M
                detect_ble_packets<<<nblocks, nthreads>>>(
                    binbuf + b*num_samps + ch*samps_per_chan,
                    samps_per_chan, blechan, conn_list, LE1M_SRATE);
                // LE 2M
                //detect_ble_packets<<<nblocks, nthreads>>>(
                    //binbuf + b*num_samps + ch*samps_per_chan,
                    //samps_per_chan, blechan, conn_list, LE2M_SRATE);
                bfscan_channel<<<nblocks, nthreads>>>(
                    binbuf + b*num_samps + ch*samps_per_chan,
                    samps_per_chan, blechan, conn_list, LE1M_SRATE);
            }
        }
    }
}
  
