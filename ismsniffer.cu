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
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <csignal>
#include <complex>
#include <cmath>

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <uhd/device.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread_priority.hpp>

#include "usrpcluster.hpp"
#include "doublebuffer.hpp"
#include "bleprocessor.h"
#include "pcapsaver.h"

static const size_t num_rep = 20000;
static const double timeout = 3.0;

typedef std::complex<double> iqsamp_t;

namespace po = boost::program_options; 


volatile bool stopsig = false;
void sigint_handler(int) {stopsig = true;}

void streaming(uhd::rx_streamer::sptr rxstream,
               DoubleBuffer<iqsamp_t*> *const buf_ctrl,
               const size_t samps_per_buf)
{
    uhd::set_thread_priority_safe(1);

    iqsamp_t* buf_ptr;
    iqsamp_t* old_buf_ptr;

    uhd::rx_metadata_t md;
    size_t num_rx_samps = 0;

    while (stopsig == false) {
        // Select buffer(s) to write.
        buf_ptr = *(buf_ctrl->start_writing());

        // Receive samples
        num_rx_samps += rxstream->recv(buf_ptr, samps_per_buf, md, timeout);

        // Release buffer and check for overflows
        buf_ctrl->end_writing();
        if (buf_ptr == old_buf_ptr)
            std::cerr << "An overflow has occurred" << std::endl;

        old_buf_ptr = buf_ptr;
        
        // Throw exception if errors occur
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
            std::cerr << std::endl << "OVERFLOW DETECTED" << std::endl;
        else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
            throw std::runtime_error(md.strerror());
    }
}


void register_new_packets(uint8_t* bits,
                          pcapmerger& pc,
                          size_t nsamps_per_mboard,
                          size_t samps_per_chan,
                          size_t num_mboards,
                          size_t num_samps_rx,
                          size_t* pktcount)
{
    const size_t header_size = 2;
    const size_t aa_length = 32;

    // Iterate over BLE channels looking for packets.
    for (size_t j = 0; j < samps_per_chan; j++) {
        for (size_t n = 0; n < num_mboards; n++) {
            for (size_t ch = 0; ch < DECFACTOR; ch++) {
                size_t i = n*nsamps_per_mboard + ch*samps_per_chan + j;
                if (bits[i] & 0xc0) {
                    // Found a new packet, check LE PHY.
                    size_t srate = (bits[i] & 0x80) ? LE1M_SRATE : LE2M_SRATE;
                    pktcount[n * DECFACTOR + ch]++;
                    
                    uint32_t blefreq = 0, blechan = 0;

                    int central_freq_int = (int) (usrp_tune_freqs[n]/1e6);
                    int freqidx = (i%nsamps_per_mboard)/samps_per_chan;
                    if (freqidx <= DECFACTOR/2)
                        blefreq = central_freq_int + freqidx*2;
                    else
                        blefreq = central_freq_int + (-DECFACTOR + freqidx)*2;
                    blechan = freq2chan(blefreq);

                    // Extract access address
                    size_t offset = i + 8*srate;
                    uint32_t aa = 0x00;
                    for (size_t c = 0; c < aa_length; ++c)
                        aa |= (bits[offset+c*srate] & 0x1) << c;

                    offset += aa_length*srate;
                    size_t payload_size = 0;
                    uint8_t header[header_size] = {0};
                    uint8_t pdu[260] = {0};
    
                    // Extract header and read payload length.
                    for (int byte = 0; byte < header_size; ++byte)
                        for (int b = 0; b < 8; ++b)
                            header[byte] |=
                                (bits[offset+(byte*8+b)*srate] & 0x1) << (7-b);
                    ble_dewhiten(header, blechan, header_size);
                    swap_nbytes(header, header_size);
                    payload_size = header[1];

                    // Extract BLE frame without checking CRC validity again.
                    size_t pdu_size = header_size + payload_size;
                    for (int byte = 0; byte < pdu_size+3; ++byte)
                        for (int b = 0; b < 8; ++b)
                            pdu[byte] |=
                                (bits[offset + (byte*8+b)*srate] & 0x1) << (7-b);
                    ble_dewhiten(pdu, blechan, pdu_size+3);
                    swap_nbytes(pdu, pdu_size);

                    uint32_t timenow_us = (num_samps_rx + j) / srate;
                    uint32_t timeusec = ((uint32_t) timenow_us) % 1000000;
                    uint32_t timesec = ((uint32_t) timenow_us - timeusec) / 1000000;
                    struct timeval ts = {timesec, timeusec};
                    pc.addpacket(&ts, blechan, aa, pdu_size+3, pdu, 0);
                    
                    // Ensure duplicate packets are discarded
                    for (int kk = 0; kk < srate; kk++) bits[i+1] &= 0x1;
                }
                else if (bits[i] & 0x10) {
                    size_t srate = LE1M_SRATE;
                    pktcount[n * DECFACTOR + ch]++;
                    
                    uint32_t blefreq = 0, blechan = 0;

                    int central_freq_int = (int) (usrp_tune_freqs[n]/1e6);
                    int freqidx = (i%nsamps_per_mboard)/samps_per_chan;
                    if (freqidx <= DECFACTOR/2)
                        blefreq = central_freq_int + freqidx*2;
                    else
                        blefreq = central_freq_int + (-DECFACTOR + freqidx)*2;
                    blechan = freq2chan(blefreq);

                    // Extract access address
                    size_t offset = i + 8*srate;
                    uint32_t aa = 0x00;
                    for (size_t c = 0; c < aa_length; ++c)
                        aa |= (bits[offset+c*srate] & 0x1) << c;

                    offset += aa_length*srate;
                    size_t payload_size = 0;
                    uint8_t header[header_size] = {0};
                    uint8_t pdu[260] = {0};
    
                    // Extract header and read payload length.
                    for (int byte = 0; byte < header_size; ++byte)
                        for (int b = 0; b < 8; ++b)
                            header[byte] |=
                                (bits[offset+(byte*8+b)*srate] & 0x1) << (7-b);
                    ble_dewhiten(header, blechan, header_size);
                    swap_nbytes(header, header_size);
                    payload_size = header[1];

                    if (payload_size != 0) continue;

                    // Extract BLE frame without checking CRC validity again.
                    size_t pdu_size = header_size;
                    for (int byte = 0; byte < pdu_size+3; ++byte)
                        for (int b = 0; b < 8; ++b)
                            pdu[byte] |=
                                (bits[offset + (byte*8+b)*srate] & 0x1) << (7-b);
                    ble_dewhiten(pdu, blechan, pdu_size+3);
                    swap_nbytes(pdu, pdu_size);

                    uint32_t timenow_us = (num_samps_rx + j) / srate;
                    uint32_t timeusec = ((uint32_t) timenow_us) % 1000000;
                    uint32_t timesec = ((uint32_t) timenow_us - timeusec) / 1000000;
                    std::cout << boost::format("%u%06u NewAA: %08X") % timesec % timeusec % aa;
                    std::cout << std::endl;

                    // Ensure duplicate packets are discarded
                    for (int kk = 0; kk < srate; kk++) bits[i+1] &= 0x1;
                }
            }
        }
    }
}


int UHD_SAFE_MAIN(int argc, char *argv[])
{
    // Set thread priority and escape signal.
    uhd::set_thread_priority_safe(1);
    signal(SIGINT, &sigint_handler);

    // Variables to be set by command line options.
    std::string usrp_args, fname, subdev, channel_string, antenna;
    double gain;
    bool sync_request, debug;

    // Set up program options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Display this help message")
        ("usrp-args", po::value<std::string>(&usrp_args),
            "USRP device arguments in a comma-separated list")
        ("gain", po::value<double>(&gain)->default_value(40),
            "rx gain (in dB)")
        ("subdev", po::value<std::string>(&subdev)->default_value("A:A"),
            "USRP subdevice (eg. \"A:A\" or \"A:A A:B\")")
        ("chan", po::value<std::string>(&channel_string)->default_value("0"),
            "rx channels (eg. \"0\" or \"0,1\")")
        ("antenna", po::value<std::string>(&antenna)->default_value("TX/RX"),
            "rx antenna (eg. \"TX/RX\" or \"RX2\")")
        ("file", po::value<std::string>(&fname)->default_value("trace.pcap"),
            "name of output pcap file")
        ("sync", po::value<bool>(&sync_request)->default_value(false),
            "use common clock for synchronization")
        ("debug", po::value<bool>(&debug)->default_value(false),
            "print info on recvd pkts per channel");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Display program info.
    std::cout << "Bluetooth Low Energy Wide-band sniffer\n" << std::endl;

    // Print the help message.
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return ~0;
    }

    // Create a new USRP cluster
    UsrpCluster usrp(usrp_args, subdev, sync_request);
    const size_t num_mboards = usrp.get_num_mboards();
    std::cout << boost::format("\nDetected %u mboards\n") % num_mboards
              << std::endl;

    const double sampling_rate = 40e6;

    // SDR front-end configuration.
    usrp.set_chan(channel_string);
    usrp.set_freq(usrp_tune_freqs);
    usrp.set_rate(sampling_rate);
    usrp.set_gain(gain);
    usrp.set_ant(antenna);

    // Initialize receving streamer
    const std::string cpu_fmt = "fc64"; // cpu: double
    const std::string otw_fmt = "sc16"; // otw: int16
    usrp.init_rx_streamer(cpu_fmt, otw_fmt);

    // Check if all USRPs are locked to the 10 MHz reference signal
    if (sync_request == false) {
        std::cerr
            << "[WARNING] Not using ref clock; timestamps will be misaligned."
            << std::endl;
    } else if (usrp.all_mboards_locked() == false) {
        // sync_request is true in this case
        std::cerr
            << "[ERROR] Some mboards are not locked to the ref clock!"
            << std::endl;
        return ~0;
    }

    // Allocate data buffers
    const size_t nsamps_per_mboard = num_rep * usrp.get_max_num_samps();
    const size_t nsamps_overall = nsamps_per_mboard * num_mboards;
    iqsamp_t* h_iqbuffer_a;
    iqsamp_t* h_iqbuffer_b;
    iqsamp_t* d_iqbuffer;
    uint8_t* h_binbuffer;
    uint8_t* d_binbuffer;

    cudaMallocHost((void**)&h_iqbuffer_a, nsamps_overall * sizeof(iqsamp_t));
    cudaMallocHost((void**)&h_iqbuffer_b, nsamps_overall * sizeof(iqsamp_t));
    cudaMallocHost((void**)&h_binbuffer, nsamps_overall * sizeof(uint8_t));
    cudaMalloc((void**)&d_iqbuffer, nsamps_overall * sizeof(iqsamp_t));
    cudaMalloc((void**)&d_binbuffer, nsamps_overall * sizeof(uint8_t));

    // Access control for the double buffer
    std::vector<DoubleBuffer<iqsamp_t*>> buf_ctrl(num_mboards);
    for (size_t i = 0; i < num_mboards; ++i) {
        buf_ctrl[i].m_buf[0] = h_iqbuffer_a + i*nsamps_per_mboard;
        buf_ctrl[i].m_buf[1] = h_iqbuffer_b + i*nsamps_per_mboard;
    }

    // Initialize data structures on the GPU
    init_decoder(nsamps_overall);

    // Reset timer on all the boards
    usrp.set_time_next_pps(0.0);

    // Setup streaming parameters
    const double t_start = 2.0;
    uhd::stream_cmd_t stream_start(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_start.num_samps = 0;
    stream_start.stream_now = false;
    stream_start.time_spec = uhd::time_spec_t(t_start);

    // Start streaming
    std::vector<uhd::rx_streamer::sptr> rxstreamer = usrp.get_rx_streamer();
    boost::thread_group stream_group;
    for (size_t i = 0; i < num_mboards; ++i) {
        rxstreamer[i]->issue_stream_cmd(stream_start);
        // Producer threads
        stream_group.create_thread(boost::bind(&streaming,
            rxstreamer[i], &buf_ctrl[i], nsamps_per_mboard));
    }
    std::cout
        << boost::format("start streaming in %.1lf s") % t_start
        << std::endl;

    // CUDA Streams
    cudaStream_t stream_h2d, stream_d2h;
    cudaStreamCreate(&stream_h2d);
    cudaStreamCreate(&stream_d2h);

    // Consumer thread
    iqsamp_t** readbuf;

    const size_t samps_per_chan = nsamps_per_mboard / DECFACTOR;

    size_t pktcount[num_mboards*DECFACTOR] = {0};

    pcapmerger pc;
    int err = pc.create(fname.c_str());
    if (err) {
        std::cerr << "Failed to create pcap saver" << std::endl;
        stopsig = true;
    }

    size_t num_samps_rx = 0;

    while (stopsig == false) {
        
        // Claim buffer for reading (wait until ready).
        do readbuf = buf_ctrl[0].start_reading();
        while (readbuf == nullptr);

        
        cudaMemcpyAsync(d_iqbuffer, *readbuf, nsamps_overall * sizeof(iqsamp_t),
                   cudaMemcpyHostToDevice, stream_h2d);
        cudaMemcpyAsync(h_binbuffer, d_binbuffer, nsamps_overall * sizeof(uint8_t),
                   cudaMemcpyDeviceToHost, stream_d2h);
        
        cudaStreamSynchronize(stream_h2d); // wait until iqsamps are ready
        iq_processing(d_iqbuffer, d_binbuffer, nsamps_per_mboard, num_mboards);

        cudaStreamSynchronize(stream_d2h); // wait until bits are ready
        chan_processing(d_iqbuffer, d_binbuffer, nsamps_per_mboard, num_mboards);

        // Save new packets to file.
        register_new_packets(h_binbuffer, pc, nsamps_per_mboard,
            samps_per_chan, num_mboards, num_samps_rx, pktcount);
        
        // Display how many packets have been decoded.
        if (debug) {
            for (int k = 0; k < num_mboards*DECFACTOR; k++) 
                std::cout << boost::format("%3d ") % pktcount[k];
            std::cout << std::endl;
        }

        num_samps_rx += samps_per_chan;
        buf_ctrl[0].end_reading();
 
    }

    std::cout << "\n\nShutting down...\n" << std::endl;
    stream_group.join_all();
    free_decoder();

    cudaStreamDestroy(stream_h2d);
    cudaStreamDestroy(stream_d2h);

    // Save data to pcap file.
    pc.flush();
    pc.finalise();

    // Free allocated memory.
    cudaFreeHost(h_iqbuffer_a);
    cudaFreeHost(h_iqbuffer_b);
    cudaFreeHost(h_binbuffer);
    cudaFree(d_iqbuffer);
    cudaFree(d_binbuffer);

    std::cout << "\nDone.\n" << std::endl;
    return 0;
}
