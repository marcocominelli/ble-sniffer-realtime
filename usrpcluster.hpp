//
// Copyright 2019 Marco Cominelli
//

#ifndef USRP_CLUSTER_HPP
#define USRP_CLUSTER_HPP

#include <string>
#include <vector>

#include <uhd/usrp/multi_usrp.hpp>

typedef std::complex<double> iqsamp_t;

// UsrpCluster is designed to handle multiple multi_usrp objects.
// This is useful if one wants to overcome the limitation of having at most one
// USRP Bxxx device per multi_usrp object. For example, the operation of
// multiple USRP B210 devices can be syncrhonized with a UsrpCluster object.
class UsrpCluster
{
public:
    UsrpCluster(std::string usrp_args);
    UsrpCluster(std::string usrp_args, std::string subdev);
    UsrpCluster(std::string usrp_args, std::string subdev, bool sync_req);
    ~UsrpCluster() {};

    // Set frequency, rate and gain across all boards
    void set_freq(const std::vector<double> &freq);
    void set_rate(double rate);
    void set_gain(double gain);
    void set_ant(std::string ant);
    void set_chan(std::string channel_list);

    // Get frequency, rate and gain across all boards
    double get_freq() {return m_freq;}
    double get_rate() {return m_rate;}
    double get_gain() {return m_gain;}

    // Get current number of synchronised motherboards
    size_t get_num_mboards() {return usrp.size();}

    // Get handle to rx streamer
    std::vector<uhd::rx_streamer::sptr>& get_rx_streamer() {return rxstreamer;}

    // Set timestamp on all the boards
    void set_time_next_pps(double time);

    // Check if all motherboards are time-locked
    bool all_mboards_locked();

    // Initialize rx streamer
    void init_rx_streamer(std::string cpu_fmt, std::string otw_fmt);

    // Get maximum number of samples streamed by each motherboard
    size_t get_max_num_samps();

    // Get number of rx channels
    size_t get_num_rx_channels() {return channel_nums.size();}


private:
    std::vector<uhd::usrp::multi_usrp::sptr> usrp;
    std::vector<uhd::rx_streamer::sptr> rxstreamer;

    double m_freq;
    double m_rate;
    double m_gain;
    std::string m_ant;
    std::vector<size_t> channel_nums;

    bool sync_required;
};

#endif //USRP_CLUSTER_HPP
