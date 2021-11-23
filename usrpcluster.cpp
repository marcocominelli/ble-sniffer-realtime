//
// Copyright 2019 Marco Cominelli
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
#include <string>
#include <vector>
#include <csignal>
#include <complex>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/algorithm/string.hpp>

#include <uhd/device.hpp>
#include <uhd/exception.hpp>
#include <uhd/usrp_clock/multi_usrp_clock.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/device_addr.hpp>

#include "usrpcluster.hpp"

void sync_thread(uhd::usrp::multi_usrp::sptr mboard,
                 uhd::time_spec_t time,
                 boost::barrier &bar)
{
    bar.wait();
    mboard->set_time_next_pps(time);
    return;
}

void streaming(std::vector<uhd::rx_streamer::sptr> rxstream,
               std::vector<std::vector<iqsamp_t>> buffer,
               size_t idx,
               bool &stopsig)
{
    double timeout_time = 5.0;
    uhd::rx_metadata_t md;

    while (stopsig == false) {
        // Receive samples
        rxstream[idx]->recv(buffer[idx], buffer[idx].size(), md, timeout_time);

        // Throw exception if errors occur
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            std::cerr << std::endl << "OVERFLOW DETECTED" << std::endl;
        } else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::cout << std::endl;
            throw std::runtime_error(md.strerror());
        }
    }
}


UsrpCluster::UsrpCluster(std::string usrp_args)
{
    // Parse arguments to create UsrpCluster
    std::vector<std::string> args_list;
    boost::split(args_list, usrp_args, boost::is_any_of(", "),
                 boost::token_compress_on);

    // Add devices to UsrpCluster
    for (const auto& mboard_args: args_list)
        usrp.push_back(uhd::usrp::multi_usrp::make(mboard_args));

    // Set external clock and time sources for all motherboards
    const std::string ref = "external";
    for (auto& mboard: usrp) {
        mboard->set_clock_source(ref);
        mboard->set_time_source(ref);
    }

    sync_required = true;

    // Allow some time for setting up the hardware
    boost::this_thread::sleep(boost::posix_time::seconds(1));
}

UsrpCluster::UsrpCluster(std::string usrp_args, std::string subdev)
{
    // Parse arguments to create UsrpCluster
    std::vector<std::string> args_list;
    boost::split(args_list, usrp_args, boost::is_any_of(", "),
                 boost::token_compress_on);

    // Add devices to UsrpCluster
    for (const auto& mboard_args: args_list)
        usrp.push_back(uhd::usrp::multi_usrp::make(mboard_args));

    // Select subdevices
    for (auto& mboard: usrp) {
        mboard->set_rx_subdev_spec(subdev);
    }

    // Set external clock and time sources for all motherboards
    const std::string ref = "external";
    for (auto& mboard: usrp) {
        mboard->set_clock_source(ref);
        mboard->set_time_source(ref);
    }

    sync_required = true;

    // Allow some time for setting up the hardware
    boost::this_thread::sleep(boost::posix_time::seconds(1));
}

UsrpCluster::UsrpCluster(std::string usrp_args, std::string subdev,
                         bool sync_req)
{
    // Parse arguments to create UsrpCluster
    std::vector<std::string> args_list;
    boost::split(args_list, usrp_args, boost::is_any_of(", "),
                 boost::token_compress_on);

    // Add devices to UsrpCluster
    for (const auto& mboard_args: args_list)
        usrp.push_back(uhd::usrp::multi_usrp::make(mboard_args));

    // Select subdevices
    for (auto& mboard: usrp) {
        mboard->set_rx_subdev_spec(subdev);
    }

    // Set external clock and time sources for all motherboards
    const std::string ref = "external";
    for (auto& mboard: usrp) {
        mboard->set_clock_source(ref);
        mboard->set_time_source(ref);
    }

    // Set synchronization
    sync_required = sync_req;

    // Allow some time for setting up the hardware
    boost::this_thread::sleep(boost::posix_time::seconds(1));
}


void UsrpCluster::set_time_next_pps(double time)
{
    uhd::time_spec_t start(time);
    if (sync_required == false) {
        for (auto& mboard: usrp) mboard->set_time_now(start);
        return;
    }
    boost::barrier bar(usrp.size());
    boost::thread_group sync_group;
    for (auto& mboard: usrp) {
        sync_group.create_thread(
            boost::bind(&sync_thread, mboard, start, boost::ref(bar)));
    }
    sync_group.join_all();
    boost::this_thread::sleep(boost::posix_time::seconds(1));
}


void UsrpCluster::set_freq(const std::vector<double> &freq)
{
    for (size_t i = 0; i < usrp.size(); ++i)
        for (auto& channel: channel_nums)
            usrp[i]->set_rx_freq(freq[i], channel);
}


void UsrpCluster::set_rate(double rate)
{
    m_rate = rate;
    for (auto& mboard: usrp)
        for (auto& channel: channel_nums)
            mboard->set_rx_rate(m_rate, channel);
}


void UsrpCluster::set_gain(double gain)
{
    m_gain = gain;
    for (auto& mboard: usrp)
        for (auto& channel: channel_nums)
            mboard->set_rx_gain(m_gain, channel);
}


void UsrpCluster::set_ant(std::string ant)
{
    m_ant = ant;
    for (auto& mboard: usrp)
        for (auto& channel: channel_nums)
            mboard->set_rx_antenna(m_ant, channel);
}


void UsrpCluster::set_chan(std::string channel_list)
{
    std::vector<std::string> channels;
    boost::split(channels, channel_list, boost::is_any_of("\"',"));
    for (size_t j = 0; j < channels.size(); ++j) {
            channel_nums.push_back(boost::lexical_cast<size_t>(channels[j]));
    }
}


bool UsrpCluster::all_mboards_locked()
{
    bool all_locked = true;

    for (auto& mboard: usrp) {
        bool is_locked = mboard->get_mboard_sensor("ref_locked").to_bool();
        if (is_locked == false) all_locked = false;
    }

    return all_locked;
}


void UsrpCluster::init_rx_streamer(std::string cpu_fmt, std::string otw_fmt)
{
    uhd::stream_args_t stream_args(cpu_fmt, otw_fmt);
    stream_args.channels = channel_nums;
    for (const auto& mboard: usrp)
        rxstreamer.push_back(mboard->get_rx_stream(stream_args));
}


size_t UsrpCluster::get_max_num_samps()
{
    std::vector<size_t> num_samps;
    for (const auto& mboard: rxstreamer)
        num_samps.push_back(mboard->get_max_num_samps());

    return *std::min_element(num_samps.begin(), num_samps.end());
}

