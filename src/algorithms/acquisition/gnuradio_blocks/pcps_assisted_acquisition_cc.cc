/*!
 * \file pcps_assisted_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition with assistance and multi-dwells
 * \authors <ul>
 *          <li> Javier Arribas, 2013. jarribas(at)cttc.es
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "pcps_assisted_acquisition_cc.h"
#include <sstream>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "nco_lib.h"
#include "concurrent_map.h"
#include "gnss_signal_processing.h"
#include "control_message_factory.h"
#include "gps_acq_assist.h"

extern concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

using google::LogMessage;

pcps_assisted_acquisition_cc_sptr pcps_make_assisted_acquisition_cc(
        int max_dwells, unsigned int sampled_ms, int doppler_max, int doppler_min, long freq,
        long fs_in, int samples_per_ms, boost::shared_ptr<gr::msg_queue> queue, bool dump,
        std::string dump_filename)
{

    return pcps_assisted_acquisition_cc_sptr(
            new pcps_assisted_acquisition_cc(max_dwells, sampled_ms, doppler_max, doppler_min, freq,
                    fs_in, samples_per_ms, queue, dump, dump_filename));
}



pcps_assisted_acquisition_cc::pcps_assisted_acquisition_cc(
        int max_dwells, unsigned int sampled_ms, int doppler_max, int doppler_min, long freq,
        long fs_in, int samples_per_ms, boost::shared_ptr<gr::msg_queue> queue, bool dump,
        std::string dump_filename) :
		        gr::block("pcps_assisted_acquisition_cc",
		                gr::io_signature::make(1, 1, sizeof(gr_complex)),
		                gr::io_signature::make(0, 0, sizeof(gr_complex)))
{
    d_sample_counter = 0;    // SAMPLE COUNTER
    d_active = false;
    d_queue = queue;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_sampled_ms = sampled_ms;
    d_config_doppler_max = doppler_max;
    d_config_doppler_min=doppler_min;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    // HS Acquisition
    d_max_dwells = max_dwells;
    d_gnuradio_forecast_samples = d_fft_size*4;
    d_input_power = 0.0;
    d_state = 0;
    d_disable_assist = false;
    //todo: do something if posix_memalign fails
    if (posix_memalign((void**)&d_carrier, 16, d_fft_size * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_fft_codes, 16, d_fft_size * sizeof(gr_complex)) == 0){};

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;
}



void pcps_assisted_acquisition_cc::set_doppler_step(unsigned int doppler_step)
{
    d_doppler_step = doppler_step;
}



void pcps_assisted_acquisition_cc::free_grid_memory()
{
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            delete[] d_grid_data[i];
            delete[] d_grid_doppler_wipeoffs[i];
        }
    delete d_grid_data;
}



pcps_assisted_acquisition_cc::~pcps_assisted_acquisition_cc()
{
    free(d_carrier);
    free(d_fft_codes);
    delete d_ifft;
    delete d_fft_if;
    if (d_dump)
        {
            d_dump_file.close();
        }
}



void pcps_assisted_acquisition_cc::set_local_code(std::complex<float> * code)
{
    memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex)*d_fft_size);
}



void pcps_assisted_acquisition_cc::init()
{
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_input_power = 0.0;
    d_state = 0;

    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    volk_32fc_conjugate_32fc_a(d_fft_codes, d_fft_if->get_outbuf(), d_fft_size);
}



void pcps_assisted_acquisition_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = d_gnuradio_forecast_samples ; //set the required available samples in each call
}



void pcps_assisted_acquisition_cc::get_assistance()
{
    Gps_Acq_Assist gps_acq_assisistance;
    if (global_gps_acq_assist_map.read(this->d_gnss_synchro->PRN, gps_acq_assisistance)==true)
        {
            //TODO: use the LO tolerance here
            if (gps_acq_assisistance.dopplerUncertainty >= 1000)
                {
                    d_doppler_max = gps_acq_assisistance.d_Doppler0 + gps_acq_assisistance.dopplerUncertainty*2;
                    d_doppler_min = gps_acq_assisistance.d_Doppler0 - gps_acq_assisistance.dopplerUncertainty*2;
                }
            else
                {
                    d_doppler_max = gps_acq_assisistance.d_Doppler0 + 1000;
                    d_doppler_min = gps_acq_assisistance.d_Doppler0 - 1000;
                }
            this->d_disable_assist = false;
            std::cout << "Acq assist ENABLED for GPS SV "<< this->d_gnss_synchro->PRN <<" (Doppler max,Doppler min)=("
                      << d_doppler_max << "," << d_doppler_min << ")" << std::endl;
        }
    else
        {
            this->d_disable_assist = true;
            std::cout << "Acq assist DISABLED for GPS SV "<< this->d_gnss_synchro->PRN << std::endl;
        }
}



void pcps_assisted_acquisition_cc::reset_grid()
{
    d_well_count = 0;
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            for (unsigned int j = 0; j < d_fft_size; j++)
                {
                    d_grid_data[i][j] = 0.0;
                }
        }
}



void pcps_assisted_acquisition_cc::redefine_grid()
{
    if (this->d_disable_assist == true)
        {
            d_doppler_max = d_config_doppler_max;
            d_doppler_min = d_config_doppler_min;
        }
    // Create the search grid array
    d_num_doppler_points = floor(std::abs(d_doppler_max-d_doppler_min)/d_doppler_step);

    d_grid_data = new float*[d_num_doppler_points];
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            d_grid_data[i] = new float[d_fft_size];
        }

    // create the carrier Doppler wipeoff signals
    int doppler_hz;
    float phase_step_rad;
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_points];
    for (int doppler_index = 0; doppler_index < d_num_doppler_points; doppler_index++)
        {
            doppler_hz = d_doppler_min + d_doppler_step*doppler_index;
            // doppler search steps
            // compute the carrier doppler wipe-off signal and store it
            phase_step_rad = (float)GPS_TWO_PI*doppler_hz / (float)d_fs_in;
            d_grid_doppler_wipeoffs[doppler_index] = new gr_complex[d_fft_size];
            fxp_nco(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, 0, phase_step_rad);
        }
}



double pcps_assisted_acquisition_cc::search_maximum()
{
    float magt = 0.0;
    float fft_normalization_factor;
    int index_doppler = 0;
    unsigned int tmp_intex_t;
    unsigned int index_time = 0;

    for (int i=0;i<d_num_doppler_points;i++)
        {
            volk_32f_index_max_16u_a(&tmp_intex_t,d_grid_data[i],d_fft_size);
            if (d_grid_data[i][tmp_intex_t] > magt)
                {
                    magt = d_grid_data[i][index_time];
                    index_doppler = i;
                    index_time = tmp_intex_t;
                }
        }

    // Normalize the maximum value to correct the scale factor introduced by FFTW
    fft_normalization_factor = (float)d_fft_size * (float)d_fft_size;
    magt = magt / (fft_normalization_factor * fft_normalization_factor);

    // 5- Compute the test statistics and compare to the threshold
    d_test_statistics = 2 * d_fft_size * magt /(d_input_power*d_well_count);

    // 4- record the maximum peak and the associated synchronization parameters
    d_gnss_synchro->Acq_delay_samples = (double)index_time;
    d_gnss_synchro->Acq_doppler_hz = (double)(index_doppler*d_doppler_step + d_doppler_min);
    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

    // Record results to file if required
    if (d_dump)
        {
            std::stringstream filename;
            std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write
            filename.str("");
            filename << "../data/test_statistics_" << d_gnss_synchro->System
                     << "_" << d_gnss_synchro->Signal << "_sat_"
                     << d_gnss_synchro->PRN << "_doppler_" <<  d_gnss_synchro->Acq_doppler_hz << ".dat";
            d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
            d_dump_file.write((char*)d_grid_data[index_doppler], n); //write directly |abs(x)|^2 in this Doppler bin?
            d_dump_file.close();
        }

    return d_test_statistics;
}



float pcps_assisted_acquisition_cc::estimate_input_power(gr_vector_const_void_star &input_items)
{
    const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
    // 1- Compute the input signal power estimation
    float* p_tmp_vector;
    if (posix_memalign((void**)&p_tmp_vector, 16, d_fft_size * sizeof(float)) == 0){};
    volk_32fc_magnitude_squared_32f_u(p_tmp_vector, in, d_fft_size);

    const float* p_const_tmp_vector = p_tmp_vector;
    float power;
    volk_32f_accumulator_s32f_a(&power, p_const_tmp_vector, d_fft_size);
    free(p_tmp_vector);
    return ( power / (float)d_fft_size);
}



int pcps_assisted_acquisition_cc::compute_and_accumulate_grid(gr_vector_const_void_star &input_items)
{
    // initialize acquisition algorithm
    const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " "
               << d_gnss_synchro->PRN
               << " ,sample stamp: " << d_sample_counter << ", threshold: "
               << d_threshold << ", doppler_max: " << d_doppler_max
               << ", doppler_step: " << d_doppler_step;

    // 2- Doppler frequency search loop
    float* p_tmp_vector;
    if (posix_memalign((void**)&p_tmp_vector, 16, d_fft_size * sizeof(float)) == 0){};

    for (int doppler_index = 0; doppler_index < d_num_doppler_points; doppler_index++)
        {
            // doppler search steps
            // Perform the carrier wipe-off
            volk_32fc_x2_multiply_32fc_u(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs[doppler_index], d_fft_size);
            // 3- Perform the FFT-based convolution  (parallel time search)
            // Compute the FFT of the carrier wiped--off incoming signal
            d_fft_if->execute();

            // Multiply carrier wiped--off, Fourier transformed incoming signal
            // with the local FFT'd code reference using SIMD operations with VOLK library
            volk_32fc_x2_multiply_32fc_a(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

            // compute the inverse FFT
            d_ifft->execute();

            // save the grid matrix delay file
            volk_32fc_magnitude_squared_32f_a(p_tmp_vector, d_ifft->get_outbuf(), d_fft_size);
            const float*  old_vector = d_grid_data[doppler_index];
            volk_32f_x2_add_32f_a(d_grid_data[doppler_index], old_vector, p_tmp_vector, d_fft_size);
        }
    free(p_tmp_vector);
    return d_fft_size;
}
int pcps_assisted_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    /*!
     * TODO: 	High sensitivity acquisition algorithm:
     * 			State Mechine:
     * 			S0. StandBy. If d_active==1 -> S1
     * 			S1. GetAssist. Define search grid with assistance information. Reset grid matrix -> S2
     * 			S2. ComputeGrid. Perform the FFT acqusition doppler and delay grid.
     * 				Accumulate the search grid matrix (#doppler_bins x #fft_size)
     * 				Compare maximum to threshold and decide positive or negative
     * 				If T>=gamma -> S4 else
     * 				If d_well_count<max_dwells -> S2
     * 				else if !disable_assist -> S3
     * 				else -> S5.
     * 			S3. RedefineGrid. Open the grid search to unasisted acquisition. Reset counters and grid. -> S2
     * 			S4. Positive_Acq: Send message and stop acq -> S0
     * 			S5. Negative_Acq: Send message and stop acq -> S0
     */

    switch (d_state)
    {
    case 0: // S0. StandBy
        if (d_active == true) d_state = 1;
        d_sample_counter += ninput_items[0]; // sample counter
        consume_each(ninput_items[0]);
        break;
    case 1: // S1. GetAssist
        get_assistance();
        redefine_grid();
        reset_grid();
        d_sample_counter += ninput_items[0]; // sample counter
        consume_each(ninput_items[0]);
        d_state = 2;
        break;
    case 2: // S2. ComputeGrid
        int consumed_samples;
        consumed_samples = compute_and_accumulate_grid(input_items);
        d_well_count++;
        if (d_well_count >= d_max_dwells)
            {
                d_state=3;
            }
        d_sample_counter += consumed_samples;
        consume_each(consumed_samples);
        break;
    case 3: // Compute test statistics and decide
        d_input_power = estimate_input_power(input_items);
        d_test_statistics = search_maximum();
        if (d_test_statistics > d_threshold)
            {
                d_state = 5;
            }
        else
            {
                if (d_disable_assist == false)
                    {
                        d_disable_assist = true;
                        std::cout << "Acq assist DISABLED for GPS SV "<< this->d_gnss_synchro->PRN << std::endl;
                        d_state = 4;
                    }
                else
                    {
                        d_state = 6;
                    }
            }
        d_sample_counter += ninput_items[0]; // sample counter
        consume_each(ninput_items[0]);
        break;
    case 4: // RedefineGrid
        free_grid_memory();
        redefine_grid();
        reset_grid();
        d_sample_counter += ninput_items[0]; // sample counter
        consume_each(ninput_items[0]);
        d_state = 2;
        break;
    case 5: // Positive_Acq
        DLOG(INFO) << "positive acquisition";
        DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
        DLOG(INFO) << "sample_stamp " << d_sample_counter;
        DLOG(INFO) << "test statistics value " << d_test_statistics;
        DLOG(INFO) << "test statistics threshold " << d_threshold;
        DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
        DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
        DLOG(INFO) << "input signal power " << d_input_power;
        d_active = false;
        // Send message to channel queue //0=STOP_CHANNEL 1=ACQ_SUCCESS 2=ACQ_FAIL
        d_channel_internal_queue->push(1); // 1-> positive acquisition
        free_grid_memory();
        // consume samples to not block the GNU Radio flowgraph
        d_sample_counter += ninput_items[0]; // sample counter
        consume_each(ninput_items[0]);
        d_state = 0;
        break;
    case 6: // Negative_Acq
        DLOG(INFO) << "negative acquisition";
        DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
        DLOG(INFO) << "sample_stamp " << d_sample_counter;
        DLOG(INFO) << "test statistics value " << d_test_statistics;
        DLOG(INFO) << "test statistics threshold " << d_threshold;
        DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
        DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
        DLOG(INFO) << "input signal power " << d_input_power;
        d_active = false;
        // Send message to channel queue //0=STOP_CHANNEL 1=ACQ_SUCCESS 2=ACQ_FAIL
        d_channel_internal_queue->push(2); // 2-> negative acquisition
        free_grid_memory();
        // consume samples to not block the GNU Radio flowgraph
        d_sample_counter += ninput_items[0]; // sample counter
        consume_each(ninput_items[0]);
        d_state = 0;
        break;
    default:
        d_state = 0;
        break;
    }

    return 0;
}
