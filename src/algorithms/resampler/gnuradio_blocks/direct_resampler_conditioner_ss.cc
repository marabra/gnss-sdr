/*!
 * \file direct_resampler_conditioner_ss.cc
 * \brief Nearest neighborhood resampler with
 *        short input and short output
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
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


#include "direct_resampler_conditioner_ss.h"
#include <algorithm>
#include <iostream>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "gps_sdr_signal_processing.h"


using google::LogMessage;

direct_resampler_conditioner_ss_sptr direct_resampler_make_conditioner_ss(
        double sample_freq_in, double sample_freq_out)
{

    return direct_resampler_conditioner_ss_sptr(
            new direct_resampler_conditioner_ss(sample_freq_in,
                    sample_freq_out));
}

direct_resampler_conditioner_ss::direct_resampler_conditioner_ss(
        double sample_freq_in, double sample_freq_out) :
    gr::block("direct_resampler_make_conditioner_ss", gr::io_signature::make(1,
            1, sizeof(short)), gr::io_signature::make(1, 1, sizeof(short))),
            d_sample_freq_in(sample_freq_in), d_sample_freq_out(
                    sample_freq_out), d_phase(0), d_lphase(0), d_history(1)
{

    // Computes the phase step multiplying the resampling ratio by 2^32 = 4294967296
    if (d_sample_freq_in >= d_sample_freq_out)
    {
        d_phase_step = (unsigned int)floor((double)4294967296.0
                * sample_freq_out / sample_freq_in);
    }
    else
    {
        d_phase_step = (unsigned int)floor((double)4294967296.0
                * sample_freq_in / sample_freq_out);
    }

    set_relative_rate(1.0 * sample_freq_out / sample_freq_in);
    set_output_multiple(1);
}

direct_resampler_conditioner_ss::~direct_resampler_conditioner_ss()
{

}

void direct_resampler_conditioner_ss::forecast(int noutput_items,
        gr_vector_int &ninput_items_required)
{

    int nreqd = std::max((unsigned)1, (int)((double)(noutput_items + 1)
            * sample_freq_in() / sample_freq_out()) + history() - 1);
    unsigned ninputs = ninput_items_required.size();

    for (unsigned i = 0; i < ninputs; i++)
    {
        ninput_items_required[i] = nreqd;
    }
}

int direct_resampler_conditioner_ss::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{

    const CPX *in = (const CPX *)input_items[0];
    CPX *out = (CPX *)output_items[0];

    int lcv = 0;
    int count = 0;

    if (d_sample_freq_in >= d_sample_freq_out)
    {
        while ((lcv < noutput_items))
        {
            if (d_phase <= d_lphase)
            {
                out[lcv] = *in;
                lcv++;
            }

            d_lphase = d_phase;
            d_phase += d_phase_step;
            in++;
            count++;
        }
    }
    else
    {
        while ((lcv < noutput_items))
        {
            d_lphase = d_phase;
            d_phase += d_phase_step;
            if (d_phase <= d_lphase)
            {
                in++;
                count++;
            }
            out[lcv] = *in;
            lcv++;
        }
    }

    consume_each(std::min(count, ninput_items[0]));
    return lcv;
}
