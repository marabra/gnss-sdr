/*!
 * \file galileo_e1_signal_processing.h
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_COMPASS_B1_SIGNAL_PROCESSING_H_
#define GNSS_SDR_COMPASS_B1_SIGNAL_PROCESSING_H_

#include <complex>
#include <iostream>
#include <gnuradio/math.h>
#include "Compass_B1.h"
#include "gnss_signal_processing.h"

/*!
 * \brief This function generates Compass B1 code (one sample per chip).
 *
 */

void compass_b1_code_gen_int(int* _dest, char _Signal[3], signed int _prn,
        unsigned int _chip_shift);

/*
 *  Generates complex Compass B1 code for the desired SV ID and sampled to specific sampling frequency
 */
//function galileo_code_gen_complex_sampled taken as reference, but the cboc has been remove from the required input
void compass_b1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
        unsigned int _prn, signed int _fs, unsigned int _chip_shift, bool _secondary_flag);



//FROM GALILEO
//void galileo_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
//        bool _cboc, unsigned int _prn, signed int _fs, unsigned int _chip_shift,
//
//        bool _secondary_flag);
//FROM GPS
//void gps_l1_ca_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift);

#endif /* GNSS_SDR_GALILEO_E1_SIGNAL_PROCESSING_H_ */