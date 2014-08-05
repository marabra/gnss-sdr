/*!
 * \file galileo_e1_signal_processing.cc
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "compass_b1_signal_processing.h"
#include <string>


void
compass_b1_code_gen_int(int* _dest, char _Signal[3], signed int _prn,
        unsigned int _chip_shift)
{
	std::cout<< "(src/algorithms/libs/compass_b1_signal_processing)Entered in compass_b1_code_gen_int."<< std::endl;
    std::string _compass_signal = _Signal;
    std::cout<< "(src/algorithms/libs/compass_b1_signal_processing) Compass signal in compass_b1_code_gen_int: " << _compass_signal << std::endl;
    std::cout<< "(src/algorithms/libs/compass_b1_signal_processing) Compass signal in compass_b1_code_gen_int: PRN = " << _prn << std::endl;
    std::cout<< "(src/algorithms/libs/compass_b1_signal_processing) Compass signal in compass_b1_code_gen_int: _chip_shift = " << _chip_shift << std::endl;
    signed int prn = _prn - 1;
    int index = 0;
    //int* dest = _dest;

    /* A simple error check */
    if ((_prn < 1) || (_prn > 37))
        {
            return;
        }

    if (_compass_signal.rfind("1B") != std::string::npos && _compass_signal.length() >= 2) // which is the meaning of _compass_signal.length() >= 2???
        {

            for (size_t i = 0; i < Compass_B1_PRIMARY_CODE[prn].length(); i++)
                {

                    hex_to_binary_converter(&_dest[index],
                            Compass_B1_PRIMARY_CODE[prn].at(i));
                    int bin_to_print=_dest[index];
                    //std::cout<<bin_to_print;
                    index = index + 4;
                }

        }
    else
        {
            return;
        }
}



//void
//galileo_e1_gen(std::complex<float>* _dest, int* _prn, char _Signal[3])
//{
//    std::string _galileo_signal = _Signal;
//    const unsigned int _codeLength = 12 * Galileo_E1_B_CODE_LENGTH_CHIPS;
//    const float alpha = sqrt(10.0 / 11.0);
//    const float beta = sqrt(1.0 / 11.0);
//
//    std::complex<float> sinboc_11[12*4092]; //  _codeLength not accepted by Clang
//    std::complex<float> sinboc_61[12*4092];
//
//    galileo_e1_sinboc_11_gen(sinboc_11, _prn, _codeLength); //generate sinboc(1,1) 12 samples per chip
//    galileo_e1_sinboc_61_gen(sinboc_61, _prn, _codeLength); //generate sinboc(6,1) 12 samples per chip
//
//    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
//        {
//            for (unsigned int i = 0; i < _codeLength; i++)
//                {
//                    _dest[i] = alpha * sinboc_11[i] + beta * sinboc_61[i];
//                }
//        }
//    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
//        {
//            for (unsigned int i = 0; i < _codeLength; i++)
//                {
//                    _dest[i] = alpha * sinboc_11[i] - beta * sinboc_61[i];
//                }
//        }
//    else
//        return;
//}
//

//FROM GALILEO
void
compass_b1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
        unsigned int _prn, signed int _fs, unsigned int _chip_shift,
        bool _secondary_flag)
  {
	std::cout<< "Entered in compass_b1_code_gen_complex_sampled " << std::endl;
    // This function is based on the GNU software GPS for MATLAB in Kay Borre's book

    std::string _Compass_signal = _Signal;
    unsigned int _samplesPerCode;
    const int _codeFreqBasis = Compass_B1_CODE_CHIP_RATE_HZ; //2.046e6 Hz
    unsigned int _codeLength = Compass_B1_CODE_LENGTH_CHIPS; //2046.0 chip

    std::cout<< "(src/algorithms/libs/compass_b1_signal_processing) Compass signal in compass_b1_code_gen_complex_sampled: Compass_signal=" << _Compass_signal << std::endl;
    std::cout<< "(src/algorithms/libs/compass_b1_signal_processing) Compass signal in compass_b1_code_gen_complex_sampled: PRN = " << _prn << std::endl;
    std::cout<< "(src/algorithms/libs/compass_b1_signal_processing) Compass signal in compass_b1_code_gen_complex_sampled: _chip_shift = " << _chip_shift << std::endl;

    int primary_code_B1_chips[(int)Compass_B1_CODE_LENGTH_CHIPS];

    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));  //in this case if fs=4000000, sample per code is 4000
    //const int _samplesPerChip = (_cboc == true) ? 12 : 2;
    const int _samplesPerChip =_samplesPerCode/Compass_B1_CODE_LENGTH_CHIPS; // if fs=4000000, sample per chip is 4000/2046=1.955

    const unsigned int delay = (((int)Compass_B1_CODE_LENGTH_CHIPS - _chip_shift)
                                % (int)Compass_B1_CODE_LENGTH_CHIPS)
                                * _samplesPerCode / Compass_B1_CODE_LENGTH_CHIPS;

    compass_b1_code_gen_int(primary_code_B1_chips, _Signal, _prn, 0); //generate Compass E1 code, 1 sample per chip

    std::complex<float>* _signal_B1;

    _codeLength = _samplesPerChip * Compass_B1_CODE_LENGTH_CHIPS; //1.995*2046 = 4000
    _signal_B1 = new std::complex<float>[_codeLength];

//    if (_cboc == true)
//        {
//            galileo_e1_gen(_signal_E1, primary_code_E1_chips, _Signal); //generate cboc 12 samples per chip
//        }
//    else
//        {
//            galileo_e1_sinboc_11_gen(_signal_E1, primary_code_E1_chips,
//                    _codeLength); //generate sinboc(1,1) 2 samples per chip
//        }

    if (_fs != _samplesPerChip * _codeFreqBasis) //in this case fs=1.955*2046000=4000000
        {
            std::complex<float>* _resampled_signal = new std::complex<float>[_samplesPerCode];
            resampler(_signal_B1, _resampled_signal, _samplesPerChip * _codeFreqBasis, _fs,
                    _codeLength, _samplesPerCode); //resamples code to fs

            delete[] _signal_B1;
            _signal_B1 = _resampled_signal;
        }

    // add NH modulation on Compass primary code
    //if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
    //{

        std::complex<float>* _signal_B1_secondary = new std::complex<float>
                                                    [(int)Compass_NH_SECONDARY_CODE_LENGTH
                                                    * _samplesPerCode];

        for (unsigned int i = 0; i < (int)Compass_NH_SECONDARY_CODE_LENGTH; i++)
            {
                for (unsigned k = 0; k < _samplesPerCode; k++)
                    {
                        _signal_B1_secondary[i*_samplesPerCode + k] = _signal_B1[k]
                                * (Compass_NH_SECONDARY_CODE.at(i) == '0'
                                   ? std::complex<float>(1,0) : std::complex<float>(-1,0));
                    }
            }

        _samplesPerCode *= (int)Compass_NH_SECONDARY_CODE_LENGTH;

        delete[] _signal_B1;
        _signal_B1 = _signal_B1_secondary;
    //}

    for (unsigned int i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i+delay)%_samplesPerCode] = _signal_B1[i];
        }

    delete[] _signal_B1;
}

void
compass_b1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
        unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    compass_b1_code_gen_complex_sampled(_dest, _Signal, _prn,
                                        _fs, _chip_shift, false);
}
