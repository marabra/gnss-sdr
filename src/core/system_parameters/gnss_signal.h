/*!
 * \file gnss_signal.h
 * \brief  Implementation of the Gnss_Signal class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GNSS_SIGNAL_H_
#define GNSS_SDR_GNSS_SIGNAL_H_

#include "gnss_satellite.h"
#include <string>

/*!
 * \brief This class represents a GNSS signal.
 *
 * It contains information about the space vehicle and the specific signal.
 */
class Gnss_Signal
{
private:
    Gnss_Satellite satellite;
    std::string signal;
public:
    Gnss_Signal();
    Gnss_Signal(Gnss_Satellite satellite_, std::string signal_);
    ~Gnss_Signal();
    std::string get_signal() const; //!<  Get the satellite system {"GPS", "GLONASS", "SBAS", "Galileo", "Compass"}
    Gnss_Satellite get_satellite() const;    //!< Get the Gnss_Satellite associated to the signal
    friend bool operator== (const Gnss_Signal &, const Gnss_Signal &);    //!< operator== for comparison
    friend std::ostream& operator<<(std::ostream &, const Gnss_Signal &); //!< operator<< for pretty printing
};

#endif
