/*******************************************************************************
 * This file is part of openWNS (open Wireless Network Simulator)
 * _____________________________________________________________________________
 *
 * Copyright (C) 2004-2007
 * Chair of Communication Networks (ComNets)
 * Kopernikusstr. 5, D-52074 Aachen, Germany
 * phone: ++49-241-80-27910,
 * fax: ++49-241-80-22242
 * email: info@openwns.org
 * www: http://www.openwns.org
 * _____________________________________________________________________________
 *
 * openWNS is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License version 2 as published by the
 * Free Software Foundation;
 *
 * openWNS is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

#ifndef OFDMAPHY_RECEIVER_MIMO_NOCORRELATIONZF_HPP
#define OFDMAPHY_RECEIVER_MIMO_NOCORRELATIONZF_HPP

#include <OFDMAPHY/receiver/mimo/ICalculationStrategy.hpp>

namespace ofdmaphy { namespace receiver { namespace mimo {

    /**
	 * @brief No correlation between antennas at all
	 */
    class NoCorrelationZF :
        public ICalculationStrategy
    {

    public:
        NoCorrelationZF(const wns::pyconfig::View&,
                        Receiver* rx,
                        wns::logger::Logger*);

        virtual std::vector<wns::Ratio>
        getPostProcessingSINRFactor(rise::TransmissionObjectPtr t);

    private:
        wns::logger::Logger* logger;
        const Receiver* rx;

    };
}
}
}
#endif // not defined __RISE_OFDMA_HPP



