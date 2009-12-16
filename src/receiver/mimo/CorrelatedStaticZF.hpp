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

#ifndef OFDMAPHY_RECEIVER_MIMO_CORRELATEDSTATICZF_HPP
#define OFDMAPHY_RECEIVER_MIMO_CORRELATEDSTATICZF_HPP

#include <OFDMAPHY/receiver/mimo/ICalculationStrategy.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <complex>

namespace ofdmaphy { namespace receiver { namespace mimo {

    /**
	 * @brief IMTAStatic correlation between antennas
	 */
    class CorrelatedStaticZF :
        public ICalculationStrategy
    {

    public:
        CorrelatedStaticZF(const wns::pyconfig::View&,
                           Receiver* rx,
                           wns::logger::Logger*);

        virtual std::vector<wns::Ratio>
        getPostProcessingSINRFactor(rise::TransmissionObjectPtr t);

        virtual double
        getArrayOrientation() const
            {
                return this->arrayOrientation;
            };

        virtual double
        getAntennaSpacing() const
            {
                return this->antennaSpacing;
            }

        virtual double
        getAngleSpread() const
            {
                return this->angleSpread;
            }

    private:
        std::vector<wns::Ratio>
        getCorrelationLoss(int n, double angle, double spacing, double spread);

        wns::logger::Logger* logger;
        Receiver* rx;
        double arrayOrientation;
        double antennaSpacing;
        double angleSpread;

    };
}
}
}
#endif // not defined __RISE_OFDMA_HPP



