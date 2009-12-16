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

#ifndef OFDMAPHY_RECEIVER_FTFADINGASPECT_HPP
#define OFDMAPHY_RECEIVER_FTFADINGASPECT_HPP

#include <OFDMAPHY/receiver/ReceiverBase.hpp>

#include <RISE/scenario/ftfading/FTFading.hpp>

#include <WNS/PowerRatio.hpp>
#include <WNS/pyconfig/View.hpp>

namespace ofdmaphy { namespace receiver {

        class MeasurementAspect;

    /**
	 * @brief implements the FTFading Aspect of a MultiCarrier receiver
	 */
    class FTFadingAspect :
        virtual public ReceiverBase
    {
        friend class MeasurementAspect;

    public:
        FTFadingAspect(const wns::pyconfig::View& config);
        virtual ~FTFadingAspect();

        bool
        FTFadingIsActive() const;

    protected:
        // next lines by [afo]:
        /** @brief get the current (frequency-and-time dependent) fading level of one subcarrier
		 * [+3..-inf] dB
		 */
        virtual wns::Ratio
        getFTFading(int _subCarrier);

        virtual wns::Ratio
        getFTFading(wns::node::Interface* source, int _subCarrier);

        // next line by [afo]:
        /** @brief single FTFading object */
        rise::scenario::ftfading::FTFading* ftfading;

    private:
        bool active;
        simTimeType samplingTime;
    };

}
}

#endif // not defined __RISE_OFDMA_HPP



