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

#ifndef OFDMAPHY_RECEIVER_RECEIVERBASE_HPP
#define OFDMAPHY_RECEIVER_RECEIVERBASE_HPP

#include <RISE/scenario/ftfading/FTFading.hpp>

#include <WNS/node/Interface.hpp>
#include <WNS/PowerRatio.hpp>
#include <WNS/logger/Logger.hpp>
#include <WNS/pyconfig/View.hpp>
#include <map>
#include <vector>

namespace ofdmaphy { namespace receiver {
    struct PerSourceContainer {
        int packetCount;
        rise::scenario::ftfading::FTFading* ftfading;
        wns::Ratio quasiStaticPathLoss;
        std::vector<wns::Power> interferenceVector;
    };

    /**
	 * @brief abstract base class for all aspects of OFDMA receiver
	 */
    class ReceiverBase
    {
    public:
        /** @brief Constructor */
        ReceiverBase();

        /** @brief Constructor with pyconfig */
        ReceiverBase(const wns::pyconfig::View& config);

        /** @brief Destructor */
        virtual ~ReceiverBase();

        virtual int
        getCurrentNumberOfSubCarriers() const = 0;

        virtual wns::Power
        getNoisePerSubChannel() const = 0;

    protected:
        wns::logger::Logger logger;

        /**
         * @brief information element for measurements and fading maps from the
         * source to its measured pathloss,ftfading
         */
        typedef std::map<wns::node::Interface*, PerSourceContainer> PerSourceMap;

        /** @brief container for multiple objects, one per source */
        PerSourceMap perSourceMap;
    };
}
}

#endif // not defined OFDMAPHY_RECEIVER_RECEIVERBASE_HPP



