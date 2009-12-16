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

#ifndef OFDMAPHY_RECEIVER_MIMO_ICALCULATIONSTRATEGY_HPP
#define OFDMAPHY_RECEIVER_MIMO_ICALCULATIONSTRATEGY_HPP

#include <WNS/StaticFactory.hpp>
#include <WNS/pyconfig/View.hpp>
#include <WNS/logger/Logger.hpp>
#include <WNS/PowerRatio.hpp>

#include <RISE/misc/pointer.hpp>

#include <boost/numeric/ublas/matrix.hpp>

namespace ofdmaphy { namespace receiver {
        class Receiver;
}}


namespace ofdmaphy { namespace receiver { namespace mimo {

    /**
	 * @brief Interface for the correlation strategy
	 */
    class ICalculationStrategy
    {

    public:
        ICalculationStrategy(const wns::pyconfig::View&,
                             Receiver* rx,
                             wns::logger::Logger*)
            {};

        virtual ~ICalculationStrategy()
            {};

        virtual std::vector<wns::Ratio>
        getPostProcessingSINRFactor(rise::TransmissionObjectPtr t) = 0;

    };

    /** @brief Special creator for ICalculationStrategy */
    template <typename T, typename KIND = T>
    class ICalculationStrategyCreator:
        public ICalculationStrategyCreator<KIND, KIND>
    {
    public:
        virtual KIND*
        create(const wns::pyconfig::View& _config,
               Receiver* _rx,
               wns::logger::Logger* _logger)
            {
                return new T(_config, _rx, _logger);
            }
    };

    template <typename KIND>
    class ICalculationStrategyCreator<KIND, KIND>
    {
    public:
        virtual
        ~ICalculationStrategyCreator() {};

        virtual KIND*
        create(const wns::pyconfig::View& _config,
               Receiver* rx,
               wns::logger::Logger* _logger) = 0;
    };

    typedef ICalculationStrategyCreator<ICalculationStrategy> CalculationStrategyCreator;
    typedef wns::StaticFactory<CalculationStrategyCreator> CalculationStrategyFactory;

}
}
}
#endif // not defined __RISE_OFDMA_HPP



