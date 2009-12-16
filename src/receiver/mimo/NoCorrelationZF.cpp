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

#include <OFDMAPHY/receiver/mimo/NoCorrelationZF.hpp>
#include <OFDMAPHY/receiver/Receiver.hpp>


using namespace ofdmaphy::receiver::mimo;

STATIC_FACTORY_REGISTER_WITH_CREATOR(NoCorrelationZF,
                                     ICalculationStrategy,
                                     "NoCorrelationZF",
                                     ICalculationStrategyCreator);

NoCorrelationZF::NoCorrelationZF(const wns::pyconfig::View& _config,
                                 Receiver* _rx,
                                 wns::logger::Logger* _logger):
    ICalculationStrategy(_config, _rx, _logger),
    logger(_logger),
    rx(_rx)
{

}

std::vector<wns::Ratio>
NoCorrelationZF::getPostProcessingSINRFactor(rise::TransmissionObjectPtr t)
{
    int n_SS = t->getNumberOfSpatialStreams();
    int n_RX = rx->getOFDMAStation()->getNumAntennas();

    if(n_SS > n_RX)
    {
        // More streams than receive antennas -> no usefull reception possible
        return std::vector<wns::Ratio>(n_SS, wns::Ratio::from_factor(1e-12));
    }

    MESSAGE_SINGLE(NORMAL, *logger, "postProcessingSINR with n_SS " << n_SS << " and n_Rx " << n_RX << "-> " << double(n_RX - n_SS + 1)/double(n_SS));

    return std::vector<wns::Ratio>(n_SS,
                                   wns::Ratio::from_factor(double(n_RX - n_SS + 1)/double(n_SS)));
}



