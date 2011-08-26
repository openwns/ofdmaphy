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

#ifndef OFDMAPHY_RECEIVER_OFDMAASPECT_HPP
#define OFDMAPHY_RECEIVER_OFDMAASPECT_HPP

#include <OFDMAPHY/receiver/ReceiverBase.hpp>

#include <RISE/receiver/MultiCarrier.hpp>
#include <RISE/receiver/ReceiverInterface.hpp>

#include <WNS/PowerRatio.hpp>

#include <vector>

namespace rise { namespace medium {
        class PhysicalResource;
    } // medium
} // rise

namespace ofdmaphy { namespace receiver {
    /**
	 * @brief implements the OFDMA Aspect of a MultiCarrier receiver
	 */
    class OFDMAAspect :
        virtual public ReceiverBase,
        virtual public rise::receiver::ReceiverInterface,
        virtual public rise::receiver::MultiCarrierAspect
    {
        typedef std::vector<rise::medium::PhysicalResource*> PhysicalResourceContainer;

    public:
        OFDMAAspect(wns::Ratio rnf);

        /** @brief allocate physicalResources for all OFDMA subCarriers */
        virtual void
        tune(double f, double b, int numberOfSubCarriers);

        /** @brief get noise power on any subCarrier (-174dBm/Hz*BW+receiverNoiseFigure) */
        virtual wns::Power
        getNoisePerSubChannel() const;

        /** @brief get noise power on specific subCarrier (-174dBm/Hz*BW+receiverNoiseFigure) */
        virtual wns::Power
        getNoise(int subCarrier) const;

        /** @brief get receiverNoiseFigure: degradation of received signal quality due to imperfections */
        wns::Ratio
        getNoiseFigure() const;

        /** @brief translate frequency to subcarrier */
        virtual int
        getSubCarrierIndex(double f);

    protected:
        /** @brief for each subCarrier there is a physicalResource in this container */
        PhysicalResourceContainer physicalResources;

        /** @brief number of subCarriers of this OFDMA system */
        int
        getCurrentNumberOfSubCarriers() const
            { return currentNumberOfSubCarriers; }

    private:
        /** @brief receiverNoiseFigure is set by the constructor */
        wns::Ratio receiverNoiseFigure;

        /** @brief number of OFDMA subcarriers */
        int currentNumberOfSubCarriers;

        /** @brief bandwidth of one subcarrier */
        double carrierBandwidth;
        double lowestFrequency;
        double firstCarrierMidFrequency;
    };

}
}

#endif // not defined __RISE_OFDMA_HPP



