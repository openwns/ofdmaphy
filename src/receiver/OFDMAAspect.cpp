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

#include <OFDMAPHY/receiver/OFDMAAspect.hpp>

#include <RISE/medium/Medium.hpp>
#include <RISE/medium/PhysicalResource.hpp>

using namespace ofdmaphy::receiver;

OFDMAAspect::OFDMAAspect(wns::Ratio rnf) :
    physicalResources(),
    receiverNoiseFigure(rnf),
    currentNumberOfSubCarriers(0),
    carrierBandwidth(0.0),
    lowestFrequency(0.0),
    firstCarrierMidFrequency(0.0)
{
}

void
OFDMAAspect::tune(double f, double b, int numberOfSubCarriers)
{
    while(!physicalResources.empty())
    {
        (*physicalResources.begin())->detach(this);
        physicalResources.erase(physicalResources.begin());
    }

    assure(physicalResources.empty(), "Tuning not possible. Receiver is already tuned!");

    currentNumberOfSubCarriers = numberOfSubCarriers;
    carrierBandwidth = b/numberOfSubCarriers;
    lowestFrequency = f - b/2;
    firstCarrierMidFrequency = lowestFrequency + carrierBandwidth/2;

    // allocate physicalResources for all subCarriers:
    for(int32_t i=0; i<numberOfSubCarriers; ++i)
    {
        double midFrequency = firstCarrierMidFrequency + i*carrierBandwidth;
        rise::medium::Medium* m = rise::medium::Medium::getInstance();
        rise::medium::PhysicalResource* p = m->getPhysicalResource(midFrequency, carrierBandwidth);
        physicalResources.push_back(p);
        p->attach(this);
    }
}

wns::Power
OFDMAAspect::getNoise(int subCarrier) const
{
    assure(subCarrier >= 0, "No negative numbers for subCarrier allowed");
    assure(subCarrier < currentNumberOfSubCarriers, "No such subCarrier! Too high!");
    assure(subCarrier < (int)physicalResources.size(), "No such subCarrier! Too high!");

    return getNoisePerSubChannel(); // faster
}

wns::Power
OFDMAAspect::getNoisePerSubChannel() const
{
    //thermal noise: -174 dBm at room temperature (290 K) in a 1 Hz bandwidth (BW)
    wns::Power noise = wns::Power::from_dBm(-174);
    noise += wns::Ratio::from_factor(carrierBandwidth*1E6);
    //receiver noise figure: degradation of received signal quality due to imperfections
    noise += receiverNoiseFigure;
    return noise;
}

wns::Ratio
OFDMAAspect::getNoiseFigure() const
{
    return receiverNoiseFigure;
}

int
OFDMAAspect::getSubCarrierIndex(double f)
{
    int i = (int)((f-lowestFrequency)/carrierBandwidth);
    assure(i>=0,"getSubCarrierIndex("<<f<<") = "<<i<<" is negative");
    return i;
}
