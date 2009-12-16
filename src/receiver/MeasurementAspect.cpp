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

#include <OFDMAPHY/receiver/MeasurementAspect.hpp>
#include <OFDMAPHY/Station.hpp>

#include <RISE/transceiver/transmitter.hpp>

using namespace ofdmaphy::receiver;

// constructor
MeasurementAspect::MeasurementAspect(const wns::pyconfig::View& config) :
    doMeasurementUpdates(config.get<bool>("doMeasurementUpdates")),
    measurementUpdateInterval(0.0),
    measurementUpdateOffset(0.0)
{
    // somehow ReceiverBase(config) does not initialize the logger
    if (logger.getModuleName().compare("unspecified")==0)
        logger=wns::logger::Logger(config.getView("logger")); // workaround.

    if (doMeasurementUpdates)
    {
        // can also be done if ftfading == NULL
        if (!config.isNone("measurementUpdateInterval")) 
        {
            measurementUpdateInterval = config.get<simTimeType>("measurementUpdateInterval");

            if (measurementUpdateInterval==0.0) 
            {
                doMeasurementUpdates=false;
            }
            measurementUpdateOffset = config.get<simTimeType>("measurementUpdateOffset");
        }
        else
        {
            assure(false,"no measurementUpdateInterval in config");
        }
    }
    else
    {
        measurementUpdateInterval=0.0;
    }
}

// destructor
MeasurementAspect::~MeasurementAspect()
{
}

void
MeasurementAspect::startRegularMeasurementUpdates()
{
    // trigger event scheduler
    assure (measurementUpdateInterval>0.0,
            "wrong measurementUpdateInterval");
    assure (measurementUpdateInterval+measurementUpdateOffset>=0.0,
            "wrong measurementUpdateOffset");

    // the first measurement comes at this absolute time
    setTimeout(measurementUpdateOffset+measurementUpdateInterval);
}

void
MeasurementAspect::onTimeout()
{
    doMeasurementsNow();
    setTimeout(measurementUpdateInterval); // schedule next event
}

void
MeasurementAspect::registerSource(wns::node::Interface* source)
{
    if (not doMeasurementUpdates)
    {
        return;
    }

    assure(source!=NULL,"source==NULL");

    int numberOfSubChannels = getCurrentNumberOfSubCarriers();
    PerSourceMap::iterator perSourceMapFound = perSourceMap.find(source);

    if (perSourceMapFound == perSourceMap.end())
    {
        // not found
        MESSAGE_SINGLE(NORMAL, logger, "registerSource("<<source->getName()<<") successful");

        PerSourceContainer myPerSourceContainer;
        myPerSourceContainer.packetCount = 0;
        myPerSourceContainer.ftfading = NULL;
        myPerSourceContainer.quasiStaticPathLoss = wns::Ratio::from_dB(0.0);
        wns::Power interferenceTemplate = getNoisePerSubChannel();
        myPerSourceContainer.interferenceVector = std::vector<wns::Power>(numberOfSubChannels,interferenceTemplate);
        perSourceMap[source] = myPerSourceContainer; // insert (copy)
    }
}

wns::node::Interface*
MeasurementAspect::registerSource(rise::TransmissionObjectPtr t)
{
    rise::Transmitter* transmitter = t->getTransmitter();
    assure(transmitter!=NULL,"transmitter==NULL");

    Station* sourceStation = dynamic_cast<Station*>(transmitter->getStation());
    assure(sourceStation!=NULL,"sourceStation==NULL");

    wns::node::Interface* sourceNode = sourceStation->getNode();
    registerSource(sourceNode);
    return sourceNode;
}

PerSourceContainer&
MeasurementAspect::getPerSourceContainer(wns::node::Interface* source)
{
    assure(source!=NULL,"source==NULL");

    PerSourceMap::iterator perSourceMapFound = perSourceMap.find(source);
    assure(perSourceMapFound!= perSourceMap.end(),
           "cannot find source "<<source->getName()<<" in perSourceMap");

    PerSourceContainer& myPerSourceContainer = (*perSourceMapFound).second; // no copy, just reference
    return myPerSourceContainer;
}

void
MeasurementAspect::saveMeasuredFlatPathloss(wns::node::Interface* source, wns::Ratio pathloss)
{
    assure(source!=NULL,"source==NULL");

    PerSourceMap::iterator perSourceMapFound = perSourceMap.find(source);

    if (perSourceMapFound!= perSourceMap.end())
    {
        PerSourceContainer& myPerSourceContainer = (*perSourceMapFound).second;
        myPerSourceContainer.quasiStaticPathLoss = pathloss;
        myPerSourceContainer.packetCount++;
        MESSAGE_SINGLE(NORMAL, logger, "saveMeasuredFlatPathloss("<<source->getName()<<","<<pathloss<<"): count="<<myPerSourceContainer.packetCount);
    }
    else
    {
        MESSAGE_SINGLE(NORMAL, logger, "saveMeasuredFlatPathloss("<<source->getName()<<","<<pathloss<<"): unregistered source");
    }
}
