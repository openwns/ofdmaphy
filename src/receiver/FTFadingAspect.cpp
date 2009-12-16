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

#include <OFDMAPHY/receiver/FTFadingAspect.hpp>

//#include <RISE/scenario/Scenario.hpp>

using namespace ofdmaphy::receiver;

// constructor
FTFadingAspect::FTFadingAspect(const wns::pyconfig::View& config) :
    //ReceiverBase(config),
    ftfading(NULL),
    active(false),
    samplingTime(0.0)
{
    // somehow ReceiverBase(config) does not initialize the logger
    if (logger.getModuleName().compare("unspecified")==0)
        logger=wns::logger::Logger(config.getView("logger")); // workaround.

    assure(config.knows("FTFadingStrategy"),"FTFadingStrategy unknown");
    if (!config.isNone("FTFadingStrategy"))
    {
        wns::pyconfig::View ftfadingView = config.get("FTFadingStrategy");

        assure(ftfadingView.knows("ftfadingName"),"ftfadingName unknown");
        if (!ftfadingView.isNone("ftfadingName"))
        {
            std::string ftfadingName = ftfadingView.get<std::string>("ftfadingName");
            assure(ftfadingName.length()>0,"invalid ftfadingName");
            assure(logger.getModuleName().compare("unspecified")!=0,"logger problem");

            MESSAGE_SINGLE(NORMAL, logger, "Establishing ftfading strategy "<<ftfadingName);

            /** @brief create FTFading object from factory by name */
            rise::scenario::ftfading::FTFading::FTFadingCreator* FTFadingCreator =
                rise::scenario::ftfading::FTFading::FTFadingFactory::creator(ftfadingName);

            ftfading = FTFadingCreator->create(ftfadingView);
            assure(ftfading!=NULL,"Error getting ftfading");

            samplingTime = ftfading->getSamplingTime();
            assure(samplingTime>=0.0,"samplingTime must be positive: t="<<samplingTime);

            // samplingTime of all ftfading processes should be the same
            if (samplingTime > 0.0)
            {
                MESSAGE_SINGLE(NORMAL, logger, "Retrieved ftfading (strategy "<<ftfadingName<<") from Broker.");
                active=true;
            }
            else
            {
                // effectively fading is off
                MESSAGE_SINGLE(NORMAL, logger, "Retrieved ftfading (strategy "<<ftfadingName<<") from Broker. Switching off.");
                delete ftfading; ftfading=NULL;
                active=false;
            }
        }
    } // else ftfading = NULL;
}

// destructor
FTFadingAspect::~FTFadingAspect()
{
}

bool FTFadingAspect::FTFadingIsActive() const
{
    return active && (ftfading!=NULL);
}

wns::Ratio FTFadingAspect::getFTFading(int _subCarrier)
{
    assure(ftfading,"no FTFading object");
    assure(_subCarrier>=0,"subCarrier must be positive");

    // positive value = constructive; negative value = destructive
    return ftfading->getFTFading(_subCarrier);
}

wns::Ratio FTFadingAspect::getFTFading(wns::node::Interface* source, int _subCarrier)
{
    assure(ftfading!=NULL,"no FTFading object");
    assure(_subCarrier>=0,"subCarrier must be positive");
    assure(source!=NULL,"source==NULL");

    //PerSourceContainer myPerSourceContainer = perSourceMap[source];
    PerSourceMap::iterator perSourceMapFound = perSourceMap.find(source);
    assure(perSourceMapFound!= perSourceMap.end(),"cannot find source in perSourceMap");

    PerSourceContainer& myPerSourceContainer = (*perSourceMapFound).second;

    rise::scenario::ftfading::FTFading* myFTFading = myPerSourceContainer.ftfading;

    if (myFTFading==NULL)
    {
        // create on demand
        // copy the single fading object for all sources
        myFTFading = ftfading;
        // TODO: make individual
        myPerSourceContainer.ftfading = myFTFading;
        MESSAGE_SINGLE(NORMAL, logger, "FTFadingAspect::getFTFading("<<source->getName()<<","<<_subCarrier<<"): new ftFading");
    }

    assure(myFTFading!=NULL,"myFTFading==NULL");

    // positive value = constructive; negative value = destructive
    return myFTFading->getFTFading(_subCarrier);
}

