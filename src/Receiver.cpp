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

#include <OFDMAPHY/Receiver.hpp>
#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/Sender.hpp>

#include <RISE/medium/Medium.hpp>
#include <RISE/medium/PhysicalResource.hpp>
#include <RISE/scenario/Scenario.hpp>
#include <RISE/manager/systemmanager.hpp>
#include <RISE/antenna/Antenna.hpp>
#include <RISE/transceiver/transmitter.hpp>
#include <RISE/transceiver/cache/idvectorcache.hpp>

#include <WNS/probe/bus/ProbeBus.hpp>
#include <WNS/probe/bus/ProbeBusRegistry.hpp>
#include <WNS/probe/bus/ContextCollector.hpp>

#include <valarray>

using namespace ofdmaphy;

ReceiverBase::ReceiverBase()
{
}

ReceiverBase::ReceiverBase(const wns::pyconfig::View& config) :
    logger(config.getView("logger"))
    //nSectors(config.get<int>("nSectors"))
{
    assure(logger.getModuleName().compare("unspecified")!=0,"logger problem");
    //std::cout << "ReceiverBase.logger: "<<logger.getModuleName()<<"."<<logger.getLoggerName()<<std::endl;
}

ReceiverBase::~ReceiverBase()
{
}

OFDMAAspect::OFDMAAspect(wns::Ratio rnf) :
    physicalResources(),
    receiverNoiseFigure(rnf),
    currentNumberOfSubCarriers(0),
    carrierBandwidth(0.0),
    lowestFrequency(0.0),
    firstCarrierMidFrequency(0.0)
{
}

void OFDMAAspect::tune(double f, double b, int numberOfSubCarriers)
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

wns::Power OFDMAAspect::getNoise(int subCarrier) const
{
    assure(subCarrier >= 0, "No negative numbers for subCarrier allowed");
    assure(subCarrier < currentNumberOfSubCarriers, "No such subCarrier! Too high!");
    assure(subCarrier < (int)physicalResources.size(), "No such subCarrier! Too high!");

    return getNoisePerSubChannel(); // faster
}

wns::Power OFDMAAspect::getNoisePerSubChannel() const
{
    //thermal noise: -174 dBm at room temperature (290 K) in a 1 Hz bandwidth (BW)
    wns::Power noise = wns::Power::from_dBm(-174);
    noise += wns::Ratio::from_factor(carrierBandwidth*1E6);
    //receiver noise figure: degradation of received signal quality due to imperfections
    noise += receiverNoiseFigure;
    return noise;
}

// receiver noise figure: degradation of received signal quality due to imperfections
wns::Ratio OFDMAAspect::getNoiseFigure() const
{
    return receiverNoiseFigure;
}

/* method to get a specific subCarrier for getting its FTFading (required for FTFading) */
int
OFDMAAspect::getSubCarrierIndex(double f)
{
    int i = (int)((f-lowestFrequency)/carrierBandwidth);
    assure(i>=0,"getSubCarrierIndex("<<f<<") = "<<i<<" is negative");
    return i;
}

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
    assure (measurementUpdateInterval>0.0, "wrong measurementUpdateInterval");
    assure (measurementUpdateInterval+measurementUpdateOffset>=0.0, "wrong measurementUpdateOffset");

    //MESSAGE_SINGLE(NORMAL, logger, "startRegularMeasurementUpdates: interval="<<measurementUpdateInterval<<"s");
    setTimeout(measurementUpdateOffset+measurementUpdateInterval); // the first measurement comes at this absolute time
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
    if (!doMeasurementUpdates) 
        return;

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

/* simplified interface if only TransmissionObjectPtr is known */
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
    assure(perSourceMapFound!= perSourceMap.end(),"cannot find source "<<source->getName()<<" in perSourceMap");

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

/****************************************************************************************/

Receiver::Receiver(const wns::pyconfig::View& config, rise::Station* s) :
    ReceiverBase(config),
    OFDMAAspect(config.get<wns::Ratio>("receiverNoiseFigure")),
    FTFadingAspect(config),
    MeasurementAspect(config),
    LossCalculation(config),
    station(s),
    propagationCache(new rise::IdVectorCache(this)),
    wraparoundShiftVectors(NULL),
    nSectors(config.get<int>("nSectors"))
{
    this->startObserving(s);

    if (measurementUpdatesAreOn())
    {
        // can also be done if ftfading == NULL
        if ((getMeasurementUpdateInterval()==0.0) && (ftfading))
        {
            setMeasurementUpdateInterval(ftfading->getSamplingTime());
        }
        else if (FTFadingIsActive())
        {
            assure(getMeasurementUpdateInterval() == ftfading->getSamplingTime(),
                   "measurementUpdateInterval="<<getMeasurementUpdateInterval()<<" != ftfading::samplingTime="<<ftfading->getSamplingTime());
        }
        else
        {

        }

        assure(getMeasurementUpdateInterval()>0.0,
               "doMeasurementUpdates=True but measurementUpdateInterval=0");

        MESSAGE_SINGLE(NORMAL, logger, "doMeasurementUpdates in intervals of "<<getMeasurementUpdateInterval()<<"s, offset="<<getMeasurementUpdateOffset());
        startRegularMeasurementUpdates();
    }

    // handle wraparaoundShifts:
    wraparoundShiftVectors = s->getSystemManager()->getWraparoundShiftVectors();

    // Is wraparound switched on? Answer is: doWraparound = (wraparoundShiftVectors->size()>0) ? true:false;
    MESSAGE_SINGLE(NORMAL, logger, "wraparound: using "<<wraparoundShiftVectors->size()<<" ShiftVectors");

    // use the wraparoundShiftVectors like this:
    int shiftListLength = wraparoundShiftVectors->size();
    for(int i=0; i<shiftListLength; i++)
    {
        wns::geometry::Vector v = (*wraparoundShiftVectors)[i];
        const std::valarray<double> va = v.get();
        MESSAGE_SINGLE(NORMAL, logger, "wraparoundShiftVectors["<<i<<"]=("<<va[0]<<","<<va[1]<<")");
    }
}

Receiver::~Receiver()
{
    delete propagationCache;
    activeTransmissions.clear();
    // cleanup FTfading object:
    if (ftfading)
    {
        delete ftfading;
        ftfading=NULL;
    }
}

// this overloads a method from rise::Receiver
wns::Power Receiver::getRxPower(const rise::TransmissionObjectPtr& t)
{
    wns::Power rxPower = getRxPower(t, getCurrentReceivePattern(t));
    MESSAGE_BEGIN(VERBOSE, logger, m , "getRxPower");
    m  << "(from: " << dynamic_cast<Station*>(t->getTransmitter()->getStation())->getNode()->getName() << ") = "<< rxPower;
    MESSAGE_END();
    return rxPower;
}

wns::Ratio Receiver::getQuasiStaticPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern)
{
    double frequency = t->getPhysicalResource()->getFrequency();
    rise::Transmitter* transmitter = t->getTransmitter();
    assure(transmitter!=NULL,"transmitter==NULL");
    //Station* transmitterOFDMAStation = dynamic_cast<Station*>(transmitter->getStation());
    //assure(transmitterOFDMAStation != NULL,"Transmitter must have an OFDMAStation"); // new [rs] 24.10.2007

    wns::Ratio transmittersAntennaGain = t->getTransmittersAntennaGain(getStation()->getPosition());
    // Idea: wns::Ratio transmittersAntennaGain = t->getTransmittersAntennaGain(getStation()->getPosition()->shift(x,y));
    // Idea: wns::Ratio transmittersAntennaGain = t->getTransmittersAntennaGain(getStation()->getPosition()+wns::PositionOffset(x,y,0));
    // Idea: instead of (x,y) we could use wns::PositionOffset()
    MESSAGE_SINGLE(VERBOSE,logger, "  TransmittersAntennaGain: " << transmittersAntennaGain);

    // pure static path loss (from propagationCache):
    wns::Ratio purePathLoss = getLoss(transmitter, frequency);
    MESSAGE_SINGLE(VERBOSE,logger, "  PurePathLoss: " << purePathLoss);

    wns::Ratio receiverAntennaGain;
    if (pattern == wns::service::phy::ofdma::PatternPtr()) 
    {
        // no pattern = no beamforming
        // static antenna
        receiverAntennaGain =  getStation()->getAntenna()->getGain(transmitter->getAntenna()->getPosition(), wns::service::phy::ofdma::PatternPtr());
        MESSAGE_SINGLE(VERBOSE,logger, "  ReceiverAntennaGain of Static Pattern: " << receiverAntennaGain);
    }
    else
    {
        // beamforming on
        //assure(dynamic_cast<Station*>(getStation()), "station is not an OFDMA station (and may have no beamforming antenna)");
        //receiverAntennaGain = dynamic_cast<Station*>(getStation())->getBFAntenna()->getGain(t->getTransmitter()->getAntenna()->getPosition(), pattern);
        Station* receiverOFDMAStation = dynamic_cast<Station*>(getStation());
        assure(receiverOFDMAStation!=NULL, "station is not an OFDMA station (and may have no beamforming antenna)");
        receiverAntennaGain = receiverOFDMAStation->getBFAntenna()->getGain(transmitter->getAntenna()->getPosition(), pattern);
        MESSAGE_SINGLE(VERBOSE,logger, "  ReceiverAntennaGain of Beamformed Pattern: " << receiverAntennaGain);
    }
    // TODO: receiverOFDMAStation->getBFAntenna()->getGain() should deliver the same as
    // getStation()->getAntenna()->getGain()
    // try to simplify this please
    // assure(receiverAntennaGain1 == receiverAntennaGain2,"error")
    wns::Ratio pathLoss = purePathLoss - transmittersAntennaGain - receiverAntennaGain;
    // todo: store it in a cache?
    MESSAGE_SINGLE(VERBOSE,logger, "  getQuasiStaticPathLoss() = " << pathLoss);
    return pathLoss;
}

wns::Ratio Receiver::getFullPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern)
{
    wns::Ratio pathLoss = getQuasiStaticPathLoss(t,pattern);
    if (FTFadingIsActive())
    {
        double frequency = t->getPhysicalResource()->getFrequency();
        int subChannelIndex = getSubCarrierIndex(frequency);
        // ^ can we store this somewhere? Used later for PowerMeasurement
        //wns::Ratio ftFadingGain = getFTFading(subChannelIndex);

        Station* sourceStation = dynamic_cast<Station*>(t->getTransmitter()->getStation());
        assure(sourceStation!=NULL,"sourceStation==NULL");

        wns::node::Interface* sourceNode = sourceStation->getNode();
        assure(sourceNode!=NULL,"sourceNode==NULL");

        wns::Ratio ftFadingGain = getFTFading(sourceNode,subChannelIndex);
        MESSAGE_SINGLE(VERBOSE,logger, "  FTFading: " << ftFadingGain);
        pathLoss -= ftFadingGain;
    }
    MESSAGE_SINGLE(VERBOSE,logger, "  getFullPathLoss() = " << pathLoss);
    return pathLoss;
}

wns::Power Receiver::getRxPower(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern)
{
    assure(t, "no existing transmission object");
    MESSAGE_BEGIN(VERBOSE, logger, m, "Receiver::getRxPower(User ");
    std::string userName; // only needed if VERBOSE
    Station* OFDMAStation = dynamic_cast<Station*>(t->getTransmitter()->getStation());
    if (OFDMAStation == NULL)
        userName = "Unknown";
    else
        userName = OFDMAStation->getNode()->getName();
    m << userName <<"): ";
    MESSAGE_END();

    wns::Power rxPower = t->getTxPower();
    MESSAGE_SINGLE(NORMAL,logger, "TxPower: " << rxPower);

    wns::Ratio fullPathLoss = getFullPathLoss(t,pattern);
    MESSAGE_SINGLE(NORMAL, logger, "PathLoss FUll (inside getRxPower) " << fullPathLoss );

    rxPower -= fullPathLoss;
    MESSAGE_SINGLE(NORMAL,logger, "RxPower: " << rxPower);
    return rxPower;
}

// used for ofdmaStation->getBFAntenna()->setPowerReceivedForStation(transmitterStation, getUnfilteredRxPower(t));
wns::Power Receiver::getUnfilteredRxPower(const rise::TransmissionObjectPtr& t)
{
    assure(t, "no existing transmission object");

    wns::Power rxPower = t->getTxPower();

    rxPower -= getLoss(t->getTransmitter(), t->getPhysicalResource()->getFrequency());
    rxPower += t->getTransmittersAntennaGain(getStation()->getPosition());

    if (ftfading)
    {
        int subBandIndex = getSubCarrierIndex(t->getPhysicalResource()->getFrequency());
        rxPower += getFTFading(subBandIndex); // TODO: adapt
    }

    MESSAGE_BEGIN(VERBOSE, logger, m , "Unfiltered RxPower: ");
    m << rxPower << " (i.e. without receiver Antenna Gain)";
    MESSAGE_END();

    return rxPower;
}

wns::Power Receiver::getAllRxPower(const int subCarrier)
{
    wns::Power rxPower;
    assure(subCarrier >= 0, "Cannot measure RxPower of a subCarrier < 0\n");
    assure(subCarrier <= getCurrentNumberOfSubCarriers(), "subCarrier is too large\n");

    // Add noise
    rxPower += getNoise(subCarrier);

    // Add all current transmissions
    rise::medium::PhysicalResource::TransmissionObjectIterator itr;
    for (itr=physicalResources[subCarrier]->getTOBegin(); itr!=physicalResources[subCarrier]->getTOEnd(); ++itr)
    {
        rxPower += getRxPower(*itr);
    }

    return rxPower;
}


wns::Power Receiver::getAllRxPower()
{
    wns::Power rxPower;
    assure(getCurrentNumberOfSubCarriers() > 0, "Cannot measure RxPower without any subCarrier.\n");

    // foreach subcarrier...
    for(int SubCarrierIndex=0; SubCarrierIndex<getCurrentNumberOfSubCarriers(); ++SubCarrierIndex)
    {
        // ...add noise and transmissions
        rxPower += getAllRxPower(SubCarrierIndex);
    }
    return rxPower;
}


wns::Power Receiver::getInterference(const rise::TransmissionObjectPtr& t)
{
    // Init with thermal noise plus receiver noise figure
    MESSAGE_SINGLE(NORMAL, logger, "---------------- Starting Interference Calculation -----------------");
    wns::Power interference = getNoisePerSubChannel();
    MESSAGE_SINGLE(NORMAL, logger, "Noise: " << interference );
    MESSAGE_SINGLE(NORMAL, logger, "nSectors: " << nSectors );

    rise::medium::PhysicalResource::TransmissionObjectIterator itr;
    wns::service::phy::ofdma::PatternPtr currentPattern = getCurrentReceivePattern(t);

    // Sum up the power of all active Transmissions itr on this PhysicalResource
    for(itr=t->getPhysicalResource()->getTOBegin(); itr!=t->getPhysicalResource()->getTOEnd(); ++itr)
    {
        if (*itr == t)
        {
            // own transmission
            //MESSAGE_SINGLE(VERBOSE, logger,
            //"ofdmaphy::Receiver::getInterference(): own transmission");

            // exclude from addition here, as getRxPower() might be costly.
        }
        else
        {
            if (nSectors == 1)
            {
                interference += getRxPower(*itr, currentPattern);
            }
            else if (nSectors == 3)
            {
                wns::Position interfering = (*itr)->getTransmitter()->getAntenna()->getPosition();
                Station* ofdmaStation = getOFDMAStation();

                MESSAGE_SINGLE(NORMAL, logger, ofdmaStation->getNode()->getName());

                if( (ofdmaStation->getNode()->getName().at(0)) == 'A' )
                {
                    MESSAGE_SINGLE(NORMAL, logger, "AP is the receiver" );
                    wns::Position BSpos = ofdmaStation->getBFAntenna()->getPosition();

                    double phi = ((interfering - BSpos).getAzimuth()*180)/M_PI;

                    MESSAGE_BEGIN(NORMAL, logger, m, "Angle between ");
                    m << (*itr)->getTransmitter()->getStation()->getStationId() + 1;
                    m << " and " << ofdmaStation->getNode()->getNodeID() - 1;
                    m << " is: " << phi;
                    MESSAGE_END();

                    if(phi <= 90 && phi >= -30) //for three sectors
                    {
                        MESSAGE_BEGIN(NORMAL, logger, m, "Interference due to ");
                        m << (*itr)->getTransmitter()->getStation()->getStationId() + 1;
                        m << " in the signal of station ";
                        m << t->getTransmitter()->getStation()->getStationId() + 1;
                        m << " at the receiver station "<< ofdmaStation->getNode()->getNodeID() - 1;
                        m <<" : "<< getRxPower(*itr, currentPattern);
                        MESSAGE_END();

                        interference += getRxPower(*itr, currentPattern);
                    }
                }
                else
                {
                    MESSAGE_SINGLE(NORMAL, logger, "UT is the receiver" );
                    wns::Position SSpos = ofdmaStation->getBFAntenna()->getPosition();

                    double phi = ((SSpos-interfering).getAzimuth()*180)/M_PI;

                    MESSAGE_BEGIN(NORMAL, logger, m, "Angle between ");
                    m << ofdmaStation->getNode()->getNodeID() - 1;
                    m << " and " << (*itr)->getTransmitter()->getStation()->getStationId() + 1;
                    m << " is: " << phi;
                    MESSAGE_END();

                    if(phi <= 90 && phi >= -30)//for three sectors
                    {
                        interference += getRxPower(*itr, currentPattern);

                        MESSAGE_BEGIN(NORMAL, logger, m, "Interference due to ");
                        m << (*itr)->getTransmitter()->getStation()->getStationId() + 1;
                        m << " inside the signal of station ";
                        m << t->getTransmitter()->getStation()->getStationId() + 1;
                        m << " at the receiver station ";
                        m << ofdmaStation->getNode()->getNodeID() -1;
                        m <<" : "<< getRxPower(*itr, currentPattern);
                        MESSAGE_END();
                    }
                }
            } // for 3 sectors
            else
            {
                interference += getRxPower(*itr, currentPattern);
            }
            MESSAGE_BEGIN(VERBOSE, logger, m, "");
            m << "RxPower from StationId ";
            m << (*itr)->getTransmitter()->getStation()->getStationId();
            m << " = " << getRxPower(*itr, currentPattern);
            MESSAGE_END();
        }
    }

    MESSAGE_SINGLE(VERBOSE, logger, "------- Finished Interference Calculation, Result: " << interference << " -----------------");
    return interference;
}

void
Receiver::insertReceivePattern(wns::node::Interface* node, wns::service::phy::ofdma::PatternPtr pattern)
{
    currentReceivePatterns[node] = pattern;
}

void
Receiver::removeReceivePattern(wns::node::Interface* node)
{
    currentReceivePatterns.erase(node);
}

void
Receiver::setCurrentReceivePatterns(std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> _currentReceivePatterns)
{
    // old patterns are no longer valid
    currentReceivePatterns = _currentReceivePatterns;
}

wns::service::phy::ofdma::PatternPtr
Receiver::getCurrentReceivePattern(const rise::TransmissionObjectPtr& t) const
{
    // station must not be an ofdmaphy station, but it must be a node provider
    {
        Station* tmpStation = dynamic_cast<ofdmaphy::Station*>(t->getTransmitter()->getStation());
        if (tmpStation)
            return getCurrentReceivePattern(tmpStation->getNode());
    }

    {
        Sender* tmpStation = dynamic_cast<ofdmaphy::Sender*>(t->getTransmitter()->getStation());
        if (tmpStation)
            return getCurrentReceivePattern(tmpStation->getMyNode());
    }

    throw wns::Exception("Unable to determine station type of sender");
    //return wns::service::phy::ofdma::PatternPtr();
}

wns::service::phy::ofdma::PatternPtr
Receiver::getCurrentReceivePattern(wns::node::Interface* pStack) const
{
    MESSAGE_SINGLE(VERBOSE, logger, "Searching ReceivePattern for node: " << pStack->getName());
    std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr>::const_iterator itr;
    itr = currentReceivePatterns.find(pStack);

    if( itr == currentReceivePatterns.end()) {
        // omni-directional reception performed with static antenna
        return  wns::service::phy::ofdma::PatternPtr();
    }
    else
    {
        // pattern pointed to the transmitter
        MESSAGE_SINGLE(VERBOSE, logger, "Found ReceivePattern for node: " << pStack->getName());
        return itr->second;
    }
}

void Receiver::writeCacheEntry(rise::PropCacheEntry& cacheEntry, rise::Transmitter* t, double freq)
{
    //	rise::Station& rxStation = *getStation();
    //	rise::antenna::Antenna& rxAntenna = *rxStation.getAntenna();
    //	rise::antenna::Antenna& txAntenna = *t->getAntenna();

    wns::Ratio pl = getPathloss(*t, freq);
    wns::Ratio sh = getShadowing(*t);

    cacheEntry.setPathloss(pl);
    cacheEntry.setShadowing(sh);

    // receive antenna gain at transmitter's position
    //wns::Ratio r1 = rxAntenna.getGain(txAntenna.getPosition(), NULL);
    // transmit antenna gain at receiver's position
    //wns::Ratio r2 = txAntenna.getGain(rxAntenna.getPosition(), NULL);
    cacheEntry.setAntennaGain(wns::Ratio::from_dB(0));
    cacheEntry.setValid(true);
}

/** @brief PathlossCalculationInterface (from propagation cache) */
wns::Ratio Receiver::getLoss(rise::Transmitter* t, double f)
{
    MESSAGE_BEGIN(VERBOSE, logger, m , "  PathLossFromPropagationCache: ");
    m <<  propagationCache->getLoss(t, f);
    MESSAGE_END();

    return propagationCache->getLoss(t, f);
}

void Receiver::positionChanged()
{
    propagationCache->invalidatePropagationEntries();
}

void Receiver::positionWillChange()
{
    signalLevelsChange();
}

// triggered by PhysicalResourceObserver::notify in RISE/PhysicalResource.cpp
void Receiver::notify(rise::TransmissionObjectPtr t)
{
    if (transmissionForMe(t))
    {
        //Station* ofdmaStation = getOFDMAStation();
        //wns::node::Interface* sourceNode = ofdmaStation->getNode();
        //registerSource(sourceNode);
        wns::node::Interface* sourceNode = registerSource(t); // for OFDMA
                                                              // measurements

        if (t->getIsStart())
        {
            // start of transmission
            MESSAGE_BEGIN(NORMAL, logger, m, "ofdmaphy::Receiver::notify(): startOfTransmission");

            if (Station* ofdmaStation = dynamic_cast<Station*>(t->getTransmitter()->getStation())) 
            {
                m << " from " << ofdmaStation->getNode()->getName();
            }
            MESSAGE_END();

            add(t);
            activeTransmissions.push_back(t);
        }
        else
        {
            // end of transmission
            if (find(activeTransmissions.begin(), activeTransmissions.end(), t) != activeTransmissions.end())
            {
                MESSAGE_BEGIN(NORMAL, logger, m, "ofdmaphy::Receiver::notify(): endOfTransmission");
                if (Station* ofdmaStation = dynamic_cast<Station*>(t->getTransmitter()->getStation()))
                {
                    m << " from " << ofdmaStation->getNode()->getName();
                }
                MESSAGE_END();

                endOfTransmission(t);
                // ^ inherited from TimeWeightedTransmissionAveraging::endOfTransmission
                // inherited from rise::Receiver, from rise::ReceiverInterface::TransmissionAveragingStrategy

                wns::Ratio omniAttenuation;
                wns::service::phy::ofdma::PatternPtr pattern = getCurrentReceivePattern(t);
                if (pattern == wns::service::phy::ofdma::PatternPtr()) // empty pattern
                    omniAttenuation = wns::Ratio::from_dB(0);
                else
                    omniAttenuation = pattern->getOmniAttenuation();

                // pass received thing upwards in the stack:
                Station* ofdmaStation = getOFDMAStation();

                //const wns::service::phy::phymode::PhyModeInterface& transmissionObject->getPhyMode();

                // obsolete [rs]:
                //ofdmaStation->receiveData(t->getPayload(), getAveragedRxPower(t), getAveragedInterference(t), omniAttenuation);
                /* [rs] new code using PowerMeasurementPtr */
                wns::Power rxPower = getAveragedRxPower(t); // inherited from rise::receiver::TimeWeightedTransmissionAveraging
                wns::Power interference = getAveragedInterference(t);
                int subChannelIndex = getSubCarrierIndex(t->getPhysicalResource()->getFrequency());

                if (doMeasurementUpdates)
                { // the periodic "big" ones
                    wns::Ratio pathLoss = getQuasiStaticPathLoss(t, getCurrentReceivePattern(t)); // determine (again)
                    saveMeasuredFlatPathloss(sourceNode,pathLoss);
                    //saveMeasuredInterference(sourceNode,subChannelIndex,getAveragedInterference(t));
                }
                //MESSAGE_SINGLE(NORMAL, logger,
                //"ofdmaphy::Receiver::receiveData(): preparing
                //PowerMeasurementPtr");

                wns::service::phy::power::PowerMeasurementPtr rxPowerMeasurementPtr =
                    wns::SmartPtr<rise::receiver::PowerMeasurement>
                    (new rise::receiver::PowerMeasurement(t,
                                                          sourceNode,
                                                          rxPower,
                                                          interference,
                                                          omniAttenuation,
                                                          subChannelIndex)
                        );
                MESSAGE_SINGLE(NORMAL, logger, "PowerMeasurement="<<*rxPowerMeasurementPtr);
                ofdmaStation->receiveData(/* sdu */t->getPayload(), rxPowerMeasurementPtr);

                // update list of receive power of stations in the BFAntenna
                // periodically. Note that we have to store unfiltered Rx Power,
                // i.e. not taking into account any antenna gains at this receiver
                if (ofdmaStation->beamformingEnabled())
                {
                    Station* transmitterStation = static_cast<Station*>(t->getTransmitter()->getStation());
                    ofdmaStation->getBFAntenna()
                        ->setPowerReceivedForStation(transmitterStation, getUnfilteredRxPower(t));
                    ofdmaStation->getBFAntenna()
                        ->setTxPowerForStation(transmitterStation, t->getTxPower());
                }

                remove(t);
                assure(find(activeTransmissions.begin(), activeTransmissions.end(),t) != activeTransmissions.end(),
                       "Mismatch in Transmission Notifications!");
                activeTransmissions.remove(t);
            } else {
                // Do nothing, we haven't heard the beginning of the
                // transmission. This can happen if we tune to a new frequency
                // and bandwidth. Then transmissions are already active on a
                // PhyiscalResource. We will be notified about their end of
                // transmission but haven't heard the start ...
            }
        }
    } // transmission was for me
    else
    {
        signalLevelsChange();
    } // transmission was not for me

    // For each started and stopped transmission, the Received Signal Strenght
    // (RSS) at the receiver changes. Upper FUs can observe the RSS to detect a
    // busy channel.
    // The signalling is made only if observers are present, as the operation
    // (adding dBm values) is costly and must be done for every packet at every receiver.
    if (this->wns::Subject<RSSInterface>::hasObservers())
    {
        // For getAllRxPower, the notify() comes always too early (see
        // rise::medium::PhysicalResource): first the receiver is notified, then
        // the transmission is added or removed. Hence, we have to add/substract the
        // new/old transmission. The mobility is not effected, as both
        // getAllRxPower() and getRxPower() return a 'snapshot' of now
        wns::Power newReceivedSignalStrength = getAllRxPower();
        if (t->getIsStart())
        {
            newReceivedSignalStrength += getRxPower(t);
        }
        else
        {
            assure(newReceivedSignalStrength > getRxPower(t), "receivedSignalStrength is too low for current ongoing transmission\n");
            newReceivedSignalStrength -= getRxPower(t);
        }

        if (newReceivedSignalStrength != receivedSignalStrength)
        {
            receivedSignalStrength = newReceivedSignalStrength;
            MESSAGE_SINGLE(NORMAL, logger, "notify: New carrier-sense of " << receivedSignalStrength);
            this->wns::Subject<RSSInterface>::forEachObserver(OnNewRSS(receivedSignalStrength));
        }
    }
} // Receiver::notify()

void
Receiver::doMeasurementsNow()
{
    int numberOfSubChannels=getCurrentNumberOfSubCarriers();
    MESSAGE_SINGLE(NORMAL,logger, "doMeasurementsNow(): "<<perSourceMap.size()<<" sources, "<<numberOfSubChannels<<" subChannels:");

    Station* ofdmaStation = getOFDMAStation();
    wns::Power interferenceTemplate = getNoisePerSubChannel(); // only Noise
    // algorithm: iterate over all (relevant) sources (all at BS, 1 at UT)

    for (PerSourceMap::iterator perSourceIterator = perSourceMap.begin();
         perSourceIterator != perSourceMap.end();
         ++perSourceIterator)
    {
        wns::node::Interface *source = perSourceIterator->first;
        assure(source!=NULL,"source==NULL");

        PerSourceContainer& perSourceContainer = perSourceIterator->second;
        wns::Ratio quasiStaticPathLoss = perSourceContainer.quasiStaticPathLoss;
        rise::scenario::ftfading::FTFading* ftfading = perSourceContainer.ftfading; // Pointer

        assure((int)perSourceContainer.interferenceVector.size()==numberOfSubChannels,
               "wrong interferenceVector.size="<<perSourceContainer.interferenceVector.size());

        // if (perSourceContainer.packetCount < 10) // irrelevant
        MESSAGE_SINGLE(NORMAL, logger, "  source="<<source->getName()<<": PL="<<quasiStaticPathLoss<<", c="<<perSourceContainer.packetCount);
        wns::service::phy::power::OFDMAMeasurementPtr ofdmaMeasurementPtr =
            //wns::SmartPtr<ofdmaphy::OFDMAMeasurement>
            ofdmaphy::OFDMAPHYMeasurementPtr
            (new ofdmaphy::OFDMAMeasurement(source, numberOfSubChannels,
                                            /*valid_for*/measurementUpdateInterval,
                                            quasiStaticPathLoss,
                                            ftfading,
                                            perSourceContainer.interferenceVector, // big copy
                                            logger));

        // send measurements
        ofdmaStation->measurementUpdate(source,ofdmaMeasurementPtr);
        // clear members for next cycle:
        //for(int subChannel=0; subChannel<numberOfSubChannels; ++subChannel)
        // perSourceContainer.interferenceVector[subChannel] = noiseOnSubChannel; // copy
        perSourceContainer.interferenceVector = std::vector<wns::Power>(numberOfSubChannels,interferenceTemplate); // new vector object
    } // forall sources
}

bool
Receiver::isReceiving() const
{
    return (!activeTransmissions.empty());
}

void Receiver::mobilityUpdate(rise::Transmitter* t)
{
    signalLevelsChange();
    propagationCache->invalidatePropagationEntries(t);
}

void Receiver::tune(double f, double b, int numberOfSubCarriers)
{
    activeTransmissions.clear();
    this->removeAll();

    MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Receiver::tune(f="<<f<<",b="<<b<<",#SC="<<numberOfSubCarriers<<")");

    OFDMAAspect::tune(f, b, numberOfSubCarriers); // inherited
    //currentNumberOfSubCarriers = numberOfSubCarriers;

    // Change of frequency means new received signal strength
    if (this->wns::Subject<RSSInterface>::hasObservers())
    {
        wns::Power newReceivedSignalStrength = getAllRxPower();
        if (newReceivedSignalStrength != receivedSignalStrength)
        {
            receivedSignalStrength = newReceivedSignalStrength;
            MESSAGE_SINGLE(NORMAL, logger, "tune: New carrier-sense of " << receivedSignalStrength);
            this->wns::Subject<RSSInterface>::forEachObserver(OnNewRSS(receivedSignalStrength));
        }
    }
}



