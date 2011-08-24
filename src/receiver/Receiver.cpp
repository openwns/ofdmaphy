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

#include <OFDMAPHY/receiver/Receiver.hpp>

#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/Sender.hpp>
#include <OFDMAPHY/OFDMAMeasurement.hpp>

#include <RISE/medium/PhysicalResource.hpp>
#include <RISE/scenario/Scenario.hpp>
#include <RISE/transceiver/transmitter.hpp>
#include <RISE/transceiver/cache/idvectorcache.hpp>
#include <RISE/receiver/PowerMeasurement.hpp>

#include <valarray>

using namespace ofdmaphy::receiver;

Receiver::Receiver(const wns::pyconfig::View& config, rise::Station* s) :
    ReceiverBase(config),
    OFDMAAspect(config.get<wns::Ratio>("receiverNoiseFigure")),
    FTFadingAspect(config),
    MeasurementAspect(config),
    LossCalculation(config),
    station(s),
    propagationCache(new rise::IdVectorCache(this)),
    activeTransmissions(),
    wraparoundShiftVectors(NULL),
    nSectors(config.get<int>("nSectors")),
    fastFadeInterference(config.get<bool>("fastFadeInterference"))
{
    activeTransmissions.clear();
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

    mimoProcessing = mimo::CalculationStrategyFactory::creator(
        config.get<std::string>("mimoProcessing.__plugin__"))->create(config.getView("mimoProcessing"),
                                                                      this,
                                                                      &logger);

    s->setReceiverType(getPropagationCharacteristicId());
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
wns::Power
Receiver::getRxPower(const rise::TransmissionObjectPtr& t)
{
    wns::Power rxPower = getRxPower(t, getCurrentReceivePattern(t));
    MESSAGE_BEGIN(VERBOSE, logger, m , "getRxPower");
    m  << "(from: " << dynamic_cast<Station*>(t->getTransmitter()->getStation())->getNode()->getName() << ") = "<< rxPower;
    MESSAGE_END();
    return rxPower;
}

wns::Ratio
Receiver::getQuasiStaticPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern)
{
    double frequency = t->getPhysicalResource()->getFrequency();
    rise::Transmitter* transmitter = t->getTransmitter();
    assure(transmitter!=NULL,"transmitter==NULL");

    wns::Ratio transmittersAntennaGain = t->getTransmittersAntennaGain(getStation()->getAntenna()->getPosition());
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
        Station* receiverOFDMAStation = dynamic_cast<Station*>(getStation());
        assure(receiverOFDMAStation!=NULL, "station is not an OFDMA station (and may have no beamforming antenna)");
        receiverAntennaGain = receiverOFDMAStation->getBFAntenna()->getGain(transmitter->getAntenna()->getPosition(), pattern);
          // superposition of sector and beamfomning pattern
        receiverAntennaGain +=  getStation()->getAntenna()->getGain(transmitter->getAntenna()->getPosition(), wns::service::phy::ofdma::PatternPtr());
        MESSAGE_SINGLE(VERBOSE,logger, "  ReceiverAntennaGain of Beamformed Pattern: " << receiverAntennaGain);
    }
    // TODO: receiverOFDMAStation->getBFAntenna()->getGain() should deliver the same as
    // getStation()->getAntenna()->getGain()
    wns::Ratio pathLoss = purePathLoss - transmittersAntennaGain - receiverAntennaGain;

    // todo: store it in a cache?
    MESSAGE_SINGLE(VERBOSE,logger, "  getQuasiStaticPathLoss() = " << pathLoss);

    return pathLoss;
}

wns::Ratio
Receiver::getFullPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern)
{
    wns::Ratio pathLoss = getQuasiStaticPathLoss(t,pattern);

    MESSAGE_SINGLE(VERBOSE,logger, "  getFullPathLoss() = " << pathLoss);
    return pathLoss;
}

wns::Power
Receiver::getRxPower(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern)
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
    MESSAGE_SINGLE(NORMAL, logger, "TxPower: " << rxPower);

    wns::Ratio fullPathLoss = getFullPathLoss(t,pattern);
    MESSAGE_SINGLE(NORMAL, logger, "PathLoss Full (inside getRxPower) " << fullPathLoss );

    rxPower -= fullPathLoss;
    MESSAGE_SINGLE(NORMAL, logger, "RxPower: " << rxPower);
    return rxPower;
}

wns::Power
Receiver::getUnfilteredRxPower(const rise::TransmissionObjectPtr& t)
{
    assure(t, "no existing transmission object");

    wns::Power rxPower = t->getTxPower();

    rxPower -= getLoss(t->getTransmitter(), t->getPhysicalResource()->getFrequency());
    rxPower += t->getTransmittersAntennaGain(getStation()->getAntenna()->getPosition());

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

wns::Power
Receiver::getAllRxPower(const int subCarrier)
{
    wns::Power rxPower;
    assure(subCarrier >= 0, "Cannot measure RxPower of a subCarrier < 0\n");
    assure(subCarrier <= getCurrentNumberOfSubCarriers(), "subCarrier is too large\n");

    // Add noise
    rxPower += getNoise(subCarrier);

    // Add all current transmissions
    rise::medium::PhysicalResource::TransmissionObjectIterator itr;
    for (itr=physicalResources[subCarrier]->getTOBegin();
         itr!=physicalResources[subCarrier]->getTOEnd();
         ++itr)
    {
        rxPower += getRxPower(*itr);
    }

    return rxPower;
}


wns::Power
Receiver::getAllRxPower()
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
    for(itr=t->getPhysicalResource()->getTOBegin();
        itr!=t->getPhysicalResource()->getTOEnd();
        ++itr)
    {
        if (*itr == t)
        {
            // Do not include power of this transmission 
        }
        else
        {
            if (nSectors == 1)
            {
                wns::Power iPower = getRxPower(*itr, currentPattern);
                if(fastFadeInterference)
                {
                    iPower += getFastFading(*(*itr)->getTransmitter(), 
                        (*itr)->getPhysicalResource()->getFrequency());
                }
                interference += iPower;
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
    MESSAGE_SINGLE(VERBOSE, logger, "Receiver::insertReceivePattern for node: "<< node->getName());
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
    MESSAGE_SINGLE(VERBOSE, logger, "Receiver::setCurrentReceivePatterns");
    currentReceivePatterns = _currentReceivePatterns;
}

wns::service::phy::ofdma::PatternPtr
Receiver::getCurrentReceivePattern(const rise::TransmissionObjectPtr& t) const
{
    // station must not be an ofdmaphy station, but it must be a node provider
    Station* tmpStation = dynamic_cast<ofdmaphy::Station*>(t->getTransmitter()->getStation());
    if (tmpStation)
    {
        return getCurrentReceivePattern(tmpStation->getNode());
    }

    Sender* tmpSender = dynamic_cast<ofdmaphy::Sender*>(t->getTransmitter()->getStation());
    if (tmpSender)
    {
        return getCurrentReceivePattern(tmpSender->getMyNode());
    }

    throw wns::Exception("Unable to determine station type of sender");
}

wns::service::phy::ofdma::PatternPtr
Receiver::getCurrentReceivePattern(wns::node::Interface* pStack) const
{
    MESSAGE_SINGLE(VERBOSE, logger, "Searching ReceivePattern for node: " << pStack->getName());
    std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr>::const_iterator itr;
    itr = currentReceivePatterns.find(pStack);

    if( itr == currentReceivePatterns.end())
    {
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
    cacheEntry.setPathloss(getPathloss(*t, freq));
    cacheEntry.setShadowing(getShadowing(*t));
    cacheEntry.setAntennaGain(wns::Ratio::from_dB(0));
    cacheEntry.setValid(true);
}

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
    // Only if we were set to receive mode we listen to notifications
    if (! (getOFDMAStation()->isReceptionEnabled()))
    {
       return;
    }
    // Do not receive from myself
    if(t->getTransmitter()->getStation()->getStationId() == getStation()->getStationId())
    {
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
            if (t->getIsStart())
                return;
            wns::Power newReceivedSignalStrength = getAllRxPower();
            assure(newReceivedSignalStrength > getRxPower(t), "receivedSignalStrength is too low for current ongoing	transmission\n");
            newReceivedSignalStrength -= getRxPower(t);
            if (newReceivedSignalStrength != receivedSignalStrength)
            {
                // the new signal strength is propagated with a delay
                receivedSignalStrength = newReceivedSignalStrength;
                this->signalNewReceivedSignalStrength();
            }
        }
        return;
    }

    if (transmissionForMe(t))
    {
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

                wns::Ratio omniAttenuation = wns::Ratio::from_dB(0);
                wns::service::phy::ofdma::PatternPtr pattern = getCurrentReceivePattern(t);
                if (pattern != wns::service::phy::ofdma::PatternPtr())
                {
                    // not an empty pattern
                    omniAttenuation = pattern->getOmniAttenuation();
                }

                Station* ofdmaStation = getOFDMAStation();

                if (doMeasurementUpdates)
                {
                    // the periodic "big" ones
                    wns::Ratio pathLoss = getQuasiStaticPathLoss(t, getCurrentReceivePattern(t)); // determine (again)
                    saveMeasuredFlatPathloss(sourceNode,pathLoss);
                }

                std::vector<wns::Ratio> postProcessingSINRFactor(1, wns::Ratio::from_factor(1.0));
                // minimize calculation if MIMO is not used at all
                if(!ofdmaStation->beamformingEnabled() && (ofdmaStation->getNumAntennas() > 1 || t->getNumberOfSpatialStreams() > 1))
                {
                    postProcessingSINRFactor = mimoProcessing->getPostProcessingSINRFactor(t);
                }
                // Get Interference + Noise
                wns::Power ipn = getAveragedInterference(t);
                wns::Power noise = getNoisePerSubChannel();
                wns::Ratio iot = ipn / noise;

                wns::Ratio fastFadingGain;

                fastFadingGain = getFastFading(*t->getTransmitter(), t->getPhysicalResource()->getFrequency());

                wns::geometry::Point him = t->getTransmitter()->getStation()->getAntenna()->getPosition();
                wns::geometry::Point me = getStation()->getAntenna()->getPosition();
                double distance = (him - me).getR();

                wns::service::phy::power::PowerMeasurementPtr rxPowerMeasurementPtr =
                    wns::SmartPtr<rise::receiver::PowerMeasurement>
                    (new rise::receiver::PowerMeasurement(t,
                                                          sourceNode,
                                                          getAveragedRxPower(t) * fastFadingGain,
                                                          ipn,
                                                          iot,
                                                          fastFadingGain,
                                                          omniAttenuation,
                                                          distance,
                                                          postProcessingSINRFactor));
                MESSAGE_SINGLE(NORMAL, logger, "PowerMeasurement="<<*rxPowerMeasurementPtr);

                // pass received payload upwards in the stack:
                ofdmaStation->receiveData(t->getPayload(),
                                          rxPowerMeasurementPtr);

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
            }
            else
            {
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
            // the new signal strength is propagated with a delay
            receivedSignalStrength = newReceivedSignalStrength;
            this->signalNewReceivedSignalStrength();
        }
    }
} // Receiver::notify()

void
Receiver::signalNewReceivedSignalStrength()
{
    MESSAGE_SINGLE(NORMAL, logger, "notify: New carrier-sense of " << receivedSignalStrength);
    this->wns::Subject<RSSInterface>::forEachObserver(OnNewRSS(receivedSignalStrength));
}

void
Receiver::updateRequest()
{
    receivedSignalStrength = getAllRxPower();
    this->wns::Subject<RSSInterface>::forEachObserver(OnNewRSS(receivedSignalStrength));
}

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
        rise::scenario::ftfading::FTFading* ftfading = perSourceContainer.ftfading;

        assure((int)perSourceContainer.interferenceVector.size()==numberOfSubChannels,
               "wrong interferenceVector.size="<<perSourceContainer.interferenceVector.size());

        MESSAGE_SINGLE(NORMAL, logger, "  source="<<source->getName()<<": PL="<<quasiStaticPathLoss<<", c="<<perSourceContainer.packetCount);
        wns::service::phy::power::OFDMAMeasurementPtr ofdmaMeasurementPtr =
            ofdmaphy::OFDMAPHYMeasurementPtr
            (new ofdmaphy::OFDMAMeasurement(source,
                                            numberOfSubChannels,
                                            measurementUpdateInterval,
                                            quasiStaticPathLoss,
                                            ftfading,
                                            perSourceContainer.interferenceVector, // big copy
                                            logger));


        // send measurements
        ofdmaStation->measurementUpdate(source,ofdmaMeasurementPtr);

        // new vector object
        perSourceContainer.interferenceVector =
            std::vector<wns::Power>(numberOfSubChannels,interferenceTemplate);
    } // forall sources
}

bool
Receiver::isReceiving() const
{
    return (!activeTransmissions.empty());
}

// very useful for debugging:
std::string
Receiver::printActiveTransmissions() const
{
    assure(this!=NULL,"Houston, we have a problem");
    std::stringstream s;
    s << "activeTransmissions =" << std::endl;
    //TransmissionObjectPtrList activeTransmissions;
    for ( TransmissionObjectPtrList::const_iterator iter = activeTransmissions.begin();
          iter != activeTransmissions.end(); ++iter)
    {
        rise::TransmissionObjectPtr transmissionObject = *iter; // SmartPtr
        s << transmissionObject->toString() << std::endl;
        
    }
    return s.str();
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



