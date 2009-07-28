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

#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/Manager.hpp>
#include <OFDMAPHY/Receiver.hpp>
#include <OFDMAPHY/Component.hpp>

#include <RISE/scenario/Scenario.hpp>
#include <RISE/receiver/PowerMeasurement.hpp>
#include <RISE/transmissionobjects/unicasttransmissionobject.hpp>
#include <RISE/transmissionobjects/broadcasttransmissionobject.hpp>
#include <RISE/transmissionobjects/transmissionobjectbf.hpp>
#include <RISE/manager/metasystemmanager.hpp>

#include <WNS/service/phy/power/OFDMAMeasurement.hpp> // to be derived from
#include <WNS/PowerRatio.hpp>
#include <WNS/SmartPtr.hpp>
#include <WNS/node/component/Component.hpp>


using namespace ofdmaphy;

// a new Station is created in constructor of ofdmaphy::Component
Station::Station(Component* _component, const wns::pyconfig::View& pyConfigView) :
    rise::Station(pyConfigView),
    //log("Station"),
    eirpLimited(pyConfigView.get<bool>("eirpLimited")),
    logger(pyConfigView.get("logger")),
    systemManager(dynamic_cast<SystemManager*>(rise::MetaSystemManager::getInstance()->getSystemManagerBySystemName(pyConfigView.get<std::string>("systemManagerName")))),
    transmitter(NULL),
    receiver(NULL),
    powerAdmission(this),
    maxTxPowerPerSubband(pyConfigView.get<wns::Power>("txPower")),
    totalPower(pyConfigView.get<wns::Power>("totalPower")),
    tuneRx(),
    tuneTx(),
    reverseState(false),
    beamformingAntenna(NULL),
    supportsBeamforming(false),
    activeTransmissions(),
    handler(NULL),
    measurementHandler(NULL),
    component(_component) // to be removed, only for Node retrieval
{
//    MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station contruction. total txPower="<<txPower);

    assure(systemManager, "Was not able to acquire SystemManager");

    if (! pyConfigView.isNone("beamformingAntenna"))
    {
        beamformingAntenna = new rise::antenna::Beamforming(pyConfigView.getView("beamformingAntenna"), this);
        supportsBeamforming = true;
    }
    else
    {
        supportsBeamforming = false;
    }

    assure(pyConfigView.len("receiver") == 1,
           "Only one receiver supported at the moment!");

    this->receiver = new Receiver(pyConfigView.getView("receiver", 0), this);
    this->transmitter = new Transmitter<Station>(pyConfigView.getView("transmitter", 0), this, getAntenna());

    double txFrequency = pyConfigView.get<double>("txFrequency");
    double rxFrequency = pyConfigView.get<double>("rxFrequency");
    double bandwidth   = pyConfigView.get<double>("bandwidth");
    int numberOfSubCarrier = pyConfigView.get<int>("numberOfSubCarrier");

    tuneRx.frequency = rxFrequency;
    tuneRx.bandwidth = bandwidth;
    tuneRx.numberOfSubCarrier = numberOfSubCarrier;

    tuneTx.frequency = txFrequency;
    tuneTx.bandwidth = bandwidth;
    tuneTx.numberOfSubCarrier = numberOfSubCarrier;

    this->setRxTune( tuneRx );
    this->setTxTune( tuneTx );

    // Start to observe the receiver for onNewRSS calls
    //this->wns::Observer<RSSInterface>::startObserving(receiver);

    MESSAGE_BEGIN(NORMAL, logger, m, "ofdmaphy::Station constructed: ");
    m << "#SC="<<numberOfSubCarrier
	  <<", fTx="<<txFrequency
	  <<", fRx="<<rxFrequency
	  <<", b="<<bandwidth
	  <<", Ptotal="<<totalPower
	  <<", PmaxSubband="<<maxTxPowerPerSubband;
	  // TODO: nominal power per subBand?
    MESSAGE_END();

	assure(maxTxPowerPerSubband <= totalPower, "Inconsistent txPower settings.");

}

Station::~Station()
{
    activeTransmissions.clear();
    delete receiver;
    delete transmitter;
    if (beamformingAntenna)
        delete beamformingAntenna;
}

void
Station::registerRSSHandler(wns::service::phy::ofdma::RSSHandler* _rssHandler)
{
    assure(_rssHandler, "must be non-NULL");
    rssHandler = _rssHandler;
    this->wns::Observer<RSSInterface>::startObserving(this->receiver);
}

rise::antenna::Beamforming*
Station::getBFAntenna() const {
    assure (beamformingEnabled(), "Beamforming is not supported in the current configuration");
    assure (beamformingAntenna, "Beamforming is not supported in the current configuration");
    return beamformingAntenna;
}

bool
Station::beamformingEnabled() const
{
    return supportsBeamforming;
}

void
Station::onNodeCreated()
{
	if (beamformingEnabled() == false)
		return;

	assure(
		getAntenna()->getPosition() == getBFAntenna()->getPosition(),
		"Both antennas must have the same position since the beamforming"
		"algorithm will use the position of the Station::antenna!");
}

void
Station::startBroadcast(wns::osi::PDUPtr sdu, int subBand, wns::Power requestedPower, wns::service::phy::phymode::PhyModeInterfacePtr phyModePtr)
{
	// check the requested txPower (per subBand) (and modify if needed)
	wns::Power txPower = this->powerAdmission->admit(requestedPower);
	assure(phyModePtr,"phyModePtr==NULL");

	// broadcast transmission (non-beamforming with static antenna)
	Broadcast bto(new rise::BroadcastTransmissionObject(transmitter,
							    sdu,
							    txPower,
								phyModePtr,
							    uint32_t(1)));

 	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station::startBroadcast(subBand="<<subBand<<",P="<<txPower<<",M&C="<<*phyModePtr<<") Broadcast");
	transmitter->startTransmitting(bto, subBand);
	assure(activeTransmissions.find(sdu) == activeTransmissions.end(), "Transmission for this PDU already active");
	activeTransmissions[sdu]= bto;
}

void
Station::startBroadcast(wns::osi::PDUPtr sdu, int subBand, wns::Power requestedPower)
{
 	MESSAGE_SINGLE(VERBOSE, logger, "ofdmaphy::Station::startBroadcast(): WARNING: using old interface without PhyMode");
	startBroadcast(sdu, subBand, requestedPower, wns::service::phy::phymode::emptyPhyModePtr());
}

void
Station::startUnicast(wns::osi::PDUPtr sdu, wns::node::Interface* _recipient, int subBand, wns::Power requestedPower, wns::service::phy::phymode::PhyModeInterfacePtr phyModePtr)
{
	// unicast transmission (non-beamforming with static antenna)
	assure(_recipient != NULL, "Invalid Recipient");
	assure(phyModePtr,"phyModePtr==NULL");

	// check the requested txPower (and modify if needed)
	wns::Power txPower = powerAdmission->admit(requestedPower);

	Station* recipient = systemManager->getStation(_recipient);

	Unicast uto =
		Unicast(new rise::UnicastTransmissionObject(transmitter,
								 recipient->receiver,
								 uint32_t(1),
								 sdu,
								 txPower,
								 phyModePtr));
 	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station::startUnicast(subBand="<<subBand<<",P="<<txPower<<",M&C="<<*phyModePtr<<")");
	// call RISE transmitter
	transmitter->startTransmitting(uto, subBand);
	assure(activeTransmissions.find(sdu) == activeTransmissions.end(), "Transmission for this PDU already active");
	activeTransmissions[sdu]= uto;
}

void
Station::startUnicast(wns::osi::PDUPtr sdu, wns::node::Interface* _recipient, int subBand, wns::Power requestedPower)
{
 	MESSAGE_SINGLE(VERBOSE, logger, "ofdmaphy::Station::startUnicast(): WARNING: using old interface without PhyMode");
	startUnicast(sdu, _recipient, subBand, requestedPower, wns::service::phy::phymode::emptyPhyModePtr());
}

void
Station::stopTransmission(wns::osi::PDUPtr sdu, int
#ifndef WNS_NO_LOGGING
						  subBand
#endif
)
{
	assure(activeTransmissions.find(sdu) != activeTransmissions.end(), "No active transmission with this PDU");
	assure(activeTransmissions.find(sdu)->second->getPayload() == sdu, "Wrong pdu"); // hoy:???

 	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station::stopTransmission(subBand=" << subBand << ")");
	std::map<wns::osi::PDUPtr, Transmission>::iterator itr = activeTransmissions.find(sdu);

	// Remove transmission
	// Transmission is a SmartPtr -> Transmission() creates NULL pointer
	Transmission t = itr->second;
	activeTransmissions.erase(itr);

	transmitter->stopTransmitting(t);
}

wns::CandI
Station::getCurrentCandI(wns::osi::PDUPtr sdu)
{
	//for testing purpose only
	assure(activeTransmissions.find(sdu) != activeTransmissions.end(), "No active transmission with this PDU");
	assure(activeTransmissions.find(sdu)->second->getPayload() == sdu, "Wrong pdu"); // hoy:???

	std::map<wns::osi::PDUPtr, Transmission>::iterator itr = activeTransmissions.find(sdu);
	wns::CandI candi;

	candi.I = wns::dynamicCast<rise::UnicastTransmissionObject, rise::TransmissionObject>(itr->second)->getReceiver()->getInterference(itr->second);
	candi.C = wns::dynamicCast<rise::UnicastTransmissionObject, rise::TransmissionObject>(itr->second)->getReceiver()->getRxPower(itr->second);

	return candi;
}

rise::SystemManager*
Station::getSystemManager() const
{
	return systemManager;
}

void
Station::receiveData(wns::osi::PDUPtr sdu, wns::service::phy::power::PowerMeasurementPtr rxPowerMeasurementPtr)
{
	//MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station::receiveData(rxPower="<< rxPowerMeasurementPtr->getRxPower()<<")");
	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station::receiveData(): calling handler->onData("<<rxPowerMeasurementPtr->getString()<<")");
	//rxPowerMeasurementPtr->setSystemManager(systemManager); // needed to determine sourceNode
	handler->onData(sdu, rxPowerMeasurementPtr);
}

void
Station::measurementUpdate(wns::node::Interface* source, wns::service::phy::power::OFDMAMeasurementPtr rxPowerMeasurementPtr)
{
	// source == rxPowerMeasurementPtr->source are the same
	assure(source == rxPowerMeasurementPtr->getSourceNode(),"measurement source mismatch");
	if (source != NULL)
		MESSAGE_SINGLE(NORMAL,logger, "Station::measurementUpdate("<<source->getName()<<")");
	//assure(measurementHandler != NULL, "no measurementHandler registered (NULL)");
	if (measurementHandler!=NULL) { // only if service was requested
		measurementHandler->onMeasurementUpdate(source, rxPowerMeasurementPtr);
	} else {
		MESSAGE_SINGLE(NORMAL,logger, "Station::measurementUpdate: no measurementHandler !!!");
	}
}

void
Station::onNewRSS(wns::Power rss)
{
    if(activeTransmissions.empty())
    {
        MESSAGE_SINGLE(VERBOSE, logger, "onNewRSS() with rss " << rss);
        // Relay the information to the observer
        this->rssHandler->onRSSChange(rss);
        //this->wns::Subject<CarrierSensing>::forEachObserver(OnCS(rss));
    }
    else
    {
        MESSAGE_SINGLE(VERBOSE, logger, "onNewRSS() with rss " << rss << ", but own transmission ongoing");
    }

}

std::map<wns::node::Interface*, wns::Ratio>
Station::calculateSINRsRx(const std::vector<wns::node::Interface*>& combination,
                          wns::Power iInterPlusNoise)
{
    // prepare empty container for the result
    std::map<wns::node::Interface*, wns::Ratio> returnValue;

    // call the function that calculates signal and interference separately
    std::map<wns::node::Interface*, wns::CandI > candis = calculateCandIsRx(combination, iInterPlusNoise);

    // calculate SINRs accordingly
    for (std::map<wns::node::Interface*, wns::CandI>::iterator itr = candis.begin();
         itr != candis.end();
         itr++)
    {
        returnValue[itr->first] = itr->second.C / itr->second.I;
    }

    return returnValue;
}

std::map<wns::node::Interface*, wns::CandI >
Station::calculateCandIsRx(const std::vector<wns::node::Interface*>& combination,
                           wns::Power iInterPlusNoise)
{
    assure( supportsBeamforming, "Beamforming not supported in current configuration" );
    assure( beamformingAntenna , "No Beamforming Antenna present");
    assure( iInterPlusNoise.get_mW() > 0, "beam pattern calculation requires positive level of omni-directional interference");

    std::map<rise::Station*, wns::CandI > candis;
    std::map<wns::node::Interface*, wns::CandI > returnValue;

    std::vector<rise::Station*> vec = std::for_each(combination.begin(),
                                                    combination.end(),
                                                    ConvertNode<Station*, std::vector<rise::Station*> >(this)).result;

    candis = beamformingAntenna->calculateCandIsRx(vec, iInterPlusNoise);

    for (std::map<rise::Station*, wns::CandI >::iterator itr = candis.begin();
         itr != candis.end();
         itr++)
    {
        assureType(itr->first, Station*);
        returnValue[static_cast<Station*>(itr->first)->getNode()] = itr->second;
    }

    return returnValue;
}

std::map<wns::node::Interface*, wns::Ratio>
Station::calculateSINRsTx(const std::map<wns::node::Interface*, wns::Power>& Station2NoisePlusIintercell,
                          wns::Power x_friendlyness,
                          wns::Power intendedTxPower)
{
    std::map<wns::node::Interface*, wns::Ratio> returnValue;

    std::map<wns::node::Interface*, wns::CandI > candis = calculateCandIsTx(Station2NoisePlusIintercell,
                                                                            x_friendlyness,
                                                                            intendedTxPower);

    for (std::map<wns::node::Interface*, wns::CandI>::const_iterator itr = candis.begin();
         itr != candis.end();
         itr++)
    {
        returnValue[itr->first] = itr->second.C / itr->second.I;
    }

    return returnValue;
}

std::map<wns::node::Interface*, wns::CandI >
Station::calculateCandIsTx(const std::map<wns::node::Interface*, wns::Power>& Station2NoisePlusIintercell,
                           wns::Power x_friendlyness,
                           wns::Power intendedTxPower)
{
    assure( supportsBeamforming, "Beamforming not supported in current configuration" );
    assure( beamformingAntenna , "No Beamforming Antenna present");
    assure(x_friendlyness.get_mW() > 0, "friendlyness factor to reduce generated interference (sidelobes) must be > 0");

    std::map<rise::Station*, wns::Power> map;
    std::map<rise::Station*, wns::CandI > candis;
    std::map<wns::node::Interface*, wns::CandI > returnValue;

    for (std::map<wns::node::Interface*, wns::Power>::const_iterator itr = Station2NoisePlusIintercell.begin();
         itr != Station2NoisePlusIintercell.end();
         itr++)
    {
        map[systemManager->getStation(itr->first)] = itr->second;
    }

    candis = beamformingAntenna->calculateCandIsTx(map, x_friendlyness, intendedTxPower, eirpLimited);

    for (std::map<rise::Station*, wns::CandI>::iterator itr = candis.begin();
         itr != candis.end();
         itr++)
    {
        assureType(itr->first, Station*);
        returnValue[static_cast<Station*>(itr->first)->getNode()] = itr->second;
    }

    return returnValue;
}

wns::service::phy::ofdma::PatternPtr
Station::calculateAndSetBeam(wns::node::Interface *id,
                             const std::vector<wns::node::Interface*>& undesired,
                             wns::Power omniPower)
{
    assure( supportsBeamforming, "Beamforming not supported in current configuration" );
    assure( beamformingAntenna , "No Beamforming Antenna present");
    assure( omniPower.get_mW() > 0, "beam pattern calculation requires positive level of omni-directional power");

    std::vector<rise::Station*> vec = std::for_each(undesired.begin(),
                                                    undesired.end(),
                                                    ConvertNode<Station*, std::vector<rise::Station*> >(this)).result;

    Station* r = systemManager->getStation(id);

    return beamformingAntenna->calculateAndSetBeam(r, vec, omniPower);
}

double
Station::estimateDoA(wns::node::Interface *id)
{
    assure( supportsBeamforming, "Beamforming not supported in current configuration" );
    assure( beamformingAntenna , "No Beamforming Antenna present");
    return( beamformingAntenna->estimateDoA(systemManager->getStation(id)));
}

void
Station::startTransmission(wns::osi::PDUPtr sdu,
                           wns::node::Interface* _recipient,
                           int subBand,
                           wns::service::phy::ofdma::PatternPtr pattern,
                           wns::Power requestedTxPower,
                           const wns::service::phy::phymode::PhyModeInterfacePtr phyModePtr)
{
    // unicast beamforming transmission with beamforming antenna
    assure( supportsBeamforming, "Beamforming not supported in current configuration" );
    assure( beamformingAntenna , "No Beamforming Antenna present");
    assure(_recipient != NULL, "Invalid Recipient");
    assure(pattern != wns::service::phy::ofdma::PatternPtr(), "not a valid pattern");
    assure(phyModePtr,"phyModePtr==NULL");

    // check the requested txPower (and modify if needed)
    wns::Power txPower = powerAdmission->admit(requestedTxPower);

    MESSAGE_BEGIN(NORMAL, logger, m, "new PDU with txPower of ");
    m << requestedTxPower.get_dBm();
    m << " dBm, \n the current sum Tx power is " << this->getSumPower().get_dBm();
    m << " dBm, \n the available total Tx power is " << this->getMaxOutputPower().get_dBm() << "dBm";
    MESSAGE_END();

    Station* recipient = systemManager->getStation(_recipient);

    BeamformingTransmission bfto =
        BeamformingTransmission(new rise::TransmissionObjectBF(transmitter,
                                                               recipient->receiver,
                                                               this->getBFAntenna(),
                                                               sdu,
                                                               requestedTxPower,
                                                               phyModePtr,
                                                               pattern,
                                                               uint32_t(1)));
    MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Station::startTransmissions(subBand="<<subBand<<",P="<<txPower<<",M&C="<<*phyModePtr<<") BF");
    transmitter->startTransmitting(bfto, subBand);

    assure(activeTransmissions.find(sdu) == activeTransmissions.end(), "Transmission for this PDU already active");
    activeTransmissions[sdu] = bfto;

    // Write Pattern to output file
    MESSAGE_BEGIN(NORMAL, logger, m, "Drawing radiation Pattern for transmission from ");
    m << this->getNode()->getName() << " to " << _recipient->getName();
    std::string fileName = std::string("patterns/")+this->getNode()->getName()+"_"+_recipient->getName()+".pattern";
    this->getBFAntenna()->drawRadiationPattern(fileName, pattern);
    MESSAGE_END();
}

void
Station::startTransmission(wns::osi::PDUPtr sdu,
                           wns::node::Interface* _recipient,
                           int subBand,
                           wns::service::phy::ofdma::PatternPtr pattern,
                           wns::Power requestedTxPower)
{
    MESSAGE_SINGLE(VERBOSE, logger, "ofdmaphy::Station::startTransmission(): WARNING: using old interface without PhyMode");
    startTransmission(sdu, _recipient, subBand, pattern, requestedTxPower, wns::service::phy::phymode::emptyPhyModePtr());
}

bool
Station::isReceiving() const
{
    return receiver->isReceiving();
}

void
Station::setRxTune(const wns::service::phy::ofdma::Tune& rxTune)
{
    tuneRx = rxTune;
    receiver->tune(tuneRx.frequency,
                   tuneRx.bandwidth,
                   tuneRx.numberOfSubCarrier);
}

void
Station::setTxTune(const wns::service::phy::ofdma::Tune& txTune)
{
    tuneTx = txTune;
    transmitter->tune(tuneTx.frequency,
                      tuneTx.bandwidth,
                      tuneTx.numberOfSubCarrier);
}

void
Station::setTxRxSwap(bool reverse)
{
    if (tuneRx == tuneTx)
    {
        reverseState = reverse;
        return; // nothing to swap
    }

    if (reverse)
    {
        if (reverseState)
        {
            // do nothing
        }
        else
        {
            // reverse current setup
            wns::service::phy::ofdma::Tune oldRxTune = this->getRxTune();

            this->setRxTune( this->getTxTune() );
            this->setTxTune( oldRxTune );
        }
    }
    else
    {
        if (reverseState)
        {
            // revert back
            // reverse current setup
            wns::service::phy::ofdma::Tune oldRxTune = this->getRxTune();

            this->setRxTune( this->getTxTune() );
            this->setTxTune( oldRxTune );
        }
        else
        {
            // do nothing, stay reverse
        }
    }

    // remember the state
    reverseState = reverse;

    MESSAGE_BEGIN(NORMAL, logger, m, "ofdmaphy::setTxRxSwap(reverse="<< (reverse ? "true" : "false" ) << "): ");
    m << "fTx="<<tuneTx.frequency<<", fRx="<<tuneRx.frequency;
    MESSAGE_END();
}

void
Station::setCurrentReceivePatterns(std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> _receivePatterns)
{
	// if the source station of the transmission  is not found
    // in _receivePatterns, the receiver will use instead of
	// the beamforming antenna the non-beamforming antenna for reception

	// @todo make known to the outside world whether we are capable of
	// beamforming or not

	// If someone tries to clear our pattern cache while we are not capable of
	// beamforming anyway, we just ignore it. All other functions where
	// beamforming would be actively required will raise an exception anyway in
	// this case
	if ( supportsBeamforming == false and _receivePatterns.empty())
		return;

	assure( supportsBeamforming, "Beamforming not supported in current configuration" );
	assure( beamformingAntenna , "No Beamforming Antenna present");
	receiver->setCurrentReceivePatterns(_receivePatterns);
}

void
Station::insertReceivePattern(wns::node::Interface* _node, wns::service::phy::ofdma::PatternPtr _pattern)
{
	assure( supportsBeamforming, "Beamforming not supported in current configuration" );
	assure( beamformingAntenna , "No Beamforming Antenna present");
	receiver->insertReceivePattern(_node, _pattern);
}

void
Station::removeReceivePattern(wns::node::Interface* _node)
{
	assure( supportsBeamforming, "Beamforming not supported in current configuration" );
	assure( beamformingAntenna , "No Beamforming Antenna present");
	receiver->removeReceivePattern(_node);
}

void
Station::setTxPowerForStation(wns::node::Interface* stack, wns::Power _txPower)
{
	assure( supportsBeamforming, "Beamforming not supported in current configuration" );
	assure( beamformingAntenna , "No Beamforming Antenna present");
	beamformingAntenna->setTxPowerForStation(systemManager->getStation(stack), _txPower);
}

void
Station::setPowerReceivedForStation(wns::node::Interface* stack, wns::Power _rxPower)
{
	assure( supportsBeamforming, "Beamforming not supported in current configuration" );
	assure( beamformingAntenna , "No Beamforming Antenna present");
	beamformingAntenna->setPowerReceivedForStation(systemManager->getStation(stack), _rxPower);
}


void
Station::startReceiving(){
// must be implemented since it is abstract
}

void
Station::stopReceiving(){
// must be implemented since it is abstract
}

wns::node::Interface*
Station::getNode()
{
	return component->getNode();
}

wns::Power
Station::getSumPower() const
{
    wns::Power sumPower = wns::Power().from_mW(0.0);
    for ( std::map<wns::osi::PDUPtr, Transmission>::const_iterator itr = activeTransmissions.begin();
          itr!= activeTransmissions.end();
          itr++)
    {
        sumPower += itr->second->getTxPower();
    }
    MESSAGE_SINGLE(NORMAL,logger,"Current Sum Power is: "<<sumPower);
    return sumPower;
}

wns::Power
Station::admit(const wns::Power& requestedPower) const
{
    wns::Power request = requestedPower;
    wns::Power currentPower = this->getSumPower();
    wns::Power maxPower = this->getMaxOutputPower();

// 	// Check per-subband limitations
// 	if (request > maxTxPowerPerSubband)
// 	{
// 		MESSAGE_SINGLE(NORMAL, logger, "Cut requested power of "<<request<<" to max Power per subband of "<<maxTxPowerPerSubband);
// 		// Cut to meet limit
// 		request = maxTxPowerPerSubband;
// 	}

    // Check overall limitations
    if (currentPower + request <= maxPower)
    {
        MESSAGE_SINGLE(NORMAL, logger, "Admitting transmission with txPower: "<<request);
        return request;
    }

    // Cut to meet limit
    wns::Power cutPower = maxPower-currentPower;

    MESSAGE_SINGLE(NORMAL, logger, "Cut requested power of "<<request<<" to "<<cutPower<<" due to Overall Power constraints");
    MESSAGE_SINGLE(NORMAL, logger, "Admitting transmission with txPower: "<<cutPower);

    assure(cutPower != wns::Power(), "You may not initiate a transmission with 0 mW");
    return cutPower;
}


