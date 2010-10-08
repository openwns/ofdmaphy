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

#include <OFDMAPHY/Sender.hpp>
#include <OFDMAPHY/Scanner.hpp>

#include <RISE/transmissionobjects/broadcasttransmissionobject.hpp>
#include <RISE/manager/metasystemmanager.hpp>

#include <WNS/PowerRatio.hpp>
#include <WNS/node/component/Component.hpp>
#include <WNS/probe/bus/ContextProvider.hpp>
#include <WNS/probe/bus/ContextProviderCollection.hpp>
#include <WNS/distribution/DiscreteUniform.hpp>

#include <boost/bind.hpp>

using namespace ofdmaphy;

STATIC_FACTORY_REGISTER_WITH_CREATOR(
	Scanner,
	wns::node::component::Interface,
	"ofdmaphy.Scanner",
	wns::node::component::ConfigCreator
	);

Scanner::Receiver::Receiver(const wns::pyconfig::View& config, Scanner* s) :
    ofdmaphy::receiver::Receiver(config, s),
	scanner(s),
	bsID(0),
    margin(wns::Ratio::from_dB(-250.0))
{
}

Scanner::Receiver::~Receiver()
{
}

void
Scanner::Receiver::notify(rise::TransmissionObjectPtr to)
{
	add(to);
	transmissions.push_back(to);
	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Receiver::notify(): startOfTransmission");
}

void
Scanner::Receiver::positionChanged()
{
    ofdmaphy::receiver::Receiver::positionChanged();

    if(transmissions.empty())
        return;

    wns::Position currentPosition = getStation()->getAntenna()->getPosition();

    assure(margin >= wns::Ratio::from_dB(0.0), "Margin must be >= 0 dB");

    wns::Ratio nullMargin = wns::Ratio::from_dB(0);
    wns::Ratio maxSINR = wns::Ratio::from_dB(-250);
    wns::Power maxRxPwr = wns::Power::from_dBm(-250);
    wns::Ratio minPathloss = wns::Ratio::from_dB(300);
    double assocdistance = 0.0;

    std::multimap<wns::Ratio, rise::TransmissionObjectPtr> sinrSortedTransmissions;                                    

	for (TOList::const_iterator itr = transmissions.begin();
		 itr != transmissions.end();
		 ++itr)
	{
		Transmitter<Sender>* transmitter = 
			dynamic_cast<Transmitter<Sender>*>((*itr)->getTransmitter());

        wns::Position bsPosition = transmitter->getAntenna()->getPosition();
        double distance = (bsPosition - currentPosition).abs();
		wns::Power txp    = (*itr)->getTxPower();
		wns::Power rxp    = this->getRxPower(*itr);
		wns::Power interf = this->getInterference(*itr);
		wns::Ratio sinr   = rxp / interf;
        wns::Ratio pathloss = this->getQuasiStaticPathLoss((*itr), wns::service::phy::ofdma::PatternPtr());
        pathloss += wns::Ratio::from_dB(2.0);

        if(margin > nullMargin)
            sinrSortedTransmissions.insert(std::pair<wns::Ratio, rise::TransmissionObjectPtr>(sinr, *itr));

        maxSINR = std::max(maxSINR, sinr);
        maxRxPwr = std::max(maxRxPwr, rxp);
        if (pathloss < minPathloss)
        {
            minPathloss = pathloss;
            assocdistance = distance;
        }

		bsID = transmitter->getStation()->getStationId();

		MESSAGE_BEGIN(NORMAL, logger, m, "Measured ");

		std::string msName = scanner->getMyNode()->getName();
		std::string bsName = transmitter->getStation()->getMyNode()->getName();

        m << bsName << "(bsId=" << bsID << ") by " << msName << "(msId=" << scanner->getStationId() << "):\n"
		  << "TxPower: " << txp.get_dBm() << "\n"
		  << "RxPower: " << rxp.get_dBm() << "\n"
		  << "Interference+Noise: " << interf.get_dBm() << "\n"
		  << "SINR:" << sinr.get_dB() << "\n";
		MESSAGE_END();

		rxpContextCollector->put(rxp.get_dBm());
		sinrContextCollector->put(sinr.get_dB());
        pathlossContextCollector->put(-1 * pathloss.get_dB());
	}

    if(margin > nullMargin)
    {
        wns::Ratio marginSINR = maxSINR - margin;

        std::multimap<wns::Ratio, rise::TransmissionObjectPtr>::iterator it_bound;

        it_bound = sinrSortedTransmissions.lower_bound(marginSINR);
        assure(it_bound != sinrSortedTransmissions.end(), "Maximum SINR not in sorted SINR container");
        
        std::vector<rise::TransmissionObjectPtr> candidates;
        
        std::multimap<wns::Ratio, rise::TransmissionObjectPtr>::iterator it;
        for(it = it_bound; it != sinrSortedTransmissions.end(); it++)
            candidates.push_back(it->second);

        wns::distribution::DiscreteUniform rng(0, candidates.size() - 1);
        rise::TransmissionObjectPtr serving =  candidates[rng()];

		wns::Power rxp    = this->getRxPower(serving);
		wns::Power interf = this->getInterference(serving);
		wns::Ratio sinr   = rxp / interf;

        maxRxpContextCollector->put(rxp.get_dBm());
        maxSINRContextCollector->put(sinr.get_dB());
        minPathlossContextCollector->put(-1 * this->getQuasiStaticPathLoss((serving), 
            wns::service::phy::ofdma::PatternPtr()).get_dB());

		Transmitter<Sender>* transmitter = 
			dynamic_cast<Transmitter<Sender>*>((serving)->getTransmitter());
        wns::Position bsPosition = transmitter->getAntenna()->getPosition();
        distanceContextCollector->put((bsPosition - currentPosition).abs());

		MESSAGE_BEGIN(NORMAL, logger, m, "Selected " << transmitter->getStation()->getMyNode()->getName());
		m << "Number of candidates: " << candidates.size() << "\n"
		  << "SINR:" << sinr.get_dB() << "\n";
		MESSAGE_END();
    }
    else
    {
        maxRxpContextCollector->put(maxRxPwr.get_dBm());
        maxSINRContextCollector->put(maxSINR.get_dB());
        minPathlossContextCollector->put(-1 * minPathloss.get_dB());
        distanceContextCollector->put(assocdistance);
    }
}

wns::Power
Scanner::Receiver::getRxPower(const rise::TransmissionObjectPtr& to)
{
    return ofdmaphy::receiver::Receiver::getRxPower(to, wns::service::phy::ofdma::PatternPtr());
}

wns::service::phy::ofdma::PatternPtr
Scanner::Receiver::getCurrentReceivePattern(const rise::TransmissionObjectPtr& /*t*/)
{
	return  wns::service::phy::ofdma::PatternPtr();
}

void
Scanner::Receiver::initProbes(const wns::pyconfig::View& config)
{
    MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Scanner::Receiver initializing PutDecorators");

	wns::probe::bus::ContextProviderCollection localcpc(&scanner->getMyNode()->getContextProviderCollection());
	
	localcpc.addProvider(wns::probe::bus::contextprovider::Constant("MSID", scanner->getStationId()));

	localcpc.addProvider(wns::probe::bus::contextprovider::Callback("BSID", boost::bind(&Scanner::Receiver::Receiver::getBSID, this)));

	rxpContextCollector = wns::probe::bus::ContextCollectorPtr(
		new wns::probe::bus::ContextCollector(localcpc,
											  config.get<std::string>("rxpProbeName")));

	sinrContextCollector = wns::probe::bus::ContextCollectorPtr(
		new wns::probe::bus::ContextCollector(localcpc,
											  config.get<std::string>("sinrProbeName")));

    pathlossContextCollector = wns::probe::bus::ContextCollectorPtr(
        new wns::probe::bus::ContextCollector(localcpc,
                                              config.get<std::string>("pathlossProbeName")));
    
    maxRxpContextCollector = wns::probe::bus::ContextCollectorPtr(
        new wns::probe::bus::ContextCollector(localcpc,
                                              config.get<std::string>("maxRxpProbeName")));

    maxSINRContextCollector = wns::probe::bus::ContextCollectorPtr(
        new wns::probe::bus::ContextCollector(localcpc,
                                              config.get<std::string>("maxSINRProbeName")));

    minPathlossContextCollector = wns::probe::bus::ContextCollectorPtr(
        new wns::probe::bus::ContextCollector(localcpc,
                                              config.get<std::string>("minPathlossProbeName")));

    distanceContextCollector = wns::probe::bus::ContextCollectorPtr(
        new wns::probe::bus::ContextCollector(localcpc,
                                              config.get<std::string>("distanceProbeName")));
}

void Scanner::Receiver::setMargin(wns::Ratio m)
{
    margin = m;
    assure(margin >= wns::Ratio::from_dB(0.0), "Margin must be >= 0 dB");
}

int
Scanner::Receiver::getBSID()
{
	return bsID;
}


Scanner::Scanner(wns::node::Interface* _node, const wns::pyconfig::View& pyConfigView) :
	rise::Station(pyConfigView),
	wns::node::component::Component(_node, pyConfigView),
	logger(pyConfigView.get("logger")),
	systemManager(rise::MetaSystemManager::getInstance()->getSystemManagerBySystemName(pyConfigView.get<std::string>("systemManagerName"))),
	receiver(NULL)
{
  	assure(systemManager, "Was not able to acquire SystemManager");
}

void
Scanner::doStartup()
{
	wns::pyconfig::View pyConfigView = this->getConfig();
	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Scanner construction");
	systemManager->addStation(this);
	this->receiver = new Receiver(pyConfigView.getView("receiver", 0), this);
	this->receiver->initProbes(pyConfigView);
	this->receiver->setMargin(pyConfigView.get<wns::Ratio>("margin"));

	double rxFrequency = pyConfigView.get<double>("rxFrequency");
	double bandwidth = pyConfigView.get<double>("bandwidth");
	int numberOfSubCarrier = pyConfigView.get<int>("numberOfSubCarrier");

 	receiver->tune(rxFrequency,
				   bandwidth,
				   numberOfSubCarrier);

    MESSAGE_BEGIN(NORMAL, logger, m, "ofdmaphy::Scanner constructed: ");
    m << "#SC="<<numberOfSubCarrier<<", fRx="<<rxFrequency<<",  b="<<bandwidth;
    MESSAGE_END();
}

Scanner::~Scanner()
{
	delete receiver;
}

void
Scanner::onNodeCreated()
{
	this->setMobility( getService<rise::scenario::mobility::MobilityInterface*>("mobility"));
}

void
Scanner::onWorldCreated()
{
}


void
Scanner::onShutdown()
{
}

rise::SystemManager*
Scanner::getSystemManager() const
{
	return systemManager;
}

wns::node::Interface*
Scanner::getMyNode() const
{
	return this->getNode();
}



