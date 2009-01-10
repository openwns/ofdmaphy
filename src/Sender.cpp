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
#include <OFDMAPHY/Transmitter.hpp>

#include <RISE/transmissionobjects/broadcasttransmissionobject.hpp>
#include <RISE/manager/metasystemmanager.hpp>
#include <RISE/plmapping/PhyMode.hpp>

#include <WNS/PowerRatio.hpp>
#include <WNS/node/component/Component.hpp>

using namespace ofdmaphy;

STATIC_FACTORY_REGISTER_WITH_CREATOR(
	Sender,
	wns::node::component::Interface,
	"ofdmaphy.Sender",
	wns::node::component::ConfigCreator
	);

Sender::Sender(wns::node::Interface* _node, const wns::pyconfig::View& pyConfigView) :
	rise::Station(pyConfigView),
	wns::node::component::Component(_node, pyConfigView),
	subBand(pyConfigView.get<int>("subBand")),
	logger(pyConfigView.get("logger")),
	systemManager(dynamic_cast<rise::SystemManager*>(rise::MetaSystemManager::getInstance()->getSystemManagerBySystemName(pyConfigView.get<std::string>("systemManagerName")))),
	transmitter(NULL),
	txPower(pyConfigView.get<wns::Power>("txPower"))
{
	assure(systemManager, "Was not able to acquire SystemManager");
}

void
Sender::doStartup()
{
	/**
	 * @todo in the future we need adjustable power per subBand [rs]
	 */
    MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Sender construction. total txPower="<<txPower);
	systemManager->addStation(this);

	this->transmitter = new Transmitter<Sender>(pyConfigView.getView("transmitter", 0), this, getAntenna());

	double txFrequency = pyConfigView.get<double>("txFrequency");
	double bandwidth = pyConfigView.get<double>("bandwidth");
	int numberOfSubCarrier = pyConfigView.get<int>("numberOfSubCarrier");

 	transmitter->tune(txFrequency,
					  bandwidth,
					  numberOfSubCarrier);

    MESSAGE_BEGIN(NORMAL, logger, m, "ofdmaphy::Sender constructed: ");
	m << "#SC="<<numberOfSubCarrier<<", fTx="<<txFrequency<<"MHz,  b="<<bandwidth<<"MHz, P="<<txPower;
    MESSAGE_END();
}

Sender::~Sender()
{
	delete transmitter;
}

void
Sender::onNodeCreated()
{
	this->setMobility( getService<rise::scenario::mobility::MobilityInterface*>("mobility"));
}

void
Sender::onWorldCreated()
{
	this->startTransmission();
}


void
Sender::onShutdown()
{
}


void
Sender::startTransmission()
{
	Broadcast bto(new rise::BroadcastTransmissionObject(transmitter,
														wns::osi::PDUPtr(),
														txPower,
														wns::service::phy::phymode::emptyPhyModePtr(),
														rise::TransmissionObject::downlink));

 	MESSAGE_SINGLE(NORMAL, logger, "ofdmaphy::Sender::startTransmission(subBand=" << subBand << ") Broadcast");
	transmitter->startTransmitting(bto, subBand);
}

rise::SystemManager*
Sender::getSystemManager() const
{
	return systemManager;
}

wns::node::Interface*
Sender::getMyNode() const
{
	return this->getNode();
}



