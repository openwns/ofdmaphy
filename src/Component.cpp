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

#include <OFDMAPHY/Component.hpp>
#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/Manager.hpp>

using namespace ofdmaphy;

STATIC_FACTORY_REGISTER_WITH_CREATOR(
	Component,
	wns::node::component::Interface,
	"ofdmaphy.Component",
	wns::node::component::ConfigCreator
	);

Component::Component(
	wns::node::Interface* node,
	const wns::pyconfig::View& pyConfigView) :

	wns::node::component::Component(node, pyConfigView),
	logger(pyConfigView.get("logger")),
	station(new Station(this, pyConfigView.get("ofdmaStation")))
{
	dynamic_cast<SystemManager*>(station->getSystemManager())->addStation(node, station);
}

void
Component::doStartup()
{
	// register station as a PHY DataTransmissionService
	this->addService(
		this->getConfig().get<std::string>("dataTransmission"),
		station);

	this->addService(
		this->getConfig().get<std::string>("notification"),
		station);

	// new [rs] to support CQI measurements:
	this->addService(
		this->getConfig().get<std::string>("measurements"),
		station);

	MESSAGE_SINGLE(NORMAL, logger, "OFDMA PHY Component created");
	MESSAGE_SINGLE(VERBOSE, logger, "registered services: "
				   << this->getConfig().get<std::string>("dataTransmission") << ", "
				   << this->getConfig().get<std::string>("notification") << ", "
				   << this->getConfig().get<std::string>("measurements")
		);
}


Component::~Component()
{
	delete station;
}


void
Component::onNodeCreated()
{
	// some things we can only do after the mobility Service is present
	station->setMobility( getService<rise::scenario::mobility::MobilityInterface*>("mobility"));
	station->onNodeCreated();
}

void
Component::onWorldCreated()
{
	dynamic_cast<ofdmaphy::SystemManager*>(station->getSystemManager())->initAntennas(station);
}

void
Component::onShutdown()
{
}

wns::node::Interface*
Component::getNode() const
{
	return wns::node::component::Component::getNode();
}


