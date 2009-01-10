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

#ifndef OFDMAPHY_SENDER_HPP
#define OFDMAPHY_SENDER_HPP

#include <OFDMAPHY/Transmitter.hpp>

#include <RISE/antenna/Antenna.hpp>
#include <RISE/stations/station.hpp>
#include <RISE/manager/systemmanager.hpp>

#include <WNS/node/component/Component.hpp>
#include <WNS/logger/Logger.hpp>

namespace ofdmaphy {

	class Sender:
		public rise::Station,
		public wns::node::component::Component
	{
		typedef rise::BroadcastTransmissionObjectPtr Broadcast;

	public:
		Sender(wns::node::Interface* node, const wns::pyconfig::View& pyConfigView);

		virtual
		~Sender();

		virtual void
		onNodeCreated();

		virtual void
		onWorldCreated();

		virtual void
		onShutdown();

		virtual rise::SystemManager*
		getSystemManager() const;

		virtual wns::node::Interface*
		getMyNode() const;

	private:
		virtual void
		doStartup();

		virtual void
 		startTransmission();

		//! receive data and forward back to higher layers
		virtual void receiveData(wns::osi::PDUPtr, wns::Power, wns::Power, wns::Ratio)
		{
		}

		int subBand;

		wns::logger::Logger logger;
		rise::SystemManager* systemManager;
		Transmitter<Sender>* transmitter;
		wns::Power txPower;
	};
} // ofdmaphy

#endif // NOT defined OFDMAPHY_SENDER_HPP


