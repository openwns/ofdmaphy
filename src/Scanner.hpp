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

#ifndef OFDMAPHY_SCANNER_HPP
#define OFDMAPHY_SCANNER_HPP

#include <OFDMAPHY/Receiver.hpp>

#include <RISE/antenna/Antenna.hpp>
#include <RISE/stations/station.hpp>
#include <RISE/manager/systemmanager.hpp>

#include <WNS/node/component/Component.hpp>
#include <WNS/logger/Logger.hpp>
#include <WNS/PositionObserver.hpp>
#include <WNS/probe/bus/ContextCollector.hpp>

namespace ofdmaphy {

	class Scanner:
		public rise::Station,
		public wns::node::component::Component
	{
		typedef rise::BroadcastTransmissionObjectPtr Broadcast;

		class Receiver :
			public ofdmaphy::Receiver
		{
			typedef std::list<rise::TransmissionObjectPtr> TOList;
		public:
			Receiver(const wns::pyconfig::View& config, Scanner* s);

			virtual
			~Receiver();

			virtual void
			notify(rise::TransmissionObjectPtr to);

			virtual void
			positionChanged();

			virtual wns::Power
			getRxPower(const rise::TransmissionObjectPtr& to);

			virtual wns::service::phy::ofdma::PatternPtr
			getCurrentReceivePattern(const rise::TransmissionObjectPtr&);

			void
			initProbes(const wns::pyconfig::View&);

			int
			getBSID();
		private:
			Scanner* scanner;

			TOList transmissions;

			int bsID;

			wns::probe::bus::ContextCollectorPtr rxpContextCollector;
			wns::probe::bus::ContextCollectorPtr sinrContextCollector;
            wns::probe::bus::ContextCollectorPtr pathlossContextCollector;
            wns::probe::bus::ContextCollectorPtr maxRxpContextCollector;
            wns::probe::bus::ContextCollectorPtr maxSINRContextCollector;
            wns::probe::bus::ContextCollectorPtr minPathlossContextCollector;
            wns::probe::bus::ContextCollectorPtr losNLOSRatioContextCollector;
		};

	public:
		Scanner(wns::node::Interface* node, const wns::pyconfig::View& pyConfigView);

		virtual
		~Scanner();

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
		virtual void receiveData(wns::osi::PDUPtr, wns::Power, wns::Power, wns::Ratio)
		{
		}

		virtual void
		doStartup();

		wns::logger::Logger logger;
		rise::SystemManager* systemManager;
		Receiver* receiver;
	};
} // ofdmaphy

#endif // NOT defined OFDMAPHY_SENDER_HPP


