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

#ifndef OFDMAPHY_TESTS_OFDMASTATIONDROPIN_HPP
#define OFDMAPHY_TESTS_OFDMASTATIONDROPIN_HPP

#include <OFDMAPHY/tests/SystemManagerDropIn.hpp>

#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/Transmitter.hpp>
#include <RISE/transceiver/receiver.hpp>
#include <RISE/transceiver/transmitter.hpp>
#include <RISE/scenario/mobility/None.hpp>
#include <WNS/service/phy/ofdma/Measurements.hpp>
#include <WNS/service/phy/ofdma/MeasurementHandler.hpp>
#include <WNS/pyconfig/Parser.hpp>
#include <WNS/pyconfig/View.hpp>
#include <WNS/pyconfig/helper/Functions.hpp>
#include <WNS/node/tests/Stub.hpp>


namespace ofdmaphy { namespace tests {
	class SystemManagerDropIn;
	typedef Transmitter<Station> Transmitter;
	class OFDMAStationDropIn :
		public Station
	{
	public:
		OFDMAStationDropIn(SystemManagerDropIn* _sm) :
			Station(NULL, wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Station",
																			"OFDMAStationDropIn")),
			sm(_sm),
			transmitter2(NULL),
			receiver2(NULL),
			node(new wns::node::tests::Stub())
		{
			sm->addStation(node, this);

			wns::pyconfig::View mobilityView = wns::pyconfig::helper::createViewFromDropInConfig("rise.Mobility",
																								 "MobilityDropin");
			setMobility( new rise::scenario::mobility::None(mobilityView) );
			onNodeCreated();
		}

		virtual ~OFDMAStationDropIn() {}

		virtual void move()
		{}

		void set(rise::receiver::ReceiverInterface* r)
		{
			receiver2 = r;
		}

		void set(Transmitter* t)
		{
			transmitter2 = t;
		}

		SystemManagerDropIn* getSystemManager() const
		{
			return sm;
		}

		void
		receiveData(wns::osi::PDUPtr /*sdu*/, wns::Ratio /*r*/)
		{
			// do nothing in testReceiver
		}

		void
		receiveData(wns::osi::PDUPtr /*sdu*/, wns::Power /*c*/, wns::Power /*i*/, wns::Ratio /*oA*/)
		{
			// do nothing in testReceiver
		}

		void
		receiveData(wns::osi::PDUPtr /*sdu*/, wns::service::phy::power::PowerMeasurementPtr /*rxPowerMeasurementPtr*/)
		{
			// do nothing in testReceiver
		}

		wns::node::Interface*
		getNode()
		{
			return node;
		}

	private:
		SystemManagerDropIn* sm;
		Transmitter* transmitter2;
		rise::receiver::ReceiverInterface* receiver2;
		wns::node::Interface* node;

	};
}
}


#endif // OFDMAPHY_TESTS_OFDMASTATIONDROPIN_HPP



