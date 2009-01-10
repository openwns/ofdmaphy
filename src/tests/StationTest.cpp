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

#include <OFDMAPHY/tests/SystemManagerDropIn.hpp>
#include <OFDMAPHY/tests/OFDMAStationDropIn.hpp>
#include <OFDMAPHY/Transmitter.hpp>
#include <OFDMAPHY/Receiver.hpp>

#include <RISE/medium/Medium.hpp>

#include <WNS/node/Node.hpp>
#include <WNS/TestFixture.hpp>

namespace ofdmaphy { namespace tests {
	class StationTest
		: public wns::TestFixture
	{
		CPPUNIT_TEST_SUITE( StationTest );
		CPPUNIT_TEST( testSetGetTune );
		CPPUNIT_TEST( testSetSwap );
		CPPUNIT_TEST_SUITE_END();

		SystemManagerDropIn* systemManagerDropIn;
		OFDMAStationDropIn* station1;
		Receiver* ofdma1;
		Transmitter* trans1;
	public:
		StationTest(){};
		~StationTest(){};

		void prepare()
		{
			rise::medium::Medium::getInstance()->reset();
			systemManagerDropIn = new SystemManagerDropIn();
			station1 = new OFDMAStationDropIn(systemManagerDropIn);
			ofdma1   = new Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
						station1);
			trans1   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), 
						   station1, station1->getAntenna());
			station1->set(ofdma1);
			station1->set(trans1);
			ofdma1->removeAll();
			wns::simulator::getEventScheduler()->reset();

		}

		void cleanup()
		{
			delete systemManagerDropIn;
		}

		void testSetGetTune()
		{
			wns::service::phy::ofdma::Tune txTune;
			txTune.frequency = 5000;
			txTune.bandwidth = 100;
			txTune.numberOfSubCarrier = 128;

			wns::service::phy::ofdma::Tune rxTune;
			rxTune.frequency = 4000;
			rxTune.bandwidth = 100;
			rxTune.numberOfSubCarrier = 128;

			station1->setTxTune(txTune);
			CPPUNIT_ASSERT( txTune == station1->getTxTune() );
			station1->setRxTune(rxTune);
			CPPUNIT_ASSERT( rxTune == station1->getRxTune() );
		}

		void testSetSwap()
		{
			wns::service::phy::ofdma::Tune txTune;
			txTune.frequency = 5000;
			txTune.bandwidth = 100;
			txTune.numberOfSubCarrier = 128;

			wns::service::phy::ofdma::Tune rxTune;
			rxTune.frequency = 4000;
			rxTune.bandwidth = 100;
			rxTune.numberOfSubCarrier = 128;

			station1->setTxTune(txTune);
			CPPUNIT_ASSERT( txTune == station1->getTxTune() );
			station1->setRxTune(rxTune);
			CPPUNIT_ASSERT( rxTune == station1->getRxTune() );

			station1->setTxRxSwap(true);
			CPPUNIT_ASSERT( txTune == station1->getRxTune() );
			CPPUNIT_ASSERT( rxTune == station1->getTxTune() );
			station1->setTxRxSwap(true); // this shouldn't change anything
			CPPUNIT_ASSERT( txTune == station1->getRxTune() );
			CPPUNIT_ASSERT( rxTune == station1->getTxTune() );
			station1->setTxRxSwap(false); // change back to normal
		}
	};
}}

CPPUNIT_TEST_SUITE_REGISTRATION( ofdmaphy::tests::StationTest );



