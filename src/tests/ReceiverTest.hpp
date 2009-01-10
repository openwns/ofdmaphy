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

#ifndef OFDMAPHY_TESTS_OFDMATEST_HPP
#define OFDMAPHY_TESTS_OFDMATEST_HPP

#include <OFDMAPHY/tests/SystemManagerDropIn.hpp>
#include <OFDMAPHY/tests/OFDMAStationDropIn.hpp>
#include <OFDMAPHY/Transmitter.hpp>
#include <OFDMAPHY/Receiver.hpp>

#include <WNS/node/Node.hpp>
#include <WNS/TestFixture.hpp>

namespace ofdmaphy { namespace tests {
	class OFDMATest
		: public wns::TestFixture
	{
		CPPUNIT_TEST_SUITE( OFDMATest );

		CPPUNIT_TEST( testGetNoise );
		CPPUNIT_TEST( testGetRxPower );
		CPPUNIT_TEST( testGetRxPowerBF );
		CPPUNIT_TEST( testAnalogToBeamformingTest );
		CPPUNIT_TEST( testGetAllRxPower );
		CPPUNIT_TEST( testGetInterference );
		CPPUNIT_TEST( testNotify );
		CPPUNIT_TEST( testMobilityUpdate );
		CPPUNIT_TEST( testPositionChange );
		CPPUNIT_TEST( testSetGetReceivePattern );
		CPPUNIT_TEST_SUITE_END();
	public:
		OFDMATest();
		~OFDMATest();
		void prepare();
		void cleanup();
		void testGetNoise();
 		void testGetRxPower();
 		void testGetRxPowerBF();
 		void testAnalogToBeamformingTest();
		void testGetAllRxPower();
 		void testGetInterference();
 		void testNotify();
 		void testMobilityUpdate();
 		void testPositionChange();
 		void testSetGetReceivePattern();
	private:
		SystemManagerDropIn* systemManagerDropIn;
		OFDMAStationDropIn* station1;
 		OFDMAStationDropIn* station2;
 		OFDMAStationDropIn* station3;
 		OFDMAStationDropIn* station4;
		Receiver* ofdma1;
 		Receiver* ofdma2;
 		Receiver* ofdma3;
		Transmitter* trans1;
 		Transmitter* trans2;
 		Transmitter* trans3;
 		Transmitter* trans4;
 		Transmitter* trans5;
	};
} // tests
} // ofdmaphy

#endif // NOT defined OFDMAPHY_TESTS_OFDMATEST_HPP


