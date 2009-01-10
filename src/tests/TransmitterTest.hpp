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

#ifndef OFDMAPHY_TESTS_OFDMATRANSMITTERTEST_HPP
#define OFDMAPHY_TESTS_OFDMATRANSMITTERTEST_HPP

#include <OFDMAPHY/tests/SystemManagerDropIn.hpp>
#include <OFDMAPHY/tests/OFDMAStationDropIn.hpp>
#include <OFDMAPHY/Transmitter.hpp>
#include <OFDMAPHY/Receiver.hpp>

#include <WNS/node/Node.hpp>
#include <WNS/service/phy/ofdma/DataTransmission.hpp>
#include <WNS/TestFixture.hpp>

namespace ofdmaphy { namespace tests {
	class TransmitterTest
		: public wns::TestFixture
	{
		CPPUNIT_TEST_SUITE( TransmitterTest );

		CPPUNIT_TEST( testGetRxPowerBF );
		CPPUNIT_TEST( testAnalogToBeamformingTest );
		CPPUNIT_TEST_SUITE_END();
	public:
		TransmitterTest();
		~TransmitterTest();
		void prepare();
		void cleanup();
		void testGetRxPowerBF();
		void testAnalogToBeamformingTest();
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
	};
}}

#endif // OFDMAPHY_TESTS_TRANSMITTERTEST_HPP


