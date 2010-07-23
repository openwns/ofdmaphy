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

#include "TransmitterTest.hpp"

#include <RISE/medium/Medium.hpp>
#include <RISE/transmissionobjects/broadcasttransmissionobject.hpp>
#include <RISE/transmissionobjects/transmissionobjectbf.hpp>
#include <RISE/medium/PhysicalResource.hpp>
#include <WNS/events/NoOp.hpp>

#include <WNS/node/tests/Stub.hpp>
#include <WNS/scheduler/SchedulerTypes.hpp>
#include <WNS/TestFixture.hpp>
#include <WNS/pyconfig/Parser.hpp>
#include <WNS/SmartPtr.hpp>

using namespace ofdmaphy;
using namespace ofdmaphy::tests;

CPPUNIT_TEST_SUITE_REGISTRATION( TransmitterTest );

TransmitterTest::TransmitterTest() :
	wns::TestFixture(),
	systemManagerDropIn(NULL),
	station1(NULL),
 	station2(NULL),
 	station3(NULL),
 	station4(NULL),
	ofdma1(NULL),
	ofdma2(NULL),
 	ofdma3(NULL),
	trans1(NULL),
 	trans2(NULL),
 	trans3(NULL)
{
}

TransmitterTest::~TransmitterTest()
{
}

void TransmitterTest::prepare()
{
	rise::medium::Medium::getInstance()->reset();

	systemManagerDropIn = new SystemManagerDropIn();
	station1 = new OFDMAStationDropIn(systemManagerDropIn);
 	station2 = new OFDMAStationDropIn(systemManagerDropIn);
 	station3 = new OFDMAStationDropIn(systemManagerDropIn);
 	station4 = new OFDMAStationDropIn(systemManagerDropIn);
    ofdma1   = new receiver::Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
				station1);
	ofdma2   = new receiver::Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
				station2);
	ofdma3   = new receiver::Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
				station3);
	trans1   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station1, station1->getAntenna());
 	trans2   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station2, station2->getAntenna());
 	trans3   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station3, station3->getAntenna());

	station1->set(ofdma1);
	station1->set(trans1);

	station2->set(ofdma2);
	station2->set(trans2);

	station3->set(ofdma3);
	station3->set(trans3);

	ofdma1->removeAll();
	ofdma2->removeAll();
	ofdma3->removeAll();
	wns::simulator::getEventScheduler()->reset();
	//if you are testing beamforming antennas, be sure that the setup is not
	//symetric to the y-axis!!!
	station1->moveTo(wns::Position(100,100,0));
	station2->moveTo(wns::Position(200,200,0));
	station3->moveTo(wns::Position(130,70,0));
	ofdma1->tune(5000, 100, 2048);
	ofdma2->tune(5000, 100, 2048);
	ofdma3->tune(5000, 100, 2048);
}

void TransmitterTest::cleanup()
{
	delete systemManagerDropIn;
}

void TransmitterTest::testGetRxPowerBF()
{
	std::map<wns::node::Interface*, rise::antenna::PatternPtr> currentReceivePatterns;
	wns::Power txPower = wns::Power::from_dBm(20);
	// tune transmitters
	trans1->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans3->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	// tune station1 (I don't know why)
	wns::service::phy::ofdma::Tune txTune;
	txTune.frequency = 5000-50+(100/2048.0)/2;
	txTune.bandwidth = 100/2048.0;
	txTune.numberOfSubCarrier = 1;
	station1->setTxTune(txTune);

	// assure that the received power for both stations is known at the
	// receiver, so start two transmissions
	rise::BroadcastTransmissionObjectPtr bto2(
		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), txPower));
	trans2->startTransmitting(bto2, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getRxPower(bto2).get_dBm(), 1E-5);
	trans2->stopTransmitting(bto2);
	// sent a second bto
	rise::BroadcastTransmissionObjectPtr bto3(
		new rise::BroadcastTransmissionObject(trans3, wns::osi::PDUPtr(), txPower));
	trans3->startTransmitting(bto3, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();
	//station 3 is closer so the RxPower is higher
	WNS_ASSERT_MAX_REL_ERROR(-58.8849, ofdma1->getRxPower(bto3).get_dBm(), 1E-5);
	trans3->stopTransmitting(bto3);

	//optimize pattern at receiving station 1 for station 2
	std::vector<wns::node::Interface*> undesired2;
	undesired2.push_back(station3->getNode());
	rise::antenna::PatternPtr pattern2;
	pattern2 = station1->calculateAndSetBeam(station2->getNode(), undesired2, ofdma1->getNoise(0));

	//start first SDMA transmission
	wns::SmartPtr<rise::TransmissionObjectBF> bfto4(
			new rise::TransmissionObjectBF(trans1,
						       ofdma2,
						       station1->getBFAntenna(),
						       wns::osi::PDUPtr(),
						       txPower,
						       pattern2));
	trans1->startTransmitting(bfto4, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	WNS_ASSERT_MAX_REL_ERROR(-64.3654, ofdma2->getRxPower(bfto4).get_dBm(), 1E-3);
	// Rx power is higher with adaptive pattern due to receive antenna gain
 	WNS_ASSERT_MAX_REL_ERROR(ofdma2->getNoise(0).get_dBm(), ofdma2->getInterference(bfto4).get_dBm(), 1E-5);

  	trans1->stopTransmitting(bfto4);

 	//start second reception
	//optimize pattern at transmitting station 1 for station 2
	std::vector<wns::node::Interface*> undesired3;
	undesired3.push_back(station2->getNode());
	rise::antenna::PatternPtr pattern3;
	pattern3 = station1->calculateAndSetBeam(station3->getNode(), undesired3, ofdma1->getNoise(0));

	wns::SmartPtr<rise::TransmissionObjectBF> bfto5(
			new rise::TransmissionObjectBF(trans1,
						       ofdma3,
						       station1->getBFAntenna(),
						       wns::osi::PDUPtr(),
						       txPower,
						       pattern3));
 	trans1->startTransmitting(bfto5, 0);
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();
 	// due to antenna gain in look direction, receive power is increased compared
 	// to omni transmission
 	WNS_ASSERT_MAX_REL_ERROR(-53.9078, ofdma3->getRxPower(bfto5).get_dBm(), 1E-5);
  	//interference is very close to noise (-127.113dB)
  	WNS_ASSERT_MAX_REL_ERROR(ofdma3->getNoise(0).get_dBm(), ofdma3->getInterference(bfto5).get_dBm(), 1E-5);

 	trans1->stopTransmitting(bfto5);

	//start the same two transmissions in SDMA mode
	wns::osi::PDUPtr pdu2 = wns::osi::PDUPtr(new wns::osi::PDU());
	wns::node::Interface* recipient2 = station2->getNode();
	wns::osi::PDUPtr pdu3 = wns::osi::PDUPtr(new wns::osi::PDU());
	wns::node::Interface* recipient3 = station3->getNode();

	// generate grouping that consists of one group with station 2 and
	// station 3
	wns::scheduler::Grouping grouping;
	wns::scheduler::Group myGroup;
    myGroup[wns::scheduler::UserID(station2->getNode())] = wns::CandI();
    myGroup[wns::scheduler::UserID(station3->getNode())] = wns::CandI();
	grouping.groups.push_back(myGroup);
    grouping.userGroupNumber[wns::scheduler::UserID(station2->getNode())] = 0;
    grouping.userGroupNumber[wns::scheduler::UserID(station3->getNode())] = 0;
    grouping.patterns[wns::scheduler::UserID(station2->getNode())] = pattern2;
    grouping.patterns[wns::scheduler::UserID(station3->getNode())] = pattern3;

	WNS_ASSERT_MAX_REL_ERROR( 0.5 , grouping.shareOfPowerPerStreams(0).get_factor() , 1E-3 );
	WNS_ASSERT_MAX_REL_ERROR( 0.5 , grouping.shareOfPowerPerStreams(myGroup).get_factor() , 1E-3 );
	WNS_ASSERT_MAX_REL_ERROR( grouping.eirpReductionOfPower(0).get_factor() ,grouping.eirpReductionOfPower(myGroup).get_factor() , 1E-5 );
	WNS_ASSERT_MAX_REL_ERROR( 0.1898 ,grouping.eirpReductionOfPower(myGroup).get_factor() , 1E-3);

 	// test reduction due to concurrent streams
	// implemented in struct Grouping and scheduling strategies
	wns::Power txPowerPerStream = txPower;
	txPowerPerStream += grouping.shareOfPowerPerStreams(0);

 	station1->startTransmission(pdu2, recipient2, 0, pattern2, txPowerPerStream );
 	station1->startTransmission(pdu3, recipient3, 0, pattern3, txPowerPerStream );
  	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
  	wns::simulator::getEventScheduler()->processOneEvent();

 	// 3dB loss due to half of the Tx power
  	WNS_ASSERT_MAX_REL_ERROR(-67.375, station1->getCurrentCandI(pdu2).C.get_dBm(), 1E-3);
  	WNS_ASSERT_MAX_REL_ERROR(-56.918, station1->getCurrentCandI(pdu3).C.get_dBm(), 1E-3);
 	station1->stopTransmission(pdu2, 0);
 	station1->stopTransmission(pdu3, 0);

 	// test reduction due to concurrent streams and additional EIRP restrictions
	txPowerPerStream = txPower;
	txPowerPerStream += grouping.shareOfPowerPerStreams(myGroup);
	txPowerPerStream += grouping.eirpReductionOfPower(myGroup);

 	station1->startTransmission(pdu2, recipient2, 0, pattern2, txPowerPerStream );
 	station1->startTransmission(pdu3, recipient3, 0, pattern3, txPowerPerStream );
  	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
  	wns::simulator::getEventScheduler()->processOneEvent();

 	// loss due to missing antenna gain
  	WNS_ASSERT_MAX_REL_ERROR(-74.592, station1->getCurrentCandI(pdu2).C.get_dBm(), 1E-3);
  	WNS_ASSERT_MAX_REL_ERROR(-64.134, station1->getCurrentCandI(pdu3).C.get_dBm(), 1E-3);
 	station1->stopTransmission(pdu2, 0);
 	station1->stopTransmission(pdu3, 0);
}

void TransmitterTest::testAnalogToBeamformingTest()
{
	std::map<wns::node::Interface*, wns::Power> stack2NoisePlusIintercell;
	std::map<wns::node::Interface*, wns::Ratio> sinrs;
	wns::Power txPower = wns::Power::from_dBm(-40);
	wns::Power noise = wns::Power::from_dBm(-93);
	wns::Power x_friendlyness = noise;
	wns::Power zero = wns::Power::from_mW(0.0);
	rise::antenna::PatternPtr pid2, pid3, pid4;
	std::vector<wns::node::Interface*> undesired;

	station1->moveTo(wns::Position(0,0,0));
	station2->moveTo(wns::Position(30,20,0));
	station3->moveTo(wns::Position(100,10,0));
	station4->moveTo(wns::Position(40,100,0));

	//the RxPower of the stations is measured durung reception of first packets
	station1->getBFAntenna()->setPowerReceivedForStation(station2, wns::Power::from_dBm(-70)); //pathloss of 30dB
	station1->getBFAntenna()->setPowerReceivedForStation(station3, wns::Power::from_dBm(-50)); //pathloss of 10dB
	station1->getBFAntenna()->setPowerReceivedForStation(station4, wns::Power::from_dBm(-60)); //pathloss of 20dB 

	//the TxPower of the station is needed for the estimation
	station1->getBFAntenna()->setTxPowerForStation(station2, txPower);
	station1->getBFAntenna()->setTxPowerForStation(station3, txPower);
	station1->getBFAntenna()->setTxPowerForStation(station4, txPower);

	// only station 2 with noise
	stack2NoisePlusIintercell.clear();
	stack2NoisePlusIintercell[station2->getNode()] = noise;

	sinrs = station1->calculateSINRsTx(stack2NoisePlusIintercell, x_friendlyness, txPower);
	CPPUNIT_ASSERT( sinrs.find(station2->getNode()) != sinrs.end());
	// (-40dBm-30dB+7dBi)-(-93dBm) approx. 30dB
 	WNS_ASSERT_MAX_REL_ERROR((* sinrs.find(station2->getNode()) ).second.get_dB(), 29.8498, 1E-3); 

	// three stations in SDMA without noise
	stack2NoisePlusIintercell.clear();
	stack2NoisePlusIintercell[station2->getNode()] = zero;
	stack2NoisePlusIintercell[station3->getNode()] = zero;
	stack2NoisePlusIintercell[station4->getNode()] = zero;
	sinrs = station1->calculateSINRsTx(stack2NoisePlusIintercell, x_friendlyness, txPower);

	CPPUNIT_ASSERT(sinrs.find(station2->getNode()) != sinrs.end());
 	CPPUNIT_ASSERT_DOUBLES_EQUAL(38.29, (* sinrs.find(station2->getNode())).second.get_dB(), 1E-2);

	CPPUNIT_ASSERT(sinrs.find(station3->getNode()) != sinrs.end());
	CPPUNIT_ASSERT_DOUBLES_EQUAL(35.87, (* sinrs.find(station3->getNode()) ).second.get_dB(), 1E-2);

	CPPUNIT_ASSERT(sinrs.find(station4->getNode()) != sinrs.end());
	CPPUNIT_ASSERT_DOUBLES_EQUAL(42.75, (* sinrs.find(station4->getNode()) ).second.get_dB(), 1E-2);


	undesired.clear();
 	undesired.push_back(station3->getNode());
	undesired.push_back(station4->getNode());

//	CPPUNIT_ASSERT( station1->getBFAntenna()->getNumBeams() == 0 );
	pid2 = station1->calculateAndSetBeam(station2->getNode(), undesired, noise);
//	CPPUNIT_ASSERT( station1->getBFAntenna()->getNumBeams() == 1 );
//	CPPUNIT_ASSERT( station1->getBFAntenna()->isBeamSet(pid2) );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station2->getBFAntenna()->getPosition(), pid2).get_factor() > 0.9 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station3->getBFAntenna()->getPosition(), pid2).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station4->getBFAntenna()->getPosition(), pid2).get_factor() < 0.1 );

	undesired.clear();
 	undesired.push_back(station2->getNode());
	undesired.push_back(station4->getNode());
	pid3 = station1->calculateAndSetBeam(station3->getNode(), undesired, noise);
//	CPPUNIT_ASSERT( station1->getBFAntenna()->getNumBeams() == 2 );
//	CPPUNIT_ASSERT( station1->getBFAntenna()->isBeamSet(pid3) );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station2->getBFAntenna()->getPosition(), pid3).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station3->getBFAntenna()->getPosition(), pid3).get_factor() > 0.9 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station4->getBFAntenna()->getPosition(), pid3).get_factor() < 0.1 );

	undesired.clear();
 	undesired.push_back(station2->getNode());
	undesired.push_back(station3->getNode());
	pid4 = station1->calculateAndSetBeam(station4->getNode(), undesired, noise);
//	CPPUNIT_ASSERT( station1->getBFAntenna()->getNumBeams() == 3 );
//	CPPUNIT_ASSERT( station1->getBFAntenna()->isBeamSet(pid4) );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station2->getBFAntenna()->getPosition(), pid4).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station3->getBFAntenna()->getPosition(), pid4).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station4->getBFAntenna()->getPosition(), pid4).get_factor() > 0.86 );
	//exact value is 0.861857

// 	station1->getBFAntenna()->removeBeam(pid2);
// 	station1->getBFAntenna()->removeBeam(pid3);
// 	station1->getBFAntenna()->removeBeam(pid4);
// 	CPPUNIT_ASSERT( station1->getBFAntenna()->getNumBeams() == 0 );

}


