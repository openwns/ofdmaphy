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

#include "ReceiverTest.hpp"

#include <RISE/medium/Medium.hpp>
#include <RISE/transmissionobjects/broadcasttransmissionobject.hpp>
#include <RISE/medium/PhysicalResource.hpp>
#include <RISE/plmapping/PhyMode.hpp>
#include <WNS/events/NoOp.hpp>

#include <WNS/TestFixture.hpp>
#include <WNS/pyconfig/Parser.hpp>
#include <WNS/pyconfig/helper/Functions.hpp>
#include <WNS/node/tests/Stub.hpp>

#include <iostream>

using namespace ofdmaphy;
using namespace ofdmaphy::tests;

CPPUNIT_TEST_SUITE_REGISTRATION( OFDMATest );

OFDMATest::OFDMATest() :
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
 	trans3(NULL),
 	trans4(NULL),
 	trans5(NULL)
{
}

OFDMATest::~OFDMATest()
{
}

void OFDMATest::prepare()
{
	rise::medium::Medium::getInstance()->reset();

	//wns::service::phy::phymode::PhyModeInterface* phyModePtr;

	systemManagerDropIn = new SystemManagerDropIn();
	station1 = new OFDMAStationDropIn(systemManagerDropIn);
 	station2 = new OFDMAStationDropIn(systemManagerDropIn);
 	station3 = new OFDMAStationDropIn(systemManagerDropIn);
 	station4 = new OFDMAStationDropIn(systemManagerDropIn);
	ofdma1   = new Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
				station1);
	ofdma2   = new Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
				station2);
	ofdma3   = new Receiver(wns::pyconfig::helper::createViewFromDropInConfig("ofdmaphy.Receiver", "ReceiverDropIn"),
				station3);
	trans1   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station1, station1->getAntenna());
 	trans2   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station2, station2->getAntenna());
 	trans3   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station3, station3->getAntenna());
 	trans4   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station4, station4->getAntenna());
 	trans5   = new Transmitter(wns::pyconfig::helper::createViewFromDropInConfig("rise.Transmitter", "TransmitterDropIn"), station2, station2->getAntenna());
 	station1->set(ofdma1);
 	station1->set(trans1);

  	station2->set(ofdma2);
  	station2->set(trans2);
  	station2->set(trans5);

 	station3->set(ofdma3);
 	station3->set(trans3);

 	station4->set(trans4);
	ofdma1->removeAll();
 	ofdma2->removeAll();
 	ofdma3->removeAll();

	wns::simulator::getEventScheduler()->reset();

	//if you are testing linear beamforming antennas, be sure that the setup is not
	//symetric to the y-axis!!!
	station1->moveTo(wns::Position(100,100,0));
 	station2->moveTo(wns::Position(200,200,0));
 	station3->moveTo(wns::Position(350,110,0));
 	ofdma1->tune(5000, 100, 2048);
}

void OFDMATest::cleanup()
{
	delete systemManagerDropIn;
}

void OFDMATest::testGetNoise()
{
	// assume -174 dBm / Hz
 	double bgNoiseForOneCarrier = -127.113299;
 	for(int i=0; i < 2048; ++i) {
  		WNS_ASSERT_MAX_REL_ERROR(bgNoiseForOneCarrier, ofdma1->getNoise(i).get_dBm(), 1E-5);
 	}
}

void OFDMATest::testGetRxPower()
{
	// tune two transmitters to one frequency
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	// start a transmission
	rise::BroadcastTransmissionObjectPtr bto(
		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));

	trans2->startTransmitting(bto, 0);

	// move forward in time
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	// check power
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getRxPower(bto).get_dBm(), 1E-5);

	// sent a second bto, ten times the power, same frequency
	rise::BroadcastTransmissionObjectPtr bto2(
		new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));
	trans5->startTransmitting(bto2, 0);

	// move forward in time
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	// check power
	WNS_ASSERT_MAX_REL_ERROR(-59.3424, ofdma1->getRxPower(bto2).get_dBm(), 1E-5);
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getRxPower(bto).get_dBm(), 1E-5);

	// stop transmiitting bto2
	trans5->stopTransmitting(bto2);

	// send a third bto, ten times the power, other frequency
	rise::BroadcastTransmissionObjectPtr bto3(
		new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));

	// retune transmitter
	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans5->startTransmitting(bto3, 0);

	// check power
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getRxPower(bto).get_dBm(), 1E-5);
	WNS_ASSERT_MAX_REL_ERROR(-59.3424, ofdma1->getRxPower(bto3).get_dBm(), 1E-5);

	// move forward in time (must proceed, can't remove transmission objects
	// if no time has elapsed)
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	// remove transmission objects from physical resource
	trans5->stopTransmitting(bto3);
	trans2->stopTransmitting(bto);
}

void OFDMATest::testGetRxPowerBF()
{
	std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> currentReceivePatterns;
	wns::Power txPower = wns::Power::from_dBm(20);
	wns::Power noise = ofdma1->getNoise(0); // noise equals -127.113dBm
	wns::Power x_friendlyness = noise;
	std::vector<wns::node::Interface*> combination;

	// tune transmitters of two different stations to one frequency
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1); // Transmitter::tune(f=4950.02,b=0.0488281,#SC=1)
	trans3->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	// assure that the received power for both stations is known at the
	// receiver, so start two transmissions
	rise::BroadcastTransmissionObjectPtr bto2(
		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), txPower));
	trans2->startTransmitting(bto2, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent(); // now = 0.5
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getRxPower(bto2).get_dBm(), 1E-5);
	trans2->stopTransmitting(bto2);
	// sent a second bto
	rise::BroadcastTransmissionObjectPtr bto3(
		new rise::BroadcastTransmissionObject(trans3, wns::osi::PDUPtr(), txPower));
	trans3->startTransmitting(bto3, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent(); // now = 1.0
	WNS_ASSERT_MAX_REL_ERROR(-74.2979, ofdma1->getRxPower(bto3).get_dBm(), 1E-5);
	trans3->stopTransmitting(bto3);

// only station 2 with noise
	//estimate SINR 
	//first the TxPower of the stations must be known at the beamforming antenna
	station1->setTxPowerForStation(station2->getNode(), txPower);
	station1->setTxPowerForStation(station3->getNode(), txPower);

//estimate SINR directly at Antenna
	std::vector<rise::Station*> combinationStations;
	combinationStations.push_back(station2);
	std::map<rise::Station*, wns::CandI> candisStations;
	candisStations = station1->getBFAntenna()->calculateCandIsRx(combinationStations, noise);
	CPPUNIT_ASSERT(candisStations.find(station2) != candisStations.end());
 	WNS_ASSERT_MAX_REL_ERROR(63.246, ((*candisStations.find(station2)).second.C / (*candisStations.find(station2)).second.I).get_dB(), 1E-3);

	//optimize pattern at receiving station 1 for station 2
	std::vector<wns::node::Interface*> undesired2;
	undesired2.push_back(station3->getNode());
	wns::service::phy::ofdma::PatternPtr pattern2;
	pattern2 = station1->calculateAndSetBeam(station2->getNode(), undesired2, noise);

        //set receive pattern for station 2
	currentReceivePatterns.clear();
	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> (station2->getNode(), pattern2));
	ofdma1->setCurrentReceivePatterns(currentReceivePatterns);
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station2->getNode()) == pattern2);
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station3->getNode()) == wns::service::phy::ofdma::PatternPtr());

	//start receiving
	rise::BroadcastTransmissionObjectPtr bto4(
		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), txPower));
	trans2->startTransmitting(bto4, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent(); // now = 1.5

 	WNS_ASSERT_MAX_REL_ERROR(noise.get_dBm(), ofdma1->getInterference(bto4).get_dBm(), 1E-5);

	//double OLD_rxPower = ofdma1->OLD_getRxPower(bto4).get_dBm();
	//WNS_ASSERT_MAX_REL_ERROR(-64.1138, OLD_rxPower, 1E-3); // -97.074 ???
	double rxPower = ofdma1->getRxPower(bto4).get_dBm();
	WNS_ASSERT_MAX_REL_ERROR(-64.1138, rxPower, 1E-3); // -97.074 ???

	//check estimated value (estimate=57.7709 dB, measurement=57.7583 dB)
 	WNS_ASSERT_MAX_REL_ERROR( ((*candisStations.find(station2)).second.C / (*candisStations.find(station2)).second.I).get_dB(), (ofdma1->getRxPower(bto4).get_dBm() - ofdma1->getInterference(bto4).get_dBm()), 1E-2);

	//start second parallel reception without adaptive pattern
	rise::BroadcastTransmissionObjectPtr bto5(
		new rise::BroadcastTransmissionObject(trans3, wns::osi::PDUPtr(), txPower));
	trans3->startTransmitting(bto5, 0);
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent(); // now = 2.0

	WNS_ASSERT_MAX_REL_ERROR(-64.1138, ofdma1->getRxPower(bto4).get_dBm(), 1E-5); //optimized reception
	WNS_ASSERT_MAX_REL_ERROR(-74.2979, ofdma1->getRxPower(bto5).get_dBm(), 1E-5); //omni reception

 	//optimal interference is close to thermal noise (-127.113dB)
 	WNS_ASSERT_MAX_REL_ERROR(-117.555, ofdma1->getInterference(bto4).get_dBm(), 1E-3);
	//non-adaptive interference should be as large as (ofdma1->getRxPower(bto4, NULL) + noise
 	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getInterference(bto5).get_dBm(), 1E-5);

 	trans2->stopTransmitting(bto4);
	trans3->stopTransmitting(bto5);


// station 2 and 3 in SDMA
//estimate SINR directly at Antenna
	combinationStations.clear();
	combinationStations.push_back(station2);
	combinationStations.push_back(station3);
	candisStations = station1->getBFAntenna()->calculateCandIsRx(combinationStations, noise);
	CPPUNIT_ASSERT( candisStations.find(station2) != candisStations.end());
	CPPUNIT_ASSERT( candisStations.find(station3) != candisStations.end());
 	WNS_ASSERT_MAX_REL_ERROR(53.441, ((*candisStations.find(station2)).second.C / (*candisStations.find(station2)).second.I).get_dB(), 1E-3);
 	WNS_ASSERT_MAX_REL_ERROR(60.301, ((*candisStations.find(station3)).second.C / (*candisStations.find(station3)).second.I).get_dB(), 1E-3);


        //set additional receive pattern for station 3
	std::vector<wns::node::Interface*> undesired3;
	undesired3.push_back(station2->getNode());
	wns::service::phy::ofdma::PatternPtr pattern3;
	pattern3 = station1->calculateAndSetBeam(station3->getNode(), undesired3, noise);

 	currentReceivePatterns.clear();
 	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr>
				      (station2->getNode(), pattern2));
 	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr>
				      (station3->getNode(), pattern3));
 	ofdma1->setCurrentReceivePatterns(currentReceivePatterns);
 	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station2->getNode()) == pattern2);
 	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station3->getNode()) == pattern3);

 	//start receiving in SDMA
 	rise::BroadcastTransmissionObjectPtr bto6(
 		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), txPower));
 	trans2->startTransmitting(bto6, 0);
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();

 	rise::BroadcastTransmissionObjectPtr bto7(
 		new rise::BroadcastTransmissionObject(trans3, wns::osi::PDUPtr(), txPower));
 	trans3->startTransmitting(bto7, 0);
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();

 	WNS_ASSERT_MAX_REL_ERROR(-64.113, ofdma1->getRxPower(bto4).get_dBm(), 1E-3);
 	WNS_ASSERT_MAX_REL_ERROR(-66.8121, ofdma1->getRxPower(bto5).get_dBm(), 1E-5);

 	//interference should be minimized close to noise level
 	WNS_ASSERT_MAX_REL_ERROR(-117.555, ofdma1->getInterference(bto6).get_dBm(), 1E-3);
 	WNS_ASSERT_MAX_REL_ERROR(noise.get_dBm(), ofdma1->getInterference(bto7).get_dBm(), 1E-5);

	//check estimated value
	// disabled: fails with given max error:
	// expected: 52.4402 actual: 52.4396
//    	WNS_ASSERT_MAX_REL_ERROR(( *sinrsStations.find(station2) ).second.get_dB(),
//   				 (ofdma1->getRxPower(bto6).get_dBm() - ofdma1->getInterference(bto6).get_dBm()), 1E-5);
 	WNS_ASSERT_MAX_REL_ERROR(((*candisStations.find(station3)).second.C / (*candisStations.find(station3)).second.I).get_dB(), (ofdma1->getRxPower(bto7).get_dBm() - ofdma1->getInterference(bto7).get_dBm()), 1E-5);

 	trans2->stopTransmitting(bto6);
 	trans3->stopTransmitting(bto7);
}

void OFDMATest::testAnalogToBeamformingTest()
{
	std::map<wns::node::Interface*, wns::CandI> candis;
	std::vector<wns::node::Interface*> combination;
	wns::Power txPower = wns::Power::from_dBm(-40);
	wns::Power noise = wns::Power::from_dBm(-93);
	wns::Power x_friendlyness = noise;
	wns::service::phy::ofdma::PatternPtr pid2, pid3, pid4;
	std::vector<wns::node::Interface*> undesired;

	station1->moveTo(wns::Position(0,0,0));
	station2->moveTo(wns::Position(30,20,0));
	station3->moveTo(wns::Position(100,10,0));
	station4->moveTo(wns::Position(40,100,0));

	//the RxPower of the stations is measured at during reception of first packets
	station1->getBFAntenna()->setPowerReceivedForStation(station2, wns::Power::from_dBm(-70)); //pathloss of 30dB
	station1->getBFAntenna()->setPowerReceivedForStation(station3, wns::Power::from_dBm(-50)); //pathloss of 10dB
	station1->getBFAntenna()->setPowerReceivedForStation(station4, wns::Power::from_dBm(-60)); //pathloss of 20dB 

	// only station 2 with noise
	combination.clear();
 	combination.push_back(station2->getNode());

	std::vector<rise::Station*> combinationStations;
	combinationStations.push_back(station2);
	std::map<rise::Station*, wns::CandI> candisStations;
	candisStations = station1->getBFAntenna()->calculateCandIsRx(combinationStations, noise);
	CPPUNIT_ASSERT( candisStations.find(station2) != candisStations.end());

	//(-40dBm -30dB + 6.85dBi) - (-93dBm) = 29.85dB
 	WNS_ASSERT_MAX_REL_ERROR(29.8498, ((*candisStations.find(station2)).second.C / (*candisStations.find(station2)).second.I).get_dB(), 1E-3); 

	// three stations in SDMA without noise
// 	combination.clear();
//  	combination.push_back(station2->getNode());
//  	combination.push_back(station3->getNode());
// 	combination.push_back(station4->getNode());
// 	sinrs = station1->calculateSINRsRx(combination, noise);
// 	CPPUNIT_ASSERT( sinrs.find(station2->getNode()) != sinrs.end());
//  	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.90243, (*sinrs.find(station2->getNode()) ).second.get_dB(), 1E-3);
// 	//ideally this should be -70dBm-Iinter, thus Iinter = -77.90243dBm
// 	CPPUNIT_ASSERT( sinrs.find(station3->getNode()) != sinrs.end());
//  	CPPUNIT_ASSERT_DOUBLES_EQUAL(42.518, (* sinrs.find(station3->getNode()) ).second.get_dB(), 1E-3);
// 	//ideally this should be -50dBm-Iinter, thus Iinter = -92.518dBm
// 	CPPUNIT_ASSERT( sinrs.find(station4->getNode()) != sinrs.end());
//  	CPPUNIT_ASSERT_DOUBLES_EQUAL(21.212, (* sinrs.find(station4->getNode()) ).second.get_dB(), 1E-3);
// 	//ideally this should be -60dBm-Iinter, thus Iinter = -81.212dBm

	combinationStations.clear();
	combinationStations.push_back(station2);
	combinationStations.push_back(station3);
	combinationStations.push_back(station4);
	candisStations = station1->getBFAntenna()->calculateCandIsRx(combinationStations, noise);
	CPPUNIT_ASSERT( candisStations.find(station2) != candisStations.end());

	CPPUNIT_ASSERT( candisStations.find(station2) != candisStations.end());
 	CPPUNIT_ASSERT_DOUBLES_EQUAL(15.434, ((*candisStations.find(station2)).second.C / (*candisStations.find(station2)).second.I).get_dB(), 1E-3);

	CPPUNIT_ASSERT( candisStations.find(station3) != candisStations.end());
 	CPPUNIT_ASSERT_DOUBLES_EQUAL(49.698, ((*candisStations.find(station3)).second.C / (*candisStations.find(station3)).second.I).get_dB(), 1E-2);

	CPPUNIT_ASSERT( candisStations.find(station4) != candisStations.end());
 	CPPUNIT_ASSERT_DOUBLES_EQUAL(28.567, ((*candisStations.find(station4)).second.C / (*candisStations.find(station4)).second.I).get_dB(), 1E-3);

	undesired.clear();
 	undesired.push_back(station3->getNode());
	undesired.push_back(station4->getNode());

	pid2 = station1->calculateAndSetBeam(station2->getNode(), undesired, noise);
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station2->getBFAntenna()->getPosition(), pid2).get_factor() > 0.9 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station3->getBFAntenna()->getPosition(), pid2).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station4->getBFAntenna()->getPosition(), pid2).get_factor() < 0.1 );

	undesired.clear();
 	undesired.push_back(station2->getNode());
	undesired.push_back(station4->getNode());
	pid3 = station1->calculateAndSetBeam(station3->getNode(), undesired, noise);
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station2->getBFAntenna()->getPosition(), pid3).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station3->getBFAntenna()->getPosition(), pid3).get_factor() > 0.9 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station4->getBFAntenna()->getPosition(), pid3).get_factor() < 0.1 );

	undesired.clear();
 	undesired.push_back(station2->getNode());
	undesired.push_back(station3->getNode());
	pid4 = station1->calculateAndSetBeam(station4->getNode(), undesired, noise);
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station2->getBFAntenna()->getPosition(), pid4).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station3->getBFAntenna()->getPosition(), pid4).get_factor() < 0.1 );
	CPPUNIT_ASSERT( station1->getBFAntenna()->getGain(station4->getBFAntenna()->getPosition(), pid4).get_factor() > 0.86 );
	//exact value is 0.861857

// 	CPPUNIT_ASSERT( station1->getBFAntenna()->getNumBeams() == 0 );

}


void OFDMATest::testGetAllRxPower()
{
 	// tune two transmitters to one frequency
 	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
 	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	wns::Power allRxPower;
 	// total rx power on each subCarrier has to be equal to thermal noise
 	for(int i=0; i < 2048; ++i) {
 		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
        // total rx power has to be equal to total thermal noise
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);

 	// start a low power transmission to see if thermal noise is included:
 	rise::BroadcastTransmissionObjectPtr btoNoise(
 		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(-38)));
 	trans2->startTransmitting(btoNoise, 0);

 	// move forward in time
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();

 	WNS_ASSERT_MAX_REL_ERROR(-127.3424, ofdma1->getRxPower(btoNoise).get_dBm(), 1E-5);
 	// check power
	WNS_ASSERT_MAX_REL_ERROR(-124.216, ofdma1->getAllRxPower(0).get_dBm(), 1E-5);

 	// total rx power has to be equal to thermal noise
	allRxPower = wns::Power::from_dBm(-124.216);
 	for(int i=1; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);

 	// stop transmission
 	trans2->stopTransmitting(btoNoise);

 	// total rx power has to be equal to thermal noise
	allRxPower = wns::Power::from_mW(0);
 	for(int i=0; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);

 	// start a transmission
 	rise::BroadcastTransmissionObjectPtr bto(
 		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
 	trans2->startTransmitting(bto, 0);

 	// move forward in time
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();

 	// check power
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getAllRxPower(0).get_dBm(), 1E-5);

 	// rest has to be equal to noise
	allRxPower = wns::Power::from_dBm(-69.3424);
	for(int i=1; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);


 	// sent a second bto, ten times the power, same frequency
 	rise::BroadcastTransmissionObjectPtr bto2(
 		new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));
 	trans5->startTransmitting(bto2, 0);

 	// move forward in time
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();

 	// check power
	WNS_ASSERT_MAX_REL_ERROR(-58.9285, ofdma1->getAllRxPower(0).get_dBm(), 1E-5);

 	// rest has to be equal to noise
	allRxPower = wns::Power::from_dBm(-58.9285);
 	for(int i=1; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);


 	trans5->stopTransmitting(bto2);
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getAllRxPower(0).get_dBm(), 1E-5);

 	// rest has to be equal to noise
	allRxPower = wns::Power::from_dBm(-69.3424);
 	for(int i=1; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);

 	// sent a third bto, ten times the power, other frequency
 	rise::BroadcastTransmissionObjectPtr bto3(
 		new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));

 	// retune transmitter
 	trans5->tune(5000-50+(100/2048.0)*(3.0/2), 100/2048.0, 1);
 	trans5->startTransmitting(bto3, 0);

 	// check power
 	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getAllRxPower(0).get_dBm(), 1E-5);
 	WNS_ASSERT_MAX_REL_ERROR(-59.3425, ofdma1->getAllRxPower(1).get_dBm(), 1E-5);

 	// rest has to be equal to noise
	allRxPower = wns::Power::from_dBm(-69.3424);
	allRxPower += wns::Power::from_dBm(-59.3425);
 	for(int i=2; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);

 	// move forward in time (must proceed, can't remove transmission objects
 	// if no time has elapsed)
 	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
 	wns::simulator::getEventScheduler()->processOneEvent();

 	// remove transmission objects from physical resource
 	trans5->stopTransmitting(bto3);

 	// rest has to be equal to noise
	allRxPower = wns::Power::from_dBm(-69.3424);
 	for(int i=1; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
 	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);


 	// remove transmission objects from physical resource
 	trans2->stopTransmitting(bto);
 	// rest has to be equal to noise
	allRxPower = wns::Power::from_mW(0);
 	for(int i=0; i < 2048; ++i) {
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(i).get_dBm(), ofdma1->getAllRxPower(i).get_dBm(), 1E-5);
		allRxPower += ofdma1->getNoise(i);
	}
	WNS_ASSERT_MAX_REL_ERROR(allRxPower.get_dBm(), ofdma1->getAllRxPower().get_dBm(), 1E-5);
}

void OFDMATest::testGetInterference()
{
	// tune two transmitters to one frequency
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	// start a transmission
	rise::BroadcastTransmissionObjectPtr bto(
		new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
	trans2->startTransmitting(bto, 0);

	// move forward in time
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	// check interference
	WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getInterference(bto).get_dBm(), 1E-5);

	// sent a second bto, ten times the power, same frequency
	rise::BroadcastTransmissionObjectPtr bto2(
		new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));
	trans5->startTransmitting(bto2, 0);

	// move forward in time
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	// check power
	WNS_ASSERT_MAX_REL_ERROR(-69.3424, ofdma1->getInterference(bto2).get_dBm(), 1E-5);
	WNS_ASSERT_MAX_REL_ERROR(-59.3424, ofdma1->getInterference(bto).get_dBm(), 1E-5);

	// stop transmiitting bto2
	trans5->stopTransmitting(bto2);

	// send a third bto, ten times the power, other frequency
	rise::BroadcastTransmissionObjectPtr bto3(
		new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));

	// retune transmitter to second subcarrier
	trans5->tune(5000-50+(100/2048.0)*(3.0/2), 100/2048.0, 1);
	trans5->startTransmitting(bto3, 0);

	// check power
 	WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getInterference(bto).get_dBm(), 1E-5);
 	WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(1).get_dBm(), ofdma1->getInterference(bto3).get_dBm(), 1E-5);

	// move forward in time (must proceed, can't remove transmission objects
	// if no time has elapsed)
	wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
	wns::simulator::getEventScheduler()->processOneEvent();

	// remove transmission objects from physical resource
	trans5->stopTransmitting(bto3);
	trans2->stopTransmitting(bto);
}

void OFDMATest::testNotify()
{
	// tune two transmitters to one frequency
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	{
		// start a transmission
		rise::BroadcastTransmissionObjectPtr bto(
			new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
		trans2->startTransmitting(bto, 0);

		// move forward in time
		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto).get_dBm(), 1E-5);

		// check power
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getRxPower(bto).get_dBm(), ofdma1->getAveragedRxPower(bto).get_dBm(), 1E-5);

		// stop tramsission
		trans2->stopTransmitting(bto);
	}

	{
		// start a second transmission
		rise::BroadcastTransmissionObjectPtr bto(
			new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(30)));
		trans2->startTransmitting(bto, 0);

		// move forward in time
		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto).get_dBm(), 1E-5);

		// check power
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getRxPower(bto).get_dBm(), ofdma1->getAveragedRxPower(bto).get_dBm(), 1E-5);

		trans2->stopTransmitting(bto);
	}

	{
		// Send one transmissionobject and add another one later
		rise::BroadcastTransmissionObjectPtr bto3(
			new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
		trans2->startTransmitting(bto3, 0);

		// move forward in time
		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto3).get_dBm(), 1E-5);

		// check power
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getRxPower(bto3).get_dBm(), ofdma1->getAveragedRxPower(bto3).get_dBm(), 1E-5);


		// start another transmission
		rise::BroadcastTransmissionObjectPtr bto4(
			new rise::BroadcastTransmissionObject(trans5, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
		trans5->startTransmitting(bto4, 0);

		// move forward in time
		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		CPPUNIT_ASSERT( ofdma1->isReceiving() );

		// now check interference (half the time was noise, half the time
		// rxPower of bto4)
		WNS_ASSERT_MAX_REL_ERROR((ofdma1->getRxPower(bto4).get_mW()+ofdma1->getNoise(0).get_mW())/2.0, ofdma1->getAveragedInterference(bto3).get_mW(), 1E-5);

		trans2->stopTransmitting(bto3);
		CPPUNIT_ASSERT( ofdma1->isReceiving() );
		trans5->stopTransmitting(bto4);
		CPPUNIT_ASSERT( !ofdma1->isReceiving() );
	}
}

void OFDMATest::testMobilityUpdate()
{
	// tune two transmitters to one frequency
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	{
		// start a transmission
		rise::BroadcastTransmissionObjectPtr bto(
			new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
		trans2->startTransmitting(bto, 0);

		// Move
		station2->moveTo(wns::Position(90,80,10));

		// Move back
		station2->moveTo(wns::Position(100,100,0));

		// move forward in time
		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		wns::Power oldRxPower = ofdma1->getRxPower(bto);
		// Move
		station2->moveTo(wns::Position(50,50,0));
		wns::Power newRxPower = ofdma1->getRxPower(bto);

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto).get_dBm(), 1E-5);

		// check averaged power (must be the old since we did not
		// proceed in time)
		WNS_ASSERT_MAX_REL_ERROR(oldRxPower.get_dBm(), ofdma1->getAveragedRxPower(bto).get_dBm(), 1E-5);

		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto).get_dBm(), 1E-5);

		// check power (has changed since we did proceed in time)
		WNS_ASSERT_MAX_REL_ERROR((oldRxPower+newRxPower).get_mW()/2.0, ofdma1->getAveragedRxPower(bto).get_mW(), 1E-5);

		// stop tramsission
		trans2->stopTransmitting(bto);
	}
}

void OFDMATest::testPositionChange()
{
	// tune two transmitters to one frequency
	trans2->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);
	trans5->tune(5000-50+(100/2048.0)/2, 100/2048.0, 1);

	{
		// start a transmission
		rise::BroadcastTransmissionObjectPtr bto(
			new rise::BroadcastTransmissionObject(trans2, wns::osi::PDUPtr(), wns::Power::from_dBm(20)));
		trans2->startTransmitting(bto, 0);

		// Move
		station1->moveTo(wns::Position(10,20,10));

		// Move back
		station1->moveTo(wns::Position(0,0,0));

		// move forward in time
		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		wns::Power oldRxPower = ofdma1->getRxPower(bto);
		// Move
		station1->moveTo(wns::Position(50,50,0));
		wns::Power newRxPower = ofdma1->getRxPower(bto);

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto).get_dBm(), 1E-5);

		// check averaged power (must be the old since we did not
		// proceed in time)
		WNS_ASSERT_MAX_REL_ERROR(oldRxPower.get_dBm(), ofdma1->getAveragedRxPower(bto).get_dBm(), 1E-5);

		wns::simulator::getEventScheduler()->scheduleDelay(wns::events::NoOp(), 0.5);
		wns::simulator::getEventScheduler()->processOneEvent();

		// check interference
		WNS_ASSERT_MAX_REL_ERROR(ofdma1->getNoise(0).get_dBm(), ofdma1->getAveragedInterference(bto).get_dBm(), 1E-5);

		// check power (has changed since we did proceed in time)
		WNS_ASSERT_MAX_REL_ERROR((oldRxPower+newRxPower).get_mW()/2.0, ofdma1->getAveragedRxPower(bto).get_mW(), 1E-5);

		// stop tramsission
		trans2->stopTransmitting(bto);
	}
}


void
OFDMATest::testSetGetReceivePattern()
{
	std::vector<wns::node::Interface*> undesired;
	undesired.push_back(station3->getNode());
	wns::service::phy::ofdma::PatternPtr pattern1, pattern2;

	// must not be an optimized pattern
	// patternPtr = station1.calculateAndSetBeam(station2->getNode(), undesired, wns::Power::from_dBm(-127));

        //set receive pattern for station 2
	std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> currentReceivePatterns;
	currentReceivePatterns.clear();
	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> (station2->getNode(), pattern1));
	ofdma1->setCurrentReceivePatterns(currentReceivePatterns);

	//get receive pattern for station 2
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station2->getNode()) == pattern1);
	//get receive pattern (NULL=omni) for station 3
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station3->getNode()) == wns::service::phy::ofdma::PatternPtr());

        //set receive pattern for station 3 (implicitly reset station 2)
	currentReceivePatterns.clear();
	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> (station3->getNode(), pattern1));
	ofdma1->setCurrentReceivePatterns(currentReceivePatterns);

	//get receive pattern (NULL=omni) for station 2
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station2->getNode()) ==  wns::service::phy::ofdma::PatternPtr());
	//get receive pattern for station 3
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station3->getNode()) == pattern1);

        //set receive pattern for station 2 and 3
	currentReceivePatterns.clear();
	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> (station2->getNode(), pattern1));
	currentReceivePatterns.insert(std::pair<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> (station3->getNode(), pattern2));
	ofdma1->setCurrentReceivePatterns(currentReceivePatterns);

	//get receive pattern for station 2
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station2->getNode()) == pattern1);
	//get receive pattern for station 3
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station3->getNode()) == pattern2);

	ofdma1->removeReceivePattern(station2->getNode());
	CPPUNIT_ASSERT(ofdma1->getCurrentReceivePattern(station2->getNode()) == wns::service::phy::ofdma::PatternPtr());
}


