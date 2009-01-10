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

#include <OFDMAPHY/OFDMAMeasurement.hpp>
#include <WNS/events/scheduler/Interface.hpp>
#include <RISE/scenario/ftfading/FTFading.hpp>
#include <fstream>
#include <iomanip>

using namespace ofdmaphy;

OFDMAMeasurement::OFDMAMeasurement(wns::node::Interface* _source,
								   int _numberOfSubChannels,
								   simTimeType _timeOfValidity,
								   wns::Ratio _quasiStaticPathLoss,
								   rise::scenario::ftfading::FTFading* _ftfading,
								   std::vector<wns::Power> _interferenceVector,
								   wns::logger::Logger& _logger
	) :
	source(_source),
	numberOfSubChannels(_numberOfSubChannels),
	timeOfValidity(_timeOfValidity),
	quasiStaticPathLoss(_quasiStaticPathLoss),
	ftfading(_ftfading),
	logger(_logger)
{
	assure(source!=NULL,"source==NULL");
	assure(numberOfSubChannels>0,"wrong numberOfSubChannels="<<numberOfSubChannels);
	assure(timeOfValidity > 0.0,"OFDMAMeasurement is valid too shortly ("<<timeOfValidity<<"s)");
	//assure(ftfading!=NULL,"ftfading==NULL"); // ftfading==NULL means no fading (0dB)
	timestamp = wns::simulator::getEventScheduler()->getTime();
	//wns::Power interferenceTemplate = wns::Power::from_mW(0.0); // from_dBm()
 	//interferencePlusNoise = std::vector<wns::Power>(numberOfSubChannels,interferenceTemplate);
	interferencePlusNoise = _interferenceVector; // big copy
}

OFDMAMeasurement::~OFDMAMeasurement()
{
}

const wns::Ratio
OFDMAMeasurement::getPathLoss(int subChannel) const
{
	assure(subChannel>=0,"invalid subChannel="<<subChannel);
	assure(subChannel<numberOfSubChannels,"invalid subChannel="<<subChannel);
	wns::Ratio pathLoss = quasiStaticPathLoss;
	if (ftfading) {
		wns::Ratio ftfadingValue = ftfading->getFTFading(subChannel);
		pathLoss -= ftfadingValue;
		MESSAGE_SINGLE(NORMAL, logger, "OFDMAMeasurement::getPathLoss(subChannel="<<subChannel<<"):"
					   <<" spl="<<quasiStaticPathLoss
					   <<", ftf="<<ftfadingValue
					   <<", pathLoss="<<pathLoss);
	} else {
		MESSAGE_SINGLE(NORMAL, logger, "OFDMAMeasurement::getPathLoss(subChannel="<<subChannel<<"): pathLoss="<<pathLoss);
	}
	return pathLoss;
}

// const wns::Ratio OFDMAMeasurement::getPathLossInFuture(int subChannel, int samplingTimeOffset) const
//{
//	assure(samplingTimeOffset>=0,"invalid samplingTimeOffset="<<samplingTimeOffset);
//	assure(samplingTimeOffset<=1,"invalid samplingTimeOffset="<<samplingTimeOffset);
//}

const std::vector<wns::Ratio>
OFDMAMeasurement::getPathLoss() const
{
	// make a local object that is _copied_ to the caller:
	std::vector<wns::Ratio> pathLossVector; // temporary object
	if (ftfading)
		assure(numberOfSubChannels==ftfading->getNumberOfSubChannels(),"mismatch of numberOfSubChannels");
	for(int subChannel=0; subChannel<numberOfSubChannels; ++subChannel) {
		wns::Ratio pathLoss = quasiStaticPathLoss;
		if (ftfading) {
			wns::Ratio ftfadingValue = ftfading->getFTFading(subChannel);
			pathLoss -= ftfadingValue;
			MESSAGE_SINGLE(NORMAL, logger, "OFDMAMeasurement::getPathLoss(subChannel="<<subChannel<<"):"
						   << std::setiosflags(std::ios::dec)
						   << std::resetiosflags(std::ios::scientific)
						   << std::setprecision(2)
						   <<" spl="<<quasiStaticPathLoss
						   <<", ftf="<<ftfadingValue
						   <<", pathLoss="<<pathLoss);
		} else {
			MESSAGE_SINGLE(NORMAL, logger, "OFDMAMeasurement::getPathLoss(subChannel="<<subChannel<<"): pathLoss="<<pathLoss);
		}
		pathLossVector.push_back(pathLoss);
	}
	return pathLossVector; // copy back
}

//const std::vector<wns::Ratio> OFDMAMeasurement::getPathLossInFuture(int samplingTimeOffset) const
//{
//	assure(samplingTimeOffset>=0,"invalid samplingTimeOffset="<<samplingTimeOffset);
//	assure(samplingTimeOffset<=1,"invalid samplingTimeOffset="<<samplingTimeOffset);
//}

const wns::Power
OFDMAMeasurement::getInterferencePlusNoise(int subChannel) const
{
	assure(subChannel>=0,"invalid subChannel="<<subChannel);
	assure(subChannel<numberOfSubChannels,"invalid subChannel="<<subChannel);
	assure(subChannel<(int)interferencePlusNoise.size(),"subChannel out of bounds="<<subChannel);
	//wns::Power interference = wns::Power::from_mW(0.0);
	return interferencePlusNoise[subChannel];
}

const std::vector<wns::Power>&
OFDMAMeasurement::getInterferencePlusNoise() const
{
	//return std::vector<wns::Power>();
	return interferencePlusNoise;
}

wns::node::Interface*
OFDMAMeasurement::getSourceNode() const
{
	return source;
}

const bool
OFDMAMeasurement::isUpToDate() const
{
	simTimeType now = wns::simulator::getEventScheduler()->getTime();
	simTimeType diff = now - timestamp;
	return (diff>=0.0) && (diff<=timeOfValidity);
}

std::string
OFDMAMeasurement::getString() const
{
	std::stringstream s;
	s<<"OFDMAMeasurement: ";
	s.setf(std::ios::dec);
	s.unsetf(std::ios::scientific);
	s.precision(2);
	for(int subChannel=0; subChannel<numberOfSubChannels; ++subChannel) {
		//wns::Ratio pathLoss = getPathLoss(subChannel); // short way but verbose
		wns::Ratio pathLoss = quasiStaticPathLoss;
		if (ftfading) {
			wns::Ratio ftfadingValue = ftfading->getFTFading(subChannel);
			pathLoss -= ftfadingValue;
		}
		s<<pathLoss<<" ";
	}
	return s.str();
}


