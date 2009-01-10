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

#include <OFDMAPHY/Manager.hpp>

using namespace ofdmaphy;

SystemManager::SystemManager(const wns::pyconfig::View& pyConfigView) :
	rise::SystemManager(pyConfigView.get<std::string>("name"), pyConfigView),
	stations()
{
	this->createScenario();
}


void
SystemManager::initAntennas(ofdmaphy::Station* station)
{
	// Hack to satisfy the Beamforming Antenna Interface
	// Makes the registering station known at the antennas of the other
	// stations and vice versa.
	for (StationContainer::iterator itr = stations.begin();
	     itr != stations.end(); ++itr){
		if (station->beamformingEnabled())
		{
			station->setTxPowerForStation( itr->first, itr->second->getMaxPowerPerSubband() );
			station->setPowerReceivedForStation( itr->first,  wns::Power::from_dBm( -75 ) );
		}
		if (itr->second->beamformingEnabled())
		{
			itr->second->setTxPowerForStation( station->getNode(), station->getMaxPowerPerSubband() );
			itr->second->setPowerReceivedForStation( station->getNode(), wns::Power::from_dBm( -75 ) );
		}
	}
}



