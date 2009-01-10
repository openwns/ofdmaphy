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

#ifndef OFDMAPHY_MANAGER_HPP
#define OFDMAPHY_MANAGER_HPP

#include <OFDMAPHY/Station.hpp>
#include <RISE/manager/systemmanager.hpp>
#include <WNS/Assure.hpp>

#include <string>
#include <map>

namespace ofdmaphy {
	class SystemManager :
		public rise::SystemManager
	{
		typedef std::map<wns::node::Interface*, Station*> StationContainer;
	public:
		SystemManager(const wns::pyconfig::View& pyConfigView);

		virtual ~SystemManager()
		{}

		Station*
		getStation(wns::node::Interface* node)
		{
			StationContainer::iterator itr = stations.find(node);
			assure(itr != stations.end(), "station not registered");
			return itr->second;
		}

		void
		addStation(wns::node::Interface* node, Station* station)
		{
			assure(stations.find(node) == stations.end(), "station already registered");
			stations[node] = station;
			rise::SystemManager::addStation(station);
		}

		void
		initAntennas(ofdmaphy::Station* station);

	private:
		StationContainer stations;
	};
} // ofdmaphy

#endif // NOT defined OFDMAPHY_MANAGER_HPP



