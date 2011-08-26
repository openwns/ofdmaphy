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

#ifndef WNS_OFDMAPHY_MEASUREMENT_HPP
#define WNS_OFDMAPHY_MEASUREMENT_HPP

#include <WNS/service/phy/power/OFDMAMeasurement.hpp>
#include <WNS/logger/Logger.hpp>


namespace ofdmaphy {

	/**
	 * @brief Measurement Class for vectorized values depending on the OFDMA subchannel
	 * An implementation (non-abstract) is located in OFDMAPhy
	 */
	class OFDMAMeasurement:
		public wns::service::phy::power::OFDMAMeasurement, // Interface
		public wns::Cloneable<ofdmaphy::OFDMAMeasurement> // allow SmartPtr
	{
	public:
		OFDMAMeasurement(wns::node::Interface* _source,
						 int _numberOfSubChannels,
						 simTimeType _timeOfValidity,
						 wns::Ratio _quasiStaticPathLoss,
						 std::vector<wns::Power> _interferenceVector,
						 wns::logger::Logger& _logger
						 );
		virtual ~OFDMAMeasurement();
		/** @brief use this method if you only need the values of a few subChannels */
		/* timeOffset=+1 means look into the future for one frame (samplingTime) */
		virtual const wns::Ratio getPathLoss(int subChannel) const;
		/** @brief use this method if you need all values for all subChannels in future */
		//virtual const wns::Ratio getPathLossInFuture(int subChannel, int samplingTimeOffset=0) const = 0;
		/** @brief use this method if you need all values for all subChannels */
		virtual const std::vector<wns::Ratio> getPathLoss() const;
		/** @brief use this method if you need all values for all subChannels
		 * The integer argument can look into the future
		 * (for having a reference with optimum results) */
		//virtual const std::vector<wns::Ratio> getPathLossInFuture(int samplingTimeOffset) const;
		/** @brief use this method if you only need the values of a few subChannels */
		virtual const wns::Power getInterferencePlusNoise(int subChannel) const;
		/** @brief use this method if you need all values for all subChannels */
		virtual const std::vector<wns::Power>& getInterferencePlusNoise() const;
		// something to specify the origin (source node, transmitter)
		virtual wns::node::Interface* getSourceNode() const;
		/** @brief number of OFDM subchannels */
		virtual const int getNumberOfSubChannels() const { return numberOfSubChannels; };
		virtual const simTimeType getMeasurementTime() const { return timestamp; };
		/** @brief if you are unsure if this measurement is the current, ask here */
		virtual const bool isUpToDate() const;
		/** @brief access method for the values (string) */
		virtual std::string getString() const;
	private:
		/** @brief the time at which the measurement was taken.
		 *  You must decide whether it is still useful for you */
		wns::node::Interface* source;
		/** @brief the time at which the measurement was taken.
		 *  You must decide whether it is still useful for you */
		int numberOfSubChannels;
		simTimeType timestamp;
		simTimeType timeOfValidity;
		wns::Ratio  quasiStaticPathLoss;
		std::vector<wns::Power> interferencePlusNoise;
		wns::logger::Logger& logger; // same logger as OFDMA
	};
	/** @brief define stream operator for class */
	inline std::ostream&
	operator<< (std::ostream& s, const OFDMAMeasurement& m) {
		return s << m.getString();
	}
	/** @brief when a pointer to a OFDMAMeasurement is needed, use this one
	    (SmartPtr) which takes care of memory deallocation itself. */
	typedef wns::SmartPtr<const ofdmaphy::OFDMAMeasurement> OFDMAPHYMeasurementPtr;
	/** @brief a const reference to the OFDMAMeasurement can be used anywhere. */
	typedef const ofdmaphy::OFDMAMeasurement& OFDMAPHYMeasurementConstRef;

} // ofdmaphy
#endif // WNS_SERVICE_PHY_OFDMA_MEASUREMENT_HPP





