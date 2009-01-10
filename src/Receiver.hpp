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

#ifndef OFDMAPHY_RECEIVER_HPP
#define OFDMAPHY_RECEIVER_HPP

#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/RSSInterface.hpp>
#include <OFDMAPHY/OFDMAMeasurement.hpp>
#include <RISE/receiver/LossCalculation.hpp>
#include <RISE/receiver/MultiCarrier.hpp>
#include <RISE/receiver/ReceiverInterface.hpp>
#include <RISE/receiver/SignalAveragingStrategy.hpp>
#include <RISE/transceiver/cache/propagationcache.hpp>
#include <RISE/scenario/ftfading/FTFading.hpp>
#include <RISE/receiver/PowerMeasurement.hpp>
#include <WNS/events/CanTimeout.hpp>
#include <WNS/PowerRatio.hpp>
#include <WNS/logger/Logger.hpp>
#include <WNS/pyconfig/View.hpp>
#include <map>
#include <vector>

namespace rise { namespace medium {
	class PhysicalResource;
} // medium
} // rise

namespace ofdmaphy {
	struct PerSourceContainer {
		int packetCount;
		rise::scenario::ftfading::FTFading* ftfading;
		wns::Ratio quasiStaticPathLoss;
		std::vector<wns::Power> interferenceVector;
	};
	/**
	 * @brief base class for all aspects of OFDMA receiver
	 */
	class ReceiverBase
	{
	public:
		ReceiverBase();
		ReceiverBase(const wns::pyconfig::View& config);
		virtual ~ReceiverBase();
		virtual int getCurrentNumberOfSubCarriers() const = 0; // required
		virtual wns::Power getNoisePerSubChannel() const = 0; // required
	protected:
		wns::logger::Logger logger;
		/** @brief information element for measurements and fading */
		/** @brief maps from the source to its measured pathloss,ftfading */
		typedef std::map< wns::node::Interface*, PerSourceContainer > PerSourceMap;
		/** @brief container for multiple objects, one per source */
		PerSourceMap perSourceMap;
	};

	/**
	 * @brief implements the OFDMA Aspect of a MultiCarrier receiver
	 */
	class OFDMAAspect :
		virtual public ReceiverBase,
		virtual public rise::receiver::ReceiverInterface,
		virtual public rise::receiver::MultiCarrierAspect
	{
		typedef std::vector<rise::medium::PhysicalResource*> PhysicalResourceContainer;
	public:
		OFDMAAspect(wns::Ratio rnf);
		// MultiCarrierAspect
		/** @brief allocate physicalResources for all OFDMA subCarriers */
		virtual void tune(double f, double b, int numberOfSubCarriers);
		/** @brief get noise power on any subCarrier (-174dBm/Hz*BW+receiverNoiseFigure) */
		virtual wns::Power getNoisePerSubChannel() const;
		/** @brief get noise power on specific subCarrier (-174dBm/Hz*BW+receiverNoiseFigure) */
 		virtual wns::Power getNoise(int subCarrier) const;
		/** @brief get receiverNoiseFigure */
		wns::Ratio getNoiseFigure() const;
		/** @brief translate frequency to subcarrier (required for FTFading) */
		virtual int getSubCarrierIndex(double f);

	protected:
		/** @brief for each subCarrier there is a physicalResource in this container */
		PhysicalResourceContainer physicalResources;

		/** @brief number of subCarriers of this OFDMA system */
		int getCurrentNumberOfSubCarriers() const { return currentNumberOfSubCarriers; }

	private:
		/** @brief receiverNoiseFigure is set by the constructor */
		wns::Ratio receiverNoiseFigure;
		/** @brief number of OFDMA subcarriers */
		int currentNumberOfSubCarriers;
		/** @brief bandwidth of one subcarrier */
		double carrierBandwidth;
		double lowestFrequency;
		double firstCarrierMidFrequency;
	};

	/**
	 * @brief implements the FTFading Aspect of a MultiCarrier receiver
	 */
	class FTFadingAspect :
		virtual public ReceiverBase
	{
		friend class MeasurementAspect;
	public:
		FTFadingAspect(const wns::pyconfig::View& config);
		virtual ~FTFadingAspect();
		bool FTFadingIsActive() const;
	protected:
		// next lines by [afo]:
		/** @brief get the current (frequency-and-time dependent) fading level of one subcarrier
		 * [+3..-inf] dB
		 */
		virtual wns::Ratio getFTFading(int _subCarrier);
		virtual wns::Ratio getFTFading(wns::node::Interface* source, int _subCarrier);
		//virtual wns::Ratio getFTFading(rise::Station* source, int _subCarrier);
		// next line by [afo]:
		/** @brief single FTFading object */
		rise::scenario::ftfading::FTFading* ftfading; // single fading object; all sources are treated the same
	private:
		bool active;
		simTimeType samplingTime;
	};

	/**
	 * @brief implements the Measurement Aspect of a MultiCarrier receiver
	 */
	class MeasurementAspect :
		virtual public ReceiverBase,
		protected wns::events::CanTimeout // regular intervals
	{
		friend class FTFadingAspect;
	public:
		MeasurementAspect(const wns::pyconfig::View& config);
		virtual ~MeasurementAspect();
	protected:
		void startRegularMeasurementUpdates();
		/** @brief Periodically executed to give measurementUpdates */
		virtual void onTimeout();
		virtual void doMeasurementsNow() = 0; // to be implemented by Receiver
		virtual simTimeType getMeasurementUpdateInterval() const { return measurementUpdateInterval; }
		virtual void setMeasurementUpdateInterval(simTimeType _measurementUpdateInterval) { measurementUpdateInterval=_measurementUpdateInterval; }
		virtual simTimeType getMeasurementUpdateOffset() const { return measurementUpdateOffset; }
		virtual bool measurementUpdatesAreOn() const { return doMeasurementUpdates; }
		virtual void registerSource(wns::node::Interface* source);
		virtual PerSourceContainer& getPerSourceContainer(wns::node::Interface* source);
		/** @brief simplified interface if only TransmissionObjectPtr is known; returns node */
		virtual wns::node::Interface* registerSource(rise::TransmissionObjectPtr t);
		virtual void saveMeasuredFlatPathloss(wns::node::Interface* source, wns::Ratio pathloss);
	protected:
		/** @brief do call onMeasurementUpdate in regular intervals if true */
		bool doMeasurementUpdates;
		/** @brief time between measurement updates */
		simTimeType measurementUpdateInterval;
		simTimeType measurementUpdateOffset;
	private:
	};

	/**
	 * @brief OFDMA implementation of MultiCarrier receiver
	 */
	class Receiver :
		virtual public ReceiverBase,
		public OFDMAAspect,
		public FTFadingAspect,
		public MeasurementAspect,
		public rise::receiver::TimeWeightedTransmissionAveraging,
		protected rise::receiver::LossCalculation,
		public wns::Subject<RSSInterface>
	{
	public:
		Receiver(const wns::pyconfig::View& config, rise::Station* s);

		virtual ~Receiver();
//		std::vector<wns::Ratio> currentFTFading;

		/** @brief SignalCalculationInterface */
		virtual wns::Power getRxPower(const rise::TransmissionObjectPtr& t);

		/** @brief Sum of all RxPowers (per SubCarrier, all) */
		virtual wns::Power getAllRxPower(const int subCarrier);
		virtual wns::Power getAllRxPower();

		/** @brief InterferenceCalculationInterface
		 * determine power of all active transmissions except own
		 */
		virtual wns::Power getInterference(const rise::TransmissionObjectPtr& t);

		/** @brief PositionObserver */
		virtual void positionWillChange();
		virtual void positionChanged();

		/** @brief PhysicalResourceObserver
		 * notify is called when startTransmission and stopTransmission happens
		 * this triggers the call of getStation()->receiveData()
		 */
		virtual void notify(rise::TransmissionObjectPtr t);
		/** @brief doMeasurementsNow is called in regular intervals */
		virtual void doMeasurementsNow(); // interface from MeasurementAspect
		virtual void mobilityUpdate(rise::Transmitter* t);

		/** @brief PropagationCache interface */
		virtual void writeCacheEntry(rise::PropCacheEntry& cacheEntry, rise::Transmitter* t, double freq);

		/** @brief functor for newRSSInterface::onNewRSS calls */
		struct OnNewRSS
		{
			OnNewRSS(const wns::Power _rss):
				rss(_rss)
				{}

			void operator()(RSSInterface* rss)
				{
					// The functor calls the onNewRSS implemented by the Observer
					rss->onNewRSS(this->rss);
				}
		private:
			wns::Power rss;
		};

		// other
		void insertReceivePattern(wns::node::Interface*, wns::service::phy::ofdma::PatternPtr);
		void removeReceivePattern(wns::node::Interface*);
		void setCurrentReceivePatterns(std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> _currentReceivePatterns);
		virtual wns::service::phy::ofdma::PatternPtr getCurrentReceivePattern(const rise::TransmissionObjectPtr& t) const;
		virtual wns::service::phy::ofdma::PatternPtr getCurrentReceivePattern(wns::node::Interface* pStack) const;
		bool isReceiving() const;

		virtual void tune(double f, double b, int numberOfSubCarriers);

	protected:
		//wns::logger::Logger logger;

		wns::Power getRxPower(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern);
		wns::Power getUnfilteredRxPower(const rise::TransmissionObjectPtr& t);

	private:
		/** @brief PathlossCalculationInterface (from propagation cache) */
		virtual wns::Ratio getLoss(rise::Transmitter* t, double f);
		virtual wns::Ratio getQuasiStaticPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern);
		virtual wns::Ratio getFullPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern);

		virtual rise::Station* getStation() const
			{
				assure(station, "Not set");
				return station;
			}

		Station* getOFDMAStation() const
			{
				Station* tmp = dynamic_cast<Station*>(getStation());
				assure(tmp, "Station is not an OFDMA Station");
				return tmp;
			}

		rise::Station* station;

		rise::PropagationCache* propagationCache;

		std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> currentReceivePatterns;
		std::list<rise::TransmissionObjectPtr> activeTransmissions;

		wns::Power receivedSignalStrength;
	};
}

#endif // not defined __RISE_OFDMA_HPP



