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

#ifndef OFDMAPHY_RECEIVER_RECEIVER_HPP
#define OFDMAPHY_RECEIVER_RECEIVER_HPP

#include <OFDMAPHY/receiver/ReceiverBase.hpp>
#include <OFDMAPHY/receiver/OFDMAAspect.hpp>
#include <OFDMAPHY/receiver/FTFadingAspect.hpp>
#include <OFDMAPHY/receiver/MeasurementAspect.hpp>

#include <OFDMAPHY/receiver/mimo/ICalculationStrategy.hpp>

#include <OFDMAPHY/Station.hpp>
#include <OFDMAPHY/RSSInterface.hpp>

#include <RISE/receiver/LossCalculation.hpp>
#include <RISE/transceiver/cache/propagationcache.hpp>

#include <WNS/PowerRatio.hpp>
#include <WNS/pyconfig/View.hpp>

#include <map>

namespace ofdmaphy { namespace receiver {
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

        /** @brief SignalCalculationInterface */
        virtual wns::Power
        getRxPower(const rise::TransmissionObjectPtr& t);

        /** @brief Sum of all RxPowers (per SubCarrier) */
        virtual wns::Power
        getAllRxPower(const int subCarrier);

        /** @brief Sum of all RxPowers (all) */
        virtual wns::Power
        getAllRxPower();

        /** @brief InterferenceCalculationInterface: determine power of all
         *   active transmissions except own
         */
        virtual wns::Power
        getInterference(const rise::TransmissionObjectPtr& t);

        /** @brief PositionObserver */
        virtual void
        positionWillChange();

        virtual void
        positionChanged();

        /** @brief PhysicalResourceObserver
         *
		 * notify is called when startTransmission and stopTransmission happens
		 * this triggers the call of getStation()->receiveData()
		 */
        virtual void
        notify(rise::TransmissionObjectPtr t);

        void
        signalNewReceivedSignalStrength();

        /** @brief doMeasurementsNow is called in regular intervals */
        virtual void
        doMeasurementsNow();

        virtual void
        mobilityUpdate(rise::Transmitter* t);

        /** @brief PropagationCache interface */
        virtual void
        writeCacheEntry(rise::PropCacheEntry& cacheEntry, rise::Transmitter* t, double freq);

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

        void
        updateRequest();

        void
        insertReceivePattern(wns::node::Interface*, wns::service::phy::ofdma::PatternPtr);

        void
        removeReceivePattern(wns::node::Interface*);

        void
        setCurrentReceivePatterns(std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> _currentReceivePatterns);

        virtual wns::service::phy::ofdma::PatternPtr
        getCurrentReceivePattern(const rise::TransmissionObjectPtr& t) const;

        virtual wns::service::phy::ofdma::PatternPtr
        getCurrentReceivePattern(wns::node::Interface* pStack) const;

        bool
        isReceiving() const;

        std::string
        printActiveTransmissions() const;

        virtual void
        tune(double f, double b, int numberOfSubCarriers);

        wns::Power
        getRxPower(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern);

        wns::Power
        getUnfilteredRxPower(const rise::TransmissionObjectPtr& t);

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

        template<typename TYPE>
        TYPE* getMIMOProcessing() const
            {
                TYPE* t = dynamic_cast<TYPE*>(mimoProcessing);
                assure(t, "MIMO processing is not of the requested type");
                return t;
            }

        /** @brief PathlossCalculationInterface (from propagation cache) */
        virtual wns::Ratio
        getLoss(rise::Transmitter* t, double f);

        virtual wns::Ratio
        getQuasiStaticPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern);

        virtual wns::Ratio
        getFullPathLoss(const rise::TransmissionObjectPtr& t, wns::service::phy::ofdma::PatternPtr pattern);
    private:
        rise::Station* station;

        rise::PropagationCache* propagationCache;

        std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr> currentReceivePatterns;
        typedef std::list<rise::TransmissionObjectPtr> TransmissionObjectPtrList;
        TransmissionObjectPtrList activeTransmissions;

        wns::Power receivedSignalStrength;

        const rise::SystemManager::WraparoundShiftVectorContainer* wraparoundShiftVectors;

        int nSectors;

        mimo::ICalculationStrategy* mimoProcessing;
    };
}
}

#endif // not defined __RISE_OFDMA_HPP



