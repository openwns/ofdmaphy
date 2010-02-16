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

#ifndef OFDMAPHY_STATION_HPP
#define OFDMAPHY_STATION_HPP

#include <OFDMAPHY/Transmitter.hpp>
#include <OFDMAPHY/RSSInterface.hpp>

#include <RISE/antenna/Antenna.hpp>
#include <RISE/stations/station.hpp>
#include <RISE/misc/pointer.hpp>
#include <RISE/antenna/Beamforming.hpp>
#include <RISE/manager/systemmanager.hpp>

#include <WNS/logger/Logger.hpp>
#include <WNS/service/phy/ofdma/DataTransmission.hpp>
#include <WNS/service/phy/ofdma/Notification.hpp>
#include <WNS/service/phy/ofdma/Handler.hpp>
#include <WNS/service/phy/ofdma/Measurements.hpp>
#include <WNS/service/phy/ofdma/MeasurementHandler.hpp>
#include <WNS/service/phy/ofdma/CarrierSensing.hpp>
#include <WNS/service/phy/ofdma/Pattern.hpp>
#include <WNS/service/phy/phymode/PhyModeInterface.hpp>
#include <WNS/CandI.hpp>

namespace ofdmaphy {
    class SystemManager;
    namespace receiver {
        class Receiver;
    }
    class Component;

    namespace tests { class TransmitterTest;}

    class PowerAdmissionInterface
    {
    public:
        virtual
        ~PowerAdmissionInterface(){}

        virtual wns::Power
        admit(const wns::Power&) const = 0;
    };

    class Station :
        virtual public wns::service::phy::ofdma::DataTransmission,
        virtual public wns::service::phy::ofdma::Notification,
        virtual public wns::service::phy::ofdma::Measurements,
        public wns::Observer<RSSInterface>,
        virtual public PowerAdmissionInterface,
        public rise::Station
    {

        friend class ofdmaphy::tests::TransmitterTest;

        typedef wns::service::phy::ofdma::BFIdu bFIdu;

    public:
        Station(Component* component, const wns::pyconfig::View& pyConfigView);
        virtual ~Station();

        void
        onNodeCreated();

        /** @brief Start a broadcast transmission of sdu */
        virtual void
        startBroadcast(wns::osi::PDUPtr sdu,
                       int subBand,
                       wns::Power requestedTxPower,
                       int numberOfSpatialStreams = 1);

        /**
         * @brief Start a single-stream broadcast transmission of sdu with
         *        given phyMode
         */
        virtual void
        startBroadcast(wns::osi::PDUPtr sdu,
                       int subBand,
                       wns::Power requestedTxPower,
                       wns::service::phy::phymode::PhyModeInterfacePtr _phyModePtr);

       /** @brief Start a unicast transmission of sdu */
        virtual void
        startUnicast(wns::osi::PDUPtr sdu,
                     wns::node::Interface* _recipient,
                     int subBand,
                     wns::Power requestedTxPower,
                     int numberOfSpatialStreams = 1);

        /**
         * @brief Start a single-stream unicast transmission of sdu with
         *        given phyMode
         */
        virtual void
        startUnicast(wns::osi::PDUPtr sdu,
                     wns::node::Interface* _recipient,
                     int subBand,
                     wns::Power requestedTxPower,
                     wns::service::phy::phymode::PhyModeInterfacePtr _phyModePtr);

        virtual void
        stopTransmission(wns::osi::PDUPtr sdu, int subBand);

        /** @brief receive sdu plus measurement information and delegate to Layer2 */
        void
        receiveData(wns::osi::PDUPtr sdu, wns::service::phy::power::PowerMeasurementPtr rxPowerMeasurementPtr);

        /** @brief delegate Phy measurements from Receiver to Layer2 */
        void
        measurementUpdate(wns::node::Interface* source, wns::service::phy::power::OFDMAMeasurementPtr rxPowerMeasurementPtr);

        /** @brief observer for newRSSInterface */
        void
        onNewRSS(wns::Power rss);

        void
        updateRequest();

        void
        endTransmission(rise::TransmissionObjectPtr t);

        virtual rise::SystemManager*
        getSystemManager() const;

        simTimeType
        getArrivalTime(simTimeType time, int len);

        // beamforming interface
        // TODO: remove this function:
        virtual std::map<wns::node::Interface*, wns::Ratio>
        calculateSINRsRx(const std::vector<wns::node::Interface*>& combination,
                         wns::Power IinterPlusNoise);

        virtual std::map<wns::node::Interface*, wns::CandI >
        calculateCandIsRx(const std::vector<wns::node::Interface*>& combination,
                          wns::Power IinterPlusNoise);

        // TODO: remove this function:
        virtual std::map<wns::node::Interface*, wns::Ratio>
        calculateSINRsTx(const std::map<wns::node::Interface*, wns::Power>& Station2NoisePlusIintercell,
                         wns::Power x_friendlyness,
                         wns::Power intendedTxPower);

        virtual std::map<wns::node::Interface*, wns::CandI >
        calculateCandIsTx(const std::map<wns::node::Interface*, wns::Power>& Station2NoisePlusIintercell,
                          wns::Power x_friendlyness,
                          wns::Power intendedTxPower);

        //should be calculateAndStorePattern
        virtual wns::service::phy::ofdma::PatternPtr
        calculateAndSetBeam(wns::node::Interface *id,
                            const std::vector<wns::node::Interface*>& undesired,
                            wns::Power IinterPlusNoise);

        virtual double
        estimateDoA(wns::node::Interface *id);

        virtual void
        startTransmission(wns::osi::PDUPtr pdu,
                          wns::node::Interface* _recipient,
                          int subBand,
                          wns::service::phy::ofdma::PatternPtr pattern,
                          wns::Power requestedTxPower,
                          int numberOfSpatialStreams = 1);

        virtual void
        startTransmission(wns::osi::PDUPtr pdu,
                          wns::node::Interface* _recipient,
                          int subBand,
                          wns::service::phy::ofdma::PatternPtr pattern,
                          wns::Power requestedTxPower,
                          wns::service::phy::phymode::PhyModeInterfacePtr _phyModePtr);

        virtual bool
        isReceiving() const;

        virtual std::string
        printActiveTransmissions() const;

        virtual void
        insertReceivePattern(wns::node::Interface*, wns::service::phy::ofdma::PatternPtr);

        virtual void
        removeReceivePattern(wns::node::Interface*);

        virtual void
        setCurrentReceivePatterns(std::map<wns::node::Interface*, wns::service::phy::ofdma::PatternPtr>);

        virtual void
        startReceiving();

        virtual void
        stopReceiving();

        virtual wns::Power
        getMaxPowerPerSubband() const {return maxTxPowerPerSubband;}

        virtual wns::Power
        getMaxOutputPower() const {return totalPower;};

        bool
        isEIRPLimited() const {return eirpLimited;}

        virtual wns::service::phy::ofdma::Tune
        getRxTune() const {return tuneRx;}

        virtual wns::service::phy::ofdma::Tune
        getTxTune() const {return tuneTx;}

        virtual void
        setTxTune(const wns::service::phy::ofdma::Tune& txTune);

        virtual void
        setRxTune(const wns::service::phy::ofdma::Tune& rxTune);

        /** @brief method for fast swapping of Rx/Tx frequencies, e.g. in an FDD relay */
        virtual void
        setTxRxSwap(bool reverse);

        void
        setTxPowerForStation(wns::node::Interface* stack, wns::Power _txPower);

        void
        setPowerReceivedForStation(wns::node::Interface* stack, wns::Power _rxPower);

        rise::antenna::Beamforming*
        getBFAntenna() const;

        /** @brief Notification handler provides method onData for compounds to send upStack */
        virtual void
        registerHandler(wns::service::phy::ofdma::Handler* _handler)
            {
                assure(_handler, "must be non-NULL");
                handler = _handler;
            }

        virtual void
        registerRSSHandler(wns::service::phy::ofdma::RSSHandler* _rssHandler);

        /** @brief Measurement handler provides method onMeasurementUpdate for measurements to send upStack */
        virtual void
        registerMeasurementHandler(wns::service::phy::ofdma::MeasurementHandler* _measurementHandler)
            {
                assure(_measurementHandler, "must be non-NULL");
                measurementHandler = _measurementHandler;
            }

        virtual wns::node::Interface*
        getNode();

        bool
        beamformingEnabled() const;

        receiver::Receiver*
        getReceiver(){return receiver;};

        Transmitter<Station>*
        getTransmitter(){return transmitter;};

        virtual int
        getNumAntennas() const
            {
                return this->numAntennas;
            };

    protected:
        bool eirpLimited;

        /**
         * @brief retrieve info about the used power per subband for a certain
         * user, currently not implemented
         */
        virtual wns::Power
        getSumPower() const;

        virtual wns::Power
        admit(const wns::Power& requestedPower) const;

    private:

        void
        startTransmitting(wns::osi::PDUPtr sdu, rise::TransmissionObjectPtr txObject, int subBand);

        wns::logger::Logger logger;
        SystemManager* systemManager;
        Transmitter<Station>* transmitter;
        receiver::Receiver* receiver;
        PowerAdmissionInterface* powerAdmission;
        // maximum Tx power of the OFDMA station
        // reduced tx power allowed, e.g., controlled by DLL-based power control
        // wns::Power txPower;
        wns::Power maxTxPowerPerSubband;
        wns::Power totalPower; // for all subBands; typ. 1..40W

        /** @brief Number of antennas for MIMO */
        const int numAntennas;

        wns::service::phy::ofdma::Tune tuneRx;
        wns::service::phy::ofdma::Tune tuneTx;
        bool reverseState;

        rise::antenna::Beamforming* beamformingAntenna;
        bool supportsBeamforming;
        std::map<wns::osi::PDUPtr, rise::TransmissionObjectPtr> activeTransmissions;
        wns::service::phy::ofdma::Handler* handler;
        wns::service::phy::ofdma::RSSHandler* rssHandler;
        wns::service::phy::ofdma::MeasurementHandler* measurementHandler;
        Component* component;

        virtual wns::CandI getCurrentCandI(wns::osi::PDUPtr sdu);

        template <typename NEWTYPE, typename CONTAINER>
        class ConvertNode {
        public:
            ConvertNode(NEWTYPE c) :
                result(),
                caller(c)
                {}

            void operator()(wns::node::Interface* n) {
                result.push_back(caller->systemManager->getStation(n));
            }

            CONTAINER result;
            NEWTYPE caller;
        };

    };
} // ofdmaphy

#endif // NOT defined OFDMAPHY_STATION_HPP


