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

#ifndef OFDMAPHY_RECEIVER_MEASUREMENTASPECT_HPP
#define OFDMAPHY_RECEIVER_MEASUREMENTASPECT_HPP

#include <OFDMAPHY/receiver/ReceiverBase.hpp>

#include <RISE/misc/pointer.hpp>

#include <WNS/node/Interface.hpp>
#include <WNS/events/CanTimeout.hpp>
#include <WNS/PowerRatio.hpp>
#include <WNS/pyconfig/View.hpp>

namespace ofdmaphy { namespace receiver {

    /**
	 * @brief implements the Measurement Aspect of a MultiCarrier receiver
	 */
    class MeasurementAspect :
        virtual public ReceiverBase,
        protected wns::events::CanTimeout // regular intervals
    {
    public:
        MeasurementAspect(const wns::pyconfig::View& config);
        virtual ~MeasurementAspect();

    protected:
        void
        startRegularMeasurementUpdates();

        /** @brief Periodically executed to give measurementUpdates */
        virtual void
        onTimeout();

        /** @brief to be implemented by Receiver */
        virtual void
        doMeasurementsNow() = 0;

        virtual simTimeType
        getMeasurementUpdateInterval() const
            { return measurementUpdateInterval; }

        virtual void
        setMeasurementUpdateInterval(simTimeType _measurementUpdateInterval)
            { measurementUpdateInterval=_measurementUpdateInterval; }

        virtual simTimeType
        getMeasurementUpdateOffset() const
            { return measurementUpdateOffset; }

        virtual bool
        measurementUpdatesAreOn() const
            { return doMeasurementUpdates; }

        virtual void
        registerSource(wns::node::Interface* source);

        virtual PerSourceContainer&
        getPerSourceContainer(wns::node::Interface* source);

        /** @brief simplified interface if only TransmissionObjectPtr is known; returns node */
        virtual wns::node::Interface*
        registerSource(rise::TransmissionObjectPtr t);

        virtual void
        saveMeasuredFlatPathloss(wns::node::Interface* source, wns::Ratio pathloss);

        /** @brief do call onMeasurementUpdate in regular intervals if true */
        bool doMeasurementUpdates;

        /** @brief time between measurement updates */
        simTimeType measurementUpdateInterval;
        simTimeType measurementUpdateOffset;

    };
}
}

#endif // not defined __RISE_OFDMA_HPP



