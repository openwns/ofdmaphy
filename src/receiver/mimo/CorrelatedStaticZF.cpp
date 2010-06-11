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

#include <OFDMAPHY/receiver/mimo/CorrelatedStaticZF.hpp>
#include <OFDMAPHY/receiver/Receiver.hpp>

#include <WNS/geometry/Vector.hpp>

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <math.h>

using namespace ofdmaphy::receiver::mimo;

STATIC_FACTORY_REGISTER_WITH_CREATOR(CorrelatedStaticZF,
                                     ICalculationStrategy,
                                     "CorrelatedStaticZF",
                                     ICalculationStrategyCreator);

CorrelatedStaticZF::CorrelatedStaticZF(const wns::pyconfig::View& _config,
                                       Receiver* _rx,
                                       wns::logger::Logger* _logger):
    ICalculationStrategy(_config, _rx, _logger),
    logger(_logger),
    rx(_rx),
    arrayOrientation(_config.get<double>("arrayOrientation")),
    antennaSpacing(_config.get<double>("antennaSpacing")),
    angleSpread(_config.get<double>("angleSpread"))
{
    assure(arrayOrientation >= 0.0, "Antenna angle cannot be negative");
    assure(arrayOrientation <= M_PI, "Anntenna angle cannot exceed PI");
}

std::vector<wns::Ratio>
CorrelatedStaticZF::getPostProcessingSINRFactor(rise::TransmissionObjectPtr t)
{
    // number of spatial streams and rx antennas
    int n_SS = t->getNumberOfSpatialStreams();
    int n_RX = rx->getOFDMAStation()->getNumAntennas();

    if(n_SS > n_RX)
    {
        // More streams than receive antennas -> no usefull reception possible
        return std::vector<wns::Ratio>(n_SS, wns::Ratio::from_factor(1e-12));
    }

    // this is the MIMO Gain without any correlation
    wns::Ratio noCorrelationGain = wns::Ratio::from_factor(double(n_RX - n_SS + 1)/double(n_SS));
    MESSAGE_SINGLE(NORMAL, *logger, "CorrelatedStaticZF: Gain w/o correlation: " << noCorrelationGain);

    // get the transmitting station
    ofdmaphy::Station* txStation = dynamic_cast<ofdmaphy::Station*>(t->getTransmitter()->getStation());
    assure(txStation, "Peer is not an OFDMA station");

    // get the peer mimo-processing routine and cast it to CorrelatedStaticZF to
    // read the required performance values
    CorrelatedStaticZF* peer = txStation->getReceiver()->getMIMOProcessing<CorrelatedStaticZF>();

    // calculate Angle of Departure and Angle of Arrival by creating two vectors
    // tx->rx and rx->tx
    wns::Position txPosition = txStation->getPosition();
    wns::Position rxPosition = rx->getStation()->getPosition();

    double AoD = wns::geometry::Vector(txPosition, rxPosition).getAzimuth()
        + peer->getArrayOrientation();
    if(AoD < 0)
    {
        AoD += M_PI;
    }
    else if(AoD > M_PI)
    {
        AoD -= M_PI;
    }

    double AoA = wns::geometry::Vector(rxPosition, txPosition).getAzimuth()
        + this->arrayOrientation;
    if(AoA < 0)
    {
        AoA += M_PI;
    }
    else if(AoA > M_PI)
    {
        AoA -= M_PI;
    }

    // compute loss due to tx and rx correlation
    std::vector<wns::Ratio> txLoss = getCorrelationLoss(n_SS, AoD, peer->getAntennaSpacing(), peer->getAngleSpread());
    std::vector<wns::Ratio> rxLoss = getCorrelationLoss(n_RX, AoA, this->antennaSpacing, this->getAngleSpread());

    MESSAGE_BEGIN(NORMAL, *logger, m, "CorrelatedStaticZF:");
    m << " tx pos " << txPosition;
    m << " rx pos " << rxPosition;
    MESSAGE_END();

    MESSAGE_BEGIN(NORMAL, *logger, m, "CorrelatedStaticZF: TX");
    m << " orientation " << peer->getArrayOrientation();
    m << " AoD " << AoD/M_PI*180 << "d";
    m << " ASD " << peer->getAngleSpread() << "rad";
    m << " spacing " << peer->getAntennaSpacing();
    m << " -> loss: ";
    for(std::vector<wns::Ratio>::const_iterator it = txLoss.begin();
        it != txLoss.end();
        ++it)
    {
        m << (*it) << " ";
    }
    MESSAGE_END();

    MESSAGE_BEGIN(NORMAL, *logger, m, "CorrelatedStaticZF: RX");
    m << " orientation " << this->getArrayOrientation();
    m << " AoA " << AoA/M_PI*180 << "d";
    m << " ASA " << this->getAngleSpread() << "rad";
    m << " spacing " << this->getAntennaSpacing();
    m << " -> loss: ";
    for(std::vector<wns::Ratio>::const_iterator it = rxLoss.begin();
        it != rxLoss.end();
        ++it)
    {
        m << (*it) << " ";
    }
    MESSAGE_END();

    // calculate result vector
    std::vector<wns::Ratio> r;
    std::vector<wns::Ratio>::iterator itTx = txLoss.begin();
    std::vector<wns::Ratio>::iterator itRx = rxLoss.begin();

    for(;
        itTx != txLoss.end();
        ++itTx, ++itRx)
    {
        r.push_back(noCorrelationGain - *itTx - *itRx);
    }

    // single antenna fallback
    if(r[0].get_dB() < 0 and n_SS == 1)
    {
        std::vector<wns::Ratio> rFallBack;
        rFallBack.push_back(wns::Ratio::from_factor(1));
        MESSAGE_BEGIN(NORMAL, *logger, m, "CorreltatedStaticZF: ");
        m << "Correlation makes MIMO impossible -> use only one rx antenna";
        MESSAGE_END();
        return rFallBack;
    }

    return r;
}

std::vector<wns::Ratio>
CorrelatedStaticZF::getCorrelationLoss(int n, double angle, double spacing, double spread)
{
    using namespace boost::numeric::ublas;

    // Compute the correlation matrix m
    matrix< std::complex<double> > m(n,n);
    std::complex<double> j(0, 1.0);

    for(double p = 0; p < n; ++p)
    {
        for(double q = 0; q < n; ++q)
        {
            if(p==q)
            {
                m(p,q) = 1.0;
            }
            else
            {
                m(p,q) = std::exp(-1.0*j*2.0*M_PI*(p-q)*spacing*cos(angle)) * std::exp(-0.5*std::pow(2.0*M_PI*(q-p)*spacing*sin(angle)*spread, 2.0));
            }
        }
    }

    // Compute the inverse of m using LU-factorization
    matrix< std::complex<double> > m_inv(n,n);

    // create a permutation matrix for the LU-factorization
    permutation_matrix<std::size_t> pm(m.size1());
    int res = lu_factorize(m, pm);
    if(res != 0)
    {
        // factorization not possible
        return(std::vector<wns::Ratio>(n, wns::Ratio::from_factor(1e10)));
    }

    // create identity matrix of "inverse"
    m_inv.assign(identity_matrix< std::complex<double> >(m.size1()));

    // backsubstitute to get the inverse
    lu_substitute(m, pm, m_inv);

    std::vector<wns::Ratio> r;

    for(int i = 0; i< n; ++i)
    {
        assure(m_inv(i,i).imag() < 1e-10,
               "Complex value at position " << i << " has imag-part of " << m_inv(i,i).imag());
        r.push_back(wns::Ratio::from_factor(m_inv(i,i).real()));
    }
    return r;
}
