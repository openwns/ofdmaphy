###############################################################################
# This file is part of openWNS (open Wireless Network Simulator)
# _____________________________________________________________________________
#
# Copyright (C) 2004-2007
# Chair of Communication Networks (ComNets)
# Kopernikusstr. 5, D-52074 Aachen, Germany
# phone: ++49-241-80-27910,
# fax: ++49-241-80-22242
# email: info@openwns.org
# www: http://www.openwns.org
# _____________________________________________________________________________
#
# openWNS is free software; you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License version 2 as published by the
# Free Software Foundation;
#
# openWNS is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
###############################################################################

from openwns.logger import Logger
from openwns import dBm
from rise.Antenna import Isotropic, BFAntenna, AntennaDropIn
from ofdmaphy.Receiver import ReceiverDropIn
from ofdmaphy.Transmitter import TransmitterDropIn
from rise.Station import Station, MobileStation
from rise import Mobility
import openwns.node

class OFDMAComponent(openwns.node.Component):
    ofdmaStation = None
    # service names (interface between layer 1+2):
    dataTransmission = None # downStack
    notification     = None # upStack
    measurements     = None # upStack

    def __init__(self, node, name, station, parentLogger = None):
        # name is string for modeName, station is of class OFDMAStation (below)
        super(OFDMAComponent, self).__init__(node, name)
        self.logger = Logger("RISE", "PHY.OFDMAComponent", True, parentLogger)
        self.nameInComponentFactory = 'ofdmaphy.Component'
        self.dataTransmission = name + '_phyDataTransmission'
        self.notification = name + '_phyNotification'
        self.measurements = name + '_phyMeasurements'
        self.ofdmaStation = station

class OFDMAStation(MobileStation):
    txFrequency = 5000
    rxFrequency = 5000
    numberOfSubCarrier = 104                # for which system? TODO: make independent
    # one sub carrier is 781.2 kHz          # for which system? TODO: make independent
    # bandwidth is measured in MHz
    bandwidth = numberOfSubCarrier * 0.7812 # for which system? TODO: make independent
    beamformingAntenna = None
    txPower = "-0.17 dBm" # Power per subcarrier
    totalPower = "50.0 dBm" # theoretical maximum, re-set it to a suitable value for your station
    eirpLimited = None
    systemManagerName = None
    numAntennas = None

    def __init__(self, _receiver, _transmitter, parentLogger = None, eirpLimited = False, noOfAntenna = 1, arrayLayout = "linear", positionErrorVariance = 0.0):
        super(OFDMAStation, self).__init__([Isotropic([0,0,1.5])], _receiver, _transmitter, parentLogger = parentLogger)
        self.systemManagerName = "ofdma"
        self.eirpLimited = eirpLimited
        self.numAntennas = noOfAntenna

class OFDMABFStation(OFDMAStation):
    def __init__(self, _receiver, _transmitter, parentLogger = None, eirpLimited = False, noOfAntenna = 4, arrayLayout = "linear", positionErrorVariance = 0.0):
        super(OFDMABFStation, self).__init__(_receiver, _transmitter, parentLogger, eirpLimited, noOfAntenna, arrayLayout, positionErrorVariance)
        self.beamformingAntenna = BFAntenna(noOfAntenna, [0,0,1.5], arrayLayout, positionErrorVariance)


class OFDMAStationDropIn(OFDMAStation):
    """ This class is only for the OFDMA Test """
    def __init__(self, parentLogger = None):
        super(OFDMAStationDropIn, self).__init__([ReceiverDropIn()], [TransmitterDropIn()], parentLogger)
        self.logger = Logger("RISE", "PHY.StationDropIn", True, parentLogger)
        self.bandwidth = 1
        self.txFrequency = 1
        self.rxFrequency = 1
        self.numberOfSubCarrier = 1
        self.txPower = "20 dBm"
        self.beamformingAntenna = BFAntenna(4, [0,0,1.5], "linear", 0.0)
        self.systemManagerName = "OFDMATest"


class Sender(MobileStation, openwns.node.Component):

    txFrequency = 5000
    numberOfSubCarrier = 104                # for which system? TODO: make independent
    # one sub carrier is 781.2 kHz
    # bandwidth is measured in MHz
    bandwidth = numberOfSubCarrier * 0.7812 # for which system? TODO: make independent
    txPower = "-0.17 dBm" # Power per subcarrier
    subBand = 1
    systemManagerName = "OFDMA"

    def __init__(self, node, name, _transmitter, parentLogger = None):
        openwns.node.Component.__init__(self, node, name)
        MobileStation.__init__(self, [Isotropic([0,0,1.5])], None,  _transmitter, parentLogger = parentLogger)

        self.logger = Logger("OFDMAPHY", "Sender", True, parentLogger)
        self.nameInComponentFactory = 'ofdmaphy.Sender'


class Scanner(MobileStation, openwns.node.Component):

    rxFrequency = 5000
    numberOfSubCarrier = 104                # for which system? TODO: make independent
    # one sub carrier is 781.2 kHz
    # bandwidth is measured in MHz
    bandwidth = numberOfSubCarrier * 0.7812 # for which system? TODO: make independent
    systemManagerName = "OFDMA"

    rxpProbeName  = None
    sinrProbeName = None

    def __init__(self, node, name, _receiver, parentLogger = None):
        openwns.node.Component.__init__(self, node, name)
        MobileStation.__init__(self, [Isotropic([0,0,1.5])], _receiver,  None, parentLogger = parentLogger)

        self.logger = Logger("OFDMAPHY", name+".PHY.Scanner", True, parentLogger)
        self.nameInComponentFactory = 'ofdmaphy.Scanner'
