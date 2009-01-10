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

from openwns.pyconfig import attrsetter
from openwns.logger import Logger
from rise.Receiver import Receiver
from rise.scenario.Propagation import DropInPropagation
import rise.Scenario


from rise.scenario.FTFading import FTFadingOff

# this class is used e.g. in winprost/support/Transceiver.py
class OFDMAReceiver(Receiver):
    carrierSensing = None
    doMeasurementUpdates = False
    measurementUpdateInterval = None # if unspecified it will be taken from FTFading::samplingTime
    measurementUpdateOffset = None
    def __init__(self, propagation, propagationCharacteristicName, parentLogger = None, **kw):
        super(OFDMAReceiver, self).__init__(propagation, propagationCharacteristicName, parentLogger=parentLogger, **kw)
        self.logger = Logger("OFDMAPhy", "PHY.OFDMAReceiver", True, parentLogger)
        self.measurementUpdateOffset = 0.0
        attrsetter(self, kw)
    def switchMeasurementUpdates(self, doMeasurementUpdates,measurementUpdateInterval,measurementUpdateOffset):
        self.doMeasurementUpdates      = doMeasurementUpdates
        self.measurementUpdateInterval = measurementUpdateInterval
        self.measurementUpdateOffset   = measurementUpdateOffset

class ReceiverDropIn(OFDMAReceiver):

    def __init__(self, parentLogger = None):
        super(ReceiverDropIn, self).__init__(DropInPropagation.getInstance(), "DropIn", receiverNoiseFigure = "0 dB", FTFadingStrategy = FTFadingOff(), parentLogger=parentLogger)
        self.logger = Logger("OFDMAPhy", "PHY.ReceiverDropIn", True, parentLogger)
