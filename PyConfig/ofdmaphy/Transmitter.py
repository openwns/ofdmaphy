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
from rise.Transmitter import Transmitter
from rise.scenario.Propagation import DropInPropagation
#import rise.Scenario

class OFDMATransmitter(Transmitter):

    def __init__(self, propagation, propagationCharacteristicName, parentLogger = None, **kw):
        super(OFDMATransmitter, self).__init__(propagation, propagationCharacteristicName, parentLogger=parentLogger, **kw)
        self.logger = Logger("OFDMAPhy", "PHY.OFDMATransmitter", True, parentLogger)

class TransmitterDropIn(OFDMATransmitter):

    def __init__(self, parentLogger = None):
        super(TransmitterDropIn, self).__init__(DropInPropagation.getInstance(), "DropIn", parentLogger=parentLogger)
        self.logger = Logger("OFDMAPhy", "PHY.TransmitterDropIn", True, parentLogger)
