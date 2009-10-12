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

from openwns.module import Module
from openwns.pyconfig import attrsetter
from openwns.logger import Logger
from rise.System import System

class OFDMASystem(System):
    """ derived from RISE.System """
    name = None # my name must correspond with "name of the rise::SystemManager"

    def __init__(self, name, **remainingArgs):
        self.name = name
        attrsetter(self, remainingArgs)

class OFDMAPhy(Module):
    systems = None # will contain a list of OFDMASystem's
    def __init__(self):
        super(OFDMAPhy, self).__init__("ofdmaphy", "ofdmaphy")
        self.systems = []
        #self.logger.enabled = False # there is almost no output on this logger

    def knowsSystem(self, system):
        if system.name in [s.name for s in self.systems]:
            return True
        return False

    def updateSystem(self, system):
        if self.knowsSystem(system):
            index = [s.name for s in self.systems].index(system.name)
            self.systems.remove(self.systems[index])
        self.systems.append(system)