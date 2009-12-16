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

from math import pi
from random import gauss, uniform

# this class is used e.g. in winprost/support/Transceiver.py
class OFDMAReceiver(Receiver):
    carrierSensing = None
    doMeasurementUpdates = False
    measurementUpdateInterval = None # if unspecified it will be taken from FTFading::samplingTime
    measurementUpdateOffset = None
    wraparoundShiftVectors = None # None or empty list means wraparound off

    # configuration of the MIMO strategy
    mimoProcessing = None

    def __init__(self, propagation, propagationCharacteristicName, nSectors=1, parentLogger = None, **kw):
        super(OFDMAReceiver, self).__init__(propagation, propagationCharacteristicName,nSectors=nSectors, parentLogger=parentLogger, **kw)
        self.logger = Logger("OFDMAPhy", "PHY.OFDMAReceiver", True, parentLogger)
        self.measurementUpdateOffset = 0.0

        self.mimoProcessing = NoCorrelationZF()

        attrsetter(self, kw)

    def switchMeasurementUpdates(self, doMeasurementUpdates,measurementUpdateInterval,measurementUpdateOffset):
        self.doMeasurementUpdates      = doMeasurementUpdates
        self.measurementUpdateInterval = measurementUpdateInterval
        self.measurementUpdateOffset   = measurementUpdateOffset

class ReceiverDropIn(OFDMAReceiver):

    def __init__(self, parentLogger = None):
        super(ReceiverDropIn, self).__init__(DropInPropagation.getInstance(), "DropIn", receiverNoiseFigure = "0 dB", FTFadingStrategy = FTFadingOff(), parentLogger=parentLogger)
        self.logger = Logger("OFDMAPhy", "PHY.ReceiverDropIn", True, parentLogger)

class NoCorrelationZF:
    __plugin__ = "NoCorrelationZF"

class CorrelatedStaticZF(object):
    __plugin__ = "CorrelatedStaticZF"

    arrayOrientation = None
    """ Direction of linear antenna array (rad) with respect to x-Axis"""

    antennaSpacing = None
    """ Spacing of antenna elements in wavelenghts"""

    angleSpread = None
    """ Mean angle spread due to scatteres in near environment (rad)"""

    def __init__(self, meanAngleSpread, varAngleSpread, antennaSpacing, randomOrientation, arrayOrientation = 0.0):

        # meanAngleSpread and varAngleSpread must be given in
        # log_10(degrees)!
        if(varAngleSpread > 0.0):
            spread = gauss(meanAngleSpread, varAngleSpread)
        else:
            spread = meanAngleSpread
        # convert to rad
        self.angleSpread = (10**spread)*pi/180.0

        self.antennaSpacing = antennaSpacing

        if(randomOrientation):
            # we must only cover the half circle, as the other half is
            # the same for a ULA
            self.arrayOrientation = uniform(0.0, pi)
        else:
            self.arrayOrientation = arrayOrientation


class CorrelatedStaticZF_IMTAUMi_BS(CorrelatedStaticZF):
    def __init__(self, randomSpread = False, randomOrientation = False, arrayOrientation = 0.0):
        # values are for LoS. LoS/NLoS differentiation is not implemented (yet).
        if(randomSpread):
            varAngleSpread = 0.43
        else:
            varAngleSpread = 0.0
        super(CorrelatedStaticZF_IMTAUMi_BS, self).__init__(meanAngleSpread = 1.20,
                                                            varAngleSpread = varAngleSpread,
                                                            antennaSpacing = 0.5,
                                                            randomOrientation = randomOrientation,
                                                            arrayOrientation = arrayOrientation)

class CorrelatedStaticZF_IMTAUMi_UT(CorrelatedStaticZF):
    def __init__(self, randomSpread = False, randomOrientation = False, arrayOrientation = 0.0):
        # values are for LoS. LoS/NLoS differentiation is not implemented (yet).
        if(randomSpread):
            varAngleSpread = 0.19
        else:
            varAngleSpread = 0.0
        super(CorrelatedStaticZF_IMTAUMi_UT, self).__init__(meanAngleSpread = 1.75,
                                                            varAngleSpread = varAngleSpread,
                                                            antennaSpacing = 0.5,
                                                            randomOrientation = randomOrientation,
                                                            arrayOrientation = arrayOrientation)

