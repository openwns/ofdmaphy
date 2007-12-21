import os
import CNBuildSupport
from CNBuildSupport import CNBSEnvironment
import wnsbase.RCS as RCS

commonEnv = CNBSEnvironment(PROJNAME       = 'ofdmaphy',
                            AUTODEPS       = ['rise', 'wns'],
                            PROJMODULES    = ['TEST', 'BASE', 'MODULE'],
                            FLATINCLUDES   = False,
                            LIBRARY        = True,
                            SHORTCUTS      = True,
			    REVISIONCONTROL = RCS.GNUArch('../', 'OFDMAPhy', 'unstable', '0.3'),
                            )

commonEnv['CXXFLAGS'].remove('-Woverloaded-virtual')
Return('commonEnv')
