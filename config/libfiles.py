libname = 'ofdmaphy'
srcFiles = [
    'src/Component.cpp',
    'src/Station.cpp',
    'src/Manager.cpp',
    'src/Receiver.cpp',
    'src/Sender.cpp',
    'src/Scanner.cpp',
    'src/OFDMAMeasurement.cpp',
    'src/OFDMAPhy.cpp',
    'src/tests/ReceiverTest.cpp',
    'src/tests/TransmitterTest.cpp',
    'src/tests/StationTest.cpp',
    ]

hppFiles = [
    'src/Component.hpp',
    'src/Manager.hpp',
    'src/OFDMAMeasurement.hpp',
    'src/OFDMAPhy.hpp',
    'src/Receiver.hpp',
    'src/RSSInterface.hpp',
    'src/Scanner.hpp',
    'src/Sender.hpp',
    'src/Station.hpp',
    'src/tests/OFDMAStationDropIn.hpp',
    'src/tests/ReceiverTest.hpp',
    'src/tests/SystemManagerDropIn.hpp',
    'src/tests/TransmitterTest.hpp',
    'src/Transmitter.hpp',

    ]

pyconfigs = [
'ofdmaphy/Receiver.py',
'ofdmaphy/Station.py',
'ofdmaphy/System.py',
'ofdmaphy/Transmitter.py',
'ofdmaphy/OFDMAPhy.py',
'ofdmaphy/__init__.py',
'ofdmaphy/evaluation/__init__.py',
]
dependencies = []
Return('libname srcFiles hppFiles pyconfigs dependencies')
