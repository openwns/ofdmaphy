import os
import fnmatch
import glob
Import('env installDir includeDir')
libs,headers,pyconfigs = SConscript('config/libfiles.py')

for lib,files in libs.items():
    if len(files) != 0:
	lib = env.SharedLibrary('ofdmaphy-' + lib.lower(), files)
	env.Install(installDir, lib )

