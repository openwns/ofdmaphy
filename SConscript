import os
Import('env')
srcFiles,headers,pyconfigs = SConscript('config/libfiles.py')

lib = env.SharedLibrary('ofdmaphy', srcFiles)
env.Install(os.path.join(env.installDir, 'lib'), lib )

