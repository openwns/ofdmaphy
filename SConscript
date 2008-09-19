import os
Import('env')
srcFiles,headers,pyconfigs = SConscript('config/libfiles.py')

if env['static']:
    lib = env.StaticLibrary('ofdmaphy', srcFiles)
else:
    lib = env.SharedLibrary('ofdmaphy', srcFiles)
env.Install(os.path.join(env.installDir, 'lib'), lib )

