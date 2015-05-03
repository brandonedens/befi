# Meson build script for the interpreter.
project('befi', 'c')

flex = find_program('flex')
bison = find_program('bison')

lgen = generator(flex,
output : '@PLAINNAME@.yy.c',
arguments : ['-o', '@OUTPUT@', '@INPUT@'])

lfiles = lgen.process('befi.l')

pgen = generator(bison,
output : ['@BASENAME@.tab.c', '@BASENAME@.tab.h'],
arguments : ['@INPUT@', '--defines=@OUTPUT1@', '--output=@OUTPUT0@'])

pfiles = pgen.process('befi.y')

dep_math = find_library('m')
dep_pthread = find_library('pthread')

executable('befi', sources : [['drone.c', 'main.c'], lfiles, pfiles],
    dependencies: [dep_math, dep_pthread])
