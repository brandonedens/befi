# Meson build script for the interpreter.
project('beti', 'c')

flex = find_program('flex')
bison = find_program('bison')

lgen = generator(flex,
output : '@PLAINNAME@.yy.c',
arguments : ['-o', '@OUTPUT@', '@INPUT@'])

lfiles = lgen.process('beti.l')

pgen = generator(bison,
output : ['@BASENAME@.tab.c', '@BASENAME@.tab.h'],
arguments : ['@INPUT@', '--defines=@OUTPUT1@', '--output=@OUTPUT0@'])

pfiles = pgen.process('beti.y')

executable('beti', sources : ['main.c', lfiles, pfiles])
