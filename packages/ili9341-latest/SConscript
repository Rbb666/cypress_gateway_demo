
from building import *

src = []
cwd = GetCurrentDir()
CPPPATH = [cwd]

if GetDepend(['PKG_USING_ILI9341']):
    src += Glob('lcd_ili9341.c')

group = DefineGroup('ili9341', src, depend = [], CPPPATH = CPPPATH)

Return('group')
