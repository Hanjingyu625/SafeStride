"""Execute the LCD script with vendor APIs stubbed using system liblua 5.3.

This tests UI logic, not VisualTFT compilation or the LCD's installed firmware.
"""
import ctypes
import ctypes.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DisplayLuaTests(unittest.TestCase):
    def test_boot_live_stale_and_permissions(self):
        library = ctypes.util.find_library('lua5.3')
        if not library:
            self.skipTest('liblua5.3 is required for LCD script execution')
        lua = ctypes.CDLL(library)
        lua.luaL_newstate.restype = ctypes.c_void_p
        lua.luaL_openlibs.argtypes = [ctypes.c_void_p]
        lua.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lua.lua_pcallk.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
                                  ctypes.c_int, ctypes.c_ssize_t, ctypes.c_void_p]
        lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        lua.lua_tolstring.restype = ctypes.c_char_p
        lua.lua_close.argtypes = [ctypes.c_void_p]
        state = lua.luaL_newstate()
        self.assertTrue(state)
        lua.luaL_openlibs(state)
        source = (ROOT / 'display/ezhmi/safestride.lua').read_text(encoding='utf-8')
        setup = '''
values = {version=2, heartbeat=0, valid=63, speed=125, hands=3,
  crosswalk=3, seconds=6, distance=42, pitch=65526, tof=0, hazard=0,
  walker=2, braking=0, faults=0, host_link=1, flags=83}
texts={}; screen=-1
function set_text(page,id,value) texts[id]=value end
function set_fore_color(...) end
function set_enable(page,id,value) assert(id==11 and value==0) end
function change_screen(id) screen=id end
function get_variant(name) return values[string.sub(name,4)] end
'''
        checks = '''
on_init(); assert(screen==0)
on_systick(); assert(screen==0 and texts[3]=='? UNAVAILABLE')
values.heartbeat=1; on_systick()
assert(screen==1 and texts[1]=='1.25 km/h')
assert(texts[3]=='O READY' and texts[5]=='ENTRY ALLOWED')
assert(texts[8]=='-1.0 deg' and texts[10]=='Location: N/A')
values.flags=67; values.heartbeat=2; on_systick()
assert(texts[5]=='WAIT AT CROSSWALK') -- no explicit entry permission
values.braking=1; values.heartbeat=3; on_systick(); assert(texts[3]=='X BRAKING')
values.braking=0; values.flags=65; values.heartbeat=4; on_systick(); assert(texts[3]=='X STANDBY')
values.flags=83; values.hazard=1; values.heartbeat=5; on_systick(); assert(texts[3]=='X HAZARD')
values.hazard=0; values.valid=31; values.heartbeat=6; on_systick(); assert(texts[3]=='? UNAVAILABLE')
values.valid=63; values.heartbeat=7; on_systick(); assert(texts[3]=='O READY')
on_systick()
assert(texts[3]=='? UNAVAILABLE' and texts[1]=='-- km/h')
values.heartbeat=8; on_systick(); assert(texts[3]=='O READY')
values.host_link=0; values.heartbeat=9; on_systick(); assert(texts[3]=='? UNAVAILABLE')
values.host_link=1; values.version=1; values.heartbeat=10; on_systick()
assert(texts[4]=='Display / firmware mismatch')
values.version=2; values.pitch=nil; values.heartbeat=11; on_systick()
assert(texts[4]=='Check LCD variables')
values.pitch=0; values.heartbeat=100; values.host_link=1; values.flags=83
on_init(); on_systick(); on_systick()
assert(screen==1 and texts[1]=='1.25 km/h' and texts[3]=='O READY')
on_systick()
assert(texts[3]=='? UNAVAILABLE' and texts[4]=='Link lost / waiting')
'''
        try:
            code = (setup + source + checks).encode('utf-8')
            result = lua.luaL_loadstring(state, code)
            if result == 0:
                result = lua.lua_pcallk(state, 0, 0, 0, 0, None)
            error = lua.lua_tolstring(state, -1, None) if result else b''
            self.assertEqual(result, 0, error.decode('utf-8', 'replace'))
        finally:
            lua.lua_close(state)


if __name__ == '__main__':
    unittest.main()
