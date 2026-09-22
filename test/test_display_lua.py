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
values = {version=3, heartbeat=0, valid=63, speed=125, hands=3,
  crosswalk=3, seconds=6, distance=42, pitch=65526, tof=0, hazard=0,
  walker=2, braking=0, faults=0, host_link=1, flags=50131,
  location_0=21333, location_1=21317, location_2=20256,
  location_3=21332, location_4=19968, location_5=0, location_6=0,
  location_7=0, location_8=0, location_9=0}
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
assert(texts[3]=='O READY' and texts[5]=='CAN CROSS')
assert(texts[8]=='-1.0 deg' and texts[10]=='Location: SUSEO STN')
assert(texts[23]=='WET 77%')
function pulse() values.heartbeat=values.heartbeat+1; on_systick() end
values.flags=67; pulse()
assert(texts[5]=='WAIT') -- no explicit entry permission
values.crosswalk=2; values.seconds=0; pulse(); assert(texts[5]=='WAIT')
values.flags=3; pulse(); assert(texts[5]=='NO SIGNAL DATA')
values.crosswalk=5; values.flags=99; pulse(); assert(texts[5]=='CAUTION')
values.flags=35; pulse(); assert(texts[5]=='NO SIGNAL DATA')
values.crosswalk=4; values.flags=67; pulse(); assert(texts[5]=='CROSSING')
values.crosswalk=6; pulse(); assert(texts[5]=='CROSSING COMPLETE')
values.crosswalk=3
values.braking=1; pulse(); assert(texts[3]=='X BRAKING')
values.braking=0; values.flags=65; pulse(); assert(texts[3]=='X STANDBY')
values.flags=83; values.hazard=1; pulse(); assert(texts[3]=='X HAZARD')
values.hazard=0; values.valid=31; pulse(); assert(texts[3]=='? UNAVAILABLE')
values.valid=63; pulse(); assert(texts[3]=='O READY')
on_systick()
assert(texts[3]=='? UNAVAILABLE' and texts[1]=='-- km/h')
values.heartbeat=8; on_systick(); assert(texts[3]=='O READY')
values.host_link=0; values.heartbeat=9; on_systick(); assert(texts[3]=='? UNAVAILABLE')
values.host_link=1; values.version=1; values.heartbeat=10; on_systick()
assert(texts[4]=='Display / firmware mismatch')
values.version=3; values.pitch=nil; values.heartbeat=11; on_systick()
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
