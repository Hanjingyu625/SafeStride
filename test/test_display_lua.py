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
function start_timer(id,ms,direction,count)
  assert(id==0 and ms==100 and count==0)
end
function get_variant(name) return values[string.sub(name,4)] end
'''
        checks = '''
on_init(); assert(screen==0)
on_timer(0); assert(texts[3]=='? 확인 불가')
for i=1,21 do values.heartbeat=i; on_timer(0) end
assert(screen==1 and texts[1]=='1.25 km/h')
assert(texts[3]=='O 주행 가능' and texts[5]=='진입 가능')
assert(texts[8]=='-1.0 deg' and texts[10]=='위치: N/A')
values.flags=67; values.heartbeat=22; on_timer(0)
assert(texts[5]=='횡단보도: 대기') -- no explicit entry permission
values.braking=1; on_timer(0); assert(texts[3]=='X 감속 / 제동')
values.braking=0; values.flags=65; on_timer(0); assert(texts[3]=='X 대기')
values.flags=83; values.hazard=1; on_timer(0); assert(texts[3]=='X 위험 감지')
values.hazard=0; values.valid=31; on_timer(0); assert(texts[3]=='? 확인 불가')
values.valid=63
for i=1,10 do on_timer(0) end
assert(texts[3]=='? 확인 불가' and texts[1]=='-- km/h')
values.heartbeat=23; on_timer(0); assert(texts[3]=='O 주행 가능')
values.host_link=0; on_timer(0); assert(texts[3]=='? 확인 불가')
values.host_link=1; values.version=1; on_timer(0)
assert(texts[4]=='화면 / 펌웨어 버전 확인')
values.version=2; values.pitch=nil; on_timer(0)
assert(texts[4]=='LCD 변수 설정 확인')
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
