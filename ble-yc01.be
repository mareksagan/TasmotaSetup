#======================================================================
# yc01.be - BLE-YC01 (YIERYI / YINMIK 6-in-1 pool water monitor)
#           v4.3 - optimized, cached, debounced
#
# Configuration:
#   MAC address: change _PF_MAC to your meter's MAC
#   Profiles file: profiles.bin (generated from new_profiles.csv via gen_profiles.py)
#
# Console commands:
#   yc01read              force immediate read
#   yc01start             start measurement
#   yc01stop              stop measurement
#   yc01poll <seconds>    change read interval (default 50, min 10)
#   yc01profile [name]    set profile or list available profiles
#   yc01boost [0|1]       toggle fruiting-phase boost mode
#
# Protocol:
#  * GATT: service 0xFF01, characteristic 0xFF02 for READ + WRITE.
#  * Data frame: b[0]=1, b[1]=2, b[2]=15, then BE s16 at offsets 3..15:
#      pH   = s16(b[3..4]) / 100
#      EC   = s16(b[5..6])
#      TDS  = s16(b[7..8])
#      ORP  = s16(b[9..10])
#      Cl   = max(0, s16(b[11..12])) / 10
#      Temp = s16(b[13..14]) / 10
#      Batt = clamp(100*(s16(b[15..16])-1950)/1240, 0, 100)
#      SALT = EC * 0.55
#======================================================================

import persist
import webserver

var _PF_COUNT = 0
var _PF_FNAME = "profiles.bin"
var _PF_MAC = "414284588113"  # <-- change this to your meter's MAC
var _PF_INDEX = {}   # name -> record_index (cached at init)
var _PF_NAMES = []   # sorted names (cached at init)
var _COLOR_IDX = {"pH":0,"EC":1,"TDS":2,"ORP":3,"Cl":4,"Temp":5,"SALT":6,"Batt":7}

# ---- profile file helpers ----
def _pf_open()
    var f
    try
        f = open(_PF_FNAME, "r")
    except ..
        return nil
    end
    return f
end

def _pf_count()
    if _PF_COUNT > 0   return _PF_COUNT   end
    var f = _pf_open()
    if f == nil   return 0   end
    var h = f.readbytes(2)
    if size(h) < 2
        f.close()
        return 0
    end
    _PF_COUNT = h[0] + (h[1] << 8)
    f.close()
    return _PF_COUNT
end

def _pf_build_index()
    if size(_PF_INDEX) > 0   return   end
    var cnt = _pf_count()
    if cnt == 0   return   end
    var f = _pf_open()
    if f == nil   return   end
    import string
    for i : 0..cnt-1
        f.seek(2 + i * 109)
        var hdr = f.readbytes(41)
        if size(hdr) < 41   break   end
        var nl = hdr[0]
        var name = ""
        for j : 0..nl-1
            name += string.char(hdr[1+j])
        end
        _PF_INDEX[name] = i
        _PF_NAMES.push(name)
    end
    f.close()
end

def _pf_read_vals(idx)
    var f = _pf_open()
    if f == nil   return nil   end
    f.seek(2 + idx * 109 + 41)
    var data = f.readbytes(68)
    f.close()
    if size(data) < 68   return nil   end
    var vals = []
    for k : 0..16
        var pos = k * 4
        var iv = data[pos] + (data[pos+1] << 8) + (data[pos+2] << 16) + (data[pos+3] << 24)
        if iv > 2147483647   iv -= 4294967296   end
        vals.push(iv / 100.0)
    end
    return vals
end

def _pf_find(name)
    if size(_PF_INDEX) == 0
        _pf_build_index()
    end
    var idx = _PF_INDEX.find(name)
    if idx == nil
        return nil
    end
    return _pf_read_vals(idx)
end

def _pf_names()
    return _PF_NAMES
end

class YC01 : Driver
    var mac, mac_hex, addr_type
    var poll_s
    var last, last_ok, last_seen, rssi
    var tick, awaiting, watchdog, phase
    var fails, last_try
    var orp_offset, orp_raw
    var hold
    var connected
    var profile_name
    var boost
    var _mac_cache
    var _fmin, _fmax, _fmar
    var _persist_dirty
    var _retry_cnt
    var _retry_at

    def init(mac, poll_s, addr_type)
        import string
        if mac == nil   mac = persist.find("yc01_mac", _PF_MAC) end
        if addr_type == nil   addr_type = persist.find("yc01_addr_type", 0) end
        self.mac = mac
        self.mac_hex = string.toupper(string.replace(mac, ":", ""))
        self.addr_type = addr_type
        self.poll_s = (poll_s == nil) ? persist.find("yc01_poll", 240) : poll_s
        self.last = {"pH":0,"EC":0,"TDS":0,"ORP":0,"Cl":0,"Temp":0,"Batt":0,"SALT":0,"RSSI":0}
        self.last_ok = -1
        self.last_seen = -1
        self.rssi = 0
        self.tick = self.poll_s - 10
        self.awaiting = false
        self.watchdog = 0
        self.phase = 0
        self.fails = 0
        self.last_try = 0
        self.orp_offset = 0
        self.orp_raw = 0
        self.hold = false
        self.connected = false
        self.boost = persist.find("yc01_boost", 0) != 0
        self.profile_name = "Generic"
        self._mac_cache = nil
        self._fmin = [5.8, 1000, 640, 300, 0, 20, 0, 60]
        self._fmax = [6.2, 2500, 1600, 450, 0.2, 25, 750, 1000]
        self._fmar = [0.1, 200, 128, 50, 0.1, 2, 125, 30]
        self._persist_dirty = false
        self._retry_cnt = 0
        self._retry_at = 0
        _pf_count()
        _pf_build_index()
        self._load_profile(persist.find("yc01_profile", "Generic"))
        tasmota.add_rule("BLEOperation#read",   /v, t, m -> self._got_data(v, m))
        tasmota.add_rule("BLEOperation#notify", /v, t, m -> self._got_data(v, m))
        tasmota.add_rule("BLEOperation#state",  /v, t, m -> self._got_state(v, m))
        tasmota.add_rule("BLE#BLEDevices",      /v, t, m -> self._seen(v))
        log("YC01: v4.3 started, MAC " + self.mac)
    end

    def _mac_arg()
        if self._mac_cache == nil
            var m = self.mac
            if self.addr_type != 0   m = m + "/" + str(self.addr_type) end
            self._mac_cache = m
        end
        return self._mac_cache
    end

    def _is_ours(msg)
        if msg == nil   return true   end
        import string
        return string.find(string.toupper(str(msg)), self.mac_hex) >= 0
    end

    def read()   self._start(true)   end

    def _start(force)
        if self.awaiting   return end
        self.awaiting = true
        self.watchdog = 0
        self.last_try = tasmota.millis() / 1000
        self.phase = 3
        self._issue()
    end

    def _issue()
        var m = self._mac_arg()
        if self.phase == 3
            # No notifications (n:FF02) - meter only sends them on data change
            # which causes connection to die when data is stable.
            # Frequent reads (every 30s) keep the meter awake.
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 r go")
        end
    end

    def _finish(ok)
        self.awaiting = false
        self.phase = 0
        if ok
            self.fails = 0
            self._retry_cnt = 0
            self.last_ok = tasmota.millis() / 1000
        else
            self.fails += 1
            # Fast retry: up to 5 attempts, 5s apart
            if self._retry_cnt < 5
                self._retry_cnt += 1
                self._retry_at = tasmota.millis() / 1000 + 5
            end
        end
    end

    def _got_state(v, msg)
        if !self._is_ours(msg)   return   end
        import string
        v = str(v)
        self.watchdog = 0
        if v == "DONEREAD" || v == "DONENOTIFIED"   return   end
        if v == "DONEWRITE"   return   end
        if self.awaiting && string.find(v, "FAIL") >= 0
            self.connected = false
            self._finish(false)
        end
    end

    def _seen(v)
        if v == nil || self.mac_hex == ""   return   end
        var key = self._find_mac_key(v)
        if key == nil   return   end
        self.last_seen = tasmota.millis() / 1000
        var d = v[key]
        if d != nil && d.contains('r')   self.rssi = d['r'] end
        if self.fails > 0
            self.fails = 0
        end
    end

    def _find_mac_key(v)
        if v == nil   return nil   end
        import string
        var candidates = [self.mac_hex, string.tolower(self.mac_hex), self.mac_hex + "/0", self.mac_hex + "/1"]
        for k : candidates
            if v.contains(k)   return k   end
        end
        return nil
    end

    def _got_data(v, msg)
        if !self._is_ours(msg)   return   end
        self.watchdog = 0
        v = str(v)
        import string
        var p = string.find(v, "+")
        if p > 0   v = v[0..p-1] end
        var raw = bytes(v)
        if raw.size() < 5   return   end
        self._decode(raw)
        if raw[0] != 1   return end
        if raw[2] != 15  return end
        if raw[1] > 2 || raw.size() < 18   return end
        self._parse(raw)
        self._finish(true)
    end

    def _decode(d)
        var i = d.size() - 1
        while i > 0
            var t = d[i]
            var h1 = (t & 0x55) << 1
            var l1 = (t & 0xAA) >> 1
            t = d[i-1]
            var h0 = (t & 0x55) << 1
            var l0 = (t & 0xAA) >> 1
            d[i]   = 0xFF - (h1 | l0)
            d[i-1] = 0xFF - (h0 | l1)
            i -= 1
        end
    end

    def _parse(d)
        var ph = self._s16(d, 3) / 100.0
        var ec = self._s16(d, 5)
        var tds = self._s16(d, 7)
        self.orp_raw = self._s16(d, 9)
        var orp = self.orp_raw - self.orp_offset
        var cl_raw = self._s16(d, 11)
        var cl = (cl_raw < 0 ? 0 : cl_raw) / 10.0
        var temp = self._s16(d, 13) / 10.0
        var b = 100 * (self._s16(d, 15) - 1950) / 1240
        if b < 0   b = 0   end
        if b > 100   b = 100 end
        self.hold = (d.size() > 17) && ((d[17] >> 4) != 0)
        self.connected = true
        # Reuse pre-allocated map - just update values
        var m = self.last
        m["pH"] = ph
        m["EC"] = ec
        m["TDS"] = tds
        m["ORP"] = orp
        m["Cl"] = cl
        m["Temp"] = temp
        m["Batt"] = int(b + 0.5)
        m["SALT"] = ec * 0.55
        m["RSSI"] = self.rssi
    end

    def _s16(d, i)
        var v = (d[i] << 8) + d[i+1]
        if v > 32767   v -= 65536 end
        return v
    end

    # ---- profiles ----
    def _load_profile(name)
        if name == nil   name = "Generic" end
        var vals = _pf_find(name)
        if vals == nil
            log("YC01: profile '" + name + "' not found, using Generic", 2)
            name = "Generic"
            vals = _pf_find("Generic")
        end
        if vals == nil   return   end
        self.profile_name = name
        self._apply_vals(vals)
        if self.boost && vals[10] > 0
            self._apply_boost(vals)
        end
    end

    def _apply_vals(v)
        var ec_min = v[2], ec_max = v[3]
        var ec_mar = int((ec_max - ec_min) * 0.15 + 0.5)
        if ec_mar < 50   ec_mar = 50 end
        var tds_mar = int(ec_mar * 0.64)
        if tds_mar < 32   tds_mar = 32 end
        var cl_mar = v[6] * 0.5
        var salt_mar = v[9] * 0.15
        if salt_mar < 50   salt_mar = 50 end
        # Update arrays in-place to avoid GC
        var mn = self._fmin
        var mx = self._fmax
        var mg = self._fmar
        mn[0]=v[0]; mn[1]=ec_min; mn[2]=int(ec_min*0.64); mn[3]=v[7]; mn[4]=0; mn[5]=v[4]; mn[6]=0; mn[7]=60
        mx[0]=v[1]; mx[1]=ec_max; mx[2]=int(ec_max*0.64); mx[3]=v[8]; mx[4]=v[6]; mx[5]=v[5]; mx[6]=v[9]; mx[7]=100
        mg[0]=0.1; mg[1]=ec_mar; mg[2]=tds_mar; mg[3]=50; mg[4]=cl_mar; mg[5]=2; mg[6]=int(salt_mar+0.5); mg[7]=30
    end

    def _apply_boost(v)
        if v[10] != 1   return   end
        var ec_min = v[13], ec_max = v[14]
        var ec_mar = int((ec_max - ec_min) * 0.15 + 0.5)
        if ec_mar < 50   ec_mar = 50 end
        var tds_mar = int(ec_mar * 0.64)
        if tds_mar < 32   tds_mar = 32 end
        var mn = self._fmin
        var mx = self._fmax
        var mg = self._fmar
        mn[0]=v[11]; mn[1]=ec_min; mn[2]=int(ec_min*0.64); mn[3]=v[7]; mn[4]=0; mn[5]=v[15]; mn[6]=0; mn[7]=60
        mx[0]=v[12]; mx[1]=ec_max; mx[2]=int(ec_max*0.64); mx[3]=v[8]; mx[4]=v[6]; mx[5]=v[16]; mx[6]=v[9]; mx[7]=100
        mg[0]=0.1; mg[1]=ec_mar; mg[2]=tds_mar; mg[3]=50; mg[4]=mg[4]; mg[5]=2; mg[6]=mg[6]; mg[7]=30
    end

    def _range_color(v, key)
        var i = _COLOR_IDX.find(key)
        if i < 0   return "green" end
        var mn = self._fmin[i]
        var mx = self._fmax[i]
        var mg = self._fmar[i]
        if v < mn - mg || v > mx + mg   return "red"
        elif v < mn || v > mx           return "orange"
        end
        return "green"
    end

    # ---- scheduling ----
    def every_second()
        var now = tasmota.millis() / 1000
        # Flush dirty persist state (debounced flash writes)
        if self._persist_dirty
            persist.save()
            self._persist_dirty = false
        end
        if self.awaiting
            self.watchdog += 1
            if self.watchdog > 15   self._finish(false) end
            return
        end
        # Fast retry on failure: 5s gap, up to 5 attempts
        # Catches meter during BLE advertising windows after auto-off
        if self._retry_cnt > 0 && now >= self._retry_at
            self._retry_at = now + 5
            self._start(false)
            return
        end
        self.tick += 1
        if self.tick >= self.poll_s
            self.tick = 0
            self._start(false)
        end
    end

    # ---- web ----
    def web_sensor()
        import string
        var out = ""
        if self.last.size() == 0 || !self.connected
            out += "{s}Status{m}<span style='color:red'>Disconnected</span>{e}"
            tasmota.web_send_decimal(out)
            return
        end
        var m = self.last
        out += "{s}Status{m}<span style='color:green'>Connected</span>{e}"
        out += string.format("{s}pH{m}<span style='color:%s'>%.2f</span>{e}", self._range_color(m["pH"], "pH"), m["pH"])
        out += string.format("{s}EC{m}<span style='color:%s'>%i uS/cm</span>{e}", self._range_color(m["EC"], "EC"), m["EC"])
        out += string.format("{s}TDS{m}<span style='color:%s'>%i ppm</span>{e}", self._range_color(m["TDS"], "TDS"), m["TDS"])
        out += string.format("{s}ORP{m}<span style='color:%s'>%i mV</span>{e}", self._range_color(m["ORP"], "ORP"), m["ORP"])
        out += string.format("{s}SALT{m}<span style='color:%s'>%.1f ppm</span>{e}", self._range_color(m["SALT"], "SALT"), m["SALT"])
        out += string.format("{s}Temp{m}<span style='color:%s'>%.1f °C</span>{e}", self._range_color(m["Temp"], "Temp"), m["Temp"])
        out += string.format("{s}Chlorine{m}<span style='color:%s'>%.2f mg/L</span>{e}", self._range_color(m["Cl"], "Cl"), m["Cl"])
        out += string.format("{s}Battery{m}<span style='color:%s'>%i ％</span>{e}", self._range_color(m["Batt"], "Batt"), m["Batt"])
        tasmota.web_send_decimal(out)
    end
end

yc01 = YC01(nil, nil)
tasmota.add_driver(yc01)

# ---- console commands ----
def yc01read_cmd(cmd, idx, payload, payload_json)
    yc01.fails = 0
    yc01._start(true)
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01read", yc01read_cmd)

def yc01start_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_start()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01start", yc01start_cmd)

def yc01stop_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_stop()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01stop", yc01stop_cmd)

def yc01poll_cmd(cmd, idx, payload, payload_json)
    yc01.poll_s = int(payload)
    if yc01.poll_s < 30   yc01.poll_s = 30 end
    if yc01.poll_s > 600   yc01.poll_s = 600 end
    persist.yc01_poll = yc01.poll_s
    yc01._persist_dirty = true
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01poll", yc01poll_cmd)

def yc01profile_cmd(cmd, idx, payload, payload_json)
    var p = str(payload)
    if p == ""
        log("YC01: active profile: " + yc01.profile_name)
        log("YC01: boost: " + (yc01.boost ? "on" : "off"))
        var names = _pf_names()
        var line = ""
        for n : names
            if line != ""   line += ", "   end
            line += n
            if size(line) > 70
                log("YC01: " + line)
                line = ""
            end
        end
        if line != ""   log("YC01: " + line)   end
    else
        yc01._load_profile(p)
        persist.yc01_profile = yc01.profile_name
        yc01._persist_dirty = true
    end
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01profile", yc01profile_cmd)

def yc01boost_cmd(cmd, idx, payload, payload_json)
    var on = int(payload) != 0
    yc01.boost = on
    persist.yc01_boost = on ? 1 : 0
    yc01._persist_dirty = true
    yc01._load_profile(yc01.profile_name)
    log("YC01: boost " + (on ? "enabled" : "disabled"))
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01boost", yc01boost_cmd)

def yc01mac_cmd(cmd, idx, payload, payload_json)
    import string
    var p = str(payload)
    if p == ""
        log("YC01: MAC: " + yc01.mac + " type " + str(yc01.addr_type))
        tasmota.resp_cmnd_done()
        return true
    end
    # Parse MAC and optional type
    var parts = string.split(p, " ")
    var mac_str = parts[0]
    var atype = 0
    if size(parts) > 1   atype = int(parts[1]) end
    # Strip colons/dashes and validate
    mac_str = string.replace(string.upper(mac_str), ":", "")
    mac_str = string.replace(mac_str, "-", "")
    if size(mac_str) != 12
        log("YC01: invalid MAC, must be 12 hex digits", 2)
        tasmota.resp_cmnd_done()
        return true
    end
    for i : 0..11
        var c = mac_str[i]
        if (c < '0' || c > '9') && (c < 'A' || c > 'F')
            log("YC01: invalid MAC, must be 12 hex digits", 2)
            tasmota.resp_cmnd_done()
            return true
        end
    end
    yc01.mac = mac_str
    yc01.mac_hex = mac_str
    yc01.addr_type = atype
    yc01._mac_arg_cache = nil
    persist.yc01_mac = mac_str
    persist.yc01_addr_type = atype
    yc01._persist_dirty = true
    log("YC01: MAC set to " + mac_str + " type " + str(atype))
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01mac", yc01mac_cmd)
