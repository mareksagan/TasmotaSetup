#======================================================================
# yc01.be - BLE-YC01 (YIERYI / YINMIK 6-in-1 pool water monitor)
#           v4.0 - lean, fast, resource-efficient
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
#
# Commands: yc01read, yc01start, yc01stop, yc01poll, yc01profile
#======================================================================

import persist
import webserver

# Profile data: "name:ph_min,ph_max,ec_min,ec_max,t_min,t_max,cl_max,orp_min,orp_max,salt_max;..."
var _PROFILE_DATA = "Generic:5.8,6.2,1000,2500,20,25,0.2,300,450,750;Tomato:6.0,6.3,2000,3200,20,26,0.2,300,450,750;Cherry Tomato:6.0,6.3,1800,2800,20,26,0.2,300,450,750;Beefsteak Tomato:6.0,6.3,2000,3500,20,26,0.2,300,450,750;Roma Tomato:6.0,6.3,2000,3200,20,26,0.2,300,450,750;Heirloom Tomato:6.0,6.3,2000,3200,20,26,0.2,300,450,750;Strawberry:6.0,6.2,1400,2000,15,22,0.1,300,450,500;Basil:5.8,6.2,1200,1800,20,24,0.1,300,450,500;Mint:5.8,6.2,1200,1600,18,22,0.2,300,450,500;Lettuce:6.0,6.2,1200,2000,18,22,0.1,300,450,500;Butterhead Lettuce:6.0,6.2,1200,2000,18,22,0.1,300,450,500;Romaine Lettuce:6.0,6.2,1200,2000,18,22,0.1,300,450,500;Spinach:6.0,6.5,1200,1600,18,22,0.1,300,450,500;Cucumber:5.8,6.0,1600,2400,20,25,0.2,300,450,750;Bell Pepper:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Jalapeno:6.0,6.3,1600,2600,22,27,0.2,300,450,750;Habanero:6.0,6.3,1600,2800,22,28,0.2,300,450,750;Cayenne:6.0,6.3,1600,2600,22,27,0.2,300,450,750;Arugula:6.0,6.2,1500,1800,18,22,0.2,300,450,500;Dill:5.8,6.2,1000,1400,18,22,0.2,300,450,500;Parsley:6.0,6.5,1000,1600,18,22,0.2,300,450,500;Cilantro:6.0,6.5,1000,1400,18,22,0.2,300,450,500;Chives:6.0,6.5,1000,1400,18,22,0.2,300,450,500;Kale:6.0,6.5,1200,1800,18,22,0.2,300,450,1000;Swiss Chard:6.0,6.5,1200,1600,18,22,0.2,300,450,1000;Oregano:6.0,6.5,1200,1800,18,22,0.2,300,450,750;Thyme:6.0,6.5,1000,1600,18,22,0.2,300,450,750;Rosemary:6.0,6.5,1200,1800,18,24,0.2,300,450,750;Sage:6.0,6.5,1000,1600,18,22,0.2,300,450,750;Lemon Balm:5.8,6.2,1000,1400,18,22,0.2,300,450,750;Chamomile:6.0,6.5,800,1200,18,22,0.2,300,450,750;Microgreens:5.8,6.2,500,900,18,22,0.1,300,450,400;Bush Beans:6.0,6.3,1400,2000,18,24,0.2,300,450,500;Pole Beans:6.0,6.3,1400,2200,18,24,0.2,300,450,500;Edamame:6.0,6.3,1400,2000,18,24,0.2,300,450,750;Eggplant:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Zucchini:6.0,6.3,1600,2400,20,25,0.2,300,450,750;Cantaloupe:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Honeydew:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Watermelon:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Okra:6.0,6.5,1400,2200,22,28,0.2,300,450,750;Tomato + Basil (Companion):5.8,6.2,1400,2200,20,26,0.2,300,450,750;Lettuce + Spinach + Arugula:5.8,6.2,1200,1800,18,22,0.1,300,450,500;Bell Pepper + Eggplant:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Rosemary + Thyme + Oregano:6.0,6.5,1200,1800,18,22,0.2,300,450,750;Bush Beans + Edamame:6.0,6.3,1400,2000,18,24,0.2,300,450,500;Cantaloupe + Honeydew + Watermelon:6.0,6.3,1600,2400,22,26,0.2,300,450,750;Zucchini + Summer Squash:6.0,6.3,1600,2400,20,25,0.2,300,450,750;Bok Choy + Tatsoi + Komatsuna:6.0,6.2,1000,1400,18,22,0.1,300,450,500;Endive + Escarole + Frisee:6.0,6.2,1000,1400,18,22,0.1,300,450,500;Tomato + Bell Pepper + Eggplant:6.0,6.3,1800,2600,20,26,0.2,300,450,750"

# Profile lookup: name -> [ph_min, ph_max, ec_min, ec_max, t_min, t_max, cl_max, orp_min, orp_max, salt_max]
var _PFL = {}

def _pfl_build()
    if size(_PFL) > 0   return   end
    import string
    var d = _PROFILE_DATA
    var pos = 0
    var dlen = size(d)
    while pos < dlen
        var c = string.find(d, ":", pos)
        if c < 0   break   end
        var s = string.find(d, ";", c)
        if s < 0   s = dlen   end
        var name = d[pos..c-1]
        var vals = string.split(d[c+1..s-1], ",")
        _PFL[name] = [real(vals[0]), real(vals[1]), int(vals[2]), int(vals[3]), int(vals[4]), int(vals[5]), real(vals[6]), int(vals[7]), int(vals[8]), int(vals[9])]
        pos = s + 1
    end
end

class YC01 : Driver
    var mac, mac_hex, addr_type
    var poll_s
    var last, last_ok, last_seen, rssi
    var tick, awaiting, watchdog, phase
    var fails, last_try, last_forced_reconnect
    var orp_offset, orp_raw
    var hold
    var scan_init
    var profile_name
    var _mac_cache
    var _fmin, _fmax, _fmar  # flat range arrays

    def init(mac, poll_s, addr_type)
        import string
        if mac == nil   mac = persist.find("yc01_mac", "414284588113") end
        if addr_type == nil   addr_type = persist.find("yc01_addr_type", 0) end
        self.mac = mac
        self.mac_hex = string.toupper(string.replace(mac, ":", ""))
        self.addr_type = addr_type
        self.poll_s = (poll_s == nil) ? persist.find("yc01_poll", 50) : poll_s
        self.last = {}
        self.last_ok = -1
        self.last_seen = -1
        self.rssi = 0
        self.tick = self.poll_s - 10
        self.awaiting = false
        self.watchdog = 0
        self.phase = 0
        self.fails = 0
        self.last_try = 0
        self.last_forced_reconnect = 0
        self.orp_offset = 0
        self.orp_raw = 0
        self.hold = false
        self.scan_init = false
        self._mac_cache = nil
        _pfl_build()
        self._load_profile(persist.find("yc01_profile", "Generic"))
        tasmota.add_rule("BLEOperation#read",   /v, t, m -> self._got_data(v, m))
        tasmota.add_rule("BLEOperation#notify", /v, t, m -> self._got_data(v, m))
        tasmota.add_rule("BLEOperation#state",  /v, t, m -> self._got_state(v, m))
        tasmota.add_rule("BLE#BLEDevices",      /v, t, m -> self._seen(v))
        tasmota.add_rule("BLE",                 /v, t, m -> self._on_ble(v, m))
        log("YC01: v4.0 started, MAC " + self.mac)
    end

    # ---- BLE helpers ----
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

    # ---- connection state machine ----
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
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 n:FF02 k:60000 r go")
        end
    end

    def cmd_start()       self.send_payload([1, 1], true)            end
    def cmd_stop()        self.send_payload([1, 2, 0, 0, 0], false) end

    def send_payload(l, and_read)
        var b = bytes()
        var chk = 0
        for v : l
            b.add(v & 0xFF, 1)
            chk = chk ^ (v & 0xFF)
        end
        b.add(chk, 1)
        var m = self._mac_arg()
        tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 k:60000 w:" + b.tohex() + " go")
    end

    def _finish(ok)
        self.awaiting = false
        self.phase = 0
        if ok
            self.fails = 0
            self.last_ok = tasmota.millis() / 1000
        else
            self.fails += 1
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
            self._finish(false)
        end
    end

    def _on_ble(v, msg)
        var devices = self._extract(v)
        if devices == nil   devices = self._extract(msg) end
        if devices != nil   self._seen(devices) end
    end

    def _extract(x)
        if x == nil   return nil   end
        import json
        var m = x
        if type(m) == 'string'
            try   m = json.load(m)   except ..   return nil   end
        end
        if type(m) != 'instance' && type(m) != 'map'   return nil   end
        if m.contains("BLEDevices")   return m["BLEDevices"] end
        return nil
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

    # ---- RX frame handling ----
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
        import string
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
        var m = {"pH":ph, "EC":ec, "TDS":tds, "ORP":orp, "Cl":cl, "Temp":temp, "Batt":int(b+0.5), "SALT":ec*0.55, "RSSI":self.rssi}
        self.last = m
    end

    def _s16(d, i)
        var v = (d[i] << 8) + d[i+1]
        if v > 32767   v -= 65536 end
        return v
    end

    # ---- profiles ----
    def _load_profile(name)
        if name == nil   name = "Generic" end
        var vals = _PFL.find(name)
        if vals == nil
            name = "Generic"
            vals = _PFL.find("Generic")
        end
        self.profile_name = name
        var ec_min = vals[2], ec_max = vals[3]
        var ec_mar = int((ec_max - ec_min) * 0.15 + 0.5)
        if ec_mar < 50   ec_mar = 50 end
        var tds_mar = int(ec_mar * 0.64)
        if tds_mar < 32   tds_mar = 32 end
        var cl_mar = vals[6] * 0.5
        var salt_mar = vals[9] * 0.15
        if salt_mar < 50   salt_mar = 50 end
        self._fmin =    [vals[0], ec_min, int(ec_min*0.64), vals[7], 0,      vals[4], 0,       60]
        self._fmax =    [vals[1], ec_max, int(ec_max*0.64), vals[8], vals[6], vals[5], vals[9],  100]
        self._fmar =    [0.1,     ec_mar, tds_mar,         50,     cl_mar,  2,       int(salt_mar+0.5), 30]
    end

    # ---- scheduling ----
    def every_second()
        var now = tasmota.millis() / 1000
        if !self.scan_init && now > 10
            tasmota.cmd("BLEScan0 1")
            self.scan_init = true
        end
        if self.awaiting
            self.watchdog += 1
            if self.watchdog > 45   self._finish(false) end
            return
        end
        self.tick += 1
        if self.tick >= self.poll_s
            self.tick = 0
            self._start(false)
        end
    end

    # ---- web ----
    def _col(v, key)
        var i = -1
        if key == "pH"    i = 0
        elif key == "EC"   i = 1
        elif key == "TDS"  i = 2
        elif key == "ORP"  i = 3
        elif key == "Cl"   i = 4
        elif key == "Temp" i = 5
        elif key == "SALT" i = 6
        elif key == "Batt" i = 7
        end
        if i < 0   return "green" end
        var mn = self._fmin[i]
        var mx = self._fmax[i]
        var mg = self._fmar[i]
        if v < mn - mg || v > mx + mg   return "red"
        elif v < mn || v > mx           return "orange"
        end
        return "green"
    end

    def _span(val, color)
        return "<span style='color:" + color + "'>" + str(val) + "</span>"
    end

    def web_sensor()
        if self.last.size() == 0
            tasmota.web_send_decimal("{s}Status{m}Not connected{e}")
            return
        end
        import string
        var m = self.last
        tasmota.web_send_decimal("{s}pH{m}" + self._span(string.format("%.2f", m["pH"]), self._col(m["pH"], "pH")) + "{e}")
        tasmota.web_send_decimal("{s}EC{m}" + self._span(string.format("%i", m["EC"]), self._col(m["EC"], "EC")) + "{e}")
        tasmota.web_send_decimal("{s}TDS{m}" + self._span(string.format("%i", m["TDS"]), self._col(m["TDS"], "TDS")) + "{e}")
        tasmota.web_send_decimal("{s}ORP{m}" + self._span(string.format("%i", m["ORP"]), self._col(m["ORP"], "ORP")) + "{e}")
        tasmota.web_send_decimal("{s}Temp{m}" + self._span(string.format("%.1f", m["Temp"]), self._col(m["Temp"], "Temp")) + "{e}")
        tasmota.web_send_decimal("{s}Chlorine{m}" + self._span(string.format("%.2f", m["Cl"]), self._col(m["Cl"], "Cl")) + "{e}")
        tasmota.web_send_decimal("{s}Battery{m}" + self._span(string.format("%i", m["Batt"]), self._col(m["Batt"], "Batt")) + "{e}")
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
    if yc01.poll_s < 10   yc01.poll_s = 10 end
    if yc01.poll_s > 3600   yc01.poll_s = 3600 end
    persist.yc01_poll = yc01.poll_s
    persist.save()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01poll", yc01poll_cmd)

def yc01profile_cmd(cmd, idx, payload, payload_json)
    var p = str(payload)
    if p == ""
        log("YC01: active profile: " + yc01.profile_name)
    else
        yc01._load_profile(p)
        persist.yc01_profile = yc01.profile_name
        persist.save()
    end
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01profile", yc01profile_cmd)
