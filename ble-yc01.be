#======================================================================
# yc01.be - BLE-YC01 (YIERYI / YINMIK 6-in-1 pool water monitor)
#           v4.1 - lazy-loaded binary profiles, minimal memory
#
# Configuration:
#   MAC address: change _PF_MAC to your meter's MAC
#   Profiles file: profiles.bin (generated from new_profiles.csv)
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
# Profile binary format (profiles.bin):
#   Header: 2 bytes = num_profiles
#   Records: 1 byte name_len + 40 bytes name + 17 × 4-byte int32 (val*100)
#   Record size: 109 bytes
#
# Commands: yc01read, yc01start, yc01stop, yc01poll, yc01profile
#======================================================================

import persist
import webserver

var _PF_COUNT = 0
var _PF_FNAME = "profiles.bin"
var _PF_MAC = "414284588113"  # <-- change this to your meter's MAC

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
    var h = f.read(2)
    _PF_COUNT = int(h[0]) + (int(h[1]) << 8)
    f.close()
    return _PF_COUNT
end

def _pf_find(name)
    var cnt = _pf_count()
    var f = _pf_open()
    if f == nil   return nil   end
    var name_bytes = bytes(name)
    var name_len = size(name)
    for i : 0..cnt-1
        f.seek(2 + i * 109)
        var rec = f.read(109)
        if size(rec) < 109   break   end
        var nl = int(rec[0])
        if nl == name_len
            var match = true
            for j : 0..nl-1
                if int(rec[1+j]) != int(name_bytes[j])
                    match = false
                    break
                end
            end
            if match
                f.close()
                var vals = []
                var pos = 41
                for k : 0..16
                    var b0 = int(rec[pos])
                    var b1 = int(rec[pos+1])
                    var b2 = int(rec[pos+2])
                    var b3 = int(rec[pos+3])
                    var iv = b0 + (b1 << 8) + (b2 << 16) + (b3 << 24)
                    if iv > 2147483647   iv -= 4294967296   end
                    vals.push(iv / 100.0)
                    pos += 4
                end
                return vals
            end
        end
    end
    f.close()
    return nil
end

def _pf_names()
    return ["Generic", "CherryTomato", "BeefsteakTomato", "BellPepper_Poblano_BananaPepper", "ChiliPepper_Jalapeno_Cayenne",
        "Habanero", "Eggplant", "Okra", "BushBeans", "PoleBeans", "Peas_SnowPeas_SugarSnapPeas", "Edamame",
        "Cantaloupe_HoneydewMelon_MiniWatermelon", "Strawberry_EverbearingStrawberry_AlpineStrawberry",
        "Lettuce_ButterheadLettuce_RomaineLettuce", "Spinach_NewZealandSpinach", "Kale_CollardGreens",
        "Arugula", "Endive_Escarole_Frisee_Radicchio", "Mache", "Microgreens", "BokChoy_Senposai_YukinaSavoy",
        "Tatsoi_Komatsuna_Mibuna", "MustardGreens", "Mizuna", "Watercress", "Amaranth", "MalabarSpinach",
        "Purslane", "Sorrel", "Celtuce", "SweetPotatoVine", "GarlicChives", "Basil_ThaiBasil_HolyBasil",
        "Parsley", "Cilantro", "Dill", "Mint_Peppermint_Spearmint", "Chives", "Oregano", "Thyme_Sage_Tarragon_Marjoram",
        "Rosemary_BayLaurel", "LemonBalm", "Lemongrass", "Stevia", "Shiso", "VietnameseCoriander", "Culantro",
        "Epazote", "Lovage", "SummerSavory", "WinterSavory", "Lavender", "Chamomile", "Feverfew_Hyssop_Echinacea",
        "Fennel", "Nasturtium_Calendula_Borage_Pansy_Viola", "Petunia", "GerberaDaisy", "Zinnia", "Snapdragon",
        "Begonia_Impatiens", "SweetAlyssum", "Lobelia", "Marigold", "Dianthus", "Cornflower", "Portulaca",
        "Ginger_Turmeric", "Claytonia", "LandCress", "WelshOnion", "Daylily", "FavaBeans", "BearsGarlic",
        "Lettuce_Spinach_Kale", "Lettuce_Arugula_Chard", "Lettuce_Basil_Parsley", "Spinach_Kale_Chard",
        "Microgreens", "SaladGreens", "Lettuce_Spinach_Arugula_Radish", "BokChoy_Tatsoi_Komatsuna", "Mizuna_Mibuna_Senposai",
        "Basil_Mint_Parsley", "Cilantro_Dill_Chives", "Rosemary_Thyme_Oregano", "LemonBalm_Chamomile_Mint",
        "Mediterranean_Herb", "Tomato_Pepper_Eggplant", "CherryTomato_BellPepper", "Pepper_Cucumber",
        "Eggplant_Zucchini_Pepper", "Tomato_CherryTomato_Roma", "BellPepper_ChiliPepper_Jalapeno", "Radish_Turnip_Carrot",
        "GreenOnion_GarlicChives_Leek", "Onion_Shallot_Chive", "Petunia_Lobelia_Alyssum", "BushBeans_PoleBeans_Edamame",
        "Watercress_Purslane_Sorrel", "Radicchio_Mache", "BeetGreens_TurnipGreens_CollardGreens", "Tomato_Basil",
        "Shiso_VietnameseCoriander_Culantro", "Lavender_Rosemary_Sage"]
end

class YC01 : Driver
    var mac, mac_hex, addr_type
    var poll_s
    var last, last_ok, last_seen, rssi
    var tick, awaiting, watchdog, phase
    var fails, last_try, last_forced_reconnect
    var orp_offset, orp_raw
    var hold
    var connected
    var profile_name
    var boost
    var _mac_cache
    var _fmin, _fmax, _fmar
    var _bmin, _bmax, _bmar  # boost arrays

    def init(mac, poll_s, addr_type)
        import string
        if mac == nil   mac = persist.find("yc01_mac", _PF_MAC) end
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
        self.connected = false
        self.boost = false
        self.profile_name = "Generic"
        self._mac_cache = nil
        # default range arrays (overwritten by _load_profile)
        self._fmin = [5.8, 1000, 640, 300, 0, 20, 0, 60]
        self._fmax = [6.2, 2500, 1600, 450, 0.2, 25, 750, 100]
        self._fmar = [0.1, 200, 128, 50, 0.1, 2, 125, 30]
        _pf_count()
        self._load_profile(persist.find("yc01_profile", "Generic"))
        tasmota.add_rule("BLEOperation#read",   /v, t, m -> self._got_data(v, m))
        tasmota.add_rule("BLEOperation#notify", /v, t, m -> self._got_data(v, m))
        tasmota.add_rule("BLEOperation#state",  /v, t, m -> self._got_state(v, m))
        tasmota.add_rule("BLE#BLEDevices",      /v, t, m -> self._seen(v))
        log("YC01: v4.1 started, MAC " + self.mac + ", " + str(_pf_count()) + " profiles")
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
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 n:FF02 k:60000 r go")
        end
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
        self.connected = true
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
        var vals = _pf_find(name)
        if vals == nil
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
        self._fmin = [v[0], ec_min, int(ec_min*0.64), v[7], 0,      v[4], 0,       60]
        self._fmax = [v[1], ec_max, int(ec_max*0.64), v[8], v[6],   v[5], v[9],    100]
        self._fmar = [0.1,   ec_mar,  tds_mar,         50,    cl_mar,  2,    int(salt_mar+0.5), 30]
    end

    def _apply_boost(v)
        if v[10] != 1   return   end
        var ec_min = v[13], ec_max = v[14]
        var ec_mar = int((ec_max - ec_min) * 0.15 + 0.5)
        if ec_mar < 50   ec_mar = 50 end
        var tds_mar = int(ec_mar * 0.64)
        if tds_mar < 32   tds_mar = 32 end
        self._fmin = [v[11], ec_min, int(ec_min*0.64), v[7], 0,      v[15], 0,       60]
        self._fmax = [v[12], ec_max, int(ec_max*0.64), v[8], v[6],   v[16], v[9],    100]
        self._fmar = [0.1,    ec_mar,  tds_mar,         50,    self._fmar[4],  2,    self._fmar[6], 30]
    end

    def _range_color(v, key)
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

    # ---- scheduling ----
    def every_second()
        var now = tasmota.millis() / 1000
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
    def web_sensor()
        import string
        if self.last.size() == 0 || !self.connected
            tasmota.web_send_decimal("{s}Status{m}Disconnected{e}")
            return
        end
        var m = self.last
        tasmota.web_send_decimal("{s}pH{m}" + self._span(string.format("%.2f", m["pH"]), self._range_color(m["pH"], "pH")) + "{e}")
        tasmota.web_send_decimal("{s}EC{m}" + self._span(string.format("%i uS/cm", m["EC"]), self._range_color(m["EC"], "EC")) + "{e}")
        tasmota.web_send_decimal("{s}TDS{m}" + self._span(string.format("%i ppm", m["TDS"]), self._range_color(m["TDS"], "TDS")) + "{e}")
        tasmota.web_send_decimal("{s}ORP{m}" + self._span(string.format("%i mV", m["ORP"]), self._range_color(m["ORP"], "ORP")) + "{e}")
        tasmota.web_send_decimal("{s}SALT{m}" + self._span(string.format("%.1f ppm", m["SALT"]), self._range_color(m["SALT"], "SALT")) + "{e}")
        tasmota.web_send_decimal("{s}Temp{m}" + self._span(string.format("%.1f °C", m["Temp"]), self._range_color(m["Temp"], "Temp")) + "{e}")
        tasmota.web_send_decimal("{s}Chlorine{m}" + self._span(string.format("%.2f mg/L", m["Cl"]), self._range_color(m["Cl"], "Cl")) + "{e}")
        tasmota.web_send_decimal("{s}Battery{m}" + self._span(string.format("%i ％", m["Batt"]), self._range_color(m["Batt"], "Batt")) + "{e}")
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
        persist.save()
    end
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01profile", yc01profile_cmd)

def yc01boost_cmd(cmd, idx, payload, payload_json)
    var on = int(payload) != 0
    yc01.boost = on
    yc01._load_profile(yc01.profile_name)
    log("YC01: boost " + (on ? "enabled" : "disabled"))
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01boost", yc01boost_cmd)
