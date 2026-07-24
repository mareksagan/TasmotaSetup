#======================================================================
# yc01.be - BLE-YC01 (YIERYI / YINMIK 6-in-1 pool water monitor)
#           v3.5 - protocol matching the original working script for this
#           meter firmware (public address, simple read+notify, old frame layout).
#
# Protocol summary:
#  * GATT: service 0xFF01, single characteristic 0xFF02 for
#    read + notify + WRITE.
#  * RX frames (meter -> app): every byte de-obfuscated (see decode()).
#  * Data frame: b[0]=1, b[1]=2, b[2]=15 (BLE-YC01), then BE s16 values at
#    offsets 3..15:
#      pH   = s16(b[3..4]) / 100
#      EC   = s16(b[5..6])            (raw uS/cm)
#      TDS  = s16(b[7..8])            (ppm)
#      ORP  = s16(b[9..10])           (mV)
#      Cl   = max(0, s16(b[11..12])) / 10   (mg/L)
#      Temp = s16(b[13..14]) / 10     (C)
#      Batt = clamp(100*(s16(b[15..16])-1950)/1240, 0, 100)
#  * The meter advertises as a public address (Tasmota shows 414284588113(0)),
#    and the original working script connected without an address-type suffix,
#    so this driver defaults to public (type 0).
#
# Console commands:
#   yc01read                      force an immediate read
#   yc01start / yc01stop          start / stop measurement stream
#   yc01sync                      send time sync
#   yc01power 0|1                 meter power (0=on, 1=off - careful!)
#   yc01buzzer 0|1                buzzer off/on
#   yc01limit <temp|ec|orp|cl> <min> <max> [scale]
#   yc01calreq                    request calibration data (shows in log/UI)
#   yc01calset <std_value>        calibration: set standard (e.g. 6.86)
#   yc01calapply                  calibration: confirm/apply
#   yc01send <hex>                send raw frame (checksum appended)
#   yc01info                      print device-info + calibration state
#   yc01active 0|1                set Tasmota BLE passive (0) / active (1) scan
#   yc01find                      run a 10 s manual BLE scan to locate the meter
#
# Troubleshooting connection timeouts:
#   - The meter must be awake and advertising.  Tasmota default scan is
#     PASSIVE; the YINMIK app uses ACTIVE scan, so run:  yc01active 1
#   - Make sure no phone/app is already connected to the meter.
#   - If POWER2 controls meter power, ensure it is ON and wait a few
#     seconds for the meter to start advertising before issuing yc01find.
#======================================================================

class YC01 : Driver
    var mac, mac_hex, addr_type
    var poll_s
    var last, last_ok, last_seen, rssi
    var tick, awaiting, watchdog, phase
    var fails, last_try, last_opp
    var do_init, do_sync, last_sync
    var queue, q_read
    var cal, devinfo

    # addr_type: 0 = public (default, matches Tasmota scan display), 1 = random.
    def init(mac, poll_s, addr_type)
        import string
        self.mac = mac
        self.mac_hex = string.toupper(string.replace(mac, ":", ""))
        self.addr_type = (addr_type == nil) ? 0 : addr_type
        self.poll_s = (poll_s == nil) ? 300 : poll_s
        self.last = {}
        self.last_ok = -1
        self.last_seen = -1
        self.rssi = 0
        self.tick = self.poll_s - 20
        self.awaiting = false
        self.watchdog = 0
        self.phase = 0                      # 0 idle, 1 sync, 2 init, 3 read, 4 cmd-writes
        self.fails = 0
        self.last_try = 0
        self.last_opp = 0
        self.do_init = false                # simple read+notify works for this meter
        self.do_sync = false
        self.last_sync = 0
        self.queue = []
        self.q_read = false
        self.cal = {}
        self.devinfo = ""
        tasmota.add_rule("BLEOperation#read",     /v, t, m -> self.got_data(v, m))
        tasmota.add_rule("BLEOperation#notify",   /v, t, m -> self.got_data(v, m))
        tasmota.add_rule("BLEOperation#state",    /v, t, m -> self.got_state(v, m))
        # The BLEDevices list is nested under the BLE topic, so listen to the
        # whole BLE message and extract BLEDevices manually.  This is more
        # robust than relying on the BLE#BLEDevices trigger.
        tasmota.add_rule("BLE",                  /v, t, m -> self._on_ble_msg(v, m))
        log(string.format("YC01: driver v3.5 started, MAC %s type %i poll %is",
                          self.mac, self.addr_type, self.poll_s))
        log("YC01: hint: if the meter is not found, try active scan: BLEScan0 1")
    end

    # ---- helpers -------------------------------------------------------

    def _mac_arg()
        var m = self.mac
        if self.addr_type != nil && self.addr_type != 0
            m = m + "/" + str(self.addr_type)
        end
        return m
    end

    def _is_ours(msg)
        if msg == nil   return true   end
        import string
        return string.find(string.toupper(str(msg)), self.mac_hex) >= 0
    end

    # build a TX frame from a list of payload ints, append XOR checksum
    def _tx(l)
        var b = bytes()
        var chk = 0
        for v : l
            b.add(v & 0xFF, 1)
            chk = chk ^ (v & 0xFF)
        end
        b.add(chk, 1)
        return b.tohex()
    end

    def _sync_payload()
        var t = tasmota.time_dump()
        if t == nil || t['year'] < 2020   return nil   end
        return [0x0B, 0x02, t['year'] % 2000, t['month'], t['day'],
                t['hour'], t['min'], t['sec']]
    end

    # queue one or more TX frames (hex), optionally followed by a read
    def send_frames(frames, and_read)
        for f : frames
            self.queue.push(f)
        end
        if and_read   self.q_read = true   end
        if !self.awaiting   self._kick_cmd()   end
    end

    def send_payload(l, and_read)
        self.send_frames([self._tx(l)], and_read)
    end

    # ---- connection orchestration --------------------------------------

    def read()
        self._start(true)
    end

    def _start(force)
        if self.awaiting   return end
        var now = tasmota.millis() / 1000
        if !force && self.fails > 0
            var wait = 60 * self.fails
            if wait > 900   wait = 900   end
            if now - self.last_try < wait   return end      # backoff
        end
        self.awaiting = true
        self.watchdog = 0
        self.last_try = now
        if self.last_seen < 0
            log("YC01: WARNING - meter never seen in BLE adverts; wake it and verify the MAC (try BLEScan0 1)", 2)
        end
        if self.do_sync && now - self.last_sync > 21600 && self._sync_payload() != nil
            self.phase = 1
        elif self.do_init
            self.phase = 2
        else
            self.phase = 3
        end
        self._issue()
    end

    def _kick_cmd()
        self.awaiting = true
        self.watchdog = 0
        self.last_try = tasmota.millis() / 1000
        self.phase = 4
        self._issue()
    end

    def _issue()
        var m = self._mac_arg()
        if self.phase == 1
            var p = self._sync_payload()
            if p == nil
                self.phase = self.do_init ? 2 : 3
                self._issue()
                return
            end
            self.last_sync = tasmota.millis() / 1000
            var f = self._tx(p)
            log("YC01: BLEOp write time-sync " + f)
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 w:" + f + " go")
        elif self.phase == 2
            log("YC01: BLEOp write session-start")
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 w:" + self._tx([1, 2, 0, 0, 0]) + " go")
        elif self.phase == 3
            log("YC01: BLEOp read+notify")
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 n:FF02 r go")
        else                                        # phase 4: queued command writes
            if self.queue.size() == 0
                if self.q_read
                    self.q_read = false
                    self.phase = 3
                    self._issue()
                else
                    self.awaiting = false
                    self.phase = 0
                end
                return
            end
            var f = self.queue[0]
            self.queue.remove(0)
            log("YC01: BLEOp write " + f)
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 w:" + f + " go")
        end
    end

    def _finish(ok, reason)
        self.awaiting = false
        self.phase = 0
        if ok
            self.fails = 0
            self.last_ok = tasmota.millis() / 1000
        else
            self.fails += 1
            import string
            log(string.format("YC01: BLE operation failed: %s (fail #%i, backoff %is)",
                              str(reason), self.fails,
                              (60 * self.fails > 900) ? 900 : 60 * self.fails), 2)
        end
        if self.queue.size() > 0   self._kick_cmd()   end
    end

    def got_state(v, msg)
        if !self._is_ours(msg)   return   end
        v = str(v)
        self.watchdog = 0                           # any event resets the phase watchdog
        if v == "DONEREAD" || v == "DONENOTIFIED"   return   end
        if v == "DONEWRITE"
            if !self.awaiting   return   end
            if self.phase == 4
                self._issue()                       # next queued frame / read / finish
            elif self.phase < 3
                self.phase += 1                     # sync -> init -> read
                self._issue()
            end
            return
        end
        # FAILCONNECT / FAILREAD / FAILWRITE / ...
        if self.awaiting
            import string
            if string.find(v, "FAIL") >= 0
                if v == "FAILCONNECT"
                    # device unreachable: no point trying further phases
                    self.queue = []
                    self.q_read = false
                    self._finish(false, v)
                elif self.phase == 4
                    self._issue()                   # skip failed write, drain queue
                elif self.phase < 3
                    self.phase = 3                  # failed write must not block the read
                    self._issue()
                else
                    self.queue = []
                    self.q_read = false
                    self._finish(false, v)
                end
                return
            end
            self._finish(false, v)
        end
    end

    # Handle the full BLE topic message; extract BLEDevices if present.
    def _on_ble_msg(v, msg)
        if v == nil   return   end
        if v.contains("BLEDevices")
            self.seen_adverts(v["BLEDevices"])
        end
    end

    def seen_adverts(v)
        if v == nil || self.mac_hex == ""   return   end
        if v.contains(self.mac_hex)
            self.last_seen = tasmota.millis() / 1000
            var d = v[self.mac_hex]
            if d != nil && d.contains('r')   self.rssi = d['r']   end
            if self.fails > 0
                self.fails = 0
                log("YC01: meter seen advertising again, backoff cleared")
            end
            # If we are idle and have not read recently, connect now while the
            # meter is advertising.  Avoid hammering: at most one opportunistic
            # attempt every 45 s.
            if !self.awaiting && self.phase == 0
                var now = tasmota.millis() / 1000
                if now - self.last_opp > 45 && now - self.last_try > 45
                    self.last_opp = now
                    log("YC01: meter advertising, opportunistic connect")
                    self._start(true)
                end
            end
        end
    end

    # ---- RX frame handling ----------------------------------------------

    def got_data(v, msg)
        if !self._is_ours(msg)   return   end
        self.watchdog = 0
        v = str(v)
        import string
        var p = string.find(v, "+")
        if p > 0   v = v[0..p-1]   end
        var raw = bytes(v)
        if raw.size() < 5   return   end
        self.decode(raw)

        if raw[0] == 2                          # calibration data
            if raw[2] == 15   self.parse_cal(raw)   end
            return
        end
        if raw[0] == 3                          # device info / settings
            self.devinfo = raw.tohex()
            log("YC01: device info frame: " + self.devinfo)
            return
        end
        if raw[0] == 4                          # ACK
            log("YC01: ACK frame: " + raw.tohex())
            return
        end
        if raw[0] != 1   return   end           # not a data frame
        if raw[2] != 15  return   end           # model 15 = BLE-YC01

        if raw[1] == 3                          # config changed response
            if raw.size() > 3 && raw[3] == 0xFF
                log("YC01: config changed, requesting cal data (app behaviour)")
                self.send_payload([2, 4, 0], true)
            else
                log("YC01: config-changed frame: " + raw.tohex())
            end
            return
        end
        if raw[1] == 4                          # ACK
            log("YC01: data ACK: " + raw.tohex())
            return
        end
        if raw[1] > 2 || raw.size() < 18   return   end

        self.parse(raw, 0)
        self._finish(true, "")
    end

    # exact port of the app's RX de-obfuscation (chained, right to left)
    def decode(d)
        var i = d.size() - 1
        while i > 0
            var t  = d[i]
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

    def u16(d, i)   return (d[i] << 8) + d[i+1]   end

    def s16(d, i)
        var v = (d[i] << 8) + d[i+1]
        if v > 32767   v -= 65536   end
        return v
    end

    # Parser matching the original working script for this meter.
    # Frame layout (after decode): type=1, sub=2, model=15, then BE u16/s16
    # values at offsets 3..15.
    def parse(d, flags)
        var m = {}
        m["pH"]   = self.s16(d, 3) / 100.0
        m["EC"]   = self.s16(d, 5)
        m["TDS"]  = self.s16(d, 7)
        m["ORP"]  = self.s16(d, 9)
        var cl = self.s16(d, 11)
        m["Cl"]   = (cl < 0 ? 0 : cl) / 10.0
        m["Temp"] = self.s16(d, 13) / 10.0
        var b = 100 * (self.s16(d, 15) - 1950) / 1240
        if b < 0     b = 0   end
        if b > 100   b = 100 end
        m["Batt"] = int(b + 0.5)
        # This meter firmware does not report SALT; keep the key so web_sensor
        # stays happy but set it to 0.  EC is always raw uS/cm here.
        m["SALT"] = 0
        m["ECu"]  = "uS/cm"
        m["RSSI"] = self.rssi
        self.last = m
        import string
        log(string.format("YC01: pH=%.2f EC=%i TDS=%i ORP=%imV Cl=%.1f T=%.1fC Batt=%i%% RSSI=%i",
                          m["pH"], m["EC"], m["TDS"], m["ORP"], m["Cl"], m["Temp"], m["Batt"], self.rssi))
    end

    # type 0x02 calibration data frame (app parser, params 1..5)
    def parse_cal(d)
        import string
        var param = d[3]
        var s = ""
        if param == 1                                   # pH buffer
            var bufs = {1: "4.00", 2: "6.86", 3: "7.00", 4: "9.18", 5: "10.01"}
            s = "pH buffer " + (bufs.contains(d[4]) ? bufs[d[4]] : "None")
        elif param == 2                                 # EC standard
            var std = self.u16(d, 4)
            var sc = d[6]
            if sc == 1
                s = string.format("EC %.2f mS/cm", std / 100.0)
            elif sc == 2
                s = string.format("EC %.1f mS/cm", std / 10.0)
            else
                s = string.format("EC %i uS/cm", std)
            end
        elif param == 3
            s = string.format("ORP %i mV", self.u16(d, 4))
        elif param == 4
            s = string.format("H2 %i ppb", self.u16(d, 4))
        elif param == 5
            s = string.format("DO %.1f %%", self.u16(d, 4) / 10.0)
        else
            s = "unknown param " + str(param) + ": " + d.tohex()
        end
        self.cal[str(param)] = s
        log("YC01: cal data: " + s)
    end

    # ---- TX command builders (called by console commands) ---------------

    def cmd_start()       self.send_payload([1, 1], true)            end
    def cmd_stop()        self.send_payload([1, 2, 0, 0, 0], false) end
    def cmd_sync()
        var p = self._sync_payload()
        if p == nil
            log("YC01: no valid RTC, cannot time-sync", 2)
            return
        end
        self.send_payload(p, false)
    end
    def cmd_power(on)     self.send_payload([1, 0, on ? 0 : 1], false) end
    def cmd_buzzer(on)    self.send_payload([1, 4, on ? 0 : 1], false) end
    def cmd_calreq()      self.send_payload([2, 4, 0], true)         end
    def cmd_calapply()    self.send_payload([1, 0x13, 1, 0, 0, 0, 0], false) end

    # Tasmota defaults to passive BLE scan.  The YINMIK app uses active scan
    # (Android SCAN_MODE active), so enabling active scan can help find the
    # meter when it only exposes its name/service in scan-response packets.
    def cmd_active_scan(on)
        tasmota.cmd("BLEScan0 " + (on ? "1" : "0"))
        log("YC01: BLE scan set to " + (on ? "active" : "passive") +
            " (hint: active scan helps some phones/meters)")
    end

    # Trigger a manual 10 s BLE scan and report whether the meter is seen.
    def cmd_find()
        log("YC01: starting 10 s BLE scan for meter " + self.mac)
        tasmota.cmd("BLEScan1 10")
    end

    def cmd_calset(std)
        var v = int(std * 1000 + 0.5)
        if v < 0   v = 0   end
        if v > 20000   v = 20000   end            # app clamps to 20.0
        self.send_payload([1, 0x12, 1, 0, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF], false)
    end

    def cmd_limit(ltype, vmin, vmax, scale)
        var cmd = 0
        if ltype == "temp"
            cmd = 0x0F
        elif ltype == "ec"
            cmd = 0x0E
        elif ltype == "orp"
            cmd = 0x10
        elif ltype == "cl"
            cmd = 0x11
        else
            log("YC01: limit type must be temp|ec|orp|cl", 2)
            return
        end
        var mn = int(vmin * scale + (vmin < 0 ? -0.5 : 0.5))
        var mx = int(vmax * scale + (vmax < 0 ? -0.5 : 0.5))
        if mn < 0   mn += 65536   end
        if mx < 0   mx += 65536   end
        if mn > 65535   mn = 65535   end
        if mx > 65535   mx = 65535   end
        self.send_payload([1, cmd, 1, 0,
                           (mn >> 8) & 0xFF, mn & 0xFF,
                           (mx >> 8) & 0xFF, mx & 0xFF, 0], false)
    end

    # ---- scheduling ------------------------------------------------------

    def every_second()
        if self.awaiting
            self.watchdog += 1
            if self.watchdog > 45
                self.queue = []
                self.q_read = false
                self._finish(false, "WATCHDOG")
            end
            return
        end
        self.tick += 1
        if self.tick >= self.poll_s
            self.tick = 0
            self._start(false)
        end
    end

    # ---- presentation ----------------------------------------------------

    def _age()
        if self.last_ok < 0   return -1   end
        return tasmota.millis() / 1000 - self.last_ok
    end

    def json_append()
        if self.last.size() == 0   return end
        import json
        tasmota.response_append(',"YC01":' + json.dump(self.last))
    end

    # web_send_decimal wraps printf-style formatting, so any literal '%'
    # in our already-formatted strings must be doubled or the page crashes.
    def _ws(s)
        import string
        tasmota.web_send_decimal(string.replace(s, "%", "%%"))
    end

    def web_sensor()
        try
            import string
            # Always show driver state so the page tells us why values are missing.
            var now = tasmota.millis() / 1000
            var state = self.awaiting ? ("busy (phase " + str(self.phase) + ")") : "idle"
            self._ws("{s}YC01 driver{m}" + state + "{e}")
            self._ws(string.format("{s}Fails{m}%i{e}", self.fails))
            if self.last_seen < 0
                self._ws("{s}YC01 beacon{m}never seen{e}")
            else
                self._ws(string.format("{s}YC01 beacon last seen{m}%i s ago{e}", now - self.last_seen))
            end
            if self.last.size() == 0
                self._ws("{s}YC01{m}waiting for first read...{e}")
            else
                self._ws(string.format("{s}pH{m}%.2f{e}",                 self.last["pH"]))
                self._ws(string.format("{s}EC (%s){m}%.1f{e}",            self.last["ECu"], self.last["EC"]))
                self._ws(string.format("{s}TDS{m}%.1f ppm{e}",            self.last["TDS"]))
                self._ws(string.format("{s}SALT{m}%.1f{e}",               self.last["SALT"]))
                self._ws(string.format("{s}ORP{m}%.2f{e}",                self.last["ORP"]))
                self._ws(string.format("{s}Chlorine{m}%.2f mg/L{e}",      self.last["Cl"]))
                self._ws(string.format("{s}Temperature{m}%.1f &deg;C{e}", self.last["Temp"]))
                self._ws(string.format("{s}Battery{m}%i %%{e}",           self.last["Batt"]))
                var age = self._age()
                var stale = (age < 0 || age > self.poll_s * 2.5)
                self._ws(string.format("{s}Last update{m}%i s ago%s{e}",
                                         (age < 0) ? 0 : age, stale ? " (STALE)" : ""))
                self._ws(string.format("{s}RSSI{m}%i dBm{e}",             self.rssi))
            end
            if self.cal.size() > 0
                for k : self.cal.keys()
                    self._ws("{s}Cal " + k + "{m}" + self.cal[k] + "{e}")
                end
            end
        except .. as e
            log("YC01: web_sensor error: " + str(e), 2)
        end
    end
end

# addr_type defaults to 1 (random static) - required for a 41:.. MAC
yc01 = YC01("414284588113", 300)
tasmota.add_driver(yc01)

# ---- console commands ---------------------------------------------------

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

def yc01sync_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_sync()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01sync", yc01sync_cmd)

def yc01power_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_power(int(payload) != 0)
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01power", yc01power_cmd)

def yc01buzzer_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_buzzer(int(payload) != 0)
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01buzzer", yc01buzzer_cmd)

def yc01calreq_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_calreq()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01calreq", yc01calreq_cmd)

def yc01calset_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_calset(real(payload))
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01calset", yc01calset_cmd)

def yc01calapply_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_calapply()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01calapply", yc01calapply_cmd)

def yc01limit_cmd(cmd, idx, payload, payload_json)
    import string
    var a = string.split(str(payload), " ")
    if a.size() < 3
        log("YC01: usage: yc01limit <temp|ec|orp|cl> <min> <max> [scale]", 2)
        tasmota.resp_cmnd_done()
        return true
    end
    var sc = (a.size() > 3) ? real(a[3]) : 0.0
    if sc == 0.0
        # app defaults: temp x10, ec x100, orp x10, cl x10
        sc = (a[0] == "ec") ? 100.0 : 10.0
    end
    yc01.cmd_limit(a[0], real(a[1]), real(a[2]), sc)
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01limit", yc01limit_cmd)

def yc01send_cmd(cmd, idx, payload, payload_json)
    import string
    var h = string.toupper(string.replace(str(payload), " ", ""))
    if h == ""
        log("YC01: usage: yc01send <hex payload, checksum appended>", 2)
    else
        var l = []
        var i = 0
        while i + 1 < size(h)
            l.push(int("0x" + h[i..i+1]))
            i += 2
        end
        yc01.send_payload(l, false)
    end
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01send", yc01send_cmd)

def yc01info_cmd(cmd, idx, payload, payload_json)
    import json
    var m = {"devinfo": yc01.devinfo, "cal": yc01.cal,
             "last_seen": yc01.last_seen, "last_ok": yc01.last_ok,
             "fails": yc01.fails, "rssi": yc01.rssi}
    tasmota.response_append(json.dump(m))
    return true
end
tasmota.add_cmd("yc01info", yc01info_cmd)

def yc01active_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_active_scan(int(payload) != 0)
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01active", yc01active_cmd)

def yc01find_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_find()
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01find", yc01find_cmd)
