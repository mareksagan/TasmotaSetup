#======================================================================
# yc01.be - BLE-YC01 (YIERYI / YINMIK 6-in-1 pool water monitor)
#           v3.5 - protocol matching the original working script for this
#           meter firmware (public address, simple read+notify, old frame layout).
#
# Protocol summary (from WaterQualityApp / BLE-YC01-PROTOCOL.md):
#  * GATT: service 0xFF01, characteristic 0xFF02 for READ + WRITE.
#    The meter does NOT push notifications; the host must READ FF02.
#  * RX frames (meter -> app): every byte de-obfuscated (see decode()).
#  * Data frame: b[0]=1, b[1]=2, b[2]=15 (BLE-YC01), then BE s16 values at
#    offsets 3..15:
#      pH   = s16(b[3..4]) / 100
#      EC   = s16(b[5..6])            (raw uS/cm)
#      TDS  = s16(b[7..8])            (ppm)
#      SALT = EC * 0.55               (ppm, app/HA convention)
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
#   yc01orpcal <mV>               match displayed ORP to meter/app (e.g. 268)
#   yc01send <hex>                send raw frame (checksum appended)
#   yc01info                      print device-info + calibration state
#   yc01active 0|1                set Tasmota BLE passive (0) / active (1) scan
#   yc01find                      run a 10 s manual BLE scan to locate the meter
#   yc01poll <seconds>            change read interval (default 50, min 10)
#
# Troubleshooting connection timeouts / stale values:
#   - The meter must be awake and advertising.  Tasmota default scan is
#     PASSIVE; the YINMIK app uses ACTIVE scan, so run:  yc01active 1
#   - Make sure no phone/app is already connected to the meter.
#   - This meter is battery powered; no relay power control is needed.
#   - The meter auto-powers off after ~5 min without an active BLE connection.
#     The official app keeps a persistent connection and reads FF02 every
#     4 min.  This driver defaults to 50 s polling with keep-alive so the
#     GATT connection is reused between reads and the meter stays awake.
#   - On boot the driver automatically enables active scan and begins
#     scheduled reads of FF02 (the app keeps a connection and reads FF02).
#   - To force a start-measurement command manually:  yc01start
#   - If the meter becomes unresponsive after failed connections, remove the
#     batteries for a few seconds (the protocol notes that improper
#     disconnects can freeze the device).
#======================================================================

import persist
import webserver

# Pre-computed HTML span tags for range colors.  Avoids string concat in
# the web_sensor() hot path (called on every web UI refresh).
var _SPAN_G = "<span style='color:green'>"
var _SPAN_O = "<span style='color:orange'>"
var _SPAN_R = "<span style='color:red'>"
var _SPAN_X = "</span>"
var _S = "{s}"
var _M = "{m}"
var _E = "{e}"

class YC01 : Driver
    var mac, mac_hex, addr_type
    var poll_s, sync_max
    var last, last_ok, last_seen, rssi
    var tick, awaiting, watchdog, phase
    var fails, last_try, last_opp, last_forced_reconnect
    var do_init, do_sync, last_sync
    var queue, q_read
    var cal, devinfo
    var orp_offset, orp_raw
    var hold
    var scan_init
    var profile_name, ranges
    var _gr_cache
    var _mac_arg_cache
    var _profile_file_cache
    var _profile_names_cache
    var _parse_map
    var _flat_mins, _flat_maxs, _flat_margins
    var _persist_dirty, _persist_last_save

    # addr_type: 0 = public (default, matches Tasmota scan display), 1 = random.
    def init(mac, poll_s, addr_type)
        import string
        if mac == nil
            mac = persist.find("yc01_mac", "414284588113")
        end
        if addr_type == nil
            addr_type = persist.find("yc01_addr_type", 0)
        end
        self.mac = mac
        self.mac_hex = string.toupper(string.replace(mac, ":", ""))
        self.addr_type = addr_type
        self.poll_s = (poll_s == nil) ? persist.find("yc01_poll", 50) : poll_s
        self.sync_max = persist.find("yc01_sync_max", 240)
        if self.sync_max < 30   self.sync_max = 30   end
        if self.sync_max > 3600   self.sync_max = 3600   end
        self.profile_name = persist.find("yc01_profile", "Generic")
        self._gr_cache = nil
        self._load_profile(self.profile_name)  # builds flat arrays + ranges; file cached after
        self.last = {}
        self.last_ok = -1
        self.last_seen = -1
        self.rssi = 0
        self.tick = self.poll_s - 10
        self.awaiting = false
        self.watchdog = 0
        self.phase = 0                      # 0 idle, 1 sync, 2 init, 3 read, 4 cmd-writes
        self.fails = 0
        self.last_try = 0
        self.last_opp = 0
        self.last_forced_reconnect = 0
        self.do_init = false                # simple read+notify works for this meter
        self.do_sync = false
        self.last_sync = 0
        self.queue = []
        self.q_read = false
        self.cal = {}
        self.devinfo = ""
        self.orp_offset = 0                 # manual ORP calibration offset (mV)
        self.orp_raw = 0                    # last raw ORP before offset
        self.hold = false                   # hold flag from status byte (offset 17)
        self.scan_init = false
        self._mac_arg_cache = nil
        self._profile_file_cache = nil
        self._profile_names_cache = nil
        self._parse_map = nil
        self._persist_dirty = false
        self._persist_last_save = 0
        tasmota.add_rule("BLEOperation#read",     /v, t, m -> self.got_data(v, m))
        tasmota.add_rule("BLEOperation#notify",   /v, t, m -> self.got_data(v, m))
        tasmota.add_rule("BLEOperation#state",    /v, t, m -> self.got_state(v, m))
        # Try both triggers; one passes BLEDevices as value, the other nests it
        # inside the full BLE message.
        tasmota.add_rule("BLE#BLEDevices",        /v, t, m -> self.seen_adverts(v))
        tasmota.add_rule("BLE",                   /v, t, m -> self._on_ble_msg(v, m))
        if tasmota.is_network_up()
            self.web_add_handler()
        end
        log(string.format("YC01: driver v3.5 started, MAC %s type %i poll %is",
                          self.mac, self.addr_type, self.poll_s))
        log("YC01: active scan will be enabled automatically in ~10 s")
    end

    # ---- helpers -------------------------------------------------------

    # Profile-based hydroponic range thresholds.
    # Profiles are stored in yc01_profiles.csv as:
    #   name,pH_min,pH_max,EC_min,EC_max,Temp_min,Temp_max,Cl_max,ORP_min,ORP_max,SALT_max
    # Only the active profile is expanded into the full range map, so the Berry
    # heap stays small on ESP32.
    # Green = inside optimal range, orange = within margin of a limit, red = outside.

    def _generic_ranges()
        if self._gr_cache == nil
            self._gr_cache = {
                "pH":   {"min": 5.8,  "max": 6.2,  "margin": 0.1},
                "EC":   {"min": 1000, "max": 2500, "margin": 200},
                "TDS":  {"min": 640,  "max": 1600, "margin": 128},
                "ORP":  {"min": 300,  "max": 450,  "margin": 50},
                "Cl":   {"min": 0,    "max": 0.2,  "margin": 0.1},
                "Temp": {"min": 20,   "max": 25,   "margin": 2},
                "SALT": {"min": 0,    "max": 750,  "margin": 125},
                "Batt": {"min": 60,   "max": 100,  "margin": 30}
            }
        end
        return self._gr_cache
    end

    # Profile table is stored in a separate UFS file (yc01_profiles.csv) so the
    # driver itself stays small.  The settings page and profile lookup both read
    # the file line-by-line; only one profile line is ever in Berry RAM at a time.

    def _profile_file()
        return "yc01_profiles.csv"
    end

    # Read the entire CSV file into a cached string.  Subsequent profile lookups
    # and name listings work from this cache — zero file I/O after first read.
    # Returns nil on error.  Cache persists until _invalidate_profile_cache().
    def _read_profile_file()
        if self._profile_file_cache != nil
            return self._profile_file_cache
        end
        import string
        var fname = self._profile_file()
        var f
        try
            f = open(fname, "r")
        except .. as e
            log("YC01: cannot open " + fname + ": " + str(e), 2)
            return nil
        end
        try
            self._profile_file_cache = f.read()
        except .. as e
            log("YC01: error reading " + fname + ": " + str(e), 2)
            f.close()
            return nil
        end
        f.close()
        return self._profile_file_cache
    end

    # Invalidate all profile caches (file content, name list).  Call after
    # uploading a new CSV file.
    def _invalidate_profile_cache()
        self._profile_file_cache = nil
        self._profile_names_cache = nil
    end

    # Find the profile line matching 'name' by scanning the cached file content.
    # Returns nil if the file is missing or the profile is not found.
    def _find_profile_line(name)
        import string
        var content = self._read_profile_file()
        if content == nil   return nil   end
        var prefix = name + ","
        var lines = string.split(content, "\n")
        for line : lines
            if string.find(line, prefix) == 0
                # strip trailing \r only
                if size(line) > 0 && line[size(line)-1] == "\r"
                    return line[0..size(line)-2]
                end
                return line
            end
        end
        return nil
    end

    # Get all profile names from the cached file.  Returns a list of strings.
    # Skip the CSV header line.  Result is cached for the lifetime of the driver.
    def _get_profile_names()
        if self._profile_names_cache != nil
            return self._profile_names_cache
        end
        import string
        var names = []
        var content = self._read_profile_file()
        if content != nil
            var lines = string.split(content, "\n")
            var first = true
            for line : lines
                if first
                    first = false
                    continue
                end
                var comma = string.find(line, ",")
                if comma > 0
                    names.push(line[0..comma-1])
                end
            end
        end
        self._profile_names_cache = names
        return names
    end

    # Parse a CSV profile line into [name, ph_min, ph_max, ec_min, ec_max, t_min,
    # t_max, cl_max, orp_min, orp_max, salt_max].
    def _parse_profile_line(line)
        import string
        var parts = string.split(line, ",")
        return [parts[0],
                real(parts[1]), real(parts[2]),
                int(parts[3]), int(parts[4]),
                int(parts[5]), int(parts[6]),
                real(parts[7]),
                int(parts[8]), int(parts[9]),
                int(parts[10])]
    end

    # Build flat arrays for fast range-color lookups.  Index order:
    # 0=pH 1=EC 2=TDS 3=ORP 4=Cl 5=Temp 6=SALT 7=Batt
    # Eliminates nested map creation and string-key hashing in _range_color().
    def _build_flat_ranges(ph_min, ph_max, ec_min, ec_max, temp_min, temp_max,
                           cl_max, orp_min, orp_max, salt_max)
        var ec_margin = int((ec_max - ec_min) * 0.15 + 0.5)
        if ec_margin < 50   ec_margin = 50   end
        var tds_margin = int(ec_margin * 0.64)
        if tds_margin < 32   tds_margin = 32   end
        var cl_margin = cl_max * 0.5
        var salt_margin = salt_max * 0.15
        if salt_margin < 50   salt_margin = 50   end
        self._flat_mins    = [ph_min, ec_min, int(ec_min * 0.64), orp_min, 0,         temp_min, 0,        60]
        self._flat_maxs    = [ph_max, ec_max, int(ec_max * 0.64), orp_max, cl_max,    temp_max, salt_max, 100]
        self._flat_margins = [0.1,    ec_margin, tds_margin,       50,      cl_margin, 2,       int(salt_margin + 0.5), 30]
    end

    def _build_flat_generic()
        self._flat_mins    = [5.8, 1000, 640,  300, 0,    20,  0,   60]
        self._flat_maxs    = [6.2, 2500, 1600, 450, 0.2, 25,  750, 100]
        self._flat_margins = [0.1, 200,  128,  50,  0.1, 2,   125, 30]
    end

    # Expand a compact profile line into the full range map and flat arrays.
    def _profile_ranges_from_line(line)
        var p = self._parse_profile_line(line)
        self._build_flat_ranges(p[1], p[2], p[3], p[4], p[5], p[6],
                                p[7], p[8], p[9], p[10])
        # Build the ranges map for backward compatibility
        var ph_min = p[1], ph_max = p[2], ec_min = p[3], ec_max = p[4]
        var temp_min = p[5], temp_max = p[6]
        var cl_max = p[7]
        var orp_min = p[8], orp_max = p[9]
        var salt_max = p[10]
        var ec_margin = int((ec_max - ec_min) * 0.15 + 0.5)
        if ec_margin < 50   ec_margin = 50   end
        var tds_margin = int(ec_margin * 0.64)
        if tds_margin < 32   tds_margin = 32   end
        var cl_margin = cl_max * 0.5
        var salt_margin = salt_max * 0.15
        if salt_margin < 50   salt_margin = 50   end
        return {
            "pH":   {"min": ph_min, "max": ph_max, "margin": 0.1},
            "EC":   {"min": ec_min, "max": ec_max, "margin": ec_margin},
            "TDS":  {"min": int(ec_min * 0.64), "max": int(ec_max * 0.64), "margin": tds_margin},
            "Temp": {"min": temp_min, "max": temp_max, "margin": 2},
            "SALT": {"min": 0, "max": salt_max, "margin": int(salt_margin + 0.5)},
            "ORP":  {"min": orp_min, "max": orp_max, "margin": 50},
            "Cl":   {"min": 0, "max": cl_max, "margin": cl_margin},
            "Batt": self._generic_ranges()["Batt"]
        }
    end

    def _load_profile(name)
        if name == nil   name = "Generic"   end
        if name == "Generic"
            self.profile_name = "Generic"
            self._build_flat_generic()
            self.ranges = self._generic_ranges()
            log("YC01: active profile: Generic")
            return
        end
        var line = self._find_profile_line(name)
        if line == nil
            self.profile_name = "Generic"
            self._build_flat_generic()
            self.ranges = self._generic_ranges()
            log("YC01: profile '" + name + "' not found, using Generic")
            return
        end
        self.profile_name = name
        self.ranges = self._profile_ranges_from_line(line)
        log("YC01: active profile: " + name)
    end

    # Flat-array range color lookup.  No map creation, no string hashing — just
    # array indexing and arithmetic.  Called 8× per web_sensor() render.
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
        if i < 0   return "green"   end
        var mn = self._flat_mins[i]
        var mx = self._flat_maxs[i]
        var mg = self._flat_margins[i]
        if v < mn - mg || v > mx + mg
            return "red"
        elif v < mn || v > mx
            return "orange"
        end
        return "green"
    end

    # NOTE: _colored_val and _colored_val_unit are intentionally removed.
    # Their logic is inlined in web_sensor() for maximum performance.

    # Returns "MAC" or "MAC/addr_type".  Cached — MAC rarely changes.
    def _mac_arg()
        if self._mac_arg_cache == nil
            var m = self.mac
            if self.addr_type != nil && self.addr_type != 0
                m = m + "/" + str(self.addr_type)
            end
            self._mac_arg_cache = m
        end
        return self._mac_arg_cache
    end

    def _set_mac(mac, addr_type)
        import string
        if mac == nil   mac = ""   end
        mac = string.toupper(string.replace(str(mac), " ", ""))
        mac = string.replace(mac, ":", "")
        mac = string.replace(mac, "-", "")
        if size(mac) != 12
            log("YC01: invalid MAC, must be 12 hex digits", 2)
            return false
        end
        # simple hex validation
        for i : 0..size(mac)-1
            var c = mac[i]
            if (c < '0' || c > '9') && (c < 'A' || c > 'F')
                log("YC01: invalid MAC, must be 12 hex digits", 2)
                return false
            end
        end
        self.mac = mac
        self.mac_hex = mac
        self.addr_type = (addr_type == nil) ? 0 : int(addr_type)
        self._mac_arg_cache = nil
        persist.yc01_mac = self.mac
        persist.yc01_addr_type = self.addr_type
        self._persist_mark_dirty()
        log(string.format("YC01: MAC set to %s type %i", self.mac, self.addr_type))
        return true
    end

    # Mark persist state as dirty; actual flash write is debounced to at most
    # one per 5 s in every_second().  Avoids multiple flash writes when several
    # settings change in quick succession.
    def _persist_mark_dirty()
        self._persist_dirty = true
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
        if self.last_seen < 0 && (self.last_ok < 0 || now - self.last_ok > 120)
            log("YC01: WARNING - meter never seen in BLE adverts; wake it and verify the MAC (try BLEScan0 1)", 2)
        end
        # Simple read+notify, matching the original working script.  The
        # start-measurement command is available manually via yc01start.
        self.phase = 3
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
            # The meter exposes the sensor payload on FF02.  The original
            # working script for this meter used read+notify on the same
            # characteristic, so we do the same and accept data from either
            # the read response or a notification.  Keep-alive (k:60000)
            # reuses the GATT connection between 50-second polls.
            log("YC01: BLEOp read+notify")
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 n:FF02 k:60000 r go")
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
            tasmota.cmd("BLEOp1 M:" + m + " s:FF01 c:FF02 k:60000 w:" + f + " go")
        end
    end

    def _finish(ok, reason)
        self.awaiting = false
        self.phase = 0
        if ok
            self.fails = 0
            self.last_ok = tasmota.millis() / 1000
            self.last_forced_reconnect = 0
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
                elif self.last_ok >= self.last_try
                    # we already got sensor data during this operation; a late
                    # notify timeout/read failure is not a real failure.
                    log("YC01: BLE op reported " + v + " but data was already received")
                    self.queue = []
                    self.q_read = false
                    self._finish(true, "")
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

    # The BLE topic message contains BLEDevices and BLE stats.  Depending on
    # the exact Tasmota rule trigger, the devices map may arrive as the value
    # (BLE#BLEDevices) or inside the full message (BLE).  It may also arrive
    # as a raw JSON string, so try to parse that as well.
    def _on_ble_msg(v, msg)
        var devices = self._extract_devices(v)
        if devices == nil   devices = self._extract_devices(msg)   end
        if devices != nil
            self.seen_adverts(devices)
        end
    end

    def _extract_devices(x)
        if x == nil   return nil   end
        import json
        var m = x
        if type(m) == 'string'
            try
                m = json.load(m)
            except .. as e
                return nil
            end
        end
        if type(m) != 'instance' && type(m) != 'map'   return nil   end
        if m.contains("BLEDevices")
            return m["BLEDevices"]
        end
        return nil
    end

    # Look for our MAC in the devices map.  Keys can be upper/lower case,
    # with or without colons, and sometimes carry a type suffix like /0 or (0).
    def _find_mac_key(v)
        if v == nil   return nil   end
        # mac_hex is already upper-cased and de-colonized in init(); only
        # suffix variants are needed.
        var candidates = [self.mac_hex,
                          self.mac_hex + "/0",
                          self.mac_hex + "/1",
                          self.mac_hex + "(0)",
                          self.mac_hex + "(1)"]
        for k : candidates
            if v.contains(k)   return k   end
        end
        return nil
    end

    def seen_adverts(v)
        if v == nil || self.mac_hex == ""   return   end
        var key = self._find_mac_key(v)
        if key != nil
            self.last_seen = tasmota.millis() / 1000
            var d = v[key]
            if d != nil && d.contains('r')
                self.rssi = d['r']
                log("YC01: beacon seen, RSSI=" + str(self.rssi) + " dBm")
            else
                log("YC01: beacon seen (no RSSI in advert)")
            end
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
        if self._parse_map == nil
            self._parse_map = {"pH":0,"EC":0,"TDS":0,"ORP":0,"Cl":0,"Temp":0,"Batt":0,"ECu":"","SALT":0,"RSSI":0}
        end
        var m = self._parse_map
        m["pH"]   = self.s16(d, 3) / 100.0
        m["EC"]   = self.s16(d, 5)
        m["TDS"]  = self.s16(d, 7)
        self.orp_raw = self.s16(d, 9)
        m["ORP"]  = self.orp_raw - self.orp_offset
        var cl = self.s16(d, 11)
        m["Cl"]   = (cl < 0 ? 0 : cl) / 10.0
        m["Temp"] = self.s16(d, 13) / 10.0
        var b = 100 * (self.s16(d, 15) - 1950) / 1240
        if b < 0     b = 0   end
        if b > 100   b = 100 end
        m["Batt"] = int(b + 0.5)
        # Status byte at offset 17: high nibble is the hold flag.
        self.hold = (d.size() > 17) && ((d[17] >> 4) != 0)
        # SALT is not sent by the meter; derive it from EC using the same
        # 0.55 factor the YINMIK app / Home Assistant integration uses.
        m["ECu"]  = "uS/cm"
        m["SALT"] = self.s16(d, 5) * 0.55
        m["RSSI"] = self.rssi
        self.last = m
        import string
        var orp_msg = string.format("ORP=%imV", m["ORP"])
        if self.orp_offset != 0
            orp_msg = orp_msg + string.format(" (raw %imV, offset %imV)", self.orp_raw, self.orp_offset)
        end
        var extra = ""
        if self.hold   extra = extra + " HOLD"   end
        if m["Batt"] < 60   extra = extra + " LOWBATT"   end
        log(string.format("YC01: pH=%.2f EC=%i TDS=%i %s Cl=%.1f T=%.1fC Batt=%i%% RSSI=%i%s",
                          m["pH"], m["EC"], m["TDS"], orp_msg, m["Cl"], m["Temp"], m["Batt"], self.rssi, extra))
        if m["Batt"] < 60
            log("YC01: WARNING - battery below 60%; meter may drop connections or give erratic readings", 2)
        end
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

    # Set ORP calibration offset.  expected_mV is the value you see on the
    # meter's own display / app.  The driver subtracts (raw - expected) from
    # future raw ORP readings so the displayed value matches the meter.
    # Run without arguments to clear the offset.
    def cmd_orpcal(expected_mV)
        import string
        if expected_mV == nil
            self.orp_offset = 0
            log("YC01: ORP calibration offset cleared")
            return
        end
        if self.orp_raw == 0 && self.last.size() == 0
            log("YC01: no ORP reading yet; read the meter first (yc01read)", 2)
            return
        end
        var raw = (self.last.size() > 0) ? self.last["ORP"] + self.orp_offset : self.orp_raw
        self.orp_offset = raw - expected_mV
        log(string.format("YC01: ORP calibrated to %i mV (raw %i mV, offset %i mV)",
                          expected_mV, raw, self.orp_offset))
    end

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

    # Change the polling interval.  Shorter intervals keep sleepy meters awake.
    def cmd_poll(s)
        if s < 10   s = 10   end
        if s > 3600   s = 3600   end
        self.poll_s = s
        self.tick = 0
        persist.yc01_poll = s
        self._persist_mark_dirty()
        log("YC01: poll interval set to " + str(s) + " s")
    end

    # Change the maximum allowed sync delay.  If the last successful read is
    # older than this, the driver forces an immediate reconnect attempt.
    def cmd_sync_max(s)
        if s < 30   s = 30   end
        if s > 3600   s = 3600   end
        self.sync_max = s
        persist.yc01_sync_max = s
        self._persist_mark_dirty()
        log("YC01: max sync delay set to " + str(s) + " s")
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
        var now = tasmota.millis() / 1000    # single call per tick
        # Flush dirty persist state (debounced flash writes)
        if self._persist_dirty && (now - self._persist_last_save >= 5)
            persist.save()
            self._persist_dirty = false
            self._persist_last_save = now
        end
        # Enable active scan once BLE is up (driver init runs before BLE starts).
        # WaterQualityApp uses SCAN_MODE_LOW_LATENCY (active); this matches it.
        if !self.scan_init && now > 10
            self.cmd_active_scan(true)
            self.scan_init = true
        end
        if self.awaiting
            self.watchdog += 1
            if self.watchdog > 45
                self.queue = []
                self.q_read = false
                self._finish(false, "WATCHDOG")
            end
            return
        end
        # If the last successful read is older than the configured maximum sync
        # delay, try to recover instead of waiting for the next scheduled poll.
        # Rate-limit these recovery attempts to once every 120 s.  If the meter
        # has not been seen advertising recently, run an active scan rather than
        # hammering a connect() against a sleeping/offline device.
        if self.last_ok >= 0
            var age = now - self.last_ok
            if age > self.sync_max
                if now - self.last_forced_reconnect >= 120
                    self.last_forced_reconnect = now
                    import string
                    var beacon_age = (self.last_seen < 0) ? 9999 : now - self.last_seen
                    if beacon_age > 120
                        log(string.format("YC01: sync delay %is exceeds max %is and beacon not seen for %is, running active scan", age, self.sync_max, beacon_age), 2)
                        self.cmd_find()
                    else
                        log(string.format("YC01: sync delay %is exceeds max %is, forcing reconnect", age, self.sync_max), 2)
                        self._start(true)
                    end
                    return
                end
            end
        end
        self.tick += 1
        if self.tick >= self.poll_s
            self.tick = 0
            self._start(false)
        end
    end

    # ---- presentation ----------------------------------------------------

    def json_append()
        if self.last.size() == 0   return end
        import json
        tasmota.response_append(',"YC01":')
        tasmota.response_append(json.dump(self.last))
    end

    # Web configuration page: register route and Configuration-menu button.
    def web_add_handler()
        webserver.on("/yc01", / -> self.page_yc01())
    end

    def web_add_config_button()
        webserver.content_send("<p></p><form id=ac action='yc01' style='display: block;' method='get'><button>Hydroponics</button></form>")
    end

    def page_yc01()
        if !webserver.check_privileged_access()   return nil   end
        if webserver.has_arg("save")
            var newpoll = int(webserver.arg("poll"))
            if newpoll < 10   newpoll = 10   end
            if newpoll > 3600   newpoll = 3600   end
            self.poll_s = newpoll
            self.tick = 0
            persist.yc01_poll = newpoll
            var newsync = int(webserver.arg("syncmax"))
            if newsync < 30   newsync = 30   end
            if newsync > 3600   newsync = 3600   end
            self.sync_max = newsync
            persist.yc01_sync_max = newsync
            self._set_mac(webserver.arg("mac"), webserver.arg("atype"))
            var newprofile = webserver.arg("profile")
            self._load_profile(newprofile)
            persist.yc01_profile = self.profile_name
            self._persist_mark_dirty()
            self._invalidate_profile_cache()
            webserver.redirect("/cn?")
            return nil
        end
        var cur = self.poll_s
        var cur_sync = self.sync_max
        var cur_mac = self.mac
        var sel_pub = (self.addr_type == 0) ? " selected" : ""
        var sel_rnd = (self.addr_type == 1) ? " selected" : ""
        import string
        webserver.content_start("Hydroponics")
        webserver.content_send_style()
        webserver.content_send(
            "<fieldset><legend><b>&nbsp;Hydroponics&nbsp;</b></legend>" +
            "<form method='get' action='yc01'>" +
            "<p></p><table>" +
            "<tr><td style='width:200px'><b>Poll interval (seconds)</b></td>" +
            "<td style='width:220px'><input id='poll' name='poll' type='number' min='10' max='3600' value='" + str(cur) + "'></td></tr>" +
            "<tr><td style='width:200px'><b>Max sync delay (seconds)</b></td>" +
            "<td style='width:220px'><input id='syncmax' name='syncmax' type='number' min='30' max='3600' value='" + str(cur_sync) + "'></td></tr>" +
            "<tr><td><b>Beacon MAC</b></td>" +
            "<td><input id='mac' name='mac' maxlength='17' value='" + cur_mac + "'></td></tr>" +
            "<tr><td><b>Address type</b></td>" +
            "<td><select id='atype' name='atype'>" +
            "<option value='0'" + sel_pub + ">Public (0)</option>" +
            "<option value='1'" + sel_rnd + ">Random (1)</option>" +
            "</select></td></tr>" +
            "<tr><td><b>Profile</b></td>" +
            "<td><select id='profile' name='profile'>"
        )
        # Use cached profile names (file read once, cached in RAM).  Falls back
        # to Generic if the file is missing or empty.
        var names = self._get_profile_names()
        if names.size() == 0
            webserver.content_send("<option value='Generic' selected>Generic</option>")
        else
            for n : names
                var sel = (n == self.profile_name) ? " selected" : ""
                webserver.content_send("<option value='" + n + "'" + sel + ">" + n + "</option>")
            end
        end
        webserver.content_send(
            "</select></td></tr>" +
            "</table><p></p>" +
            "<button name='save' type='submit' class='button bgrn'>Save</button>" +
            "</form></fieldset>"
        )
        webserver.content_button(webserver.BUTTON_CONFIGURATION)
        webserver.content_stop()
    end

    def web_sensor()
        import string
        if self.last.size() == 0
            tasmota.web_send_decimal(_S + "Status" + _M + "Not connected" + _E)
            return
        end
        var now = tasmota.millis() / 1000
        # State
        var state
        if !self.awaiting
            state = "Waiting"
        elif self.phase == 1
            state = "Time sync"
        elif self.phase == 2
            state = "Starting"
        elif self.phase == 3
            var age = (self.last_ok >= 0) ? (now - self.last_ok) : -1
            state = (age > self.sync_max) ? "Reconnecting" : "Reading"
        elif self.phase == 4
            state = "Executing"
        else
            state = "busy (phase " + str(self.phase) + ")"
        end
        if self.hold   state += " (HOLD)"   end
        # Delay
        var age = (self.last_ok >= 0) ? (now - self.last_ok) : -1
        var delay = age - self.sync_max
        if delay < 0   delay = 0   end
        var delay_txt = string.format("%i s", delay)
        if delay > 0   delay_txt = "<span style='color:red'>" + delay_txt + "</span>"   end
        # Build output — inlined color + value formatting, zero function calls
        var out = _S + "Status" + _M + state + _E +
                  _S + "Sync delay" + _M + delay_txt + _E
        # Batt (idx 7)
        var v = self.last["Batt"]
        var mn = self._flat_mins[7]
        var mx = self._flat_maxs[7]
        var mg = self._flat_margins[7]
        var sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "Battery" + _M + sp + string.format("%.0f", v) + " ％" + _SPAN_X + _E
        # pH (idx 0)
        v = self.last["pH"]
        mn = self._flat_mins[0]
        mx = self._flat_maxs[0]
        mg = self._flat_margins[0]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "pH" + _M + sp + string.format("%.2f", v) + _SPAN_X + _E
        # EC (idx 1)
        v = self.last["EC"]
        mn = self._flat_mins[1]
        mx = self._flat_maxs[1]
        mg = self._flat_margins[1]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "EC" + _M + sp + string.format("%.1f", v) + " uS/cm" + _SPAN_X + _E
        # TDS (idx 2)
        v = self.last["TDS"]
        mn = self._flat_mins[2]
        mx = self._flat_maxs[2]
        mg = self._flat_margins[2]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "TDS" + _M + sp + string.format("%.1f", v) + " ppm" + _SPAN_X + _E
        # SALT (idx 6)
        v = self.last["SALT"]
        mn = self._flat_mins[6]
        mx = self._flat_maxs[6]
        mg = self._flat_margins[6]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "SALT" + _M + sp + string.format("%.1f", v) + " ppm" + _SPAN_X + _E
        # ORP (idx 3)
        v = self.last["ORP"]
        mn = self._flat_mins[3]
        mx = self._flat_maxs[3]
        mg = self._flat_margins[3]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "ORP" + _M + sp + string.format("%.2f", v) + " mV" + _SPAN_X + _E
        if self.orp_offset != 0
            out += _S + "ORP raw" + _M + str(self.orp_raw) + _E +
                   _S + "ORP offset" + _M + str(self.orp_offset) + _E
        end
        # Cl (idx 4)
        v = self.last["Cl"]
        mn = self._flat_mins[4]
        mx = self._flat_maxs[4]
        mg = self._flat_margins[4]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "Chlorine" + _M + sp + string.format("%.2f", v) + " mg/L" + _SPAN_X + _E
        # Temp (idx 5)
        v = self.last["Temp"]
        mn = self._flat_mins[5]
        mx = self._flat_maxs[5]
        mg = self._flat_margins[5]
        sp = (v < mn - mg || v > mx + mg) ? _SPAN_R : (v < mn || v > mx) ? _SPAN_O : _SPAN_G
        out += _S + "Temperature" + _M + sp + string.format("%.1f", v) + " &deg;C" + _SPAN_X + _E
        if self.cal.size() > 0
            for k : self.cal.keys()
                out += _S + "Cal " + k + _M + self.cal[k] + _E
            end
        end
        tasmota.web_send_decimal(out)
    end
end

# Load MAC / poll interval from persist, or use defaults on first boot.
yc01 = YC01(nil, nil)
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

def yc01orpcal_cmd(cmd, idx, payload, payload_json)
    var p = str(payload)
    if p == ""
        yc01.cmd_orpcal(nil)
    else
        yc01.cmd_orpcal(int(p))
    end
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01orpcal", yc01orpcal_cmd)

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
             "fails": yc01.fails, "rssi": yc01.rssi,
             "orp_raw": yc01.orp_raw, "orp_offset": yc01.orp_offset,
             "hold": yc01.hold}
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

def yc01poll_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_poll(int(payload))
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01poll", yc01poll_cmd)

def yc01syncmax_cmd(cmd, idx, payload, payload_json)
    yc01.cmd_sync_max(int(payload))
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01syncmax", yc01syncmax_cmd)

def yc01profile_cmd(cmd, idx, payload, payload_json)
    var p = str(payload)
    if p == ""
        log("YC01: active profile: " + yc01.profile_name)
    else
        yc01._load_profile(p)
        persist.yc01_profile = yc01.profile_name
        yc01._persist_mark_dirty()
    end
    tasmota.resp_cmnd_done()
    return true
end
tasmota.add_cmd("yc01profile", yc01profile_cmd)
