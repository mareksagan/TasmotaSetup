# Multi Fire Alarm Zigbee Sensor - Tasmota Berry Script
# Save as firealarm.be and load with: load("firealarm.be")
#
# CONFIGURATION: Add one entry per sensor. Index 1 = top row, index 2 = next, etc.
# The "name" must match the "Name" field in ZbReceived JSON.
var FIRE_ALARMS = [
  { "name": "SmokeDetector", "label": "Corridor" },
  # { "name": "SmokeSensor2", "label": "Hallway" },
]

# Prevent double registration if reloaded in same session
if global.fa_loaded == true
  print("fa.be already loaded, skipping")
  return
end
global.fa_loaded = true

class FireAlarmUnit
  var name
  var label
  var smoke
  var tamper
  var silenced_until
  var last_smoke_time
  
  def init(name, label)
    self.name = name
    self.label = label
    self.smoke = 0
    self.tamper = 0
    self.silenced_until = 0
    self.last_smoke_time = 0
  end
  
  def update_from_zb(d)
    var new_smoke = self.smoke
    var new_tamper = self.tamper
    var got_data = false
    
    if d.contains("ZoneStatusChange")
      var zsc = d["ZoneStatusChange"]
      new_smoke = (zsc & 0x01) ? 1 : 0
      new_tamper = (zsc & 0x04) ? 1 : 0
      got_data = true
    end
    
    if !got_data && d.contains("ZoneStatus")
      var zs = d["ZoneStatus"]
      new_smoke = (zs & 0x01) ? 1 : 0
      new_tamper = (zs & 0x04) ? 1 : 0
      got_data = true
    end
    
    if d.contains("Occupancy")
      new_smoke = d["Occupancy"] ? 1 : 0
    end
    if d.contains("Tamper")
      new_tamper = d["Tamper"] ? 1 : 0
    end
    
    if new_smoke != self.smoke || new_tamper != self.tamper
      if new_smoke == 1 && self.smoke == 0
        self.last_smoke_time = tasmota.rtc()['local']
      end
      self.smoke = new_smoke
      self.tamper = new_tamper
      return true
    end
    return false
  end
  
  def render()
    var status = "Clear"
    var sc = "var(--c_txtscc)"
    
    if self.smoke
      status = "Smoke"
      sc = "var(--c_txtwrn)"
    elif self.tamper
      status = "Tamper"
      sc = "#f57c00"
    end
    
    var si = ""
    if self.silenced_until > tasmota.rtc()['local']
      si = format(" <span style=\"color:var(--c_tab)\">· Muted %im</span>", (self.silenced_until - tasmota.rtc()['local']) / 60)
    end
    
    var left = format("<b>%s</b> <span style=\"color:var(--c_tab)\">%s</span>", self.label, self.name)
    
    if self.last_smoke_time > 0
      var ts = tasmota.time_str(self.last_smoke_time)
      var ts2 = ts[0..9] + " " + ts[11..18]
      left = left + format("<br><span style=\"color:#ff4444\">%s</span>", ts2)
    end
    
    return format("{s}%s{m}<span style=\"color:%s;font-weight:bold\">%s</span>%s{e}",
      left, sc, status, si)
  end
end

class FireAlarmManager
  var units
  var units_map
  
  def init(config)
    self.units = []
    self.units_map = {}
    for entry : config
      var name = entry["name"]
      var label = entry["label"]
      var unit = FireAlarmUnit(name, label)
      self.units.push(unit)
      self.units_map[name] = unit
    end
  end
  
  def handle_zb(value)
    var t = type(value)
    if t != 'instance' && t != 'map' return end
    
    var zb = value
    if value.contains("ZbReceived")
      zb = value["ZbReceived"]
      if type(zb) != 'instance' && type(zb) != 'map' return end
    end
    
    for unit : self.units
      if zb.contains(unit.name)
        var d = zb[unit.name]
        if type(d) == 'instance' || type(d) == 'map'
          unit.update_from_zb(d)
        end
      end
    end
  end
  
  def web_sensor()
    for unit : self.units
      tasmota.web_send_decimal(unit.render())
    end
  end
  
  def cmd(cmd_name, idx, payload, raw)
    var n = size(payload)
    var sp = -1
    var i = 0
    while i < n
      if payload[i] == 32
        sp = i
        break
      end
      i = i + 1
    end
    
    if sp < 0
      tasmota.resp_cmnd("Usage: FireAlarm <device> silence|clear")
      return true
    end
    
    var dev = payload[0..sp-1]
    
    i = sp + 1
    while i < n && payload[i] == 32
      i = i + 1
    end
    var act = payload[i..-1]
    
    if dev == "" || act == ""
      tasmota.resp_cmnd("Usage: FireAlarm <device> silence|clear")
      return true
    end
    
    if !self.units_map.contains(dev)
      tasmota.resp_cmnd(format("Unknown device: %s", dev))
      return true
    end
    
    var unit = self.units_map[dev]
    if act == "silence"
      unit.silenced_until = tasmota.rtc()['local'] + 300
      tasmota.resp_cmnd(format("%s silenced for 5 minutes", dev))
    elif act == "clear"
      unit.silenced_until = 0
      tasmota.resp_cmnd(format("%s silence cleared", dev))
    else
      tasmota.resp_cmnd("Usage: FireAlarm <device> silence|clear")
    end
    return true
  end
end

var fa_manager = FireAlarmManager(FIRE_ALARMS)

tasmota.add_rule("ZbReceived", def (value, trigger, msg)
  fa_manager.handle_zb(value)
end)

tasmota.add_cmd("FireAlarm", def (cmd, idx, payload, raw)
  return fa_manager.cmd(cmd, idx, payload, raw)
end)

class FireUIDriver
  def web_sensor()
    fa_manager.web_sensor()
  end
end

tasmota.add_driver(FireUIDriver())
