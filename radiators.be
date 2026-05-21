import string

# ==================== CONFIGURATION ====================
var th_configs = [
  { "name": "RadiatorGreen", "label": "Green" },
  { "name": "RadiatorRed",   "label": "Red" },
  { "name": "RadiatorBlue",  "label": "Blue" },
]
# =======================================================

var th_all = []       # list of ALL driver instances (allows duplicate names)
var th_by_idx = {}    # index -> driver (for commands)

def th_fmt(t)
  if t == nil
    return "--.-"
  elif t % 10 == 0
    return string.format("%d.0", t / 10)
  else
    return string.format("%d.%d", t / 10, t % 10)
  end
end

# Iterate the full list and update EVERY driver whose .name matches
def make_thermo_rule(dev_name)
  return def (v, t, p)
    if type(v) != 'instance'
      return
    end
    for drv : th_all
      if drv.name == dev_name
        if v.contains("EF00/0210")
          var n = v["EF00/0210"]
          if drv.target == nil || n != drv.target
            drv.target = n
            drv.web_sensor()
          end
        end
        if v.contains("EF00/0218")
          var n = v["EF00/0218"]
          if drv.current == nil || n != drv.current
            drv.current = n
            drv.web_sensor()
          end
        end
      end
    end
  end
end

class ThermostatDriver
  var target
  var current
  var name
  var label
  var idx
  
  def init(cfg, index)
    self.target = nil
    self.current = nil
    self.name = cfg["name"]
    self.label = cfg.contains("label") ? cfg["label"] : nil
    self.idx = index
    th_by_idx[string.format("%d", index)] = self
    th_all.push(self)
  end
  
  def web_sensor()
    var t = th_fmt(self.target)
    var cmd_base = string.format("Thermostat%d", self.idx)
    var display_label = self.label != nil ? self.label : "Thermostat"
    
    var left = string.format("<b>%s</b> <span style=\"color:var(--c_tab)\">%s</span>", display_label, self.name)
    
    var btn = "<span style=\"display:inline-flex;gap:6px;justify-content:flex-start;width:100%\">"
    btn = btn + string.format("<span onclick=\"fetch('/cm?cmnd=%s%%20off')\" style=\"display:inline-block;padding:3px 10px;border-radius:4px;background:var(--c_bkg);border:1px solid var(--c_brdr);color:inherit;cursor:pointer\">Off</span>", cmd_base)
    btn = btn + string.format("<span onclick=\"fetch('/cm?cmnd=%s%%20-')\" style=\"display:inline-block;padding:3px 12px;border-radius:4px;background:var(--c_bkg);border:1px solid var(--c_brdr);color:inherit;cursor:pointer;font-weight:bold\">−</span>", cmd_base)
    btn = btn + string.format("<span onclick=\"fetch('/cm?cmnd=%s%%20%%2B')\" style=\"display:inline-block;padding:3px 12px;border-radius:4px;background:var(--c_bkg);border:1px solid var(--c_brdr);color:inherit;cursor:pointer;font-weight:bold\">+</span>", cmd_base)
    btn = btn + string.format("<span onclick=\"fetch('/cm?cmnd=%s%%2021')\" style=\"display:inline-block;padding:3px 10px;border-radius:4px;background:var(--c_bkg);border:1px solid var(--c_brdr);color:inherit;cursor:pointer\">21°C</span>", cmd_base)
    btn = btn + "</span>"
    
    left = left + "<br>" + btn
    
    var h = string.format("{s}%s{m}<span style=\"color:var(--c_txtscc);font-weight:bold\">%s°C</span>{e}",
      left, t)
    
    tasmota.web_send_decimal(h)
  end
  
  def send_cmd(pl)
    print(string.format("[Thermostat%d] send_cmd payload='%s' name=%s current_target=%s", self.idx, pl, self.name, th_fmt(self.target)))
    
    if pl == "+"
      if self.target == nil
        print(string.format("[Thermostat%d] target unknown, reading first", self.idx))
        tasmota.cmd(string.format("ZbSend {\"Device\":\"%s\",\"Read\":{\"EF00/0210\":true}}", self.name))
        tasmota.resp_cmnd_done()
        return true
      end
      var nt = self.target + 5
      if nt > 350 nt = 350 end
      self.target = nt
      self.web_sensor()
      tasmota.cmd(string.format("ZbSend {\"Device\":\"%s\",\"Write\":{\"EF00/0210\":%d}}", self.name, nt))
      print(string.format("[Thermostat%d] sent Write EF00/0210=%d", self.idx, nt))
      tasmota.resp_cmnd_done()
    elif pl == "-"
      if self.target == nil
        print(string.format("[Thermostat%d] target unknown, reading first", self.idx))
        tasmota.cmd(string.format("ZbSend {\"Device\":\"%s\",\"Read\":{\"EF00/0210\":true}}", self.name))
        tasmota.resp_cmnd_done()
        return true
      end
      var nt = self.target - 5
      if nt < 50 nt = 50 end
      self.target = nt
      self.web_sensor()
      tasmota.cmd(string.format("ZbSend {\"Device\":\"%s\",\"Write\":{\"EF00/0210\":%d}}", self.name, nt))
      print(string.format("[Thermostat%d] sent Write EF00/0210=%d", self.idx, nt))
      tasmota.resp_cmnd_done()
    elif pl == "off"
      self.target = 50
      self.web_sensor()
      tasmota.cmd(string.format("ZbSend {\"Device\":\"%s\",\"Write\":{\"EF00/0210\":50}}", self.name))
      print(string.format("[Thermostat%d] sent Write EF00/0210=50 (off)", self.idx))
      tasmota.resp_cmnd_done()
    else
      if pl != ""
        var val = real(pl)
        if val == nil
          print(string.format("[Thermostat%d] invalid numeric payload: %s", self.idx, pl))
          tasmota.resp_cmnd_error()
          return true
        end
        var t = int(val * 10)
        if t >= 50 && t <= 350
          self.target = t
          self.web_sensor()
          tasmota.cmd(string.format("ZbSend {\"Device\":\"%s\",\"Write\":{\"EF00/0210\":%d}}", self.name, t))
          print(string.format("[Thermostat%d] sent Write EF00/0210=%d", self.idx, t))
          tasmota.resp_cmnd_done()
        else
          print(string.format("[Thermostat%d] value out of range: %d", self.idx, t))
          tasmota.resp_cmnd_error()
        end
      else
        tasmota.resp_cmnd_error()
      end
    end
    return true
  end
end

# Create drivers and register per-device ZbReceived rules
for cfg : th_configs
  var drv = ThermostatDriver(cfg, size(th_by_idx) + 1)
  tasmota.add_driver(drv)
  tasmota.add_rule("ZbReceived#" + cfg["name"], make_thermo_rule(cfg["name"]))
  print(string.format("[init] Registered Thermostat%d -> %s (label=%s)", drv.idx, drv.name, drv.label != nil ? drv.label : "Thermostat"))
end

def cmd_th(cmd, idx, pl, raw)
  var key = string.format("%d", idx)
  print(string.format("[cmd_th] cmd=%s idx=%d key=%s payload='%s'", cmd, idx, key, pl))
  if th_by_idx.contains(key)
    th_by_idx[key].send_cmd(pl)
  else
    print(string.format("[cmd_th] ERROR: no thermostat at index %d", idx))
    tasmota.resp_cmnd(string.format("Error: no thermostat at index %d", idx))
  end
  return true
end

tasmota.add_cmd("Thermostat", cmd_th)
