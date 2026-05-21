def netflip()
  if tasmota.wifi().find("up") != nil && tasmota.eth().find("up") != nil
    tasmota.cmd("Wifi 0")
  elif tasmota.wifi().find("up") == nil && tasmota.eth().find("up") == nil
    tasmota.cmd("Wifi 1")
  end
end

tasmota.add_rule("System#Boot", netflip)
tasmota.add_rule("Eth#Connected", def() tasmota.cmd("Wifi 0") end)
tasmota.add_rule("Eth#Disconnected", def() tasmota.cmd("Wifi 1") end)
