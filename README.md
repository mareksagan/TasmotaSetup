# TasmotaSetup
An example Tasmota device setup to replace Tuya devices in terms of look &amp; feel

# Energy saving
```
Rule1
  ON system#boot DO var1 0 ENDON
  ON Power1#State=0 DO Backlog RuleTimer1 0; var1 0 ENDON
  ON Power1#State=1 DO Backlog RuleTimer1 0; var1 0 ENDON
  ON Energy#Power>10 DO if (var1!=0) RuleTimer1 0; var1 0 endif ENDON
  ON Energy#Power<10 DO if (var1!=1) RuleTimer1 60; var1 1 endif ENDON
  ON Rules#Timer=1 DO Backlog var1 0; Power1 0 ENDON

Rule1 4
Rule1 1

TelePeriod 10
PowerDelta 0
Restart 1
```

# Light theme for Tasmota

```
WebColor {"WebColor":["#000000","#ffffff","#f2f2f2","#000000","#ffffff","#000000","#ffffff","#ff0000","#008000","#ffffff","#1fa3ec","#0e70a4","#d43535","#931f1f","#47c266","#5aaf6f","#ffffff","#999999","#000000","#a1d9f7"]}
```
