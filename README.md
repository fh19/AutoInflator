# AutoInflator
automatic tire inflation device w/ arduino

  Functions:
  -----------
  - Automatic inflation of tires
  - read pressure in filling pipe
  - if pressure is higher than threshold
   -> inflate/deflate until desired pressure is achieved
  - 2 Valves:
    o #1 for inflation
    o #2 for deflation
  - LCD 5110
  - on-switch is a double switch:
    o 1st to switch on power (parallel to ON_RELAY)
    o 2nd connected to sw_on (to change mode from automatic to manual operation and back)
  - plus+minus switch to set pressure or for manual operation
  - all switches connect to ground (push button)

Port assignment: see source-file



  Changelog:
  ----------

  V0.1 (2018-01-28):
  - Initialversion

  V0.2. 0.3 (2018-02-11):
  - switches per interrupt

  V0.4 (2018-02-18):
  - display optimized
  - control loop changed

  V0.5 (2018-02-18):
  - first working sample
  - manual control via interrupt
  - contropl loop improved (ratio correction)
  - manual switch off

  V0.6 (2018-02-25):
  - contropl loop improved (ratio optimization)
  - open/close-time of valve changed from 10->20ms

