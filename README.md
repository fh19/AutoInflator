# AutoInflator
Author: Falk Hecker
eMail: falk.hecker@netic.de

Automatic tire inflation device w/ arduino

  Functions:
  -----------
  - Automatic inflation of tires
  - read pressure in filling pipe
  - if pressure is higher than threshold
   -> inflate/deflate until desired pressure is achieved
   
  Hardware:
  ---------
  - Arduino Micro  
  - 2 Valves:
    - #1 for inflation
    - #2 for deflation
  - LCD 5110
  - power supply
  - relay to switch on power supply
  - 3 switches (push-buttons): on, plus and minus
    - on-switch is a double switch:
      - 1st to switch on power (parallel to ON_RELAY)
      - 2nd connected to sw_on (to change mode from automatic to manual operation and back)
    - plus+minus switch to set pressure or for manual operation
    - all switches connect to ground (push button)
  - Port assignment: see source-file


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

