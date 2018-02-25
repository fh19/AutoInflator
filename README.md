# AutoInflator
Author: Falk Hecker
eMail: falk.hecker@netic.de

Automatic tire inflation device w/ arduino

  Functions:
  -----------
  - switch on w/ pressing on-button, system will switch on on-relais
  - switch off by pushing on-button 2 times within 1 second
  - Automatic inflation of tires
  - read pressure in filling pipe
  - if pressure is higher than threshold
   -> inflate/deflate until desired pressure is achieved
  - manual inflation w/ plus/minus buttons after switching mode to manual w/ on-button
  - last set pressure and some statistics are stored in EEPROM
   
  Hardware:
  ---------
  - Arduino Micro  
  - 5/3 Valve:
    - 1st coil for inflation
    - 2nd coil for deflation
  - LCD 5110
  - 2 pressure sensors
    - 0-5V, 0-10bar (or similar)
    - output pressure
    - supply pressure (could be skipped, if value is set constant) 
  - power supply (5V + 24V)
  - relay to switch on power supply
  - relay or power stage to switch on valves
  - 3 switches (push-buttons): on, plus and minus
    - on-switch is a double switch:
      - 1st to switch on power (parallel to ON_RELAY)
      - 2nd connected to sw_on (to switch modes between automatic and manual operation)
    - plus+minus switch to set pressure (auto mode) or inflate/deflate (manual operation)
    - all switches connect to ground (push buttons)
  - Port assignment: see source-file

  Parameters:
  -----------
  - behavior (timing) of valves
  - pressure accuracy
  - pressure sensor characteristics
  - details see source-file



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

