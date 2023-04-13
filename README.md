# Battery controller for On-grid microinverter 
36V li-ion battery controller cooperating with grid-tied microinverter
## 10A 35-42V battery; 450W grid-tie microinverter

2nd version of controller HW, connected to Microinverter and four e-scooter batteries
![schema](/photo/Prototype_testing_rev2.jpg)

###  About this project
Main idea is to connect 10S 36V/42V li-ion e-scooter/e-bike battery to grid-tied PV installation.
It can be done because microinverters use low voltage input (MPP is ~40V).
So e-scooter battery has voltage exactly in operational range of microinverter and PV panel.

###  What exactly does this device?
Monitors energy consumption by microinverter. In case of inverter fault this device stores PV energy in battery.
After dusk energy stored in battery might be tranfered via inverter to mains network.

###  Why that idea?
It's because photovoltaic is very popular in village area where i've built my house. The result is that mains voltage is to high to properly operate any inverter around noon at sunny, cloudless day.

.. figure:: (/photo/energy_meter_L3voltage.jpg)
    :align: center
    :width: 70.0%

During sunny day my inverters switch off several times. That causes loses in energy production, because at possible peak production my PV array does not generate energy at all. 

Solution is to detect inverter fault and switch PV to charge battery - at the same time waiting for inverter to recover. Usually it's less than 10 minutes. But it happens dozens times during sunny day. Energy from battery is transferred to inverter (and mains) after dusk.

###  Modes of operation

user can configure controller in 3 modes:
- 1st mode is to tranfer energy from PV to inverter, charge battery only when inverter cautches fault; finally after dusk controller connects battery to microinverter to transfer stored  energy from battery to mains
- 2nd mode is to store energy in battery and connect inverter when battery is full. Then it will discharge battery after dusk
- 3rd mode is to tranfer energy from PV to inverter, charge battery only when inverter cautches fault; no battery discharge

###  Delayed discharge
My 2kW PV installation based on microinverters, uses 5 PV panels, 11A each, and 5 microinverters.
Connecting 5 batteries with controllers, I can set discharge delay independently for each controller.
result is that i can discharge them one by one thru the night, making it possible to consume that energy by fridge, lighting and other appliances.

###  Operation statistics
Firmware has built-in statistics counters, to estimate amount of energy transfered, recovered, number of microinverter faults

It suprised me, but statistics shows that some inverters have more faults than others, despite that they are all connected in parallel to the same electrical network

Real statistics after 3 days of operation:
------------------------------------------
Battery size 20Ah 10S battery, summertime, single energy bank:
- Inverter not operational 14587 seconds
- energy stored in battery due to lack of inverter operation; Wh 1781
- energy transferred from battery after dusk: Wh 1003
- energy transferred from PV to microinverter: Wh 8000
- number of inverter reset procedure launches: 6 (inverter not working > 30minutes)

Real statistics, from dec 2022 to apr 2023:
-----------------------------------------
Battery size 20Ah 10S battery, wintertime, single energy bank:
- Inverter not operational 655876 seconds (182 hours)
- battery too cold to charge 336885 seconds (93 hours) - energy not transfered to mains and not stored in battery (too cold)
- energy transferred from battery after dusk: Wh 16250
- energy transferred from PV to microinverter: Wh 84890

###  What it really does
below you see energy consumption and production as it's seen by energy distribution company.
blue is consumption, pink is production. Marked in red is what my system generates after dusk.
![schema](/photo/Energy_statistics.jpg)

###  This version is in testing phase
A few pieces assembled, in-field testing in progress.

### Simplified operation algorithm
![schema](/photo/simplified_algorythm.png)

###  what is working
- monitors Inverter operation, connects battery when inverter not operational
- after sunset properly counts delay time and after timeout connects battery to microinverter
- controls current consumption by microinverter, during daytime to decide whether charge battery or not
- controls current consumption by microinverter, during nighttime, to prevent excessive current consumption from battery: sometimes it happens -> controller resets microinverter MPP by cycling power
- overtemperature protection
- recent version (nov 2022) added under temperature charging protection

###  what is not working or not implemented
- ESP32, not implemented, HW 3v3 voltage rail is too weak to supply ESP32
- no implementation of any kind of watchdog
- no implementation of any kind of power saving mode
- SD-Card logging not implemented (to create detailed statistics or charts later)
- no RTC, might be usefull to create detailed statistics

###  HW Testing: Battery discharge
below is Pic during battery discharge ~13A, HW rev 01
![schema](/photo/20220616-192417.jpg)

below is Pic during battery discharge ~12A, HW rev 02
![schema](/photo/20220710-213613.jpg)
