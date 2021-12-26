# mavlink-antena-tracker
Antena tracker with pymavlink client and simple arduino stepper motor driver

Run mavlink_comm.py, it expects only one serial port with arduino present. Device name or anything else is not validated. This connects to mavlink "server" over TCP by default. I have it working with dragonlink this way. You may want to configure connection in different way.

Arduino board is simple motor driver, you can configure basic pulses per rotation and reverse rotation if needed. I have this running on arduino leonardo.

Stepper sample PCB layout is included either make it double sided or connect gates to header with wires. Use whatever transistors and diodes you have by hand, if you made it this far, you probably know what you are doing...

Tracker itself is very simplistic - it reads home position from mavlink and then position of vehicle. It calculates azimuth and elevation angle, sends that to arduino which drives motor. This assumes that tracker is positioned directly at home point, and motor tracker is pointed to north. It is fine if you adjust tracking by hand too. It relies on motor being accurate, there is no feedback loop to ensure correct azimuth.

I did this over weekend evenings, so there may or may not be improvements in future. Since I haven't found similar work, I will share mine.
