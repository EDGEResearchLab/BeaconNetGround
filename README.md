BeaconNetGround
===============

The system used to monitor and manage an entire network of balloons - designed specifically to work with the BEACONNet tracking messages

BEACONNet message is defined as an NMEA0183-compatible proprietary message, currently defined as:
$PEDGE,[BalloonID],[GPSDate],[GPSTime],[GPSLat],[GPSLon],[GPSAlt],[GPSSpd],[GPSCourse],[GPSNumSats],[GPSHDOP],[A1],[A2],[A3],[A4],[A5]*[CKSUM]
All GPS fields are taken from GPGGA and GPRMC sentences.  Onboard data (An) is defined as follows:
A1: Rail voltage (should be within a couple of bits of max)
A2: Temperature
A3: Pressure
A4: Humidity
A5: Ground (should be within a couple of bits of 0)

The goal of the BEACONNet message is to make it individually packetizable, so as to minimize network overhead and maximize the reliability of received sentences.  In order to achieve this, some padding and other magic may be necessary.  The sentence definition above will be updated with further info as it evolves.
