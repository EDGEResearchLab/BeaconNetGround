BeaconNetGround
===============

The system used to monitor and manage an entire network of balloons - designed specifically to work with the BEACONNet tracking messages

BEACONNet message is defined as an NMEA0183-compatible proprietary message, currently defined as:
$PEDGE,[BalloonID],[GPSDate],[GPSTime],[GPSLat],[GPSLon],[GPSAlt],[GPSSpd],[GPSCourse],[GPSNumSats],[GPSHDOP],[A1],[A2],[A3],[A4],[A5],[CPM]*[CKSUM]
All GPS fields are taken from GPGGA and GPRMC sentences.  Onboard data (An) is defined as follows:
A1: Rail voltage (should be within a couple of bits of max)
A2: Temperature
A3: Pressure
A4: Humidity
A5: Battery voltage (scaled through a 1/4 voltage divider)
CPM: Radiation counts per minute - not yet implemented, will remain at 0

This system provides two serial ports - one for command, status, and debug, and the other for reconstructed NMEA sentences, suitable for use with a COTS mapping software system, like DeLorme Street Atlas.  Both serial ports are configured for 115200/8/N/1.  The NMEA port can optionally be equipped with a bluetooth module for wireless connections.

On the command port, the user can access system features by entering a * at the terminal, then the character representing the desired function.  *? displays the help, which describes the available functions:
****M: Send user message.
****P: Send predict message.
****I: Place me on the map.
****C: Send a command. (not supported)
****B: Toggle position beacons
****D: Toggle debug mode.
****N: Show navigational and pointing info
****G: Change your station ID (beta)

Note that user messages and station IDs must not contain special characters, and are currently limited to 14 characters.

When enabled, position beacons automatically transmit a user message sentence to the rest of the network every 67 seconds.  This allows monitoring of the location of other ground stations' location without the need to manually send user messages.

The goal of each BEACONNet message is to make it individually packetizable, so as to minimize network overhead and maximize the reliability of received sentences.  In order to achieve this, some padding and other magic may be necessary.  The sentence definition above will be updated with further info as it evolves.
