#! /usr/bin/env python
'''RadarCalDroneMission.py assembles a set of mission waypoints for
flying a CODAR quadcopter calibration. Resulting mission can be
saved to a QGC ("QGroundControl") mission file for transferring to a
quadcopter; saved to a KML file for viewing in Google Earth; and/or
analyzed for flight duration, etc.

To run as a script:

With no input arguments, runs demo code:
   python RadarCalDroneMission.py

Otherwise, assembles a set of mission waypoints either by reading in an
existing QGC mission file with syntax:
   python RadarCalDroneMission.py mission_file.txt
   
or from supplied mission parameters with syntax:
   python RadarCalDroneMission.py clat clon rad hdg1 hdg2 alt transit_spd cruise_spd direction

where the input arguments are:
        clat,clon: the latitude and longitude of the CODAR antenna (centre of the circular arc).

        rad: the radius of the circular arc in metres.

        hdg1, hdg2: the True heading of the start and end of the arc, going clockwise.

        alt: the flight altitude relative to takeoff in metres.

        transit_spd: the speed flown when transiting to/from survey arc [m/s].

        cruise_spd: the speed flown along the survey arc [m/s].

        direction: Specifies whether mission will be flown in:
          a) the clockwise direction ('cw');
          b) the counterclockwise direction ('ccw');
          c) the clockwise then the counterclockwise direction in the same mission ('cw_ccw'); or
          d) the counterclockwise then the clockwise direction in the same mission ('ccw_cw').        

Flags -k, -q, -f, -a, and -c control what the script does with the waypoints it has assembled.

  -k kml_file: Output the specified KML file for viewing the mission in Google Earth.
  -q qgc_file: Output the specified QGC file (this action is not permitted when the waypoints have been read in from an existing QGC file).
  -f: Forces overwriting of either an existing KML or QGC file rather than asking user for confirmation.
  -a: Outputs an analysis of the mission (expected duration, summary of important parameters, etc.)
  -c top_speed: Sets the top speed of the drone for use in analysis. If not specified, the commanded transit speed (which may be unrealistically high) will be used.

Example steps:

  # 1) Create a mission from values and write it to a QGC file. Use
  # radius=400m, headings 239 and 66 degrees True, altitude=10m,
  # cruise speed=3m/s, transit speed=20m/s. Mission to be flown in
  # clockwise direction.
  python RadarCalDroneMission.py 48.873676 -123.291041 400 239 66 10.0 3 20 cw -q mymission_raw.txt

  # 2) Open the resulting QGC file, mymission_raw.txt, in Ardupilot
  # Mission Planner or in APM Planner 2.0, and use the GUI to move the
  # HOME position and rally points. HOME should be dragged to be close
  # to the planned takeoff location. The first rally point should be
  # dragged close (but not too close) to the takeoff location.  (this
  # is a workaround for a bug in the Open Solo operating system that
  # causes the drone to miss the initial change to transit speed if
  # the drone is launched manually before entering autonomous mode,
  # resulting in the drone flying out to the survey arc at slow
  # speed. A second speed-change command is inserted after the first
  # rally point; drag the rally point to beyond the location where you
  # plan to command autonomous mode). Save the modified mission as
  # mymission.txt.
  
  # 3) Run RadarCalDroneMission.py a second time, using the modified
  # mission file as an input argument.  Request the output of a kml
  # file and an analysis of the mission (-a). Limit the top speed of
  # the the drone to 15m/s for the analysis, regardless of the speeds
  # commanded by the operator.
  python RadarCalDroneMission.py mymission.txt -c 15 -k mymission.kml -a

'''

import os, sys
import math
from math import pi
import numpy as np
import simplekml
import pyproj
import argparse

EARTH_RADIUS_M = (6353000 + 6384000)/2. # Average of range of radii, from wikipedia
ULTRA_SLOW_SPEED = 0.000001 # m/s
DEFAULT_SPACING = 360/100 # [degrees]; 100 points for a full circle has worked well in the past.

# Drone operator can specify arbitrarily large speed values in "change speed" Mavlink commands.
# For analysis, limit speed "flown" in simulation to a realistic top speed. This will vary
# by drone--adjust using the set() method.
TOP_SPEED = 18 # Top speed observed in Oct.2024 VJOR APM when commanded to fly at 20m/s.

ALT_TYPE_ABS = 0
ALT_TYPE_REL = 3
ALT_TYPE_TER = 10
ALT_TYPE_MISSION = 2
WPT_TYPE_WPT = 16
WPT_TYPE_CHANGE_SPEED = 178
WPT_TYPE_RTL = 20

# According to MP, speed types are 0:horiz; 2:up; 3: down, with 0 the default.
# According to APM Planner, speed types are 0:AS, 1:GS, with 0 the default.
# Confusing, but bottom line is to always use speed type 0.
SPD_TYPE_VALID = 0

class MissionError(Exception):
    pass

class MissionReadError(MissionError):
    pass

class UnknownWptTypeError(MissionError):
    pass

################################################################
def hdg2latlon(hdg, clat, clon, rad):
    '''
    Converts a heading relative to the centre of a circular arc of
    radius rad metres to a latitude, longitude. Assumes a locally flat earth.

    '''
    dx = rad*math.cos(hdg)
    dy = rad*math.sin(hdg)
    lat = clat + (180./pi)*(dy/EARTH_RADIUS_M)
    lon = clon + (180./pi)*(dx/EARTH_RADIUS_M)/math.cos(clat*pi/180)

    return lat, lon


################################################################
class RadarCalDroneMission(object):
    '''Class for generating and analyzing quadcopter mission files
    for performing CODAR Antenna Pattern Measurements (APMs).

    (This class has been given a name that does not contain the acronym "APM" so as
    to avoid with the "APM" (ArduPilot Mega) quadcopter autopilot).
    
    kpb@oceannetworks.ca

    ################################################################
    # REVISION HISTORY:
    #
    # 2024-11-29, kpb -- Created.
    #
    # 2024-12-02, kpb -- Inserted an additional rally point as workaround
    # for 1st change speed waypoint being ignored.
    #
    # 2025-03-10, kpb -- Disable writing of RTL waypoint.
    #
    ################################################################

    '''
    ################################################################
    def __init__(self, *args):
        if len(args) == 1 and isinstance(args[0], str):
            # Handle case where a filename was provided.
            self.self_generated = False
            qgc_file = args[0]
            self.from_file = qgc_file
            try:
                self._load_from_file(qgc_file)
            except Exception as e:
                raise ValueError(f"Error reading file '{qgc_file}': {e}")

        elif len(args) == 9:
            self.self_generated = True
            self.from_file = None
            self.clat = args[0]
            self.clon = args[1]
            self.rad = args[2]
            self.hdg1 = args[3]
            self.hdg2 = args[4]
            self.alt = args[5]
            self.transit_spd = args[6]
            self.cruise_spd = args[7]
            self.direction = args[8]
            self.spacing = DEFAULT_SPACING
            self._create_mission_from_values()
        else:
            raise ValueError("Invalid arguments: Provide either a filename (str) or ....")

        # Top speed used for analysing planned flight times. Adjust using set().
        self.top_speed = TOP_SPEED
        
    ################################################################
    def set(self, name, val):
        # Update the mission with a new value of one of the parameters.
        do_update = True
        
        if name == 'clat':
            self.clat = val
        elif name == 'clon':
            self.clon = val
        elif name == 'rad':
            self.rad = val
        elif name == 'hdg1':
            self.hdg1 = val
        elif name == 'hdg2':
            self.hdg2 = val
        elif name == 'spacing':
            self.spacing = val
        elif name == 'alt':
            self.alt = val
        elif name == 'transit_spd':
            self.transit_spd = val
        elif name == 'cruise_spd':
            self.cruise_spd = val
        elif name == 'direction':
            self.direction = val
        elif name == 'top_speed':
            self.top_speed = val
            # A new top speed value does not require the regeneration of waypoints, as it
            # is used only in analysis.
            do_update = False
        else:
            mssg = ("Unrecognised parameter: %s" % name)
            raise ValueError(mssg)

        if do_update:
            self._create_mission_from_values()
        
    ################################################################
    def write_mission_qgc(self, qgc_file):
        '''Writes a QGroundControl mission file. Contents of wpts variable
        describe the essential attributes of the mission, but wpts
        contains only a limited subset of the variables needed for a
        QGC mission file. The missing information is supplied by
        this function in the form of default values considered
        suitable for radar calibration flights.

        QGC WPL 110
        0       1       0       16      0       0       0       0       53.4924630      -130.6370760    13.682770       1
        1       0       3       178     1.00000000      7.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.000000        1
        2       0       3       16      0.00000000      0.00000000      0.00000000      0.00000000      53.49042470     -130.63506360   20.000000       1
        ...
        67      0       3       20      0.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.000000        1

        cat /tmp/apm_mission.txt
        QGC WPL 110
        0	1	0	16	0	0	0	0	53.4924630000000008	-130.637076000000008	13.6827699999999997	1
        1	0	3	16	29	22	0	45	53.4924630000000008	-130.637076000000008	20	1
        2	0	3	178	0	99	55	0	53.4924630000000008	-130.637076000000008	20	1
        3	0	3	20	0	99	55	0	53.4924630000000008	-130.637076000000008	20	1
        
        wpt: wptnum, 1/0, alttype, wpttype, loiter_time, uncertrad, 0, yaw, lat, lon, alt, 1
        chspd: wptnum, 0, alttype, wpttype, 0?, spd, throttle, 0 (pad), lat (pad), on (pad), alt (pad), 1
        rtl: wptnum, 0, alttype (pad), wpttype, 0?, spd (pad), throttle (pad), 0 (pad), lat (pad), on (pad), alt (pad), 1
        '''

        if not self.self_generated:
            # A mission read in from a QGC file may have been stripped of unrecognised Mavlink commands,
            # so writing it to a new QGC might result in unexpected changes in flight behaviours.
            raise MissionError("Writing of QGC file only permitted for missions generated from values.")

        fid = open(qgc_file,'w')

        # Write header line.
        fid.write('QGC WPL 110\n')

        # Defaults.
        alt_type = ALT_TYPE_REL
        spd_type = SPD_TYPE_VALID
        throttle = 0
        
        # Write waypoints.
        for wpt_num in self.wpts:
            #print("wpt is", self.wpts[wpt_num])
            wpt_type = self.wpts[wpt_num]['type']
            #description = self.wpts[wpt_num]['description']

            # 2nd column is 1 on HOME waypoint, zero every other waypoint. HOME waypoint uses
            # absolute altitude type; all others use relative altitude.
            if wpt_num == 0:
                col2 = 1
                alt_type = ALT_TYPE_ABS
            else:
                col2 = 0
                alt_type = ALT_TYPE_REL

            if wpt_type == WPT_TYPE_WPT:
                lat = self.wpts[wpt_num]['lat']
                lon = self.wpts[wpt_num]['lon']   
                alt = self.wpts[wpt_num]['alt']

                # Pad uncoded values with zero values.
                speed = 0
                yaw = 0
                uncertainty_radius = 0
                last_col = 1
                loiter_time_s = 0

                # For some reason, format is slightly different for HOME waypoint.
                if wpt_num == 0:
                    # 0       1       0       16      0       0       0       0       53.4924630      -130.6370760    13.682770       1
                    out_str = ('%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%10.8f\t%10.8f\t%10.8f\t%d' % (wpt_num,col2,alt_type,wpt_type,loiter_time_s,speed,yaw,uncertainty_radius,lat,lon,alt,last_col))
                else:
                    # 2       0       3       16      0.00000000      0.00000000      0.00000000      0.00000000      53.49042470     -130.63506360   20.000000       1
                    out_str = ('%d\t%d\t%d\t%d\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%d' % (wpt_num,col2,alt_type,wpt_type,loiter_time_s,speed,yaw,uncertainty_radius,lat,lon,alt,last_col))
                
            elif wpt_type == WPT_TYPE_CHANGE_SPEED:
                speed = self.wpts[wpt_num]['speed']
                # Pad with zero values.
                lat = 0
                lon = 0
                alt = 0
                uncertainty_radius = 0
                # 1       0       3       178     1.00000000      7.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.000000        1
                out_str = ('%d\t%d\t%d\t%d\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%d' % (wpt_num,col2,alt_type,wpt_type,spd_type,speed,throttle,uncertainty_radius,lat,lon,alt,last_col))
                
            elif wpt_type == WPT_TYPE_RTL:
                # 67      0       3       20      0.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.000000        1
                # Pad with zeroes:
                loiter_time_s = 0
                speed = 0
                yaw = 0
                uncertainty_radius = 0
                lat = 0
                lon = 0
                alt = 0
                out_str = ('%d\t%d\t%d\t%d\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%10.8f\t%d' % (wpt_num,col2,alt_type,wpt_type,loiter_time_s,speed,yaw,uncertainty_radius,lat,lon,alt,last_col))
            
            fid.write('%s\n' % out_str)
        
        fid.flush()
        fid.close()
       
    ################################################################
    def _load_from_file(self, qgc_file):
        '''Reads in a QGroundControl mission file. This is lossy--only
        variables needed for analyzing an APM mission and/or creating
        a .kml file will be read in, and lines in the mission that are
        not needed for typical radar calibration flights will be
        ignored.

        Documentation of the format of QGC mission files is not easy
        to understand. The following description has been cobbled
        together from column labels and mouseover text in the Mission
        Planner and APM Planner software.

        # QGC mission files look like this. First line is a header
        # that indicates the file type and version. Spaces in all but
        # first line are actually a single space followed by a tab.
        # Example below was output by APM Planner. Other examples from
        # Mission Planner have "pad" values instead of zeroes (e.g.,
        # made-up lat/lon for speed change waypoints, made-up speeds
        # for "waypoint"-type waypoints). MP missions also have some
        # integers expressed as floats. The APM Planner format is
        # cleaner, so use that.

        QGC WPL 110
        0       1       0       16      0       0       0       0       53.4924630      -130.6370760    13.682770       1
        1       0       3       178     1.00000000      7.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.000000        1
        2       0       3       16      0.00000000      0.00000000      0.00000000      0.00000000      53.49042470     -130.63506360   20.000000       1
        ...
        67      0       3       20      0.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.00000000      0.000000        1        

        # Each line in Ardupilot parlance is a "waypoint", even if it
        # does not specify a lat/lon position. The first waypoint,
        # numbered zero, is the "HOME" waypoint.

        # The type of waypoint is indicated by the value in the 4th column:
        #   16 == a lat/lon/altitude location--a "waypoint" type of waypoint.
        #  178 == a "Change speed" waypoint.
        #   57 == a return to launch ("RTL") waypoint.
        # No other types of waypoint are recognized by this function.

        # Columns of "waypoint"-type waypoints are:
        0) waypoint number (zero based, with zeroth waypoint being "HOME")
        1) Unknown, but waypoint zero has value 1; all others have value zero.
        2) Altitude type: 0==absolute alt; 3==Relative alt; 10=="Terrain"; 2=="Mission".
        3) waypoint type: 16=="Waypoint"; 178==Change speed; 20=RTL
        4) Time [s] to loiter at this position before proceeding.
        5) Speed, m/s: 0 for every type of waypoint except "Change speed" waypoints.
        6) Desired yaw angle at waypoint (use 0).
        7) Uncertainy radius in metres (use 0).
        8) Latitude
        9) Longitude
        10) Altitude
        11) Unknown: 1 for all lines.

        # Columns of "Change speed" waypoints are:
        0) waypoint number (zero based, with zeroth waypoint being "HOME")
        1) Unknown, but waypoint zero has value 1; all others have value zero.
        2) Altitude type, but this is irrelevant for speed changes and can presumably be ignored.
        3) waypoint type: 16=="Waypoint"; 178==Change speed; 20=RTL
        4) Speed type: 0=AS; 1=GS
        5) Speed [m/s].
        6) Throttle setting: 0-100%, or -1==no change.
        Remaining 5 columns are apparently padding and can be ignored.

        # Columns of RTL waypoints are:
        0) waypoint number (zero based, with zeroth waypoint being "HOME")
        1) Unknown, but waypoint zero has value 1; all others have value zero.
        2) Altitude type, but this is irrelevant for RTL and can presumably be ignored.
        Remaining 9 columns are apparently padding and can be ignored.

        '''

        # QGC files do not include waypoint descriptions. Assign an
        # empty string to each waypoint's description as a
        # placeholder. (this means that kml files generated from
        # waypoints read in from a QGC file will not have labelled
        # waypoints, but this is unavoidable).
        QGC_WPT_DESCRIPTION = ''
        
        fid = open(qgc_file,'r')

        # Read header line.
        this_line = fid.readline().strip()
        if this_line != 'QGC WPL 110':
            raise MissionReadError("Bad header line")

        # Read remainder of lines.
        self.wpts = {}
        
        # ...Assume an impossibly slow speed at outset. This will
        # cause any analysis of the mission to predict an outrageously
        # long flight time instead of getting a divide-by-zero
        # error. The user can avoid this by ensuring that a
        # change-speed waypoint is the first waypoint after the
        # initial HOME position waypoint. This is best practice, as it
        # is not clear what speed the drone will fly at if a speed is
        # not explicitly set.
        curr_speed = ULTRA_SLOW_SPEED

        while len(this_line) > 0:
            this_line = fid.readline().strip()
            if len(this_line) > 0:
                strs = this_line.split()
                wpt_num = int(strs[0])
                alt_type = int(strs[2])
                wpt_type = int(strs[3])

                if wpt_num == 0:
                    if not alt_type == ALT_TYPE_ABS:
                        print("alt_type is", alt_type)
                        raise MissionReadError("Expect HOME altitude to be of type 'Absolute'.")
                    pass
                else:
                    if not alt_type == ALT_TYPE_REL:
                        # For safety, be inflexible about altitude type.
                        print("alt_type is", alt_type)
                        
                        raise MissionReadError("Expect all but HOME altitude to be of type 'Relative'.")
                
                if wpt_num == 1:
                    if not wpt_type == WPT_TYPE_CHANGE_SPEED:
                        # Warn the user.
                        print("For predictable flight performance, the first waypoint after HOME should be a CHANGE_SPEED waypoint. Proceeding with assumption of near-zero speed.")
                if wpt_type == WPT_TYPE_WPT:
                    # "Waypoint"-type waypoint (i.e., actual lat/lon/alt position).
                    loiter_time_s = float(strs[4])
                    if not loiter_time_s == 0:
                        raise MissionReadError("Expect no loitering at waypoints.")

                    curr_lat = float(strs[8])
                    curr_lon = float(strs[9])
                    curr_alt = float(strs[10])

                    # No speed encoded in this type of waypoint. Use the current speed value.
                    self.wpts[wpt_num] = {'type':wpt_type, 'speed':curr_speed, 'lat':curr_lat, 'lon':curr_lon, 'alt':curr_alt, 'description':QGC_WPT_DESCRIPTION}
                
                elif wpt_type == WPT_TYPE_CHANGE_SPEED:
                    # Change-speed type waypoint.
                    # ...Get error if converting string '0.000000' directly to int, so convert to float, first.
                    spd_type = int(float(strs[4]))
                    
                    if not spd_type == SPD_TYPE_VALID:
                        # Finding type "3" in BONI missions. Okay?
                        #raise MissionReadError("Expect all speeds to be of type '0'.")
                        pass
                
                    curr_speed = float(strs[5])
                    #throttle = int(strs[6])
                    throttle = int(float(strs[6]))
                    
                    if not throttle == 0:
                        # -1 should also work, but all our flights so far have been with this set to 0, and that worked.
                        # For safety, insist that other values not be allowed.
                        raise MissionReadError("Expect all throttle values to be zero.")
                    
                    # Lat/lon/alt values encoded in this type of waypoint are meaningless padding (I think). Use the values from
                    # the previous "waypoint" waypoint instead.                    
                    self.wpts[wpt_num] = {'type':wpt_type, 'speed':curr_speed, 'lat':curr_lat, 'lon':curr_lon, 'alt':curr_alt, 'description':QGC_WPT_DESCRIPTION}

                elif wpt_type == WPT_TYPE_RTL:
                    # All values encoded in an RTL line are (I think) meaningless padding. Missions flown at BONI have had
                    # lat/lon/alt values of zero, and the drone returned to launch safely. Just in case there are any
                    # unexpected results, put the HOME lat/lon values in here.
                    curr_lat = self.wpts[0]['lat']
                    curr_lon = self.wpts[0]['lon']
                    self.wpts[wpt_num] = {'type':wpt_type, 'speed':curr_speed, 'lat':curr_lat, 'lon':curr_lon, 'alt':curr_alt, 'description':QGC_WPT_DESCRIPTION}

                else:
                    mssg = ('Unexpected waypoint type %d found while loading from file.' % wpt_type)
                    raise UnknownWptTypeError(mssg)

        fid.close()
        #return wpts
        
    ################################################################
    def write_mission_kml(self, kml_file):
        '''
        Writes kml file for viewing waypoints in Google Earth.

        The "description" field of a waypoint will not qq

        '''
        kml = simplekml.Kml(open=1)

        for wpt in self.wpts:
            # Only "waypoint"-type waypoints contain valid location data.
            if self.wpts[wpt]['type'] == WPT_TYPE_WPT:
                lat = self.wpts[wpt]['lat']
                lon = self.wpts[wpt]['lon']
                alt = self.wpts[wpt]['alt']
                description = self.wpts[wpt]['description']
                pnt = kml.newpoint()

                if description.startswith('Rally') or description == 'HOME':
                    pnt.name = description
                else:
                    pnt.name = wpt
                
                pnt.altitudemode = simplekml.AltitudeMode.relativetoground
                pnt.coords = [(lon, lat, alt)]
            
        kml.save(kml_file)

    ################################################################
    def analyze_mission(self):
        '''Displays information about the mission. Total time,
        distance flown, max distance from home, min/max altitude
        commanded.

        '''

        geodesic = pyproj.Geod(ellps='WGS84')
            
        elapsed_seconds = 0
        distance_flown_m = 0
        max_dist_from_home = 0
        min_commanded_alt = math.inf
        max_commanded_alt = -math.inf
        
        curr_speed = ULTRA_SLOW_SPEED
        prev_lat = None
        prev_lon = None

        num_speed_changes = 0
        speeds = []

        home_lat = self.wpts[0]['lat']
        home_lon = self.wpts[0]['lon']
        
        for wpt in self.wpts:
            wpt_type = self.wpts[wpt]['type']
            if wpt_type == WPT_TYPE_WPT:
                curr_lat = self.wpts[wpt]['lat']
                curr_lon = self.wpts[wpt]['lon']
                _, _, dist_from_home = geodesic.inv(home_lon, home_lat, curr_lon, curr_lat)
                max_dist_from_home = max([max_dist_from_home, dist_from_home])
                
                if wpt == 0:
                    # Ignore HOME point in altitude analysis. Also, no previous
                    # lat/lon to use in distance/time calculations.
                    pass
                else:
                    curr_alt = self.wpts[wpt]['alt']
                    min_commanded_alt = min([min_commanded_alt, curr_alt])
                    max_commanded_alt = max([max_commanded_alt, curr_alt])

                    _, _, this_leg_dist_m = geodesic.inv(prev_lon, prev_lat, curr_lon, curr_lat)
                    distance_flown_m = distance_flown_m + this_leg_dist_m
                    this_leg_seconds = this_leg_dist_m/curr_speed
                    elapsed_seconds = elapsed_seconds + this_leg_seconds
                    
                prev_lat = curr_lat
                prev_lon = curr_lon
                    
            elif wpt_type == WPT_TYPE_CHANGE_SPEED:
                curr_speed = self.wpts[wpt]['speed']
                num_speed_changes = num_speed_changes + 1
                speeds.append(curr_speed)
                if curr_speed > self.top_speed:
                    print(("Warning: Commanded speed %.1f m/s is unrealistically high; limiting top speed used in analysis to %.1f m/s" % (curr_speed, self.top_speed)))
                    curr_speed = self.top_speed
                
            elif wpt_type == WPT_TYPE_RTL:
                # Add the distance back to HOME to the total.
                _, _, this_leg_dist_m = geodesic.inv(home_lon, home_lat, curr_lon, curr_lat)
                distance_flown_m = distance_flown_m + this_leg_dist_m
                this_leg_seconds = this_leg_dist_m/curr_speed
                elapsed_seconds = elapsed_seconds + this_leg_seconds
            
            else:
                mssg = ('Unexpected waypoint type %d found while analyzing mission.' % wpt_type)
                raise UnknownWptTypeError(mssg)

        elapsed_minutes = elapsed_seconds/60

        print("")
        print("ANALYSIS OF MISSION FOLLOWS:")
        
        if self.self_generated:
            print("Mission generated from values.")
        else:
            print("Mission read in from file", self.from_file)
        
        print("Total distance flown = \t\t%d metres." % (distance_flown_m))
        print("Total flight time = \t\t%d seconds (%.1f minutes)." % (elapsed_seconds,elapsed_minutes))
        print("Maximum distance from HOME = \t%d metres." % max_dist_from_home)
        print("Minimum commanded altitude = \t%.1f metres." % min_commanded_alt)
        print("Maximum commanded altitude = \t%.1f metres." %  max_commanded_alt)
        print("Number of speed changes = \t%d." % num_speed_changes)
        print("Commanded speeds:" )
        for speed in list(set(speeds)):
            print("\t\t\t%.1f m/s" % speed)
        print("Top speed permitted in analysis = %.1f m/s." % self.top_speed)



    ################################################################
    def _create_mission_from_values(self):
        '''
        Creates a set of mission waypoints.

        Necessary attributes are:

        clat,clon: the latitude and longitude of the CODAR antenna (centre of the circular arc).

        rad: the radius of the circular arc in metres.

        hdg1, hdg2: the True heading of the start and end of the arc, going clockwise.

        spacing: the angular spacing of waypoints in degrees. Value may be adjusted
        automatically to higher density to be sure to capture the hdg1, hdg2 endpoints of the arc.

        alt: the flight altitude relative to takeoff in metres.

        transit_spd: the speed flown when transiting to/from survey arc [m/s].

        cruise_spd: the speed flown along the survey arc [m/s].

        direction: Specifies whether mission will be flown in:
          a) the clockwise direction ('cw');
          b) the counterclockwise direction ('ccw');
          c) the clockwise then the counterclockwise direction in the same mission ('cw_ccw'); or
          d) the counterclockwise then the clockwise direction in the same mission ('ccw_cw').        

        Algorithm based on:
        http://stackoverflow.com/questions/15886846/python-elegant-way-of-finding-the-gps-coodinates-of-a-circle-around-a-certain-g 

        '''
        self.wpts = {}

        hdg1_math_radians, hdg2_math_radians = self.convert_headings(self.hdg1, self.hdg2)
        arc_length_degrees = abs(hdg2_math_radians - hdg1_math_radians) * 180/pi
        
        # Tweak the waypoint spacing to include endpoints. Spacing will be
        # less than or equal to specified value of spacing_degrees.
        num_arc_pts = math.ceil(arc_length_degrees/self.spacing) + 1
        #print("arc_length_degrees",arc_length_degrees,", num_arc_pts", num_arc_pts)

        # Define a clockwise arc is between heading 1 and heading 2.
        cw_arc_hdgs = np.linspace(hdg1_math_radians, hdg2_math_radians, num=num_arc_pts, endpoint=True)
        ccw_arc_hdgs = np.flip(cw_arc_hdgs)

        if self.direction.lower() == 'cw':
            arc_hdgs = cw_arc_hdgs
        elif self.direction.lower() == 'ccw':
            arc_hdgs = ccw_arc_hdgs
        elif self.direction.lower() == 'cw_ccw':
            arc_hdgs = np.concatenate([cw_arc_hdgs, ccw_arc_hdgs[1:]])
        elif self.direction.lower() == 'ccw_cw':
            arc_hdgs = np.concatenate([ccw_arc_hdgs, cw_arc_hdgs[1:]])
        else:
            raise ValueError("Unrecognised arc direction.")
        
        # Create mission waypoints.

        # ...First waypoint is HOME. No speed encoded in this type of waypoint; use the transit
        # speed as a pad value  (though this should have no effect on flight)...no, existing MP files have speed=0.
        #self.wpts[0] = {'type':WPT_TYPE_WPT, 'speed':self.transit_spd, 'lat':self.clat, 'lon':self.clon, 'alt':self.alt, 'description':'HOME'}
        self.wpts[0] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':self.clat, 'lon':self.clon, 'alt':self.alt, 'description':'HOME'}

        # ...2nd waypoint is a change-speed waypoint to go to transit (fast) speed. Pad with the HOME lat and lon and the
        # mission altitude (though these should have no effect on flight).
        self.wpts[1] = {'type':WPT_TYPE_CHANGE_SPEED, 'speed':self.transit_spd, 'lat':self.clat, 'lon':self.clon, 'alt':self.alt, 'description':'speed up'}

        # 2024-12-02, kpb -- Insert an additional rally point close to the launch site. This is a workaround for an apparent
        # bug in the 3DR Solo. In the Oct., 2024 VJOR/LPR1 APM, the commanded transit speed was 20 m/s; the actual transit
        # speed was 10 m/s outbound and 18 m/s inbound. The first change-speed waypoint was apparently being ignored--this
        # may be because I took off manually before entering autonomous mode. A second waypoint on the inbound leg can also
        # be useful for shaping the return path, so add one there, too.
        arc_first_lat, arc_first_lon = hdg2latlon(arc_hdgs[0], self.clat, self.clon, self.rad)

        # ...Put initial rally point some smallish fraction of the way
        # out to the first survey arc.  (this algorithm may not work
        # well near equator or prime meridian). User can later drag
        # waypoint to a new location in APM Planner or Mission
        # Planner, if desired. Note that (if my interpretation of the
        # bug is correct), the vehicle will not change to transit
        # speed until after this lat/lon is reached, so it should be
        # dragged fairly close to HOME.
        NEAR_RALLY_WPT_FRACTION = 0.1
        lat = self.clat + NEAR_RALLY_WPT_FRACTION*(arc_first_lat - self.clat)
        lon = self.clon + NEAR_RALLY_WPT_FRACTION*(arc_first_lon - self.clon)
        self.wpts[2] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'Rally point 1'}

        # Add change-speed waypoint after first rally point.
        self.wpts[3] = {'type':WPT_TYPE_CHANGE_SPEED, 'speed':self.transit_spd, 'lat':0, 'lon':0, 'alt':0, 'description':'speed up'}

        # ...5th waypoint is a rally point half way to first survey point. User can later drag it to a new location in
        # APM Planner or Mission Planner, if desired.
        lat = (arc_first_lat + self.clat)/2
        lon = (arc_first_lon + self.clon)/2
        self.wpts[4] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'Rally point 2'}

        # ...6th waypoint is the first point in survey arc.
        self.wpts[5] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':arc_first_lat, 'lon':arc_first_lon, 'alt':self.alt, 'description':'survey point'}
        
        # ...7th waypoint is a change-speed waypoint to switch to
        # cruise speed. Pad with the previous lat and lon and the
        # mission altitude (though these should have no effect on
        # flight)...No, BONI MP files use lat, lon = 0.
        #self.wpts[4] = {'type':WPT_TYPE_CHANGE_SPEED, 'speed':self.cruise_spd, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'slow down'}
        self.wpts[6] = {'type':WPT_TYPE_CHANGE_SPEED, 'speed':self.cruise_spd, 'lat':0, 'lon':0, 'alt':0, 'description':'slow down'}

        # ...6th waypoints onwards are arc points. Skip the first arc point, which has already been written to, above.
        wpt_num = 7
        for hdg in arc_hdgs[1:]:
            lat, lon = hdg2latlon(hdg, self.clat, self.clon, self.rad)
            #self.wpts[wpt_num] = {'type':WPT_TYPE_WPT, 'speed':self.cruise_spd, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'survey point'}
            self.wpts[wpt_num] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'survey point'}
            wpt_num = wpt_num + 1

        arc_last_lat = lat
        arc_last_lon = lon

        # ...Next waypoint after arc is to change speed to transit speed.
        #self.wpts[wpt_num] = {'type':WPT_TYPE_CHANGE_SPEED, 'speed':self.transit_spd, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'speed up'}
        self.wpts[wpt_num] = {'type':WPT_TYPE_CHANGE_SPEED, 'speed':self.transit_spd, 'lat':0, 'lon':0, 'alt':0, 'description':'speed up'}
        wpt_num = wpt_num + 1

        # ...Next waypoint is the third rally point, halfway back to HOME. Again,
        # using mean of lat and lon as a location may cause problems near
        # Equator/Prime Meridian.
        lat = (arc_last_lat + self.clat)/2
        lon = (arc_last_lon + self.clon)/2
        wpt_num = wpt_num + 1

        #self.wpts[wpt_num] = {'type':WPT_TYPE_WPT, 'speed':self.transit_spd, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'Rally point 2'}
        self.wpts[wpt_num] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'Rally point 3'}
        wpt_num = wpt_num + 1

        # ...Second inbound rally point.
        lat = self.clat + NEAR_RALLY_WPT_FRACTION*(arc_last_lat - self.clat)
        lon = self.clon + NEAR_RALLY_WPT_FRACTION*(arc_last_lon - self.clon)
        self.wpts[wpt_num] = {'type':WPT_TYPE_WPT, 'speed':0, 'lat':lat, 'lon':lon, 'alt':self.alt, 'description':'Rally point 4'}
        wpt_num = wpt_num + 1

        # ...Final point is an RTL.
        # 2025-03-10, kpb -- RTLs have possibly been causing control issues for manual landings,
        # with the drone fighting against the pilot. So do NOT add an RTL waypoint at the end of each mission.
        #self.wpts[wpt_num] = {'type':WPT_TYPE_RTL, 'speed':self.transit_spd, 'lat':self.clat, 'lon':self.clon, 'alt':self.alt, 'description':'RTL'}
        #self.wpts[wpt_num] = {'type':WPT_TYPE_RTL, 'speed':0, 'lat':0, 'lon':0, 'alt':0, 'description':'RTL'}
        
    ################################################################
    def convert_headings(self, start_heading, end_heading):
        # Converts start and end heading from compass degrees to
        # radians in "math" coordinates (positive CCW from positive x-axis).
        #
        # start_heading_math_radians, end_heading_math_radians = convert_headings(start_heading, end_heading)

        if start_heading < 0 or start_heading > 360:
            raise ValueError("Start heading must be between 0 and 360 degrees True.")

        if end_heading < 0 or end_heading > 360:
            raise ValueError("End heading must be between 0 and 360 degrees True.")

        # Convert headings to radians, positive counterclockwise from positive x-axis.
        start_heading_radians = start_heading*pi/180.
        start_heading_math_radians = math.atan2(math.cos(start_heading_radians),math.sin(start_heading_radians))
        end_heading_radians = end_heading*pi/180.
        end_heading_math_radians = math.atan2(math.cos(end_heading_radians),math.sin(end_heading_radians))

        # Express as positive values from 0 to 2pi only.
        start_heading_math_radians = start_heading_math_radians % (2*pi)
        end_heading_math_radians = end_heading_math_radians % (2*pi)

        # For a clockwise rotation, the start heading in math
        # coordinates must be *greater* than the end heading (because
        # "math" angles increase in the counterclockwise
        # direction). This will not necessarily be the case if there
        # is a zero crossing between the two headings.
        if start_heading_math_radians <= end_heading_math_radians:
            start_heading_math_radians = start_heading_math_radians + 2*pi

        return start_heading_math_radians, end_heading_math_radians
    
################################################################
def demo():
    # Use the demo code below as a guide to using this program.
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO starting.")
    
    # The following line would have to be included and uncommented in
    # any program using RadarCalDroneMission.
    
    #from RadarCalDroneMission import RadarCalDroneMission

    # Create a mission from values. The mission is a circular arc,
    # centred around the CODAR antenna location (clat, clon).
    clat = 48.87365
    clon = -123.291
    rad = 400          # radius, metres
    hdg1 = 239         # clockwise end of arc [degrees True] 
    hdg2 = 66          # counterclockwise end of arc [degrees True] 
    #spacing = 10 
    alt = 10           # Altitude above takeoff point [m]
    transit_spd = 20   # Fast [m/s]
    cruise_spd = 3     # Slow speed for survey arc [m/s]
    direction = 'ccw'  # Direction of flight ('cw', 'ccw', ...

    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Creating mission object from values.")
    #mission_from_values = RadarCalDroneMission(clat, clon, rad, hdg1, hdg2, spacing, alt, transit_spd, cruise_spd, direction)
    mission_from_values = RadarCalDroneMission(clat, clon, rad, hdg1, hdg2, alt, transit_spd, cruise_spd, direction)

    # Adjust the altitude to a different value. You can do the same to any of the other values.
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Adjusting value of one mission parameter.")
    mission_from_values.set('alt',99)

    # Write the mission to a QGC data file.
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Writing mission to QGC file 'mission_from_values.txt'.")
    mission_from_values.write_mission_qgc('mission_from_values.txt')

    # Here you would open the QGC file in Ardupilot Mission Planner (Windows only) or in APM Planner 2.0 (Windows,
    # Mac, Linux). If desired, drag the HOME position from the antenna location to the expected takeoff point.
    # Also drag the two "rally" points to suitable locations (only necessary if terrain demands it). Export the
    # the mission into the file 'mission_from_values_edited.txt'. For the sake of allowing this demo code to
    # run, the "edited" mission file will just be the unedited mission written to a new file name:
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Writing mission to QGC file 'mission_from_values_edited.txt'. In real usage, this would NOT be done--'mission_from_values_edited.txt' would be exported from either Ardupilot Mission Planner or from APM Planner 2.0.")
    mission_from_values.write_mission_qgc('mission_from_values_edited.txt')

    # Read the manually edited mission file.
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Creating mission object from file.")
    mission_from_file = RadarCalDroneMission('mission_from_values_edited.txt')

    # Output the mission into a .kml file for viewing in Google Earth as a sanity check.
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Writing mission to file 'mission_from_file.kml' for viewing in Google Earth.")
    mission_from_file.write_mission_kml('mission_from_file.kml')
    #mission_from_self_generated_file.analyze_mission()

    # Output an analysis of the mission.
    print("")
    print("-------------------------------------------------------------------")
    print("DEMO: Analyzing mission.")
    mission_from_file.analyze_mission()

    print("")
    print("-------------------------------------------------------------------")
    print("DEMO complete.")
    
################################################################
if __name__ == '__main__':

    if len(sys.argv) < 2:
        # Called with no input arguments. Run the demo.
        demo()
    else:
        # Parse input arguments.
        parser = argparse.ArgumentParser(description='Generating and analyze quadcopter mission files for performing CODAR Antenna Pattern Measurements (APMs).')
        parser.add_argument('-f', '--force', action='store_true', help='Do not ask user before overwriting output files.')
        parser.add_argument('-k','--kml_file', help=".kml output filename.")
        parser.add_argument('-q','--qgc_file', help="QGC output filename.")
        parser.add_argument('-c','--top_speed', help="Set real-world top speed of vehicle [m/s].", type=float)
        parser.add_argument('-a','--analyze', action='store_true', help="Display analysis of mission.")
        args, whats_left = parser.parse_known_args()

        if args.force:
            do_force = True
        else:
            do_force = False

        if args.top_speed:
            do_change_max_speed = True
        else:
            do_change_max_speed = False
        
        try:
            arg1 = float(whats_left[0])
            # First argument converts to a float, so it is not a filename.
            from_file = False
        except ValueError:
            # First argument does not convert to a float, so it is a string (filename).
            from_file = True

        if from_file:
            in_file = whats_left.pop(0)

            if len(whats_left)>0:
                mssg = "Unrecognised extra input arguments:" + repr(whats_left)
                raise ValueError(mssg)
            
            if args.qgc_file:
                # Don't allow creation of a QGC mission file from a mission that has been read in
                # from an existing QGC mission file. The reading in of QGC files is lossy and there
                # is no guarantee the resulting mission will run as the user expects.
                mssg = "Creating a QGC file from a mission read in from a QGC file is not permitted."
                raise ValueError(mssg)

            mission = RadarCalDroneMission(in_file)
            
        else:
            #print("whats_left is", whats_left)
            #print(len(whats_left))

            if not len(whats_left) == 9:
                mssg = ("Creating mission from values requires 9 arguments; %d passed.") % len(whats_left)
                raise ValueError(mssg)
            else:
                clat = float(whats_left.pop(0))
                clon = float(whats_left.pop(0))
                rad = float(whats_left.pop(0))
                hdg1 = float(whats_left.pop(0))
                hdg2 = float(whats_left.pop(0))
                alt = float(whats_left.pop(0))
                transit_spd = float(whats_left.pop(0))
                cruise_spd = float(whats_left.pop(0))
                direction = whats_left.pop(0)
                mission = RadarCalDroneMission(clat, clon, rad, hdg1, hdg2, alt, transit_spd, cruise_spd, direction)

        if do_change_max_speed:
            mission.set('top_speed',args.top_speed)

        # Output to kml file if requested.
        if args.kml_file:
            do_write = True

            if os.path.exists(args.kml_file):
                if do_force:
                    do_write = True
                else:
                    mssg = ("KML file %s already exists. Overwrite? [Y/N] " % args.kml_file) 
                    resp = input(mssg)
                    if resp.lower() == 'y':
                        print("Overwriting existing KML file %s. " % args.kml_file)
                    else:
                        print("Writing KML file aborted on user's request.")
                    
            if do_write:
                mission.write_mission_kml(args.kml_file)
                print("Wrote to KML file %s. " % args.kml_file)

        # Output to QGC file if requested.
        if args.qgc_file:
            do_write = True

            if from_file:
                # Not permitted. The reading in of QGC files by this script is lossy, so writing to
                # a file that could be given to a drone as a mission would be risky.
                do_write = False

            if do_write:
                if os.path.exists(args.qgc_file):
                    if do_force:
                        do_write = True
                    else:
                        mssg = ("QGC file %s already exists. Overwrite? [Y/N] " % args.qgc_file) 
                        resp = input(mssg)
                        if resp.lower() == 'y':
                            do_write = True
                        else:
                            print("Writing QGC file aborted on user's request.")
                            do_write = False

            if do_write:
                mission.write_mission_qgc(args.qgc_file)
                print("Wrote to QGC file %s. " % args.qgc_file)

        # Output analysis of mission.
        if args.analyze:
            mission.analyze_mission()

    sys.exit()

