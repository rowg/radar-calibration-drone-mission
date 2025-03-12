# radar-calibration-drone-mission
Python script that generates a drone mission for radar calibrations (APMs). 

Create, edit, and upload a mission to a drone, all from a Mac computer--no need for Microsoft-only *Mission Planner* software or manually editing a mission in an *Excel* spreadsheet.

## Background ##
The *Ardupilot Mission Planner* software ("MP" for short) can be used to create quadcopter missions incorporating circular arcs suitable for APMs. *MP* has two disadvantages, however. First, it requires considerable interactive user input to go from a full, 360-degree survey arc to a realistic, flyable mission. Second, *MP* runs only on Windows computers.

Like *Mission Planner*, the *APM Planner 2.0* software allows the user to create and edit quadcopter missions and upload them to the vehicle. The advantage of *APM Planner 2.0* is that it runs on Windows, Mac, and Linux; unfortunately it lacks *MP's* ability to generate the circular arcs needed for CODAR APMs[^1].

[^1]: note that the name "APM Planner" has nothing to do with Antenna Pattern Measurements; it stands for "ArduPilot Mega", which is one of the autopilots the program was written for.

The *RadarCalDroneMission.py* script bypasses this shortcoming by generating a mission file containing a circular arc. The user provides various parameters (radius, altitude, speed, etc.). The resulting mission file can be uploaded to a quadcopter as-is, or edited using either *Mission Planner* or *APM Planner 2.0* and uploaded afterward.

*RadarCalDroneMission.py* also analyzes the proposed mission, outputting the expected flight duration and distance for review; a .kml file can be generated, allowing the operator to view the planned mission in Google Earth as a sanity check.

## Using the software
**Quick start**: run the script with no input arguments to run demo code. Look in *RadarCalDroneMission.py* to see an example of the python commands that can be used to create a mission using *RadarCalDroneMission.py*.

Use *RadarCalDroneMission.py* as a script from the *Bash* command line, as in the following example:

```
# Create a mission, write it to a QGC (QGroundControl) file.
clat=49.01805                   # arc centre (CODAR antenna) latitude
clon=-123.1718833               # arc centre (CODAR antenna) longitude
rad=400                         # arc radius [m]
hdg1=135                        # counterclockwise end of arc [degrees True] 
hdg2=16                         # clockwise end of arc [degrees True] 
alt=10                          # Altitude above takeoff point [m]
transit_spd=20                  # Fast [m/s]
cruise_spd=3                    # Slow speed for survey arc [m/s]
direction=cw                    # Direction of flight ('cw', 'ccw', 'cw_ccw', 'ccw_cw')
q_file=vcol_cw_mission_raw.txt  # Output file name

python RadarCalDroneMission.py $clat $clon $rad $hdg1 $hdg2 $alt $transit_spd \
  $cruise_spd $direction -q $q_file
```
**Notes:**
+ The 'cw' and 'ccw' direction values correspond to clockwise and counterclockwise missions. The 'cw_ccw' direction value results in a mission that flies the specified arc first in the clockwise and then the counterclockwise direction. The 'ccw_cw' direction value results in the reverse.

+ hdg1 and hdg2, the two ends of the survey arc, can most easily be obtained by viewing the APM location in Google Earth. Use the "ruler" tool to explore options for different arc radii and angular extents.

Now open the resulting mission file (vcol_cw_mission_raw.txt) in *APM Planner 2.0* or *MP*. The "Home" position, which is initially at the CODAR antenna location (i.e., at the centre of the circular arc) is mouse-dragged to the approximate planned takeoff location. 

"Rally points" (waypoints between the APM arc and the takeoff/landing location) can also be mouse-dragged to shape the course of the vehicle to avoid obstacles during its transits. The very first rally point should be dragged to be near the takeoff location (an apparent bug in the *Open Solo* operating system for 3DR *Solo* quadcopters results in the vehicle flying at slow speed after a manual takeoff until this first waypoint is reached, even if a change-speed command appears immediately after the "Home" waypoint in the mission). Don't drag the first rally point *too* close to the takeoff location, however--it is possible that it, too, will be ignored if the vehicle has flown past it before autonomous flight begins.

<img width="1512" alt="vcol_apm_planner_raw_screenshot" src="https://github.com/user-attachments/assets/148f6d5b-837f-4208-ac6f-26df1f34159f" />
The raw mission plan created by RadarCalDroneMission.py and opened for editing in APM Planner 2.0. The "Home" location is at the centre of the survey arc, i.e., at the CODAR antenna location.
<br />
   <br />
  
<img width="1512" alt="vcol_apm_planner_edited_screenshot" src="https://github.com/user-attachments/assets/500a3cd1-b7a0-4243-a927-9a610dc1b763" />
The mission plan after editing in APM Planner 2.0. The "Home" location has been mouse-dragged to a suitable takeoff location; the 4 rally points have also been dragged to shape the quadcopter's transits to/from the survey arc.
<br />
   <br />

Save the edited mission to a file with a name like "vcol_cw_mission.txt" and then use *RadarCalDroneMission.py* to analyze the mission for flight duration, etc.

`python RadarCalDroneMission.py vcol_cw_mission.txt -k vcol_cw_mission.kml -a -c -15

The "-a" option causes the program to output an analysis of the mission; the "-c 15" option limits the top vehicle speed used in the analysis (a transit speed of 20 m/s was specified when the mission was created; the vehicle will do its best to fly this fast, but realistically, it will top out around 15 m/s). The "-k" option tells the program to output the .kml file vcol_cw_mission.kml for viewing in Google Earth.

The output from the analysis looks like this:

```
ANALYSIS OF MISSION FOLLOWS:
Mission read in from file vcol_cw_mission.txt
Total distance flown = 		2603 metres.
Total flight time = 		-173 seconds (-2.9 minutes).
Maximum distance from HOME = 	434 metres.
Minimum commanded altitude = 	10.0 metres.
Maximum commanded altitude = 	10.0 metres.
Number of speed changes = 	4.
Commanded speeds:
			3.0 m/s
			20.0 m/s
Top speed permitted in analysis = -15.0 m/s.
```

Check to see if the flight duration and distance to be flown look reasonable (this will depend on your drone and its batteries). Open the .kml file in Google Earth as a sanity check to confirm that the planned mission looks reasonable. The mission file vcol_cw_mission.txt is now ready to be written to the vehicle.

Repeat this process for the counterclockwise flight, and you are ready to proceed with an APM.
