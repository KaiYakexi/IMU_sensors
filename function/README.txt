Check the functions in the follwing order:

% Calculate kinematic quantities
jointorigins = calculateJointOrigins( markers );
segmentcos = calculateSegmentOrientation( markers,jointorigins );
jointangles = calculateJointAngles( segmentcos );

% Calculate synthetic IMU data
[position, orientation] =  generateSensorPose( markers,jointorigins,segmentcos,[0 0 0 0 0 0] ); 
[acc,gyr] = generateSensorData( position,orientation,freq );

generateSensorPose should be optimised to take 3 values for the orientation only. 
Additionally, the orientation and position values should not be fixed values but variables.



