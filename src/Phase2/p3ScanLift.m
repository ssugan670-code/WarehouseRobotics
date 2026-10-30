function h = p3ScanLift(slotZ)
%P3SCANLIFT  Carriage height [m] at which the robot camera looks at a carton slot of height slotZ during the scan.
%   The camera sits on the lift carriage (0.47 + helperVisionCameraMount height above the floor at lift 0) and looks 22 deg downwards, so a level-3 carton
%   (centre 2.33 m) is outside the picture at lift 0.  The robot therefore raises the carriage (arm stowed) until the camera is level with the slot,
%   scans, and goes on to the dock.  Level 1 and 2 need no lift (h = 0).
m = helperVisionCameraMount(); camZ0 = 0.47 + m(3);
h = slotZ - camZ0 - 0.05;
if h < 0.05, h = 0; end
h = min(h,1.80);
end
