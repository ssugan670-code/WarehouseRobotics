function m = helperVisionCameraMount()
%HELPERVISIONCAMERAMOUNT  The robot's single forward-looking camera: where it sits and how it is aimed.
% One place for the numbers, used by BOTH the visible camera body on the lift carriage
% (helperBuildMobileManipulator) and the rendered camera sensor (addRobotVisionView), so the
% body you see in the 3-D window and the picture in the vision window always agree.
%   m = [forward left up pitchDown focalPx]
%   forward/left/up : metres from the arm base on the lift carriage (left < 0 = right side); pitchDown in degrees;
%   focalPx = 300 -> wide 93 deg x 77 deg field of view (640x480 image).
% Centred on the robot's centre line (left = 0), 0.90 m above the arm base and 0.10 m ahead of it (Round 38: 10 cm higher than before so the robot's own deck/rail no longer shows at the bottom).
% The arm (shoulder, elbow, forearm, palm plate) works inside a volume that reaches ~0.5 m above the arm base,
% so a camera above that volume looks OVER the arm: stowed arm 0 % of the upper picture covered; at pick reach
% 0.55 / 0.80 m the carton is in frame (92-100 %) and 69-80 % of it unobstructed (the rest is the clamp's own palm plate);
% the horizon sits in the upper part of the frame for driving (checked with Gen3 forward kinematics).
% Rigid on the carriage: rises and falls with the lift, never pans.
m = [0.10 0.00 0.90 22 300];
end
