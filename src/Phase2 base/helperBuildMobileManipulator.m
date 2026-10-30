function [robot,qStow] = helperBuildMobileManipulator()
%HELPERBUILDMOBILEMANIPULATOR Compact AMR chassis and Kinova Gen3 as one tree.
% Simulation 3D Robot owns this single tree, so the chassis cannot drift away
% from the arm and the Translation/Rotation signals move both together.
robot=rigidBodyTree('DataFormat','column','MaxNumBodies',40);
robot.BaseName='mobile_base_root';
% A small mount lift improves the visual match to the 2.45 m shelf without
% changing any Gen3 link lengths or the IK model.
% Modest visual/physical scale increase: +13% radius and +18% deck height.
% The arm remains the real Kinova kinematic chain; link lengths are never
% cosmetically scaled away from the rigidBodyTree used by IK.
R=0.34; H=0.45; armX=0.10;
% (Round 34: the old camera mast on the chassis was removed; the robot's single camera now sits on the lift carriage, see below.)
chassis=rigidBody('mobile_chassis');
% rigidBody.addVisual cylinder parameters are [radius height].
addVisual(chassis,'Cylinder',[R H],trvec2tform([0 0 H/2]));
addVisual(chassis,'Cylinder',[R+0.01 0.04],trvec2tform([0 0 0.10]));
addVisual(chassis,'Cylinder',[0.27 0.02],trvec2tform([0 0 H+0.01]));
railH=2.05;   % visual guide rail for the lift column (behind the arm base)
addVisual(chassis,'Box',[0.07 0.16 railH],trvec2tform([armX-0.15 0 H+railH/2]));
addBody(robot,chassis,robot.BaseName);

% IK is a kinematic pose solve; it does not need gravitational dynamics.
% Leaving gravity at zero avoids dynamics validation of visual-only fixed
% bodies in the mobile composite tree.
gen3=loadrobot('kinovaGen3','DataFormat','column','Gravity',[0 0 0]);
oldBase=gen3.BaseName;
% Round 30: vertical lift column (prismatic) between the chassis and the arm, so the same Gen3 can serve rack levels 1-3.
% Joint order of the tree is [lift; q1..q7]; lift = 0 reproduces the previous fixed mount exactly.
liftMax=1.90;
carriage=rigidBody('lift_carriage'); jl=rigidBodyJoint('lift_joint','prismatic');
jl.JointAxis=[0 0 1]; jl.PositionLimits=[0 liftMax]; setFixedTransform(jl,trvec2tform([armX 0 H+0.02])); carriage.Joint=jl;
addVisual(carriage,'Box',[0.16 0.30 0.08],trvec2tform([-0.04 0 0.0]));
% Round 34: visible camera on the lift carriage (visual only - no collision geometry, no effect on IK or planning).
% Position/aim come from helperVisionCameraMount so the body and the rendered camera always agree.
cm=helperVisionCameraMount(); cf=cm(1); cl=cm(2); cu=cm(3);
postY=-0.14; if abs(cl)>=0.10, postY=cl; end          % post stands beside the arm; a centred camera hangs on a short cross-bar
addVisual(carriage,'Box',[0.06 0.10 0.02],trvec2tform([cf-0.07 postY+0.04 0.05]));                      % foot on the carriage plate
addVisual(carriage,'Box',[0.03 0.03 cu-0.04],trvec2tform([cf-0.07 postY 0.04+(cu-0.04)/2]));            % mounting post
if abs(cl-postY)>0.02, addVisual(carriage,'Box',[0.03 abs(cl-postY) 0.03],trvec2tform([cf-0.07 (cl+postY)/2 cu-0.015])); end   % cross-bar
addVisual(carriage,'Box',[0.06 0.10 0.07],trvec2tform([cf-0.07 cl cu]));                                % camera housing
addVisual(carriage,'Box',[0.03 0.05 0.05],trvec2tform([cf-0.035 cl cu]));                               % lens barrel (optical centre is 2 cm ahead of it)
addBody(robot,carriage,'mobile_chassis');
addSubtree(robot,'lift_carriage',gen3,'ReplaceBase',false);
setFixedTransform(getBody(robot,oldBase).Joint,eye(4));

% The imported Gen3 model ends at EndEffector_Link.  This fixed frame is the
% suction-pipe tip used consistently by IK, FK guards, and collision checks.
% Its local +Z axis is the pipe's approach direction.  The 0.10 m extent is
% an explicit model dimension (there was no separately supplied pipe mesh to
% measure), not an inferred visual scale.
tool=rigidBody('ToolTip');
setFixedTransform(tool.Joint,trvec2tform([0 0 0.10]));
addVisual(tool,'Cylinder',[0.015 0.10],trvec2tform([0 0 -0.05]));
% Two-finger side clamp sized for the 0.45 x 0.50 x 0.36 carton: palm plate behind the tip plane,
% two flat finger pads that straddle the carton's left/right faces (inner gap 0.524 m vs 0.50 m box).
% Fingers are fixed visual/collision geometry; the 'squeeze' is logical (see controller attach).
addVisual(tool,'Box',[0.60 0.12 0.03],trvec2tform([0 0 -0.015]));
for s=[-1 1]
    addVisual(tool,'Box',[0.02 0.12 0.18],trvec2tform([s*0.272 0 0.09]));
end
addCollision(tool,'cylinder',[0.015 0.10],trvec2tform([0 0 -0.05]));
addBody(robot,tool,'EndEffector_Link');
addCollision(chassis,'cylinder',[R H],trvec2tform([0 0 H/2]));

% Folded stow avoids the forward camera's frustum; clamp to installed limits.
qStow=[0; deg2rad([0 15 180 -130 0 55 90]).'];   % [lift; q1..q7]
% Limits belong to each rigidBodyJoint: collect the eight non-fixed joints (lift + 7).
limits=zeros(8,2); n=0;
for k=1:numel(robot.Bodies)
    joint=robot.Bodies{k}.Joint;
    if ~strcmp(joint.Type,'fixed')
        n=n+1; limits(n,:)=joint.PositionLimits;
    end
end
assert(n==8,'Phase2:KinovaJoints','Expected lift + seven Kinova joints, found %d.',n);
qStow=min(max(qStow,limits(:,1)),limits(:,2));
end
