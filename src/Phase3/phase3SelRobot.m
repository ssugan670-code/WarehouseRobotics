function sel = phase3SelRobot()
%PHASE3SELROBOT  Which robot the single fleet camera looks from right now (watched robot, else the robot that is scanning a shelf).
sel = 1;
try
    F = evalin('base','Phase3Fleet'); sel = double(F.camSel);
catch
end
end
