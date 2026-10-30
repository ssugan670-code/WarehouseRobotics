function mask = p3CamMask(hsv,colour)
%P3CAMMASK  Carton-coloured pixels of a camera picture (HSV in 0..1) - ONE gate for the camera check and the locator.
%   hue   within 0.06 of the carton hue
%   sat   inside a band around the carton saturation (carton 0.76): a paler pixel is dust / glare, a MORE saturated one is the orange staging stand
%         (stand 0.95, same hue) - so the stand below a carton is no longer mistaken for carton
%   value only a floor (0.02): a carton in the shadow of the shelf level above it is nearly black (RGB 23/15/6 at the top edge down to 7/4/1 at the bottom edge in the real picture, V = 0.09 .. 0.03) but its
%         hue and saturation are unchanged - the old gate (V >= 0.2) lost exactly those cartons ("0/8 frames show the carton")
h0 = localHsv(colour(:).');
dh = abs(hsv(:,:,1) - h0(1)); dh = min(dh,1 - dh);
sLo = max(0.25,h0(2) - 0.22); sHi = min(0.98,h0(2) + 0.14);
S = hsv(:,:,2); V = hsv(:,:,3);
sOK = S >= sLo & (S <= sHi | V < 0.15);         % in deep shadow the 8-bit colour is too coarse for a saturation ceiling (RGB 7/4/1 -> S 0.86..1.0); a stand is never that dark
mask = dh <= 0.06 & sOK & V >= 0.02;
end

function h = localHsv(rgb)
mx = max(rgb); mn = min(rgb); d = mx - mn; s = 0; if mx > 0, s = d/mx; end
if d < 1e-9, hh = 0; elseif mx == rgb(1), hh = mod((rgb(2) - rgb(3))/d,6); elseif mx == rgb(2), hh = (rgb(3) - rgb(1))/d + 2; else, hh = (rgb(1) - rgb(2))/d + 4; end
h = [hh/6 s mx];
end
