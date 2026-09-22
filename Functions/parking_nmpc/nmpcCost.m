function J = nmpcCost(U, x0, s0, buf, ref, p, w)
%NMPCCOST  How bad is this imagined future?  Lower is better.
%
%   U   : [N x 2] steering commands being considered   [front rear]
%   x0  : the car right now                            [X Y psi vx df dr]
%   s0  : how far along the path the car is right now
%   buf : [6 x 2] commands already in the delay queue
%   ref : the distance map from stage 3
%   p   : the car's numbers, including p.Ts
%   w   : the weights

Ucmd = [buf; U];                 % the queue plays out first, then the new plan
x = x0;  s = s0;  J = 0;

for k = 1:size(Ucmd,1)
    x = vehicleStep(x, [0; Ucmd(k,1); Ucmd(k,2)], p.Ts, p);   % speed fixed

    [s, ey] = nearestOnPath(ref, x(1), x(2), s);

    dpsi = x(3) - interp1(ref.s, ref.psi, s);
    epsi = atan2(sin(dpsi), cos(dpsi));
    edf  = x(5) - interp1(ref.s, ref.df, s);
    edr  = x(6) - interp1(ref.s, ref.dr, s);

    J = J + w.ey*ey^2 + w.psi*epsi^2 + w.d*(edf^2 + edr^2);
end

dU = diff([buf(end,:); U]);      % jumps between consecutive commands
J  = J + w.du*sum(dU(:).^2);
end