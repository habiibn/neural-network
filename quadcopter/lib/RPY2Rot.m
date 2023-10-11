function bRe = RPY2Rot(angles)
    varphi = angles(1);
    theta = angles(2);
    psi = angles(3);

    Rx = [1           0            0;...
          0 cos(varphi) -sin(varphi);...
          0 sin(varphi)  cos(varphi)];

    Ry = [cos(theta)  0   sin(theta);...
                   0  1            0;...
         -sin(theta)  0   cos(theta)];

    Rz = [cos(psi)    -sin(psi)    0;...
          sin(psi)     cos(psi)    0;...
          0                   0    1];


    %bRe = Rz* Ry * Rx;
    bRe = Rz * Rx * Ry;
end