function C = get_companion_matrix_sh5_3(data)
%% We build the companion matrix.
c0 = data(1)*data(2)^2*data(9)*data(10) - data(2)^3*data(10);
c1 = 2*data(1)*data(2)*data(4)*data(9)*data(10) - data(1)*data(2)*data(7)^2 - data(1)*data(2)*data(8)^2 - data(1)*data(2)*data(9)^2 + data(1)*data(2)*data(10)^2 + data(2)^2*data(3)*data(9)*data(10) - 3*data(2)^2*data(4)*data(10) + data(2)^2*data(5)*data(7) + data(2)^2*data(6)*data(8) + data(2)^2*data(9);
c2 = data(1)*data(4)^2*data(9)*data(10) - data(1)*data(4)*data(7)^2 - data(1)*data(4)*data(8)^2 - data(1)*data(4)*data(9)^2 + data(1)*data(4)*data(10)^2 - data(1)*data(9)*data(10) + 2*data(2)*data(3)*data(4)*data(9)*data(10) - data(2)*data(3)*data(7)^2 - data(2)*data(3)*data(8)^2 - data(2)*data(3)*data(9)^2 + data(2)*data(3)*data(10)^2 - 3*data(2)*data(4)^2*data(10) + 2*data(2)*data(4)*data(5)*data(7) + 2*data(2)*data(4)*data(6)*data(8) + 2*data(2)*data(4)*data(9) - data(2)*data(10);
c3 = data(3)*data(4)^2*data(9)*data(10) - data(3)*data(4)*data(7)^2 - data(3)*data(4)*data(8)^2 - data(3)*data(4)*data(9)^2 + data(3)*data(4)*data(10)^2 - data(3)*data(9)*data(10) - data(4)^3*data(10) + data(4)^2*data(5)*data(7) + data(4)^2*data(6)*data(8) + data(4)^2*data(9) - data(4)*data(10) + data(5)*data(7) + data(6)*data(8) + data(9);
C = [0,1,0;0,0,1;-c0/c3,-c1/c3,-c2/c3];
end