%%% Tests for EllipsoidFit %%%

[ofs, gain, rotM] = EllipsoidFit(XYZ);
X = XYZ(:,1);
Y = XYZ(:,2);
Z = XYZ(:,3);

% translate to (0,0,0)
XC = X - ofs(1);
YC = Y - ofs(2);
ZC = Z - ofs(3);

% rotate to XYZ axes
XYZC = [XC, YC, ZC] * rotM; 

refr = .5; % reference radius

% scale to sphere
XC = XYZC(:,1) / gain(1) * refr;
YC = XYZC(:,2) / gain(2) * refr;
ZC = XYZC(:,3) / gain(3) * refr;
figure;
subplot(2,2,1);hold on; xlabel('X')
axis equal;
grid on;
ylabel('Y')
zlabel('Z')
plot3(XC, YC, ZC);
[epx, epy, epz] = ellipsoid(ofs(1),ofs(2),ofs(3),gain(1),gain(2),gain(3));
S = surfl(epx, epy, epz);
rotev = SpinCalc('DCMtoEV', rotM, '.001', 1);
rotate(S, rotev(1:3), rotev(4));

subplot(2,2,2);hold on; plot3(X, Y, Z, 'ro');ellipsoid(ofs(1),ofs(2),ofs(3),gain(1),gain(2),gain(3));

%subplot(2,2,3);hold on; plot3(X, Y, Z, 'ro');
%subplot(2,2,4);hold on; plot3(X, Y, Z, 'ro');
% subplot(2,2,1); hold on; plot(XC,YC,'ro'); plot(X,Y,'kx');
% xlabel('X'); ylabel('Y'); axis equal; grid on;
% 
% subplot(2,2,2); hold on; plot(ZC,YC,'go'); plot(Z,Y,'kx');
% xlabel('Z'); ylabel('Y'); axis equal; grid on;
% 
% subplot(2,2,3); hold on; plot(XC,ZC,'bo'); plot(X,Z,'kx');
% xlabel('X'); ylabel('Z'); axis equal; grid on;