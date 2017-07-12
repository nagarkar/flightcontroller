% test ellipsoid fit
function [center, radii, evecs, v, chi2] = EllipsoidFitPlot( XYZ)
    % do the fitting
    [ center, radii, evecs, v, chi2 ] = ellipsoid_fit_new( XYZ, '' );
    fprintf( 'Ellipsoid center: %.5g %.5g %.5g\n', center );
    fprintf( 'Ellipsoid radii: %.5g %.5g %.5g\n', radii );
    fprintf( 'Ellipsoid evecs:\n' );
    fprintf( '%.5g %.5g %.5g\n%.5g %.5g %.5g\n%.5g %.5g %.5g\n', ...
        evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
    fprintf( 'Algebraic form:\n' );
    fprintf( '%.5g ', v );
    fprintf( '\n' );

    % draw data
    x = XYZ(:,1);
    y = XYZ(:,2);
    z = XYZ(:,3);
    figure,
    subplot(2,2,1);
    plot3( x, y, z, '.r' );
    axis equal
    ylabel('Y')
    zlabel('Z')
    xlabel('X')
    hold on;

    %draw fit
    mind = min( [ x y z ] );
    maxd = max( [ x y z ] );
    nsteps = 50;
    step = ( maxd - mind ) / nsteps;
    [ x, y, z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

    Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
              2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
              2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
    p = patch( isosurface( x, y, z, Ellipsoid, -v(10) ) );
    hold off;
    set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
    view( -70, 40 );
    %axis vis3d equal; % Makes the ellipoid look like a sphere.
    camlight;
    lighting phong;

    % Translate to spherical
    XC = XYZ(:,1) - center(1);
    YC = XYZ(:,2) - center(2);
    ZC = XYZ(:,3) - center(3);

    XYZC = [XC, YC, ZC] * evecs'; 

    XC = XYZC(:,1) / radii(1);
    YC = XYZC(:,2) / radii(2);
    ZC = XYZC(:,3) / radii(3);

    %draw fit to sphere
    subplot(2,2,2);
    plot3( XC, YC, ZC, '.r' );
    axis equal
    ylabel('Y')
    zlabel('Z')
    xlabel('X')
    hold on;

    mind = min( [ XC YC ZC ] );
    maxd = max( [ XC YC ZC ] );
    nsteps = 50;
    step = ( maxd - mind ) / nsteps;
    [ XC, YC, ZC ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

    SPHERE = XC.*XC +  YC.*YC + ZC.*ZC;
    p = patch( isosurface( XC, YC, ZC, SPHERE, 1 ) );
    hold off;
    set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
    view( -70, 40 );
    camlight;
    lighting phong;
end