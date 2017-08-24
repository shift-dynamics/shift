1;
nphi = 13;
sphi = nphi/pi;
points = [];
sntheta=[];
for ( i=0:nphi )
    phi = i/sphi;
    ntheta = round(sin(phi)*nphi);
    sntheta=[sntheta; ntheta];
    stheta = max(ntheta/pi,1);
    for ( j=0:2*ntheta-1 )
        theta = j/stheta;
        x = cos(theta)*sin(phi);
        y = sin(theta)*sin(phi);
        z = cos(phi);
        points = [points; [x,y,z]];
    endfor
endfor
sntheta
plot3(points(:,1),points(:,2),points(:,3),"^b");

%{
    double phi;
    double theta;
    double sphi;
    double stheta;
    int ntheta;
    int nphi;
    nphi = 32;
    sphi = nphi/PI;
    ntheta = 2*nphi;
    int count = 0;

    printf("sphi = %g\n", sphi);
    double dphi = PI/nphi;
    for ( double my_phi=dphi; my_phi<PI; my_phi+=dphi ) {
        int nmy_theta;
        nmy_theta = round(sin(my_phi)*ntheta);
        count += nmy_theta;
        for( int i=0; i<(64-nmy_theta)/2; i++) {
            printf("%c",' ');
        }
        for( int i=0; i<nmy_theta; i++) {
            printf("%c",'x');
        }
        for( int i=0; i<(64-nmy_theta)/2; i++) {
            printf("%c",' ');
        }
        printf("\n");
        //printf("my_phi = %g, nmy_theta %d\n", my_phi, nmy_theta);
    }

    printf("number of keywords %d\n", count);

    return 0;
%}
