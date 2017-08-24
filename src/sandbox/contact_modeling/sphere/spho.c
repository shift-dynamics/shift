#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

double dotv(double V0[3], double V1[3]) {
    return V0[0]*V1[0] + V0[1]*V1[1] + V0[2]*V1[2];
}

double normv(double V[3]){
    return (sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]));
}

double normd(double V1[3], double V2[3]){
    double V[3];
    V[0] = V2[0] - V1[0];
    V[1] = V2[1] - V1[1];
    V[2] = V2[2] - V1[2];
    return (normv(V));
}

void crossv(double V0[3], double V1[3], double V2[3]) {
    V2[0] = V0[1]*V1[2] - V0[2]*V1[1];
    V2[1] = V0[2]*V1[0] - V0[0]*V1[2];
    V2[2] = V0[0]*V1[1] - V0[1]*V1[0];
}

void unitv(double V0[3], double V1[3]) {
    double Vn = normv(V0);
    V1[0] = V0[0]/Vn;
    V1[1] = V0[1]/Vn;
    V1[2] = V0[2]/Vn;
}


int mod(i,j) {
    return (i+j)%j;
}

int main() {

    /*
    n: number of minor quad edges in one major quad (odd integer)
    n*n = number of minor quads in one major quad
    (n+1)*(n+1) = number of vertices in one major quad
    */
    int n;
    n = 5;
    /*
    m = (n-1)/2, m+1 = (n+1)/2, n = 2*m+1, n+1 = 2*(m+1)
    Divide vertices in major quad into four equal blocks 0, 1, 2, 3 in ccw order.
    Each block contains (m+1)*(m+1) vertices and m*m minor quads.
    Total vertices: 4*(m+1)*(m+1) = (n+1)*(n+1)
    Total minor quads: (2*m+1)*(2*m+1)=n*n
    Total number of minor quads and faces: 6*n*n
    Total number of vertices 6*n*n+2
    */
    int m;
    m = (n-1)/2;

    /*
    Allocate enough space to hold vertices of the six major quads for n up to
    49, which would be a maximum of 14406 faces.
    */
    double V0[50][50][3];
    double V1[50][50][3];
    double V2[50][50][3];
    double V3[50][50][3];
    double V4[50][50][3];
    double V5[50][50][3];

    /*
    Test a cube first. For similicity, vertex values range from -n to n in
    increments of 2. For n = 5, unique vertex values on and below the diagonal
    of block 0 in the top major quad stored in V0. For the unit sphere, just
    (m+1)*(m+2)/2 = 3*4/2 = 6 vertex coordinates must be computed as shown on
    and below the diagonal below. In terms of n: (n+1)*(n+3)/8 = 6*8/8 = 6
    Block 0 on and below the diagonal
                       ...               Col 3   Col 4   Col 5
    Row 5:             ...                ...     ...   [5 5 5]
    Row 4:             ...                ...   [3 3 5] [5 3 5]
    Row 3:             ...              [1 1 5] [3 1 5] [5 1 5]
    Full block 0 full
                       ...               Col 3   Col 4   Col 5
    Row 5:             ...              [1 5 5] [3 5 5] [5 5 5]
    Row 4:             ...              [1 3 5] [3 3 5] [5 3 5]
    Row 3:             ...              [1 1 5] [3 1 5] [5 1 5]
    Blocks 1 and 0 full
             Col 0    Col 1    Col 2  |  Col 3   Col 4   Col 5
    Row 5: [-5 5 5] [-3 5 5] [-1 5 5] | [1 5 5] [3 5 5] [5 5 5]
    Row 4: [-5 3 5] [-3 3 5] [-1 3 5] | [1 3 5] [3 3 5] [5 3 5]
    Row 3: [-5 1 5] [-3 1 5] [-1 1 5] | [1 1 5] [3 1 5] [5 1 5]
    Blocks 1 and 0 full
    Blocks 2 and 3 full
             Col 0    Col 1    Col 2  |  Col 3   Col 4   Col 5
    Row 5: [-5 5 5] [-3 5 5] [-1 5 5] | [1 5 5] [3 5 5] [5 5 5]
    Row 4: [-5 3 5] [-3 3 5] [-1 3 5] | [1 3 5] [3 3 5] [5 3 5]
    Row 3: [-5 1 5] [-3 1 5] [-1 1 5] | [1 1 5] [3 1 5] [5 1 5]
    –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    Row 2: [-5-1 5] [-3-1 5] [-1-1 5] | [1-1 5] [3-1 5] [5-1 5]
    Row 1: [-5-3 5] [-3-3 5] [-1-3 5] | [1-3 5] [3-3 5] [5-3 5]
    Row 0: [-5-5 5] [-3-5 5] [-1-5 5] | [1-5 5] [3-5 5] [5-5 5]
    */

    /*
    // For the time being just set the six vertex coordinate entries in V0.
    // */
    // V0[3][3][0] = 1;
    // V0[3][3][1] = 1;
    // V0[3][3][2] = 5;
    // V0[4][3][0] = 3;
    // V0[4][3][1] = 1;
    // V0[4][3][2] = 5;
    // V0[4][4][0] = 3;
    // V0[4][4][1] = 3;
    // V0[4][4][2] = 5;
    // V0[5][3][0] = 5;
    // V0[5][3][1] = 1;
    // V0[5][3][2] = 5;
    // V0[5][4][0] = 5;
    // V0[5][4][1] = 3;
    // V0[5][4][2] = 5;
    // V0[5][5][0] = 5;
    // V0[5][5][1] = 5;
    // V0[5][5][2] = 5;

    double pi = acos(-1.0);
    double phi[25];
    double dphi = pi/(2.0*n);
    printf("pi = %g, dphi = %g\n", pi, dphi);
    phi[0] = dphi/2.0;
    for (int i=0; i<m; i++) {
        phi[i+1] = phi[i] + dphi;
    }

    for (int i=0; i<m+1; i++) {
        for (int j=0; j<i+1; j++) {
            double v[3];
            v[0] = sin(phi[i])*cos(phi[j]);
            v[1] = cos(phi[i])*sin(phi[j]);
            v[2] = cos(phi[i])*cos(phi[j]);
            unitv(v, V0[i+m+1][j+m+1]);
        }
    }

    for (int i=m+2; i<n+1; i++) {
    	for (int j=m+1; j<i; j++) {
    		V0[j][i][0] = V0[i][j][1];
    		V0[j][i][1] = V0[i][j][0];
    		V0[j][i][2] = V0[i][j][2];
    	}
    }

    for (int i=0; i<m+1; i++) {
    	for (int j=m+1; j<n+1; j++) {
    		V0[i][j][0] = -V0[n-i][j][0];
    		V0[i][j][1] =  V0[n-i][j][1];
    		V0[i][j][2] =  V0[n-i][j][2];
    	}
    }

    for (int i=0; i<n+1; i++) {
    	for (int j=0; j<m+1; j++) {
    		V0[i][j][0] =  V0[i][n-j][0];
    		V0[i][j][1] = -V0[i][n-j][1];
    		V0[i][j][2] =  V0[i][n-j][2];
    	}
    }

    for (int i=0; i<n+1; i++) {
    	for (int j=0; j<n+1; j++) {
    		V3[i][j][0] =  V0[i][j][0];
    		V3[i][j][1] = -V0[i][j][2];
    		V3[i][j][2] =  V0[i][j][1];
    		V4[i][j][0] = -V3[i][j][1];
    		V4[i][j][1] =  V3[i][j][0];
    		V4[i][j][2] =  V3[i][j][2];
    		V1[i][j][0] = -V4[i][j][1];
    		V1[i][j][1] =  V4[i][j][0];
    		V1[i][j][2] =  V4[i][j][2];
    		V2[i][j][0] = -V1[i][j][1];
    		V2[i][j][1] =  V1[i][j][0];
    		V2[i][j][2] =  V1[i][j][2];
    		V5[i][j][0] =  V0[i][j][0];
    		V5[i][j][1] =  V0[i][j][1];
    		V5[i][j][2] = -V0[i][j][2];
    	}
    }


    double a[4];
    double u[4][3];
    for (int i=m; i<n; i++) {
        printf("\n");
        for (int j=m; j<i+1; j++) {
            crossv(V0[i][j], V0[i+1][j], u[0]);
            unitv(u[0], u[0]);
            crossv(V0[i+1][j], V0[i+1][j+1], u[1]);
            unitv(u[1], u[1]);
            crossv(V0[i+1][j+1], V0[i][j+1], u[2]);
            unitv(u[2], u[2]);
            crossv(V0[i][j+1], V0[i][j], u[3]);
            unitv(u[3], u[3]);
            a[0] = acos(-dotv(u[0], u[1]));
            a[1] = acos(-dotv(u[1], u[2]));
            a[2] = acos(-dotv(u[2], u[3]));
            a[3] = acos(-dotv(u[3], u[0]));
            printf("i = %d, j = %d ", i, j);
            printf("angles = [%g %g %g %g] ", a[0], a[1], a[2], a[3]);
            printf("area = %g\n", a[0]+a[1]+a[2]+a[3]-2*pi);
        }
    }

    // printf("V0\n");
    // for (int j=n; j>-1; j--) {
    //     for (int i=0; i<n+1; i++) {
    //         printf("[%g %g %g] ", V0[i][j][0], V0[i][j][1], V0[i][j][2]);
    //     }
    //     printf("\n");
    // }
    // printf("V1\n");
    // for (int j=n; j>-1; j--) {
    //     for (int i=0; i<n+1; i++) {
    //         printf("[%g %g %g] ", V1[i][j][0], V1[i][j][1], V1[i][j][2]);
    //     }
    //     printf("\n");
    // }
    // printf("V2\n");
    // for (int j=n; j>-1; j--) {
    //     for (int i=0; i<n+1; i++) {
    //         printf("[%g %g %g] ", V2[i][j][0], V2[i][j][1], V2[i][j][2]);
    //     }
    //     printf("\n");
    // }
    // printf("V3\n");
    // for (int j=n; j>-1; j--) {
    //     for (int i=0; i<n+1; i++) {
    //         printf("[%g %g %g] ", V3[i][j][0], V3[i][j][1], V3[i][j][2]);
    //     }
    //     printf("\n");
    // }
    // printf("V4\n");
    // for (int j=n; j>-1; j--) {
    //     for (int i=0; i<n+1; i++) {
    //         printf("[%g %g %g] ", V4[i][j][0], V4[i][j][1], V4[i][j][2]);
    //     }
    //     printf("\n");
    // }
    // printf("V5\n");
    // for (int j=n; j>-1; j--) {
    //     for (int i=0; i<n+1; i++) {
    //         printf("[%g %g %g] ", V5[i][j][0], V5[i][j][1], V5[i][j][2]);
    //     }
    //     printf("\n");
    // }

    FILE *fid;
    fid = fopen( "sphere5.ply", "w" );
    fprintf(fid, "ply\n");
    fprintf(fid, "format ascii 1.0\n");
    fprintf(fid, "comment VCGLIB generated\n");
    fprintf(fid, "element vertex %d\n", 6*n*n+2);
    fprintf(fid, "property float x\n");
    fprintf(fid, "property float y\n");
    fprintf(fid, "property float z\n");
    fprintf(fid, "element face %d\n", 6*n*n);
    fprintf(fid, "property list uchar int vertex_indices\n");
    fprintf(fid, "end_header\n");

    int nv = 0;
    // printf("\nStart of Vertices: Major quad V0\n");
    for (int k=1; k<m+2; k++) {
    	int i = m+k;
    	int j = m+k;
    	for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V0[i][j][0], V0[i][j][1], V0[i][j][2]);//copy(V0[i][j--]);
            fprintf(fid, "%g %g %g\n", V0[i][j][0], V0[i][j][1], V0[i][j][2]);
            i--;
        }
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V0[i][j][0], V0[i][j][1], V0[i][j][2]);//copy(V0[i--][j]);
            fprintf(fid, "%g %g %g\n", V0[i][j][0], V0[i][j][1], V0[i][j][2]);
            j--;
        }
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V0[i][j][0], V0[i][j][1], V0[i][j][2]);//copy(V0[i][j++]);
            fprintf(fid, "%g %g %g\n", V0[i][j][0], V0[i][j][1], V0[i][j][2]);
            i++;
        }
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V0[i][j][0], V0[i][j][1], V0[i][j][2]);//copy(V0[i++][j]);
            fprintf(fid, "%g %g %g\n", V0[i][j][0], V0[i][j][1], V0[i][j][2]);
            j++;
        }
    }

    // printf("\nMajor quads V0 to V4\n");

    for (int j=n-1; j>=0; j--) {
        for (int i=0; i<n; i++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V1[i][j][0], V1[i][j][1], V1[i][j][2]);//copy(V1[i][j]);
            fprintf(fid, "%g %g %g\n", V1[i][j][0], V1[i][j][1], V1[i][j][2]);
        }
        for (int i=0; i<n; i++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V2[i][j][0], V2[i][j][1], V2[i][j][2]);//copy(V2[i][j]);
            fprintf(fid, "%g %g %g\n", V2[i][j][0], V2[i][j][1], V2[i][j][2]);
        }
        for (int i=0; i<n; i++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V3[i][j][0], V3[i][j][1], V3[i][j][2]);//copy(V3[i][j]);
            fprintf(fid, "%g %g %g\n", V3[i][j][0], V3[i][j][1], V3[i][j][2]);
        }
        for (int i=0; i<n; i++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V4[i][j][0], V4[i][j][1], V4[i][j][2]);//copy(V4[i][j]);
            fprintf(fid, "%g %g %g\n", V4[i][j][0], V4[i][j][1], V4[i][j][2]);
        }
    }

//    printf("\nStart of Vertices: Major quad V5\n");
    for (int k=m; k>0; k--) {
        int i = m+k;
        int j = m+k;
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V5[i][j][0], V5[i][j][1], V5[i][j][2]);//copy(V5[i][j--]);
            fprintf(fid, "%g %g %g\n", V5[i][j][0], V5[i][j][1], V5[i][j][2]);
            i--;
        }
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V5[i][j][0], V5[i][j][1], V5[i][j][2]);//copy(V5[i--][j]);
            fprintf(fid, "%g %g %g\n", V5[i][j][0], V5[i][j][1], V5[i][j][2]);
            j--;
        }
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V5[i][j][0], V5[i][j][1], V5[i][j][2]);//copy(V5[i][j++]);
            fprintf(fid, "%g %g %g\n", V5[i][j][0], V5[i][j][1], V5[i][j][2]);
            i++;
        }
        for (int l=1; l<2*k; l++) {
            // printf("v[%d]: [%g %g %g]\n", nv++,
            // V5[i][j][0], V5[i][j][1], V5[i][j][2]);//copy(V5[i++][j]);
            fprintf(fid, "%g %g %g\n", V5[i][j][0], V5[i][j][1], V5[i][j][2]);
            j++;
        }
    }

    /*
    Each major quad is a (2m+2) by (2m+2) array of verties. Thus V0 contains m+1
    rings of vertices. Vertices in V0 were copied ccw from the center of the
    array one ring at a time moving outward. Let 0<k<m+1 represent the inner
    ring number.
    k: current inner ring number
    k+1: current outer ring
    8*k: Number of faces in kth quad strip between inner and outer rings
    4*(2*k-1): Number of vertices in inner ring
    4*(2*k+1): Number of vertices in outer ring
    4*(k-1)*(k-1): First vertex index in inner ring
    4*k*k: First vertex index in outer ring.
    */

    // printf("\nFaces on major quad 0\n");
    int nf=0;
    // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf, 0, 1, 2, 3);
    fprintf(fid, "4 %d %d %d %d\n", 0, 1, 2, 3);
    nf++;
    for (int k=1; k<m+1; k++) {
        int vi = 4*(k-1)*(k-1);
        int vo = 4*k*k;
        int ni = 4*(2*k-1);
        int no = 4*(2*k+1);
        int li = 0;
        int lo = 0;
        for (int i=0; i<4; i++) {
            // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf,
            //     vi+mod(li,  ni),
            //     vo+mod(lo-1,no),
            //     vo+mod(lo,  no),
            //     vo+mod(lo+1,no));
            fprintf(fid, "4 %d %d %d %d\n",
                vi+mod(li,  ni),
                vo+mod(lo-1,no),
                vo+mod(lo,  no),
                vo+mod(lo+1,no));
            lo+=2;
            nf++;
            for (int j=0; j<2*k-1; j++) {
                // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf,
                //     vi+mod(li,  ni),
                //     vo+mod(lo-1,no),
                //     vo+mod(lo,  no),
                //     vi+mod(li+1,ni));
                    fprintf(fid, "4 %d %d %d %d\n",
                    vi+mod(li,  ni),
                    vo+mod(lo-1,no),
                    vo+mod(lo,  no),
                    vi+mod(li+1,ni));
                li++;
                lo++;
                nf++;
            }
        }
    }

    int ni = 4*n;
    int no = 4*n;
    int vi = (n-1)*(n-1);
    int vo = vi + ni;
    // printf("\nFaces on major quads 1-4\n");
    for (int i=0; i<n; i++) {
        int li = 0;
        int lo = 0;
        for (int j=0; j<ni; j++) {
            // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf,
            //     vi+mod(li,  ni),
            //     vo+mod(lo,  no),
            //     vo+mod(lo+1,no),
            //     vi+mod(li+1,ni));
                fprintf(fid, "4 %d %d %d %d\n",
                vi+mod(li,  ni),
                vo+mod(lo,  no),
                vo+mod(lo+1,no),
                vi+mod(li+1,ni));
                li++;
                lo++;
                nf++;
        }
        vi += ni;
        vo += no;
    }

    /*
    The verties on the surface of the bottom major quad are generated in
    counterclockwise rings around the positive z-axis of the sphere just as
    they were for the other five major quads. Thus when viewed from the outside
    bottom of the sphere (the negative z-axis) these rings appear clockwise.
    To be consistent with the face order in all previous minor quad face strips,
    the faces should continue to be generated in counterclocksise order when
    viewed from the positive z-axis, but the vertex indices in each face
    definition must be assigned in clockwise order so the normals correctly
    point outward. This simply means that the rules for assigning the four
    vertex indices are slightly different. The inner and outer vertex indices
    are still incremented when moving from one face definition to the next,
    they are assigned to the faces differently.

    */

    // printf("\nFaces on major quad 5\n");

    vo = (n+1)*(n+1)+4*(n-1)*n; /*Starting index on outer ring of vertices*/
    vi = vo + 4*n; /*Starting index on inner ring of vertices*/
    no = 4*(2*m+1); /*Number of indices in outer ring of vertices*/
    ni = 4*(2*m-1); /*Number of indices in inner ring of vertcies*/
    for (int k=m; k>0; k--) { /*Current minor quad face strip working out to in*/
        // printf("k = %d, vo = %d, vi = %d, no = %d, ni = %d\n", k, vo, vi, no, ni);
        int li = 0; /*Local vertex index in the inner ring*/
        int lo = 0; /*Local vertex index in the outer ring*/
        for (int i=0; i<4; i++) { /*Side number of square minor quad face strip*/
            // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf,
            //     vi+mod(li,  ni),
            //     vo+mod(lo+1,no),
            //     vo+mod(lo,  no),
            //     vo+mod(lo-1,no));
            /*Special corner face with three vertices from outer vertex ring*/
            fprintf(fid, "4 %d %d %d %d\n",
                vi+mod(li,  ni),
                vo+mod(lo+1,no),
                vo+mod(lo,  no),
                vo+mod(lo-1,no));
            lo+=2; /*Finished with lo and lo-1 vertices so advance lo by 2*/
            nf++; /*Record new face*/
            for (int j=0; j<2*k-1; j++) { /*Regular faces on side of quad strip*/
                // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf,
                //     vi+mod(li,  ni),
                //     vo+mod(lo+1,no),
                //     vo+mod(lo,  no),
                //     vi+mod(li-1,ni));
                /*lo is now one vertex ahead of li, so the regular faces
                require two vertex indices from the inner ring in forward order,
                followed by two faces from the outer ring in reverse order*/
                fprintf(fid, "4 %d %d %d %d\n",
                    vi+mod(li,  ni),
                    vi+mod(li+1,ni),
                    vo+mod(lo,  no),
                    vo+mod(lo-1,no));
                li++; /*Move forward in inner ring for next face*/
                lo++; /*Move forward in outer ring for next face*/
                nf++; /*Record new face*/
            }
        }
        /*Finished generating current quad strip*/
        vo = vi; /*Starting index on next outer ring of vertices*/
        no = ni; /*Number of indices in next outer ring of vertices*/
        vi += ni; /*Starting index on next inner ring of vertices*/
        ni = 4*(2*k-3); /*Number of indices in next inner ring of vertcies*/
    }
    vi = 6*n*n-2; /*Starting index of last ring of vertices*/
    // printf("f[%3d]: [%3d %3d %3d %3d]\n", nf, vi+3, vi+2, vi+1, vi);
    fprintf(fid, "4 %d %d %d %d\n", vi+2, vi+1, vi, vi+3);
    nf++;

    fclose(fid);
    return 0;
}

    /*
    This program generates a quadrangulated unit sphere by slicing its surface
    with three sets of unequally spaced great circles whose poles are centered
    on the respective Cartesian x, y, and z axes. The quads are generated by
    spiraling around the z-axis in counter-clockwise order, working from the
    most positive z slice to the most negative z slice. The quads on the unit
    sphere are generated by mapping quads on the six surfaces of an inscribed
    cube onto the sphere's surface.

    So that the face of the central quad in each of the six cube faces is
    centered on an axis, let n (odd) represent the number of quad edges along
    each edge of the cube. Thus if n=5 there will be n*n=25 quad faces on each
    cube face or 6*n*n=150 quad faces total. The total number of vertices is
    6*n*n+2=152 in this example. Thus the number of faces is nF=6*n*n, the
    number of vertices is nV=nF+2, and the number of edges is nE=2*nF. Since the
    inscribed cube is mapped onto the unit sphere, these formulas for the number
    of faces, vertices and edges on the cube will be the same for the sphere.

    Since the number of quad faces is just two less than the number of vertices,
    we will use a novel method of simultaneously generating quad faces as the
    vertices are generated. The example of n=3 with 9 quad faces per cube face
    will be used to illustrate the method. If the cube is inscribed in the unit
    sphere, the length of its edges will be 2*sqrt(3)/3, and the length of each
    quad edge will be 2*sqrt(3)/(3*n). The surface area of this inscribed cube
    is 8 as compared to the surface area of the unit sphere at 4*pi=12.57....

    Start with the cube face centered on the positive z-axis, which will contain
    9 faces and 16 vertices for this example. Set lce=2*sqrt(3)/3 as the length
    of each cube edge and lqe=lce/n as the length of each quad edge. To simplify
    the following notation let dx=[lqe,0,0], dy=[0,lqe,0] and dz=[0,0,lqe]
    represent three coordinate vectors indicating incremental changes in vertex
    locations. The first four vertices for the first quad, centered on the
    positive z-axis, are v0=[lqe/2,lqe/2,lce/2], v1=[-lqe/2,lqe/2,lce/2],
    v2=[-lqe/2,-lqe/2,lce/2], and v3=[lqe/2,-lqe/2,lce/2]. These vertices,
    generated in counter-clockwise order and numbered 0 to 3, form the first
    face f0=[0,1,2,3]. The convention used throughout this development will be
    to start each new ring of vertices at the positive x and y corner.

    To automate the process of generating all vertices, an incremental method
    will be used as follows. Set v0 as above. Then generate v1 from v0 as
    v1=v0-dx. Then generate v2 from v1 as v2-v1-dy, and finally generate v3 from
    v2 by setting v3=v2+dx. These four vertices form a ring of vertices, which
    will initially be designated as the outer ring when generated, but then
    becomes the inner ring when the next ring of vertices is being generated.

    The next 12 vertices, v4-v15, in the outer ring of vertices surrounding the
    inner ring of vertices v0-v3, and the next 8 faces, f1-f8, forming a closed
    strip of faces between the inner and outer ring of vertices, are generated
    in counter-clockwise order around the z-axis on the face. The vertices
    in the outer ring will be generated, starting in the positive x and y
    corner. The new starting vertex in the outer ring of vertices will be
    diagonally adjacent to the previous starting vertex v0 in this example and
    obtained as v4=v0+dx+dy. It follows that v5=v4-dx, v6=v5-dx, v7=v6-dx,
    v8=v7-dy, v9=v8-dy, v10=v9-dy, v11=v10+dx, v12=v11+dx, v13=v12+dx,
    v14=v13+dy, and v15=v14+dy. v15 completes the outer loop of vertices and is
    right next to the first vertex, v4, in the outer ring.

    The eight faces f1-f8 are generated as a continuous, closed strip between
    the inner ring of vertices v0-v3 and outer ring of vertices v4-v15. The
    first rule for generating this set of faces is to assign vertex indices from
    v0-v3 in the inner ring in non-decreasing order as the first vertex index in
    each face definition. Then locate the first available vertex in the outer
    ring that is closest to the assigned vertex from the inner ring and make its
    index the second face entry. Then make the next vertex index in the outer
    ring the third face entry. The fourth and last vertex index in the face may
    come from the outer or inner ring, depending on which of the next available
    vertices is closer to the first vertex from the inner ring. If the last
    vertex index assigned to the current face is from the outer ring, the first
    vertex index of the next face will be the same as the first vertex index of
    the previous face. But if the last vertex index is from the inner ring, the
    first vertex index of the next face will be the same as the last vertex
    index of the previous face. The only other requirement is that the modulo
    operator must be applied to the vertex indices in the inner and outer rings
    because the quad strip forms a ring, which means that some quad faces may
    use vertex indices from both ends of the inner or outer vertex rings.

    Using the above rules we first set f1=[0,15,4,5], starting with vertex
    index 0 from the inner ring. The first available closest vertex to v0 in the
    outer ring is v15, so 15 is set to the second index in f1. Then v4 and v5 are
    determined to be the next available, closest vertices to v0 in the outer
    ring, so their indices 4 and 5 are used to complete face f1.

    The above rules indicate that the first index of face f2=[0,5,6,1] will be 0.
    While vertices v15 and v4 are also closest to v0, they are not available for
    f2, having been consumed by face f1. However, v5 is available to f2, even
    though it is included in f1, because faces f1 and f2 share a common edge
    between v0 and v5. Thus 5 becomes the second index in f2 and by the above
    rules 6 becomes the third index because v6 is the next vertex from the outer
    ring, even though vertex v1 from the inner ring is closer. Finally index 1
    from v1 becomes the fourth entry in face f2.

    The above rules then state that the first entry in f3=[1,6,7,8] will be 1
    because v1, in the inner ring, was the last entry in f2. Then we find that
    v6, v7, and v8 are the next available vertices in the outer ring, so their
    indices finish off f3. And because vertex v8 in f3 is not in the inner ring
    we start f4=[1,8,9,2] again with vertex v1 and find that v8 and v9 are
    the two available vertices from the outer ring. We then finish up with vertex
    v2 to finish face f4. Faces f5=[2,9,10,11] and f6=[2,11,12,3], and then
    f7=[3,12,13,14] and f8=[3,14,15,0] are evaluated in a similar manner. However,
    we see in the last face f8 the last vertex v0 wrapping around to the
    beginning of the inner ring. The mod function is used to simplify this
    wrapping process by allowing the algorithm to effectively index outside the
    legal range of vertex indices without having to check and make adjustments.

    This process of generating another ring of vertices followed by another ring
    of faces will continue until the first face has been completed. A total of
    (n+1)*(n+1) vertices and n*n faces will have been generated at this point.

    We continue the process of generating rings of vertices and faces around the
    completed first face of the cube, but this will be accomplished, one ring
    of vertices and one row of faces at a time across all four cube faces common
    to the four edges of the completed first cube face. The algorithms to
    generate the next 4*n*n vertices and the next 4*n*n faces common to these
    four cube faces are somewhat different than the previous ones.

    We start from the first vertex vs in the last ring and move to the first
    vertex of the first ring of vertices running parallel to the four edges of
    the face just completed. Thus vi = vs-dz. Then we generate 4*n vertices as
    n x vi<-vi-dx, n x vi<-vi-dy, n x vi<-vi+dx, n x vi<-vi+dy, where n x means
    repeat n times and vi<- means assign the quantity on the right to the next
    available vertex. The last vertex in this sequence is ignored because it
    coincides with the first vertex in the ring. This completes the first outer
    ring of vertices parallel to the last ring of vertices generated on the
    first cube face.

    In this case the inner and outer rings each have the same number of vertices
    which is 4*n, so 4*n faces in a closed quad strip will be generated between
    these two rings. The first vertex index in the first face in this ring of
    faces is set to the lowest vertex index in the inner ring. Then the closest
    avialable vertex in the outer ring is located and its index, followed by the
    next vertex index are set as the second and third face entries. Finally the
    next vertex index in the inner ring is set to the fourth face entry. This
    fourth face entry becomes the first vertex entry in the next face. The above
    process repeats for all faces around the face ring, again applying the mod
    function to account for closing the face ring.

    Then the above processes are repeated for each successive outer ring of
    vertices and another row of faces in the next ring. When the last ring of
    vertices has been generated and the last ring of faces, five of the six
    faces of the cube will have been completed. Only the sixth cube face opposite
    the first face remains to be generated, and already the vertices on its
    four outer edges have been generated from the last step. Thus only (n-1)*(n-1)
    vertices and n*n faces must be generated to complete the surface.

    To complete the final face of the cube we will be generating inner rings of
    vertices relative to outer rings, just the opposite of all previous
    processes. In essence we generate another inner ring of vertices until
    reaching the inner-most ring of vertices, which completes the last cube face.
    The last cube face will be centered on the negative z-axis, so all vertex z
    coordinates are -lce/2. The starting vertex of the outer ring is at
    vs=[lce/2, lce/2, -lce/2], so the starting vertex index of the next inner ring
    will be at vs<-vs-dx-dy. Then using the same process as for the first face
    centered on the positive z-axis, the vertices in this inner ring are generated.
    Generating the quad faces on this bottom face of the cube is a bit different
    then for the top face, because the vertex order must be reversed to keep the
    face normals pointing outward. In this situation an inner-ring vertex index
    is always in the first position in each face, and an inner-ring vertex index
    may also appear in the second position. And outer ring-indices may appear in
    the last two or last three positions in a face. While the inner-ring vertex
    indices will appear in increasing order in each face, the outer-ring indices
    will appear in decreasing order. The other rules for checking vertices are
    the same, except that faces are generated from left to right as opposed to
    from right to left in the previous cases.
    */

//   // FILE *fp;
//   // long lSize;
//   // char *buffer;
//   //
//   // fp=fopen("octant.txt", "r");
//   // if( !fp ) {
//   //   printf("ERROR: octant.txt not found.");
//   //   exit(1);
//   // }
//   //
//   // fclose(fp);
//
//     /*
//     Produce a unit sphere with uniformly spaced vertices generated one latitude at
//     a time. The vertices in each latitute start at the prime meridian. The normal
//     vectors in a unit sphere are the same as the vertex vectors, so there is no
//     need to generate a duplicate set of normal vectors.
//
//     Allow the user to specify the desired number of vertex and normal vectors in the
//     unit sphere or to specify the desired average angle between adjacent normal
//     vectors in each latitude and between latitudes.
//
//     byAngle = 1 means specify the desired angle in degrees.
//     byAngle = 0 means specify the desired number of vertex and normal vectors.
//     */
//
//     int byAngle = 0;
//     double ad;
//     double pi = acos(-1.0);
//     double ar;
//     double pa;
//     int N;
//     if (byAngle == 1){
//         ad = 10.0;           //desired angle between vertices in degrees
//         ar = ad*pi/180.0;    //desired angle between vertices in radians
//         pa = ar*ar;          //desired patch area in steradians
//         N = round(4*pi/pa);  //desired number of vertices
//     }
//     else{
//         N = 10000;           //desired number of vertices
//         pa = 4.0*pi/N;       //desired patch area in steradians
//         ar = sqrt(pa);       //desired angle between vertices in radians
//         ad = 180.0*ar/pi;    //desired angle between vertices in degrees
//     }
//
//     printf("Number of vertices: %d\n", N);
//     printf("Patch area (steradians): = %g\n", pa);
//     printf("Desired angle between normal vectors (radians): %g\n", ar);
//     printf("Desired angle between normal vectors (degrees): %g\n", ad);
//
//     /*
//     Vertices are represented as
//     x = cos(theta)cos(phi), y = sin(theta)cos(phi), z = sin(phi)
//     phi represents the latitude angle measured from the equator. Positive phi is
//     toward the north pole, and negative phi is toward the south pole.
//     theta represents the azimuth angle measured eastward from the prime meridian.
//     */
//
//     //Number of gaps between latitudes from equator to either pole.
//     int Nphi;
//     Nphi = round(pi/(2*ar));
//     //printf("Number of gaps between latitudes from equator to either pole: %d\n", Nphi);
//
//     //Actual angle between latitudes and approximately the desired angle ar.
//     double dphi;
//     dphi = pi/(2*Nphi);
//     //printf("Actual angle between latitudes: %g\n", dphi);
//
//     double V[50000][3]; //Store the x, y, and z coordinates of all vertices.
//     int NV = 0; //Record the number of vertices generated so far.
//     int NLV[2000]; //Store the number of total vertices at the end of each latitude.
//     int NL = 0;
//     NLV[NL] = 0;
//     NL++;
//
//     /*
//     Generate the sphere starting at the south pole and working upward one latitude
//     at a time. At the south pole phi = -pi/2 and theta = 0. Thus
//     */
//     V[NV][0] = 0;
//     V[NV][1] = 0;
//     V[NV][2] = -1;
//     NV++;
//     NLV[NL] = NV;
//     NL++;
//
//     /*
//     Actual angle between normal vectors on equator to obtain desired patch area.
//     This angle will be used to specify angles between vertices on each latitude.
//     */
//     double dtheta;
//     dtheta = pa/dphi;
//     //printf("Angle between normal vectors on equator: %g\n", dtheta);
//
//     /*
//     Generate one latitude at a time, working upward from the south pole to, but not
//     including, the equator. A total of Nphi-1 latitudes will be generated.
//     */
//
//     for ( int m=1; m<Nphi; m++ ){
//         double phi;
//         phi = m*dphi - pi/2; //Latitudes upward from south pole by dphi.
//         //printf("Southern hemisphere current latitude %d angle (radians): %g\n", m, phi);
//
//         /*
//         The circumfrence of each latitude is different and depends on phi. It is
//         given by 2*pi*cos(phi). Divide the circle into eaual arc increments of
//         length dtheta. The estimated number of these arc increments is obtained
//         using the round() function.
//         */
//         int Ntheta;
//         Ntheta = round(2*pi*cos(phi)/dtheta);
//         //printf("Southern hemisphere number of vertices and normal vectors: %d\n", Ntheta);
//
//         /*
//         Because of the rounding to obtain integer Ntheta, Ntheta*dtheta may be
//         slightly too small or too large to represent the full circle. Thus compute
//         more accurate values for theta as follows. Start n at 0 because first
//         vertex always falls on the prime meridian. End n at Ntheta-1 because next
//         vertex would be at 2*pi and repeat on the prime meridian.
//         */
//         for ( int n = 0; n < Ntheta; n++ ){
//             double theta = 2*pi*n/Ntheta;
//             V[NV][0] = cos(theta)*cos(phi);
//             V[NV][1] = sin(theta)*cos(phi);
//             V[NV][2] = sin(phi);
//             NV++;
//         }
//         NLV[NL] = NV;
//         NL++;
//     }
//
//     /*
//     The souther hemisphere is finished. Now do the equator. In this case we have
//     phi = 0 and x = cos(theta), y = sin(theta), z = 0
//     */
//     int Ntheta;
//     Ntheta = round(2*pi/dtheta);
//     //printf("Equator number of vertices and normal vectors: %d\n", Ntheta);
//     for ( int n = 0; n < Ntheta; n++ ){
//         double theta = 2*pi*n/Ntheta;
//         V[NV][0] = cos(theta);
//         V[NV][1] = sin(theta);
//         V[NV][2] = 0.0;
//         NV++;
//     }
//     NLV[NL] = NV;
//     NL++;
//
//     /*
//     The equator is finished so only the northern hemisphere remains. In this case
//     we work upward from just above the equator to just short of the north pole.
//     */
//     for ( int m=Nphi-1; m>0; m-- ){
//         double phi;
//         phi = pi/2 - m*dphi; //Latitudes upward from the equator by dphi.
//         //printf("Northern hemisphere current latitude %d angle (radians): %g\n", m, phi);
//
//         /*
//         The circumfrence of each latitude is different and depends on phi. It is
//         given by 2*pi*cos(phi). Divide the circle into eaual arc increments of
//         length dtheta. The estimated number of these arc increments is obtained
//         using the round() function.
//         */
//         int Ntheta;
//         Ntheta = round(2*pi*cos(phi)/dtheta);
//         //printf("Northern hemisphere number of vertices and normal vectors: %d\n", Ntheta);
//
//         /*
//         Because of the rounding to obtain integer Ntheta, Ntheta*dtheta may be
//         slightly too small or too large to represent the full circle. Thus compute
//         more accurate values for theta as follows. Start n at 0 because first
//         vertex always falls on the prime meridian. End n at Ntheta-1 because next
//         vertex would be at 2*pi and repeat on the prime meridian.
//         */
//         for ( int n = 0; n < Ntheta; n++ ){
//             double theta = 2*pi*n/Ntheta;
//             V[NV][0] = cos(theta)*cos(phi);
//             V[NV][1] = sin(theta)*cos(phi);
//             V[NV][2] = sin(phi);
//             NV++;
//         }
//         NLV[NL] = NV;
//         NL++;
//     }
//
//     //Last set the north pole vetex
//     V[NV][0] = 0;
//     V[NV][1] = 0;
//     V[NV][2] = 1;
//     NV++;
//     NLV[NL] = NV;
//     NL++;
//
//     // for ( int i=0; i<NL; i++ ){
//     //     printf("NVL[%d]= %d\n", i, NLV[i]);
//     // }
//     //
//     // for ( int i=0; i<NV; i++ ){
//     //     printf("V[%d]= [%g, %g, %g]\n", i, V[i][0], V[i][1], V[i][2]);
//     // }
//
//     // printf("Number of latitudes including poles: %d\n", NL-1);
//     // printf("Number of vertices: %d\n", NV);
//
//     /*
//     Now that we have all the vertices, we want to generate all the faces starting from
//     the south pole and working up to the north pole. The array NLV contains the
//     cumulative count of vertices at the end of each latitude circle. Thus at the ith
//     latitude starting from the south pole, it will have vertices in the range of
//     (NLV(i)+1):NLV(i+1). So when generating faces between latitudes i and i+1 we
//     use vertices (NLV(i)+1):NLV(i+1) in latitude i and vertices (NLV(i+1)+1):NLV(i+2)
//     in latitude i+1. dphi is approximately the smallest distance between two vertices
//     that could form edges of faces and sqrt(2)*dphi is approximately the largest
//     distance between two vertices that could form edges of faces. Thus it would
//     be safe to say that if the distance between two vertices between latitudes is
//     less than about sqrt(2)*dphi + eps,the edge between them would be the side of two
//     adjacent faces. Thus the number of faces formed from these valid edges and the
//     edges in the two latitudes, which are all valid, would be equal to the number of
//     valid edges between the two latitudes. With that the case, one additional face
//     can be generated as each new edge is identified.
//
//     It would be good to generate the faces in an orderly manner, working from the
//     meridian eastward until ariving back at the meridian. The vertices are also
//     arranged in this order within each latitude, so this should make it easier to
//     obtain the faces in the desired order.
//     */
//
//     int face[50000][3];
//     int Nface = 0;
//     double d0 = sqrt(2)*dphi;
//
//     //printf("Maximum angle between adjacent normal vectors: %g\n", d0);
//
//     for ( int i=1; i<NL-1; i++ ){
//         int F[2000][2];
//         int NF;
//         NF = 0;
//         int ms;
//         ms = NLV[i];
//         int me;
//         me = NLV[i+1]-1;
//         int n;
//         n = NLV[i-1];
//
//         //printf("ms: %d, me: %d, n: %d\n", ms, me, n);
//
//         for ( int m=ms; m<=me; m++ ){
//             while ( normd(V[m], V[n]) < d0+.001 ){
//                 F[NF][0] = m;
//                 F[NF][1] = n;
//                 //printf("F[%d] = [%d, %d]\n", NF, F[NF][0], F[NF][1]);
//                 NF++;
//                 if ( n < NLV[i]-1 ){
//                     if ( normd(V[m], V[n+1]) < d0+.001 ){
//                         n = n + 1;
//                     }
//                     else{
//                         break;
//                     }
//                 }
//                 else{
//                     break;
//                 }
//             }
//         }
//         if ( i>1 && i<NL-2 ){
//             F[NF][0] = F[NF-1][0];
//             F[NF][1] = F[0][1];
//             NF++;
//             //printf("F[%d] = [%d, %d]\n", NF, F[NF][0], F[NF][1]);
//         }
//         for ( int j=0; j<NF-1; j++ ){
//             if ( F[j][1] == F[j+1][1] ){
//                 face[Nface][0] = F[j][0];
//                 face[Nface][1] = F[j][1];
//                 face[Nface][2] = F[j+1][0];
//                 Nface++;
//             }
//             else{
//                 face[Nface][0] = F[j][0];
//                 face[Nface][1] = F[j][1];
//                 face[Nface][2] = F[j+1][1];
//                 Nface++;
//             }
//         }
//         int j = NF-1;
//         if ( i>1 && i<NL-2 ){
//             face[Nface][0] = F[j][0];
//             face[Nface][1] = F[0][1];
//             face[Nface][2] = F[0][0];
//             Nface++;
//         }
//         else if ( i==1 ){
//             face[Nface][0] = F[j][0];
//             face[Nface][1] = F[j][1];
//             face[Nface][2] = F[0][0];
//             Nface++;
//         }
//         else{
//             face[Nface][0] = F[j][0];
//             face[Nface][1] = F[j][1];
//             face[Nface][2] = F[0][1];
//             Nface++;
//         }
//     }
//
//     // for ( int i=0; i<Nface; i++ ){
//     //     if ( face[i][0] == face[i][1] | face[i][1] == face[i][2] | face[i][2] == face[i][0]){
//     //         printf("face %d has duplicate vertices: [%d, %d, %d]\n", i, face[i][0], face[i][1], face[i][2]);
//     //     }
//     // }
//     // for ( int i=0; i<Nface; i++ ){
//     //     if ( face[i][0] > NV-1 | face[i][1] > NV-1 | face[i][2] > NV-1)
//     //     printf("face %d has vertex index out of bounds: [%d, %d, %d]\n", i, face[i][0], face[i][1], face[i][2]);
//     // }
//     //
//     // for ( int i=0; i<Nface; i++){
//     //     printf("face %d: [%d, %d, %d]\n", i, face[i][0], face[i][1], face[i][2]);
//     // }
//
//     FILE *fid;
//     fid = fopen( "test.ply", "w" );
//     fprintf(fid, "ply\n");
//     fprintf(fid, "format ascii 1.0\n");
//     fprintf(fid, "comment VCGLIB generated\n");
//     fprintf(fid, "element vertex %d\n", NV);
//     fprintf(fid, "property float x\n");
//     fprintf(fid, "property float y\n");
//     fprintf(fid, "property float z\n");
//     fprintf(fid, "element face %d\n", Nface);
//     fprintf(fid, "property list uchar int vertex_indices\n");
//     fprintf(fid, "end_header\n");
//
//     for ( int i=0; i<NV; i++ ){
//         fprintf(fid, "%g %g %g\n", V[i][0], V[i][1], V[i][2]);
//     }
//
//     for ( int i=0; i<Nface; i++ ){
//         fprintf(fid, "3 %d %d %d\n", face[i][0], face[i][1], face[i][2]);
//     }
//
//     fclose (fid);
//
//     long lSize;
//     char *buffer;
//
//     fid=fopen("block.ply", "r");
//     if( !fid ) {
//       printf("ERROR: block.ply not found.");
//       exit(1);
//     }
//
//     fseek( fid, 0L, SEEK_END );
//     lSize = ftell( fid );
//     rewind( fid );
//
//     buffer = calloc( 1, lSize+1 );
//     if( !buffer ) {
//       fclose(fid);
//       printf("ERROR: Memory allocation failed.");
//       exit(1);
//     }
//
//     if( 1!=fread( buffer, lSize, 1, fid) ) {
//       fclose(fid);
//       free(buffer);
//       printf("ERROR: Failed to read entire file.");
//       exit(1);
//     }
//
//     char dstr[100];
//     int numV; //Number of vectors read in.
//     int numT; //Number of triangles read in.
//     int i = 0;
//     int hasNormV = 0;
//     char * line = strtok(strdup(buffer), "\n");
//     while( line ) {
//       if( strstr(line, "nx") ){
//           hasNormV = 1;
//           printf("This file has normal vectors: %s\n", line);
//       }
//       i++;
//       if( strstr(line, "element vertex") ) {
//         //printf("%s\n", line);
//         sscanf(line, "%s %s %d", dstr, dstr, &numV);
//         printf("Number of Vertices: %d\n", numV);
//       }
//       if( strstr(line, "element face") ) {
//         //printf("%s\n", line);
//         sscanf(line, "%s %s %d", dstr, dstr, &numT);
//         printf("Number of Triangles: %d\n", numT);
//       }
//       if( strstr(line, "end_header") ) {
//         line = strtok(NULL, "\n");
//         break;
//       }
//       line = strtok(NULL, "\n");
//     }
//
//     double Verts[30000][3]; //Array holding vertices.
//     double Norms[30000][3]; //Array holding normal vectors.
//     int Faces[60000][3]; //Array holding face vertex indices.
//
//     //i = 0;
//     //line = strtok(strdup(buffer), "\n");
//     while( line ) {
//       //i++;
//       //if( i == 14) {
//         int junk;
//         if ( hasNormV ) {
//             for ( int j=0; j<numV; j++ ) {
//               sscanf(line, "%lf %lf %lf %lf %lf %lf",
//                 &Verts[j][0], &Verts[j][1], &Verts[j][2],
//                 &Norms[j][0], &Norms[j][1], &Norms[j][2]);
//               line = strtok(NULL, "\n");
//             }
//         }
//         else  {
//             for ( int j=0; j<numV; j++ ) {
//               sscanf(line, "%lf %lf %lf", &Verts[j][0], &Verts[j][1], &Verts[j][2]);
//               line = strtok(NULL, "\n");
//             }
//         }
//         for ( int j=0; j<numT; j++ ) {
//           sscanf(line, "%d %d %d %d", &junk, &Faces[j][0], &Faces[j][1], &Faces[j][2]);
//           line = strtok(NULL, "\n");
//         }
//       break;
//     }
//
//     // if ( hasNormV ) {
//     //     for ( int j=0; j<numV; j++ ) {
//     //       printf("%g %g %g %g %g %g\n",
//     //         Verts[j][0], Verts[j][1], Verts[j][2], Norms[j][0], Norms[j][1], Norms[j][2]);
//     //     }
//     // }
//     // else  {
//     //     for ( int j=0; j<numV; j++ ) {
//     //         printf("%g %g %g\n", Verts[j][0], Verts[j][1], Verts[j][2]);
//     //     }
//     // }
//     // for ( int j=0; j<numT; j++ ) {
//     //   printf("%d %d %d\n", Faces[j][0], Faces[j][1], Faces[j][2]);
//     //   line = strtok(NULL, "\n");
//     // }
//
//     free(buffer);
//     fclose(fid);
//
//     fid = fopen( "block2.ply", "w" );
//     fprintf(fid, "ply\n");
//     fprintf(fid, "format ascii 1.0\n");
//     fprintf(fid, "comment VCGLIB generated\n");
//     fprintf(fid, "element vertex %d\n", NV);
//     fprintf(fid, "property float x\n");
//     fprintf(fid, "property float y\n");
//     fprintf(fid, "property float z\n");
//     fprintf(fid, "property float nx\n");
//     fprintf(fid, "property float ny\n");
//     fprintf(fid, "property float nz\n");
//     fprintf(fid, "element face %d\n", Nface);
//     fprintf(fid, "property list uchar int vertex_indices\n");
//     fprintf(fid, "end_header\n");
//
//     double nV[50000][3];
//
//     for ( int i=0; i<numV; i++ ){
//         double normV;
//         normV = normv(Norms[i]);
//         Norms[i][0]/=normV;
//         Norms[i][1]/=normV;
//         Norms[i][2]/=normV;
//     }
//
//     for ( int i=0; i<NV; i++ ) {
//         double minA;
//         minA = 1.0;
//         int k;
//         for ( int j=0; j<numV; j++ ) {
//             double align;
//             align = V[i][0]*Norms[j][0]+V[i][1]*Norms[j][1]+V[i][2]*Norms[j][2];
//             if ( align > 0.0 ){
//                 align = sqrt(1.0 - align*align);
//                 if ( align < minA ) {
//                     minA = align;
//                     k = j;
//                 }
//             }
//         }
//         // printf("%d: align: %g, index %d\n", i, minA, k);
//         nV[i][0] = Verts[k][0];
//         nV[i][1] = Verts[k][1];
//         nV[i][2] = Verts[k][2];
//         // for ( int j=0; j<numV; j++ ) {
//         //     double align;
//         //     align = V[i][0]*Norms[j][0]+V[i][1]*Norms[j][1]+V[i][2]*Norms[j][2];
//         //     align = sqrt(1.0 - align*align);
//         //     if ( align < 1.01*minA ) {
//         //         printf("index %d: %g [%g, %g, %g]\n", j, align, Verts[j][0], Verts[j][1], Verts[j][2] );
//         //     }
//         // }
//     }
//
//     for ( int i=0; i<NV; i++ ){
//         fprintf(fid, "%g %g %g %g %g %g\n",
//             nV[i][0], nV[i][1], nV[i][2], V[i][0], V[i][1], V[i][2]);
//     }
//
//     for ( int i=0; i<Nface; i++ ){
//         fprintf(fid, "3 %d %d %d\n", face[i][0], face[i][1], face[i][2]);
//     }
//
//     fclose (fid);
//
