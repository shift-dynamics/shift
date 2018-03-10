typedef struct EllipsoidSupportStruct {
  double a;
  double b;
  double c;
} EllipsoidSupportStruct;

typedef void (*ConvexHullSupportFunction)(void*, EllipsoidSupportStruct*,
                                          double[3], double[3], double[4][3],
                                          double[3]);

typedef void (*RestoreConvexHullSupport)(double[4][3], double[4][3],
                                         double[4][3], int*);

typedef struct SimplexSupportStruct {
  ConvexHullSupportFunction ConvexHullSupportFunctionA;
  ConvexHullSupportFunction ConvexHullSupportFunctionB;
  RestoreConvexHullSupport RestoreConvexHullSupportA;
  RestoreConvexHullSupport RestoreConvexHullSupportB;
  void* ConvexHullSupportStructA;
  void* ConvexHullSupportStructB;
  double (*spatialDisplacementBodyA)[3];
  double (*spatialDisplacementBodyB)[3];
  double shapeMatrixA[4][3];
  double shapeMatrixB[4][3];
  int numberSimplexPoints;
  double extremeVertexLocalA[4][3];
  double extremeVertexLocalB[4][3];
  int extremeVertexIndexA;
  int extremeVertexIndexB;
  double searchDirectionVector[3];
  int origin;
} SimplexSupportStruct;

void cross_vec(double x[3], double y[3], double z[3]);

void dif_vec(double x[3], double y[3], double z[3]);

double dot_vec(double x[3], double y[3]);

double norm_vec(double x[3]);

void u_vec(double x[3], double y[3]);

void swap_vec(double x[3], double y[3]);

void SDMPY(double DO[4][3], double DI1[4][3], double DI2[4][3]);

void simplex0(double sDV[3], double V[13][3], double extremeVertexLocalA[4][3],
              double extremeVertexLocalB[4][3], int* numberSimplexPoints,
              int* origin);
void simplex1(double sDV[3], double V[13][3], double extremeVertexLocalA[4][3],
              double extremeVertexLocalB[4][3], int* numberSimplexPoints,
              int* origin);
void simplex2(double sDV[3], double V[13][3], double extremeVertexLocalA[4][3],
              double extremeVertexLocalB[4][3], int* numberSimplexPoints,
              int* origin);
void simplex3(double sDV[3], double V[13][3], double extremeVertexLocalA[4][3],
              double extremeVertexLocalB[4][3], int* numberSimplexPoints,
              int* origin);

void EllipsoidSupportFunction(void* SimplexSupportStructV,
                              EllipsoidSupportStruct* EllipsoidSupportStructI,
                              double sDV[3], double VL[3], double SDH[4][3],
                              double VA[3]);

void RestoreEllipsoidSupport(double EVA[4][3], double SDH[4][3],
                             double WVL[4][3], int* numberSimplexPoints);

int SimplexSupportFunction(SimplexSupportStruct* SimplexSupportStructI);
