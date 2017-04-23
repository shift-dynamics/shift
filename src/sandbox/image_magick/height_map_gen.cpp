#include <Magick++.h>
#include <math.h>
#include <iostream>

double surface(double x, double y) {
  return 300.0 * sin(x * x) + 12.0 * cos(y * y);
}

int main(int argc, char** argv) {
  Magick::InitializeMagick(*argv);

  int height = 400;
  int width = 400;
  double map[height][width];
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      map[y][x] = surface(x, y);
    }
  }

  Magick::Image heightMap;
  heightMap.size("400x400");
  heightMap.type(Magick::TrueColorMatteType);
  heightMap.depth(32);
  heightMap.type(Magick::GrayscaleType);
  heightMap.defineValue("quantum", "format", "floating-point");


  /* image magick convention is that x is the left ordinate of view,
  i.e. the row index. y is the top ordinate of the view. Therefore, z
  is pointing up. This follows the ISO coordinate system for vehicle dynamics.

  star2 uses the SAE coordinate system with z pointing down. Therefore,
  x is the top ordinate of the view, y is the left ordinate of view */


  for(int x = 0; x < heightMap.columns(); x++) {
    for(int y = 0; y < heightMap.rows(); y++) {
      double z =  surface(x, y);
      heightMap.pixelColor(y, x, Magick::Color(z, z, z));
    }
  }


  heightMap.write("sinxx+cosyy.tiff");


  Magick::Image test("sinxx+cosyy.tiff");
  int w = test.columns();
  int h = test.rows();
  Magick::PixelPacket *pixels = test.getPixels(0, 0, w, h);

  double map2[test.rows() * test.columns()];

  float min = 0;
  float max = 1;

  for(int y = 0; y < heightMap.rows(); y++) {
    for(int x = 0; x < heightMap.columns(); x++) {
      Magick::Color z = pixels[w * x + y];
      map2[y * heightMap.columns() + x] = z.redQuantum();
      min = fmin(min, z.redQuantum());
      max = fmax(max, z.redQuantum());
    }
  }

  double x_offset = 15.0;
  double y_offset = 30.0;

  std::cout << "min" << min << ", max:" << max << std::endl;

  /* load values into array following sae convention with x as
  top ordinate of view (column index) and y as left ordinate of view, z
  is pointing down */



  return 0;
}
