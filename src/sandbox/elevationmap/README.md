# Synopsis

This code demonstrates the instantiation and usage of the s2_elevationmap_t datastructure. An example elevation map called
"sinxx+cosyy.tiff" is generated according to a function that is
hard coded into elevationmap.c. The resulting elevation map is saved
in 32 bit grey scale floating point tiff format.  The floating point
tiff is then read back into computer memory for bicubic or bilinear
interpolation.
