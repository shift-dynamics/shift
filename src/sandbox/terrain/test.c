#include <stdio.h>

int main() {
  for(int i = 0; i < 128; i++) {
    printf("%d & 63 = %d, ", i, i & 63);
    printf("%d >> 6 = %d\n", i, i >> 6);
  }
  return 0;
}
