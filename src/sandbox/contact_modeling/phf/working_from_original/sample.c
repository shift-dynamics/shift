#define _GNU_SOURCE
#include <assert.h>
#include <err.h>
#include <locale.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "phf.h"

// #ifndef MAX
// #define MAX(a, b) (((a) > (b)) ? (a) : (b))
// #endif

/*
typedef struct phf_string {
  void* p;
  size_t n;
} phf_string_t;
*/

int main(int argc, char** argv) {
  // original alpha = 80
  size_t alpha = 100;
  size_t lambda = 4;
  phf_seed_t seed = 42;
  int nodiv = 0;

  setlocale(LC_ALL, "C");

  phf_string_t keys[10];
  size_t count = 10;
  for(int i = 0; i < count; i++) {
    keys[i].n = 5;
    keys[i].p = malloc(8 * sizeof(char));
  }
  memcpy(keys[0].p, "10000", 6);
  memcpy(keys[1].p, "10001", 6);
  memcpy(keys[2].p, "10002", 6);
  memcpy(keys[3].p, "10003", 6);
  memcpy(keys[4].p, "10004", 6);
  memcpy(keys[5].p, "10005", 6);
  memcpy(keys[6].p, "10006", 6);
  memcpy(keys[7].p, "10007", 6);
  memcpy(keys[8].p, "10008", 6);
  memcpy(keys[9].p, "10009", 6);
  for(int i = 0; i < count; i++) {
    printf("keys[%d].n = %d, keys[%d].p = %s\n", i, (int)keys[i].n, i,
           keys[i].p);
  }

  /*
  Need to write a simple function to read in a set of keys from a file.
  Or write a function to generate a set of keys stored in keys
  phf_error_t phf_init_string(struct phf*, const phf_string_t*, const size_t,
  const size_t, const size_t, const phf_seed_t, const bool nodiv);
  struct phf {
    bool nodiv;

    phf_seed_t seed;

    size_t r; number of elements in g
    size_t m; number of elements in perfect hash
    uint32_t* g; displacement map indexed by g(k) % r

    size_t d_max; maximum displacement value in g

    enum {
      PHF_G_UINT8_MOD_R = 1,
      PHF_G_UINT8_BAND_R,
      PHF_G_UINT16_MOD_R,
      PHF_G_UINT16_BAND_R,
      PHF_G_UINT32_MOD_R,
      PHF_G_UINT32_BAND_R,
    } g_op;

    const void* g_jmp;
  }; struct phf

  */

  struct phf ctx;
  int error;
  /*
  typedef struct phf_string {
    void* p;
    size_t n;
  */

  /*
  Allocate an array of count phf_string_t keys. For each keys[i]
  malloc eight*sizeof(char) to keys[i].p.
  memcpy(keys[i].p, key, strlen(key)+1);
  */

  if((error = phf_init_string(&ctx, keys, count, lambda, alpha, seed, nodiv))) {
    errx(1, "phf_init: %s", strerror(error));
  }
  /*
  The keys array is not syncronized with the order of keys in the hash
  table. Will need to generate a list of keys dimensioned to ctx.m. Thus when an
  index for a key is returned from hash, it will need to be checked by comparing
  the key entry in the list against the key. If they match, the key was in the
  hash table.
  */
  for(int i = 0; i < count; i++) {
    phf_hash_t hash = phf_hash_string(&ctx, keys[i]);
    printf("hash = %d, keys[%d].n = %d, keys[%d].p = %s\n", (int)hash, i,
           (int)keys[i].n, i, (char*)keys[i].p);
  }

  printf("nodiv = %d\n", ctx.nodiv);
  printf("seed = %d\n", ctx.seed);
  printf("number of elements in g = %zu\n", ctx.r);
  printf("number of elements in hash = %zu\n", ctx.m);
  for(int i = 0; i < ctx.r; i++) {
    printf("g[%d] = %u\n", i, ctx.g[i]);
  }
  printf("d_max = %zu\n", ctx.d_max);

  phf_destroy(&ctx);

  return 0;
}
