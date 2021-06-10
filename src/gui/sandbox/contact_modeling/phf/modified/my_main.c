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
#include "my_extra.h"

int
//original alpha = 80
main(int argc, char** argv) {
  char* arg0 = (argc > 0) ? argv[0] : __FILE__;
  FILE* input = NULL;
  _Bool dedup = 1;
  size_t alpha = 100;
  size_t lambda = 4;
  phf_seed_t seed = 42;
  int nodiv = 0;
  int verbose = 1;
  int optc;

  setlocale(LC_ALL, "C");

  while(-1 != (optc = getopt(argc, argv, SHORTOPTS))) {
    switch(optc) {
      case '2':
        nodiv = 1;
        break;
      case 'D':
        dedup = 0;
        break;
      case 'a':
        alpha = tounsigned(optarg, 1, 100);
        break;
      case 'f':
        loadpath(prepfile(&input), optarg);
        break;
      case 'l':
        lambda = tounsigned(optarg, 0, RANGEMAX(lambda));
        break;
      case 's':
        seed = tounsigned(optarg, 0, RANGEMAX(seed));
        break;
      case 'v':
        verbose = 1;
        break;
      case 'h':
        usage(arg0, stdout);
        return 0;
      default:
        usage(arg0, stderr);
        return 1;
    }
  }

  if(!input && !isatty(fileno(stdin))) {
    loadfile(prepfile(&input), stdin);
  }

  argc -= optind;
  argv += optind;

  while(argc) {
    loadstring(prepfile(&input), *argv);
    argc--;
    argv++;
  }

  void* addr = NULL;
  size_t addrsize = 0;
  mmapfile(&addr, &addrsize, input);

  phf_string_t* keys = NULL;
  size_t count = 0, limit = 0;
  for(phf_string_t key = { 0, 0 }; nextkey(addr, addrsize, &key);) {
    addkey(&keys, &count, &limit, key);
  }

  /* find and drop any duplicate keys */
  if(dedup) {
    if(!verbose) {
      count = phf_uniq_string(keys, count);
    } else {
      qsort(keys, count, sizeof * keys, keycmp);
      for(size_t i = 0; i + 1 < count;) {
        if(0 == keycmp(&keys[i], &keys[i + 1])) {
          warnx("duplicate key (%s)", (char*)keys[i].p);
          dropkey(&keys, &count, &keys[i]);
        } else {
          i++;
        }
      }
    }
  }

  if(!count) {
    errx(1, "no keys specified");
  }

  struct phf ctx;
  int error;
  if((error = phf_init_string(&ctx, keys, count, lambda, alpha, seed, nodiv))) {
    errx(1, "phf_init: %s", strerror(error));
  }

  // char key_verify[ctx.m][5];
  // for (int i=0; i<ctx.m; i++){
  //     strcpy(key_verify[i],"null");
  // }
  int width = (int)log10(ctx.m) + 1;
  // printf("ctx.m = %zu\n", ctx.m);
  // printf("width = %d\n", width);
  for(phf_string_t key = { 0, 0 }; nextkey(addr, addrsize, &key);) {
    phf_hash_t hash = phf_hash_string(&ctx, key);
    //printf("My Version: hash = %d, key width = %d, key.p = %s\n", (int)hash, (int)key.n, (char *)key.p);
    //strcpy(key_verify[(int)hash], key.p);
    printf("%-*" PHF_PRIuHASH " %.*s\n", width, hash, (int)key.n, (char*)key.p);
  }
  // for (int i=0; i<ctx.m; i++){
  //     printf("key_verify[%d] = %s\n",i, key_verify[i]);
  //     phf_string_t keyt;
  //     keyt.n = strlen(key_verify[i]);
  //     keyt.p = key_verify[i];
  //     phf_hash_t hash = phf_hash_string(&ctx, keyt);
  //     printf("key_verify[hash] = %s, key_verify[i] = %s\n", key_verify[(int)hash], key_verify[i]);
  // }
  /*
  The keys array is not syncronized with the order of keys in the hash
  table. Will need to generate a list of keys dimensioned to ctx.m. Thus when an
  index for a key is returned from hash, it will need to be checked by comparing
  the key entry in the list against the key. If they match, the key was in the hash
  table.
  */

  // for (int i=0; i<100; i++){
  //     printf("key[%d] = %s\n", i, (char *)keys[i].p);
  // }
  // phf_string_t mykey = { 0, 0};
  // mykey.n = 4;
  // mykey.p = "2343";
  // phf_hash_t myhash = phf_hash_string(&ctx, mykey);
  // printf("My Test: myhash = %d, mykey width = %d, mykey.p = %s\n", (int)myhash, (int)mykey.n, (char *)mykey.p);
  //
  // for (int i=0; i<100; i++){
  //     double vec[3];
  //     vec[0]= (double)rand() / RAND_MAX;
  //     vec[1]= (double)rand() / RAND_MAX;
  //     vec[2]= (double)rand() / RAND_MAX;
  //     double nv = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
  //     vec[0] /= nv;
  //     vec[1] /= nv;
  //     vec[2] /= nv;
  //     int ivec[3];
  //     ivec[0] = round(7*vec[0]);
  //     ivec[1] = round(7*vec[1]);
  //     ivec[2] = round(7*vec[2]);
  //     char svec[7];
  //     sprintf(svec, "%+d%+d%+d",ivec[0], ivec[1], ivec[2]);
  //     printf("random vector = %s\n", svec);
  // }

  printf("nodiv = %d\n", ctx.nodiv);
  printf("seed = %d\n", ctx.seed);
  printf("number of elements in g = %zu\n", ctx.r);
  printf("number of elements in hash = %zu\n", ctx.m);
  for(int i = 0; i < ctx.r; i++) {
    printf("g[%d] = %u\n", i, ctx.g[i]);
  }
  printf("d_max = %zu\n", ctx.d_max);

  phf_destroy(&ctx);
  if(0 != munmap(addr, addrsize)) {
    err(1, "munmap");
  }
  fclose(input);

  return 0;
}
