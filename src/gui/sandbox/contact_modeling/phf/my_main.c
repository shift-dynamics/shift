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

#define _GNU_SOURCE
#ifndef MAX
#define MAX(a, b) (((a) > (b))? (a) : (b))
#endif
#define RANGEMAX(E) (~((0)? (E) : 0))
#define SHORTOPTS "2Da:f:l:s:vh"
#define USAGE \
  "Usage: %s [-%s] [KEY ...]\n" \
  "  -2       round r and m to power of 2\n" \
  "  -D       don't deduplicate\n" \
  "  -a PCT   hash table load factor (1%% - 100%%)\n" \
  "  -f PATH  key file\n" \
  "  -l NUM   number of keys per displacement map bucket\n" \
  "  -s SEED  seed\n" \
  "  -v       be verbose\n" \
  "  -h       print this usage\n" \
  "\n"

static void
addkey(phf_string_t** k, size_t* n, size_t* z, phf_string_t kn);

static void
dropkey(phf_string_t** _keys, size_t* count, phf_string_t* key);

static int
keycmp(const void* _a, const void* _b);

static phf_string_t*
nextkey(void* addr, size_t addrsize, phf_string_t* key);

static phf_string_t*
fgetkey(phf_string_t* key, char* buf, size_t bufsiz, FILE* fp);

static void
loadkey(FILE* input, const phf_string_t key);

static void
loadfile(FILE* input, FILE* fp);

static void
loadpath(FILE* input, const char* path);

static void
loadstring(FILE* input, char* src);

static void*
mmapfile(void** addr, size_t* size, FILE* input);

static FILE*
prepfile(FILE** input);

static uintmax_t
tounsigned(const char* src, uintmax_t min, uintmax_t max);

static void
usage(char* arg0, FILE* fp);

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

  /*
  It appears here that input contains the list of keywords, and addr will
  contain them after mmapfile transfers them from input. addrsize will
  indicate the size of addr when all keywords have been copied.
  */
  void* addr = NULL;
  size_t addrsize = 0;
  mmapfile(&addr, &addrsize, input);

  /*
  phf_string_t is a special type of string structure with two parts.
  typedef struct phf_string {
    void *p;
    size_t n;
  } phf_string_t;
  nextkey(addr, addrsize, &key); returns NULL when all keys processed.
  addkey(&keys, &count, &limit, key); puts next key into the keys array.
  The keys are inside the addr array and nextkey(addr, addrsize, &key);
  fetches them in order. Here they are transferred to the keys array.
  */
  phf_string_t* keys = NULL;
  size_t count = 0, limit = 0;
  for(phf_string_t key = { 0, 0 }; nextkey(addr, addrsize, &key);) {
    addkey(&keys, &count, &limit, key);
  }

  /*
    Find and drop any duplicate keys. This will not be used because there
    are no duplicate keys in these special key files. The keys array is sorted
    and duplicate keys are removed.
    */
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

  // Nothing to do if no keys specified.
  if(!count) {
    errx(1, "no keys specified");
  }

  /*
  struct phf {
    bool nodiv;

    phf_seed_t seed;

    size_t r; //number of elements in g
    size_t m; //number of elements in perfect hash
    uint32_t *g; //displacement map indexed by g(k) % r

    size_t d_max; //maximum displacement value in g

    enum {
      PHF_G_UINT8_MOD_R = 1,
      PHF_G_UINT8_BAND_R,
      PHF_G_UINT16_MOD_R,
      PHF_G_UINT16_BAND_R,
      PHF_G_UINT32_MOD_R,
      PHF_G_UINT32_BAND_R,
    } g_op;

    const void *g_jmp;
  }; //struct phf
  */

  struct phf ctx;
  int error;
  if((error = phf_init_string(&ctx, keys, count, lambda, alpha, seed, nodiv))) {
    errx(1, "phf_init: %s", strerror(error));
  }

  /*
  This loop writes out hashtable key information.
  */
  phf_string_t ka[1000];
  int width = (int)log10(ctx.m) + 1;
  int i = 0;
  int keycount = 0;
  for(phf_string_t key = { 0, 0 }; nextkey(addr, addrsize, &key);) {
    ka[i++] = key;
    keycount++;
    phf_hash_t hash = phf_hash_string(&ctx, key);
    printf("%-*" PHF_PRIuHASH " %.*s\n", width, hash, (int)key.n, (char*)key.p);
  }
  /*
  typedef struct phf_string {
      void *p;
      size_t n;
  } phf_string_t;
  */

  int kp[1000];
  printf("My Loop\n");
  for(int i = 0; i < keycount; i++) {
    phf_hash_t hash = phf_hash_string(&ctx, ka[i]);
    kp[(int)hash] = i;
  }

  for(int i = 0; i < keycount; i++) {
    phf_hash_t hash = phf_hash_string(&ctx, ka[i]);
    printf("%d, %s\n", kp[(int)hash], (char*)ka[i].p);
  }


  /*
  The keys array is not syncronized with the order of keys in the hash
  table. Will need to generate a list of keys dimensioned to ctx.m. Thus when an
  index for a key is returned from hash, it will need to be checked by comparing
  the key entry in the list against the key. If they match, the key was in the hash
  table.
  */

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
