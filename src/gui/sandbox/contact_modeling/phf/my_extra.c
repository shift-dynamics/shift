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
addkey(phf_string_t** k, size_t* n, size_t* z, phf_string_t kn) {
  if(!(*n < *z)) {
    size_t z1 = MAX(*z, 1) * 2;
    phf_string_t* p;

    if(z1 < *z || (SIZE_MAX / sizeof** k) < z1) {
      errx(1, "addkey: %s", strerror(ERANGE));
    }

    if(!(p = realloc(*k, z1 * sizeof** k))) {
      err(1, "realloc");
    }

    *k = p;
    *z = z1;
  }

  (*k)[(*n)++] = kn;
}

static void
dropkey(phf_string_t** _keys, size_t* count, phf_string_t* key) {
  phf_string_t* keys = *_keys, *end = &keys[*count];
  assert(key >= keys && key < end);
  memmove(key, key + 1, (end - key - 1) * sizeof * key);
  --*count;
}

static int
keycmp(const void* _a, const void* _b) {
  const phf_string_t* a = _a, *b = _b;
  return strcmp(a->p, b->p);
}

static phf_string_t*
nextkey(void* addr, size_t addrsize, phf_string_t* key) {
  char* tp = addr, *pe = tp + addrsize, *p;
  size_t n;

  if(key->p == NULL) {
    p = tp;
  } else {
    p = key->p + key->n + 1;
  }

  if(!(p < pe)) {
    return NULL;
  }
  if(!(n = strlen(p))) {
    return NULL;
  }

  key->p = p;
  key->n = n;

  return key;
}

static phf_string_t*
fgetkey(phf_string_t* key, char* buf, size_t bufsiz, FILE* fp) {
  char* p, *pe;

  do {
    if(!(p = fgets(buf, bufsiz, fp))) {
      return NULL;
    }
    pe = &buf[strlen(buf)];
    if(p < pe && pe[-1] == '\n') {
      *--pe = '\n';
    }
  } while(p == pe);

  key->p = p;
  key->n = pe - p;
  return key;
}

static void
loadkey(FILE* input, const phf_string_t key) {
  if(key.n != fwrite(key.p, 1, key.n, input)) {
    err(1, "fwrite");
  }
  if(EOF == fputc('\0', input)) {
    err(1, "fputc");
  }
}

static void
loadfile(FILE* input, FILE* fp) {
  char buf[BUFSIZ];
  phf_string_t key;

  while(fgetkey(&key, buf, sizeof buf, fp)) {
    loadkey(input, key);
  }
}

static void
loadpath(FILE* input, const char* path) {
  if(0 == strcmp(path, "-")) {
    path = "/dev/stdin";
  }

  FILE* fp;
  if(!(fp = fopen(path, "r"))) {
    err(1, "%s", path);
  }
  loadfile(input, fp);
  fclose(fp);
}

static void
loadstring(FILE* input, char* src) {
  loadkey(input, (phf_string_t) {
    src, strlen(src)
  });
}

static void*
mmapfile(void** addr, size_t* size, FILE* input) {
  if(0 != fflush(input)) {
    err(1, "fflush");
  }

  int fd = fileno(input);
  struct stat st;
  if(0 != fstat(fd, &st)) {
    err(1, "fstat");
  }

  int prot = PROT_READ;
  int flags = MAP_FILE | MAP_PRIVATE;
  if(MAP_FAILED == (*addr = mmap(NULL, st.st_size, prot, flags, fd, 0))) {
    err(1, "mmap");
  }

  *size = st.st_size;
  return *addr;
}

static FILE*
prepfile(FILE** input) {
  if(!*input && !(*input = tmpfile())) {
    err(1, "tmpfile");
  }
  return *input;
}

static uintmax_t
tounsigned(const char* src, uintmax_t min, uintmax_t max) {
  char* end = NULL;
  uintmax_t i;

  errno = 0;
  i = strtoumax(src, &end, 0);
  if((i == 0 || i == UINTMAX_MAX) && errno) {
    err(1, "%s", src);
  }
  if(i < min || i > max) {
    errx(1, "%s: not within interval [%ju, %ju]", src, min, max);
  }
  if(!end || *end != '\0') {
    errx(1, "%s: illegal character", src);
  }

  return i;
}

static void
usage(char* arg0, FILE* fp) {
  char* progname = strrchr(arg0, '/') ? strrchr(arg0, '/') + 1 : arg0;
  fprintf(fp, USAGE, progname, SHORTOPTS);
}
