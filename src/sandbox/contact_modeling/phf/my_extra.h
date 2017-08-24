#ifndef MY_EXTRA_H
#define MY_EXTRA_H

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
addkey(phf_string_t **k, size_t *n, size_t *z, phf_string_t kn);

static void
dropkey(phf_string_t **_keys, size_t *count, phf_string_t *key);

static int
keycmp(const void *_a, const void *_b);

static phf_string_t *
nextkey(void *addr, size_t addrsize, phf_string_t *key);

static phf_string_t *
fgetkey(phf_string_t *key, char *buf, size_t bufsiz, FILE *fp);

static void
loadkey(FILE *input, const phf_string_t key);

static void
loadfile(FILE *input, FILE *fp);

static void
loadpath(FILE *input, const char *path);

static void
loadstring(FILE *input, char *src);

static void *
mmapfile(void **addr, size_t *size, FILE *input);

static FILE *
prepfile(FILE **input);

static uintmax_t
tounsigned(const char *src, uintmax_t min, uintmax_t max);

static void
usage(char *arg0, FILE *fp);

#endif /* MY_EXTRA_H */
