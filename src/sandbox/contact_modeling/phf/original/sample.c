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

#ifndef MAX
#define MAX(a, b) (((a) > (b))? (a) : (b))
#endif

static void
addkey(phf_string_t **k, size_t *n, size_t *z, phf_string_t kn) {
	if (!(*n < *z)) {
		size_t z1 = MAX(*z, 1) * 2;
		phf_string_t *p;

		if (z1 < *z || (SIZE_MAX / sizeof **k) < z1)
			errx(1, "addkey: %s", strerror(ERANGE));

		if (!(p = realloc(*k, z1 * sizeof **k)))
			err(1, "realloc");

		*k = p;
		*z = z1;
	}

	(*k)[(*n)++] = kn;
}

static void
dropkey(phf_string_t **_keys, size_t *count, phf_string_t *key)
{
	phf_string_t *keys = *_keys, *end = &keys[*count];
	assert(key >= keys && key < end);
	memmove(key, key + 1, (end - key - 1) * sizeof *key);
	--*count;
}

static int
keycmp(const void *_a, const void *_b)
{
	const phf_string_t *a = _a, *b = _b;
	return strcmp(a->p, b->p);
}

static phf_string_t *
nextkey(void *addr, size_t addrsize, phf_string_t *key)
{
	char *tp = addr, *pe = tp + addrsize, *p;
	size_t n;

	if (key->p == NULL) {
		p = tp;
	} else {
		p = key->p + key->n + 1;
	}

	if (!(p < pe))
		return NULL;
	if (!(n = strlen(p)))
		return NULL;

	key->p = p;
	key->n = n;

	return key;
}

static phf_string_t *
fgetkey(phf_string_t *key, char *buf, size_t bufsiz, FILE *fp)
{
	char *p, *pe;

	do {
		if (!(p = fgets(buf, bufsiz, fp)))
			return NULL;
		pe = &buf[strlen(buf)];
		if (p < pe && pe[-1] == '\n')
			*--pe = '\n';
	} while (p == pe);

	key->p = p;
	key->n = pe - p;
	return key;
}

static void
loadkey(FILE *input, const phf_string_t key)
{
	if (key.n != fwrite(key.p, 1, key.n, input))
		err(1, "fwrite");
	if (EOF == fputc('\0', input))
		err(1, "fputc");
}

static void
loadfile(FILE *input, FILE *fp)
{
	char buf[BUFSIZ];
	phf_string_t key;

	while (fgetkey(&key, buf, sizeof buf, fp)) {
		loadkey(input, key);
	}
}

static void
loadpath(FILE *input, const char *path)
{
	if (0 == strcmp(path, "-"))
		path = "/dev/stdin";

	FILE *fp;
	if (!(fp = fopen(path, "r")))
		err(1, "%s", path);
	loadfile(input, fp);
	fclose(fp);
}

static void
loadstring(FILE *input, char *src)
{
	loadkey(input, (phf_string_t){ src, strlen(src) });
}

static void *
mmapfile(void **addr, size_t *size, FILE *input)
{
	if (0 != fflush(input))
		err(1, "fflush");

	int fd = fileno(input);
	struct stat st;
	if (0 != fstat(fd, &st))
		err(1, "fstat");

	int prot = PROT_READ;
	int flags = MAP_FILE|MAP_PRIVATE;
	if (MAP_FAILED == (*addr = mmap(NULL, st.st_size, prot, flags, fd, 0)))
		err(1, "mmap");

	*size = st.st_size;
	return *addr;
}

static FILE *
prepfile(FILE **input)
{
	if (!*input && !(*input = tmpfile()))
		err(1, "tmpfile");
	return *input;
}

static uintmax_t
tounsigned(const char *src, uintmax_t min, uintmax_t max)
{
	char *end = NULL;
	uintmax_t i;

	errno = 0;
	i = strtoumax(src, &end, 0);
	if ((i == 0 || i == UINTMAX_MAX) && errno)
		err(1, "%s", src);
	if (i < min || i > max)
		errx(1, "%s: not within interval [%ju, %ju]", src, min, max);
	if (!end || *end != '\0')
		errx(1, "%s: illegal character", src);

	return i;
}

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
usage(char *arg0, FILE *fp)
{
	char *progname = strrchr(arg0, '/')? strrchr(arg0, '/') + 1 : arg0;
	fprintf(fp, USAGE, progname, SHORTOPTS);
}

#define RANGEMAX(E) (~((0)? (E) : 0))

int
//original alpha = 80
main(int argc, char **argv)
{
	char *arg0 = (argc > 0)? argv[0] : __FILE__;
	FILE *input = NULL;
	_Bool dedup = 1;
	size_t alpha = 100;
	size_t lambda = 4;
	phf_seed_t seed = 42;
	int nodiv = 0;
	int verbose = 1;
	int optc;

	setlocale(LC_ALL, "C");

	while (-1 != (optc = getopt(argc, argv, SHORTOPTS))) {
		switch (optc) {
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

	if (!input && !isatty(fileno(stdin)))
		loadfile(prepfile(&input), stdin);

	argc -= optind;
	argv += optind;

	while (argc) {
		loadstring(prepfile(&input), *argv);
		argc--;
		argv++;
	}

	void *addr = NULL;
	size_t addrsize = 0;
	mmapfile(&addr, &addrsize, input);

	phf_string_t *keys = NULL;
	size_t count = 0, limit = 0;
	for (phf_string_t key = { 0, 0 }; nextkey(addr, addrsize, &key);) {
		addkey(&keys, &count, &limit, key);
	}

	/* find and drop any duplicate keys */
	if (dedup) {
		if (!verbose) {
			count = phf_uniq_string(keys, count);
		} else {
			qsort(keys, count, sizeof *keys, keycmp);
			for (size_t i = 0; i + 1 < count;) {
				if (0 == keycmp(&keys[i], &keys[i + 1])) {
					warnx("duplicate key (%s)", (char *)keys[i].p);
					dropkey(&keys, &count, &keys[i]);
				} else {
					i++;
				}
			}
		}
	}

	if (!count)
		errx(1, "no keys specified");

	struct phf ctx;
	int error;
	if ((error = phf_init_string(&ctx, keys, count, lambda, alpha, seed, nodiv)))
		errx(1, "phf_init: %s", strerror(error));

    // char key_verify[ctx.m][5];
    // for (int i=0; i<ctx.m; i++){
    //     strcpy(key_verify[i],"null");
    // }
	int width = (int)log10(ctx.m) + 1;
    // printf("ctx.m = %zu\n", ctx.m);
    // printf("width = %d\n", width);
	for (phf_string_t key = { 0, 0 }; nextkey(addr, addrsize, &key);) {
		phf_hash_t hash = phf_hash_string(&ctx, key);
        //printf("My Version: hash = %d, key width = %d, key.p = %s\n", (int)hash, (int)key.n, (char *)key.p);
        //strcpy(key_verify[(int)hash], key.p);
		printf("%-*" PHF_PRIuHASH " %.*s\n", width, hash, (int)key.n, (char *)key.p);
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
    for (int i=0; i<ctx.r; i++){
        printf("g[%d] = %u\n", i, ctx.g[i]);
    }
    printf("d_max = %zu\n", ctx.d_max);

	phf_destroy(&ctx);
	if (0 != munmap(addr, addrsize))
		err(1, "munmap");
	fclose(input);

	return 0;
}
