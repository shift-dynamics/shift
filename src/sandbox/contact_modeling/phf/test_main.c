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

/*
List of valid options to the program. Each option is a single character. If a :
follows an option, the option character is to be followed by an argument.
*/
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
usage(char* arg0, FILE* fp) {
  char* progname = strrchr(arg0, '/') ? strrchr(arg0, '/') + 1 : arg0;
  fprintf(fp, USAGE, progname, SHORTOPTS);
}

#define RANGEMAX(E) (~((0)? (E) : 0))

/*
* options() parses option letters and option arguments from the argv list.
* Succesive calls return succesive option letters which match one of
* those in the legal list. Option letters may require option arguments
* as indicated by a ':' following the letter in the legal list.
* for example, a legal list of "ab:c" implies that a, b and c are
* all valid options and that b takes an option argument. The option
* argument is passed back to the calling function in the value
* of the global OptArg pointer. The OptIndex gives the next string
* in the argv[] array that has not already been processed by options().
*
* options() returns -1 if there are no more option letters or if
* double SwitchChar is found. Double SwitchChar forces options()
* to finish processing options.
*
* options() returns '?' if an option not in the legal set is
* encountered or an option needing an argument is found without an
* argument following it.
*
*/

static const char SwitchChar = '-';
static const char Unknown = '?';

int OptIndex = 1;       /* first option should be argv[1] */
char* OptArg = NULL;    /* global option argument pointer */

int options(int argc, char* argv[], const char* legal) {
  static char* posn = "";  /* position in argv[OptIndex] */
  char* legal_index = NULL;
  int letter = 0;

  if(!*posn) {
    /* no more args, no SwitchChar or no option letter ? */
    if((OptIndex >= argc) ||
        (*(posn = argv[OptIndex]) != SwitchChar) ||
        !*++posn) {
      return -1;
    }
    /* find double SwitchChar ? */
    if(*posn == SwitchChar) {
      OptIndex++;
      return -1;
    }
  }
  letter = *posn++;
  if(!(legal_index = strchr(legal, letter))) {
    if(!*posn) {
      OptIndex++;
    }
    return Unknown;
  }
  if(*++legal_index != ':') {
    /* no option argument */
    OptArg = NULL;
    if(!*posn) {
      OptIndex++;
    }
  } else {
    if(*posn) {
      /* no space between opt and opt arg */
      OptArg = posn;
    } else {
      if(argc <= ++OptIndex) {
        posn = "";
        return Unknown;
      } else {
        OptArg = argv[OptIndex];
      }
    }
    posn = "";
    OptIndex++;
  }
  return letter;
}

int
//char **argv is the same as char *argv[]
//argc is count of arguments in argv[] incouding my name in argv[0]
main(int argc, char** argv) {
  char* arg0 = (argc > 0) ? argv[0] : __FILE__;
  FILE* input = NULL;
  _Bool dedup = 1;
  size_t alpha = 100;
  size_t lambda = 4;
  size_t seed = 42;
  int nodiv = 0;
  int verbose = 1;
  int optc;

  setlocale(LC_ALL, "C");

  printf("My name is %s\n", argv[0]);

  options(argc, argv, SHORTOPTS);

  int argc_copy;
  argc_copy = argc;
  char** argv_copy;
  argv_copy = argv;
  while(argc_copy--) {
    printf("%s\n", *argv_copy++);
  }

  while(-1 != (optc = getopt(argc, argv, SHORTOPTS))) {
    switch(optc) {
      case '2':
        nodiv = 1;
        printf("nodiv = %d\n", nodiv);
        break;
      case 'D':
        dedup = 0;
        printf("dedup = %d\n", dedup);
        break;
      case 'a':
        printf("alpha = %s\n", optarg);
        alpha = 80; //tounsigned(optarg, 1, 100);
        break;
      case 'f':
        printf("loadpath(prepfile())\n"); //loadpath(prepfile(&input), optarg);
        break;
      case 'l':
        printf("lambda = %s\n", optarg);
        lambda = 10; //tounsigned(optarg, 0, RANGEMAX(lambda));
        break;
      case 's':
        printf("seed = %s\n", optarg);
        seed = 43; //tounsigned(optarg, 0, RANGEMAX(seed));
        break;
      case 'v':
        verbose = 1;
        printf("verbose = %d\n", verbose);
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
    printf("loadfile(prepfile())1\n");  //loadfile(prepfile(&input), stdin);
  }

  argc -= optind;
  argv += optind;

  while(argc) {
    printf("loadstring(prepfile())2\n"); //loadstring(prepfile(&input), *argv);
    argc--;
    argv++;
  }


  exit(EXIT_SUCCESS);
// exit(EXIT_FAILURE);

}
