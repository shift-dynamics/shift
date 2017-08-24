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

int main() {

    char keys[200][5];
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    int i;
    FILE *fp;
    fp=fopen("keys", "r");
    while ( (read = getline(&line, &len, fp)) != -1 ) {
        line[strlen(line) - 1] = 0;
        strncpy(keys[i], line, strlen(line));
        printf("%s, %d\n", line, (int)strlen(keys[i]));
        i++;
    }

    return 0;
}
