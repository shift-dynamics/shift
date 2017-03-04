all: ksl_test_gcc ksl_test_clang

gcc: ksl_test_gcc

clang: ksl_test_clang

ksl_test_gcc: ksl_test.c ksl.c
	gcc ksl_test.c ksl.c -O3 -lm -lcblas -o ksl_test -I.

ksl_test_clang: ksl_test.c ksl.c
	clang ksl_test.c ksl.c -O3 -lm -lcblas -o ksl_test -I.

clean:
	rm -r ksl_test_clang ksl_test_gcc
