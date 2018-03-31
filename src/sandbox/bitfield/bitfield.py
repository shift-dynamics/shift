#!/usr/bin/env python
'''
Code to verify / debug hashing function for simplices, takes the indices
for object i and j and converts to an offset into an array of simplices
'''


def get_index(i, j):
    return int((j - 1) * j / 2 + i)


if __name__ == "__main__":
    n = 5
    for i in range(n):
        for j in range(i + 1, n):
            print(i, j, get_index(i, j))

    print((n - 1) * n / 2)
