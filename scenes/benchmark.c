#include <stdio.h>

int main() {
  long long sum = 0;
  for (int N = 0; N < 100; N++)
    for (int n = 1; n < 50000; n++) {
      int x = n;
      int iter = 0;
      while (x != 1) {
        if (x % 2 == 0)
          x = x / 2;
        else
          x = x * 3 + 1;
        iter++;
      }
      sum += iter;
    }
  printf("%lli\n", sum);
}