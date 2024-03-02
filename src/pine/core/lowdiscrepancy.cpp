#include <pine/core/lowdiscrepancy.h>

namespace pine {

psl::vector<uint16_t> compute_radical_inverse_permutations(RNG& rng) {
  auto perms = psl::vector<uint16_t>{size_t(psl::sum(Primes))};

  uint16_t* p = &perms[0];
  for (int i = 0; i < PrimeTablesize; i++) {
    for (int j = 0; j < Primes[i]; j++)
      p[j] = j;
    shuffle(p, Primes[i], 1, rng);
    p += Primes[i];
  }

  return perms;
}

}  // namespace pine