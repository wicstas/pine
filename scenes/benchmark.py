from numba import jit

@jit
def run():
    sum = 0
    for n in range(100):
        for x in range(1, 50000):
            iter = 0
            while x != 1:
                if x % 2 == 0:
                    x = x // 2
                else:
                    x = x * 3 + 1
                iter += 1
            sum += iter
    return sum

print(run())