import math

if __name__ == "__main__":
    m, e = math.frexp(1)
    print("mult=", round(m * (1 << 31)), "shift=", e)
