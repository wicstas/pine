struct unsigned_float16 {
  static uint16_t float_to_bits(float x) {
    auto u = psl::bitcast<uint32_t>(x);
    auto exp = int((u >> 23) & 0b11111111);
    exp = psl::max(exp + 0b1111 - 0b1111111, 0);
    uint16_t sig = (u >> 12) & 0b11111111111;
    return (exp << 11) | sig;
  }
  static float bits_to_float(uint16_t b) {
    auto exp = uint32_t(b >> 11);
    exp = exp + 0b1111111 - 0b1111;
    auto sig = uint32_t(b & 0b11111111111) << 12;
    return psl::bitcast<float>((exp << 23) | sig);
  }

  unsigned_float16(float x) : bits(float_to_bits(x)) {
  }
  operator float() const {
    return bits_to_float(bits);
  }
  unsigned_float16& operator+=(float rhs) {
    return *this = float(*this) + rhs;
  }
  unsigned_float16& operator-=(float rhs) {
    return *this = float(*this) - rhs;
  }
  unsigned_float16& operator*=(float rhs) {
    return *this = float(*this) * rhs;
  }
  unsigned_float16& operator/=(float rhs) {
    return *this = float(*this) / rhs;
  }

  psl::string to_string() const {
    return psl::to_string(float(*this));
  }

  uint16_t bits;
};