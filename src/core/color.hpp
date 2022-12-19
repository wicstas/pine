#pragma once
#include <core/math.hpp>

namespace pine {

template <int nSpectrumSamples, typename Derived>
struct CoefficientSpectrum {
  static constexpr int nSamples = nSpectrumSamples;
  using Coefficients = Vector<float, nSpectrumSamples>;

  CoefficientSpectrum() = default;
  explicit CoefficientSpectrum(Coefficients c) : c(c) {}

  bool isBlack() const { return c == Coefficients(0); }

  CoefficientSpectrum& operator+=(const CoefficientSpectrum& b) {
    c += b.c;
    return self();
  }
  CoefficientSpectrum& operator-=(const CoefficientSpectrum& b) {
    c -= b.c;
    return self();
  }
  CoefficientSpectrum& operator*=(const CoefficientSpectrum& b) {
    c *= b.c;
    return self();
  }
  CoefficientSpectrum& operator/=(const CoefficientSpectrum& b) {
    c /= b.c;
    return self();
  }
  CoefficientSpectrum& operator*=(float b) {
    c *= b;
    return self();
  }
  CoefficientSpectrum& operator/=(float b) {
    c /= b;
    return self();
  }

  friend Derived operator+(Derived a, const Derived& b) { return a += b; }
  friend Derived operator-(Derived a, const Derived& b) { return a -= b; }
  friend Derived operator*(Derived a, const Derived& b) { return a *= b; }
  friend Derived operator/(Derived a, const Derived& b) { return a /= b; }
  friend Derived operator*(Derived a, float b) { return a *= b; }
  friend Derived operator/(Derived a, float b) { return a /= b; }
  friend Derived operator*(float a, Derived b) { return Derived(a * b.c); }
  friend Derived operator/(float a, Derived b) { return Derived(a / b.c); }
  friend Derived sqrt(Derived x) { return Derived(sqrt(x.c)); }
  friend Derived exp(Derived x) { return Derived(exp(x.c)); }
  friend Derived log(Derived x) { return Derived(log(x.c)); }
  friend Derived pow(Derived base, float exp) {
    return Derived(pow(base.c, exp));
  }

 protected:
  Derived& self() { return (Derived&)(*this); }
  Coefficients c;
};

inline const mat3 XYZ2RGBMatrix =
    mat3(vec3(3.2405, -0.9693, 0.0557), vec3(-1.5372, 1.8760, -0.2040),
         vec3(-0.4985, 0.0416, 1.0573));
inline const mat3 RGB2XYZMatrix = inverse(XYZ2RGBMatrix);

struct XYZSpectrum;
struct RGBSpectrum;

using Spectrum = RGBSpectrum;

struct XYZSpectrum : CoefficientSpectrum<3, XYZSpectrum> {
  using Base = CoefficientSpectrum<3, XYZSpectrum>;
  using Base::Base;

  explicit operator RGBSpectrum() const;
  vec3 toRGB() { return XYZ2RGBMatrix * c; }
};

struct RGBSpectrum : CoefficientSpectrum<3, RGBSpectrum> {
  using Base = CoefficientSpectrum<3, RGBSpectrum>;
  using Base::Base;

  explicit operator XYZSpectrum() const {
    return XYZSpectrum(RGB2XYZMatrix * c);
  }
  vec3 toRGB() { return c; }
};

inline XYZSpectrum::operator RGBSpectrum() const {
  return RGBSpectrum(XYZ2RGBMatrix * c);
}

}  // namespace pine
