#include <cmath>

float f(float x) {
  return x / (powf(x, 3) + 0.1f);
}

float pdf_g(float x) {
  return x * 2;
}
float pdf_h(float x) {
  return (1 - x) * 2;
}

float inverse_cdf_g(float y) {
  return sqrtf(y);
}
float inverse_cdf_h(float y) {
  return sqrtf(y);
}
float randf() {
  return float(rand()) / RAND_MAX;
}

int main(){

  

}