#include <core/renderer.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

using namespace pine;
using namespace pybind11;

PYBIND11_MODULE(pinepy, m) {
  m.doc() = "Pine renderer";

  m.attr("Pi") = Pi;
  m.attr("E") = E;

  class_<vec2i>(m, "vec2i")
      .def(init<>())
      .def(init<int>())
      .def(init<int, int>())
      .def("__repr__", &toString<vec2i>)
      .def(self += self)
      .def(self -= self)
      .def(self *= self)
      .def(self /= self)
      .def(self *= int())
      .def(self /= int())
      .def(-self)
      .def(self + self)
      .def(self - self)
      .def(self * self)
      .def(self / self)
      .def(self * int())
      .def(self / int())
      .def(int() * self)
      .def("__getitem__",
           (const int& (vec2i::*)(int) const) & vec2i::operator[])
      .def("__setitem__", (int& (vec2i::*)(int)) & vec2i::operator[]);

  class_<vec3>(m, "vec3")
      .def(init<>())
      .def(init<float>())
      .def(init<float, float, float>())
      .def("__repr__", &toString<vec3>)
      .def(self += self)
      .def(self -= self)
      .def(self *= self)
      .def(self /= self)
      .def(self *= float())
      .def(self /= float())
      .def(-self)
      .def(self + self)
      .def(self - self)
      .def(self * self)
      .def(self / self)
      .def(self * float())
      .def(self / float())
      .def(float() * self)
      .def("__getitem__",
           (const float& (vec3::*)(int) const) & vec3::operator[])
      .def("__setitem__", (float& (vec3::*)(int)) & vec3::operator[]);

  class_<mat3>(m, "mat3")
      .def(init<>())
      .def(init<vec3, vec3, vec3>())
      .def(init<mat4>())
      .def("__repr__", &toString<mat3>)
      .def(self + self)
      .def(self - self)
      .def(self * vec3())
      .def(self * self)
      .def("__getitem__", (const vec3& (mat3::*)(int) const) & mat3::operator[])
      .def("__setitem__", (vec3 & (mat3::*)(int)) & mat3::operator[])
      .def("row", &mat3::row);

  class_<mat4>(m, "mat4")
      .def(init<>())
      .def(init<vec4, vec4, vec4, vec4>())
      .def("__repr__", &toString<mat4>)
      .def(self + self)
      .def(self - self)
      .def(self * vec4())
      .def(self * self)
      .def("__getitem__", (const vec4& (mat4::*)(int) const) & mat4::operator[])
      .def("__setitem__", (vec4 & (mat4::*)(int)) & mat4::operator[])
      .def("row", &mat4::row);

  class_<Image<vec3>>(m, "Image")
      .def(init<vec2i>())
      .def("size", &Image<vec3>::size)
      .def("width", &Image<vec3>::width)
      .def("height", &Image<vec3>::height)
      .def("nPixels", &Image<vec3>::nPixels)
      .def("__getitem__", (const vec3& (Image<vec3>::*)(vec2i) const) &
                              Image<vec3>::operator[])
      .def("__setitem__",
           (vec3 & (Image<vec3>::*)(vec2i)) & Image<vec3>::operator[]);

  class_<Shape>(m, "Shape").def(init<Sphere>()).def(init<Plane>());
  class_<Sphere>(m, "Sphere").def(init<vec3, float>());
  class_<Plane>(m, "Plane").def(init<vec3, vec3>());

  class_<Camera>(m, "Camera")
      .def(init<PinHoleCamera>())
      .def("getImage", &Camera::getImage)
      .def("imageSize", &Camera::imageSize);

  class_<PinHoleCamera>(m, "PinHoleCamera")
      .def(init<vec2i, vec3, vec3, float>());

  class_<Scene>(m, "Scene")
      .def(init<>())
      .def("getCamera", &Scene::getCamera)
      .def("setCamera", (void(Scene::*)(PinHoleCamera)) & Scene::setCamera)
      .def("addShape", (void(Scene::*)(Sphere)) & Scene::addShape)
      .def("addShape", (void(Scene::*)(Plane)) & Scene::addShape);

  class_<DebugIntegrator>(m, "DebugIntegrator").def(init<>());

  class_<Integrator>(m, "Integrator").def(init<DebugIntegrator>());
  implicitly_convertible<DebugIntegrator, Integrator>();

  class_<RayRenderer>(m, "RayRenderer")
      .def(init<Scene&, int, Integrator>())
      .def("render", &RayRenderer::render);

  m.def("dot", &dot<float, 3>);
  m.def("cross", &cross<float>);
  m.def("sqr", &sqr<float>);
  m.def("length", &length<vec3>);
  m.def("lengthSquared", &lengthSquared<vec3>);
  m.def("abs", &abs<float, 3>);
  m.def("sqrt", &sqrt<float, 3>);
  m.def("log", &log<float, 3>);
  m.def("exp", &exp<float, 3>);
  m.def("pow", &pow<float, 3>);
  m.def("clamp", &clamp<float>);
  m.def("clamp", &clamp<vec3>);
  m.def("lerp", &lerp<float, float>);
  m.def("lerp", &lerp<vec3, float>);
  m.def("normalize", &normalize<float, 3>);
  m.def("transpose", &transpose<float, 3, 3>);
  m.def("inverse", &inverse<float, 3>);
  m.def("inverse", &inverse<float, 4>);
  m.def("solve", &solve<float, 3>);
  m.def("solve", &solve<float, 4>);
  m.def("coordinateSystem", &coordinateSystem);
  m.def("lookAt", &lookAt, arg("lookFrom"), arg("lookAt"),
        arg("up") = vec3(0, 1, 0));
  m.def("saveImage", &saveImage);
}