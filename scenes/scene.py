from pinepy import *

scene = Scene()

camera = PinHoleCamera(vec2i(720, 480), vec3(0, 1, -4), vec3(0, 1, 0), Pi / 2)
scene.setCamera(camera)

for i in range(-6, 8, 2):
    scene.addShape(Sphere(vec3(i, 1, 2), 1))
scene.addShape(Plane(vec3(0, 0, 0), vec3(0, 1, 0)))

renderer = RayRenderer(scene, 1, Integrator(DebugIntegrator()))
renderer.render()

saveImage("../local/image.tga", scene.getCamera().getImage())