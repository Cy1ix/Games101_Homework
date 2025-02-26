#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{

    // Change the definition here to change resolution
    Scene scene(800, 800);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

    Material* gold = new Material(MICROFACET, Vector3f(0.0f));
    gold->Ks = Vector3f(1.000f, 0.782f, 0.344f);
    gold->roughness = 0.3f;
    Material* silver = new Material(MICROFACET, Vector3f(0.0f));
    silver->Ks = Vector3f(0.97f, 0.96f, 0.91f);
    silver->roughness = 0.3f;
    Material* copper = new Material(MICROFACET, Vector3f(0.0f));
    copper->Ks = Vector3f(0.97f, 0.74f, 0.62f);
    copper->roughness = 0.3f;

    Material* mirror = new Material(MICROFACET, Vector3f(0.0f));
    mirror->Ks = Vector3f(1.0f, 1.0f, 1.0f);
    mirror->roughness = 0.05f; // minimun value here!

    float radius = 75;
    Sphere microfacet_sphere(Vector3f(278 + 125, radius, 100), radius, silver);
    scene.Add(&microfacet_sphere);

    MeshTriangle floor("./Assignment7/models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("./Assignment7/models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("./Assignment7/models/cornellbox/tallbox.obj", white);
    MeshTriangle left("./Assignment7/models/cornellbox/left.obj", red);
    MeshTriangle right("./Assignment7/models/cornellbox/right.obj", green);
    MeshTriangle light_("./Assignment7/models/cornellbox/light.obj", light);

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene, 16);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}