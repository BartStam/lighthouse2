#pragma once

namespace lh2core
{

class Accumulator {
public:
	Accumulator() = default;
	void Rebuild(int width, int height);
	float3 Pixel(int x, int y) { return frame[y * w + x] / frame_count; }
	void addPixel(int x, int y, float3 pixel) { frame[y * w + x] += pixel; }
	void Increment() { frame_count++; }
private:
	int w; // screen width
	int frame_count;
	vector<float3> frame;
};

class Scene {
public:
	Scene() = default;
	~Scene();

	vector<float3> skyDome;
	uint skyWidth;
	uint skyHeight;

	vector<Material*> matList;
	vector<AreaLight*> areaLights;
	vector<PointLight*> pointLights;
	vector<Mesh*> meshes;
};

class RayTracer {
public:
	RayTracer() = default;
	~RayTracer();
	
	static Scene scene;
	TopLevelBVH top_level_bvh;
	Accumulator accumulator;
	int frameCount = 0;
	int N;																// Total number of triangles in the scene

	float3 Color(float3 O, float3 D, uint depth, bool outside=true);	// Trace a ray and return its color
	float3 ColorDebugBVH(float3 O, float3 D);							// Trace primary rays and color based on how many BVHs they intersected
	float3 Illumination(float3 color, float3 O);						// Given a color at a location, scale it based on visible lighting
};

} // namespace lh2core
