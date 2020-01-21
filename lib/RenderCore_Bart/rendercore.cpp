/* rendercore.cpp - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "core_settings.h"
#include <iostream>
#include <iomanip>

using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init() {
	
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget( GLTexture* target ) {
	// synchronize OpenGL viewport
	targetTextureID = target->ID;
	if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
	delete screen;
	screen = new Bitmap( target->width, target->height );

	raytracer.accumulator.Rebuild(target->width, target->height);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData, const uint* alphaFlags ) {
	Mesh* mesh;

	if (meshIdx >= raytracer.scene.meshes.size()) { // New mesh
		raytracer.scene.meshes.push_back(mesh = new Mesh(vertexCount, triangleCount));
		mesh->bvh = new BVH2(mesh);
		raytracer.triangle_count += triangleCount;
	}

	mesh = raytracer.scene.meshes[meshIdx]; // If existing mesh, assume triangle count stays the same

	// Copy vertex data
	for (int i = 0; i < vertexCount; i++) {
		mesh->vertices[i] = vertexData[i];
	}

	// Copy triangle data
	for (int i = 0; i < triangleCount; i++) {
		mesh->triangles[i] = triangleData[i];
	}

	// (Re)build the BVH. It is added to the top-level BVH in SetInstance() if required.
	DWORD trace_start = GetTickCount();
	mesh->bvh->Rebuild(); // Geometry changed, rebuild BVH
	coreStats.bvhBuildTime += GetTickCount() - trace_start;
}

void RenderCore::SetInstance(const int instanceIdx, const int meshIdx, const mat4& matrix) {
	// End of instance stream, resize instance vector if instances were removed
	if (meshIdx == -1) {
		if (raytracer.scene.instances.size() > instanceIdx) {
			raytracer.scene.instances.resize(instanceIdx);
		}
		return;
	}

	if (instanceIdx >= raytracer.scene.instances.size()) { // New instance
		Instance* instance = new Instance(raytracer.scene.meshes[meshIdx], matrix);
		raytracer.scene.instances.push_back(instance);
		raytracer.top_level_bvh.AddInstance(instance);
	}
	else { // Existing instance
		raytracer.scene.instances[instanceIdx]->mesh = raytracer.scene.meshes[meshIdx];
		raytracer.scene.instances[instanceIdx]->transform = matrix;
	}
}

void RenderCore::SetTextures(const CoreTexDesc* tex, const int textureCount) {
	
}

void RenderCore::SetMaterials(CoreMaterial* mat, const int materialCount) {
	Material* m;

	for (int i = 0; i < materialCount; i++) {
		if (i < raytracer.scene.matList.size()) {
			m = raytracer.scene.matList[i];
		}
		else {
			raytracer.scene.matList.push_back(m = new Material());
		}
		
		int texID = mat[i].color.textureID;
		if (texID == -1) {
			float r = mat[i].color.value.x;
			float g = mat[i].color.value.y;
			float b = mat[i].color.value.z;
			m->diffuse = make_float3(r, g, b);
			m->specularity = mat[i].specular.value;
			m->transmission = mat[i].transmission.value;
			m->IOR = mat[i].eta.value;

			//cout << "  Material " << i << endl;
			//cout << "    Color:        " << r << ", " << g << ", " << b << endl;
			//cout << "    Specularity:  " << m->specularity << endl;
			//cout << "    Transmission: " << m->transmission << endl;
			//cout << "    IOR:          " << m->IOR << endl;
		}
		else {
			// TODO: textures
		}
	}
}

void RenderCore::SetLights(const CoreLightTri* areaLights, const int areaLightCount, const CorePointLight* pointLights, const int pointLightCount,
	const CoreSpotLight* spotLights, const int spotLightCount, const CoreDirectionalLight* directionalLights, const int directionalLightCount) {
	// Area lights
	if (raytracer.scene.areaLights.size() > areaLightCount) { raytracer.scene.areaLights.resize(areaLightCount); }

	for (int i = 0; i < areaLightCount; i++) {
		AreaLight* areaLight;
		if (i >= raytracer.scene.areaLights.size()) { raytracer.scene.areaLights.push_back(areaLight = new AreaLight()); }
		areaLight = raytracer.scene.areaLights[i];
		areaLight->vertex0 = areaLights[i].vertex0;
		areaLight->vertex1 = areaLights[i].vertex1;
		areaLight->vertex2 = areaLights[i].vertex2;
		areaLight->normal = areaLights[i].N;
		areaLight->center = areaLights[i].centre;
		areaLight->area = areaLights[i].area;
		areaLight->radiance = areaLights[i].radiance;
	}

	// Point lights
	if (raytracer.scene.pointLights.size() > pointLightCount) { raytracer.scene.pointLights.resize(pointLightCount); }

	for (int i = 0; i < pointLightCount; i++) {
		PointLight* pointLight;
		if (i >= raytracer.scene.pointLights.size()) { raytracer.scene.pointLights.push_back(pointLight = new PointLight()); }
		pointLight = raytracer.scene.pointLights[i];
		pointLight->position = pointLights[i].position;
		pointLight->radiance = pointLights[i].radiance;
	}
}

void RenderCore::SetSkyData(const float3* pixels, const uint width, const uint height, const mat4& worldToLight) {
	assert(width == 2 * height);

	raytracer.scene.skyDome.clear();
	raytracer.scene.skyDome.resize(width * height);
	memcpy(&raytracer.scene.skyDome[0], pixels, sizeof(float3) * width * height);

	raytracer.scene.skyWidth = width;
	raytracer.scene.skyHeight = height;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render( const ViewPyramid& view, const Convergence converge ) {
	// Print some stats on the first frame
	if (raytracer.print_stats) {
		cout << endl;
		cout << "Instance count: " << raytracer.scene.instances.size() << endl;
		cout << "Mesh count:     " << raytracer.scene.meshes.size() << endl;
		cout << "Triangle count: " << raytracer.triangle_count << endl;
		cout << "Area light count: " << raytracer.scene.areaLights.size() << endl;
		cout << "BVH build time: " << coreStats.bvhBuildTime / 1000.0f << " seconds\n" << endl;
		raytracer.print_stats = false;
	}

	screen->Clear();

	raytracer.top_level_bvh.Rebuild();												// Rebuild the top-level BVH
	if (converge) { raytracer.accumulator.Rebuild(screen->width, screen->height); }	// Rebuild the accumulator if not converging

	int nx = screen->width;
	int ny = screen->height;

	float dx = 1.0f / (nx - 1);
	float dy = 1.0f / (ny - 1);

	DWORD trace_start = GetTickCount();
	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			if (RandomFloat() < raytracer.P_SAMPLE) {
				float rx = Rand(dx), ry = Rand(dy);
				float3 sx = (x * dx + rx) * (view.p2 - view.p1);			// Screen x
				float3 sy = (y * dy + dy) * (view.p3 - view.p1);			// Screen y
				float3 P = view.p1 + sx + sy;								// Point on screen
				float3 D = P - view.pos;									// Ray direction, normalized in Ray() constructor
				float3 c = raytracer.Color(view.pos, D, raytracer.DEPTH);	// Color vector
				//float3 c = raytracer.ColorDebugBVH(view.pos, D, 0.01f);		// BVH Debug mode
				raytracer.accumulator.addPixel(x, y, make_float4(c, 1));
			}

			float3 cv = raytracer.accumulator.Pixel(x, y);
			uint color = ((int)(cv.z * 255.0f) << 16) + ((int)(cv.y * 255.0f) << 8) + (int)(cv.x * 255.0f);
			screen->Plot(x, y, color);
		}
	}

	DWORD trace_time = GetTickCount() - trace_start;
	cout << "\rRender time: " << setw(4) << std::setfill(' ') << trace_time / 1000.0f << "s per frame." << std::flush;

	// copy pixel buffer to OpenGL render target texture
	glBindTexture(GL_TEXTURE_2D, targetTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown() {
	delete screen;
}

// EOF