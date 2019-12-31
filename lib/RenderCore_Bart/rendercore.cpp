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
	// Only add meshes that are not area lights. We leave area lights invisible for now.
	if (triangleData->ltriIdx == -1) {
		Mesh* mesh = new Mesh();

		mesh->vertices = new float4[vertexCount];
		mesh->vcount = vertexCount;
		memcpy(mesh->vertices, vertexData, vertexCount * sizeof(float4));

		mesh->triangles = new CoreTri[vertexCount / 3];
		memcpy(mesh->triangles, triangleData, (vertexCount / 3) * sizeof(CoreTri));

		raytracer.scene.meshes.push_back(mesh);

		raytracer.top_level_bvh.AddBVH(new BVH2(mesh));
	}
}

void RenderCore::SetInstance(const int instanceIdx, const int meshIdx, const mat4& matrix) {
}

void RenderCore::SetTextures(const CoreTexDesc* tex, const int textureCount) {
	
}

void RenderCore::SetMaterials(CoreMaterial* mat, const int materialCount) {
	cout << "SetMaterials" << endl;
	for (int i = 0; i < materialCount; i++)
	{
		Material* m;
		if (i < raytracer.scene.matList.size()) m = raytracer.scene.matList[i];
		else raytracer.scene.matList.push_back(m = new Material());
		
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
	// Add area lights to scene
	for (int i = 0; i < areaLightCount; i++) {
		if (raytracer.scene.areaLights.size() > i) { continue; }
		AreaLight* l;
		float3 v0 = areaLights[i].vertex0;
		float3 v1 = areaLights[i].vertex1;
		float3 v2 = areaLights[i].vertex2;
		float3 N = areaLights[i].N;
		float3 c = areaLights[i].centre;
		float A = areaLights[i].area;
		float3 rad = areaLights[i].radiance;
		raytracer.scene.areaLights.push_back(l = new AreaLight(v0, v1, v2, N, c, A, rad));
		cout << "Light " << i << " area: " << A << endl;
	}

	// Add point lights to scene
	for (int i = 0; i < pointLightCount; i++) {
		if (raytracer.scene.pointLights.size() > i) { continue; }
		PointLight* l;
		float3 pos = pointLights[i].position;
		float3 rad = pointLights[i].radiance;
		raytracer.scene.pointLights.push_back(l = new PointLight(pos, rad));
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
	if (raytracer.frameCount == 0) {
		cout << endl;
		cout << "Mesh count: " << raytracer.scene.meshes.size() << endl;

		raytracer.top_level_bvh.Rebuild();
		raytracer.frameCount++;

		//DWORD trace_start = GetTickCount();
		//raytracer.ConstructBVH();
		//// raytracer.PrintBVH(raytracer.pool[0]); cout << endl;
		//raytracer.frameCount++;
		//DWORD trace_time = GetTickCount() - trace_start;
		//cout << "Finished building BVH in " << trace_time / 1000.0f << " seconds." << endl;
		//cout << endl;
	}
	screen->Clear();
	if (converge) { raytracer.accumulator.Rebuild(screen->width, screen->height); }

	int depth = 8; // Maximum ray recursion depth

	int nx = screen->width;
	int ny = screen->height;

	float dx = 1.0f / (nx - 1);
	float dy = 1.0f / (ny - 1);

	raytracer.accumulator.Increment();

	DWORD trace_start = GetTickCount();
	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			float rx = Rand(dx), ry = Rand(dy);
			float3 sx = (x * dx + rx) * (view.p2 - view.p1);	// Screen x
			float3 sy = (y * dy + dy) * (view.p3 - view.p1);	// Screen y
			float3 P = view.p1 + sx + sy;						// Point on screen
			float3 D = P - view.pos;							// Ray direction, normalized in Ray() constructor
			float3 c = raytracer.Color(view.pos, D, depth);		// Color vector
			// float3 c = raytracer.ColorDebugBVH(view.pos, D); // BVH Debug mode
			raytracer.accumulator.addPixel(x, y, c);

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