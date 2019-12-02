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

using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init()
{
	// initialize core
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget( GLTexture* target )
{
	// synchronize OpenGL viewport
	targetTextureID = target->ID;
	if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
	delete screen;
	screen = new Bitmap( target->width, target->height );
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData, const uint* alphaFlags )
{	
	Mesh newMesh;
	// copy the supplied vertices; we cannot assume that the render system does not modify
	// the original data after we leave this function.
	newMesh.vertices = new float4[vertexCount];
	newMesh.vcount = vertexCount;
	memcpy( newMesh.vertices, vertexData, vertexCount * sizeof( float4 ) );
	// copy the supplied 'fat triangles'
	newMesh.triangles = new CoreTri[vertexCount / 3];
	memcpy( newMesh.triangles, triangleData, (vertexCount / 3) * sizeof( CoreTri ) );
	raytracer.scene.meshes.push_back( newMesh );
}

void RenderCore::SetTextures(const CoreTexDesc* tex, const int textureCount) {
	
}

void RenderCore::SetMaterials(CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount) {
	for (int i = 0; i < materialCount; i++)
	{
		Material* m;
		if (i < raytracer.scene.matList.size()) m = raytracer.scene.matList[i];
		else raytracer.scene.matList.push_back(m = new Material());
		int texID = matEx[i].texture[TEXTURE0];
		if (texID == -1)
		{
			float r = mat[i].diffuse_r, g = mat[i].diffuse_g, b = mat[i].diffuse_b;
			m->diffuse = make_float3(r, g, b);
			// m->diffuse = ((int)(b * 255.0f) << 16) + ((int)(g * 255.0f) << 8) + (int)(r * 255.0f);
		}
		else {
			// TODO: textures, replacement code below
			m->diffuse = make_float3(1, 1, 1);
		}
	}
}

void RenderCore::SetLights(const CoreLightTri* areaLights, const int areaLightCount, const CorePointLight* pointLights, const int pointLightCount,
	const CoreSpotLight* spotLights, const int spotLightCount, const CoreDirectionalLight* directionalLights, const int directionalLightCount) {
	// Add point lights to scene
	for (int i = 0; i < pointLightCount; i++) {
		PointLight* l;
		float3 pos = pointLights[i].position;
		float3 rad = pointLights[i].radiance;
		raytracer.scene.pointLights.push_back(l = new PointLight(pos, rad));
	}
	
	cout << "\nSetLights called." << endl;
	for (int i = 0; i < raytracer.scene.pointLights.size(); i++) {
		cout << "  Point light " << i << endl;
		cout << "    Position: " << raytracer.scene.pointLights[i]->position.x << ", " << raytracer.scene.pointLights[i]->position.y << ", " << raytracer.scene.pointLights[i]->position.z << endl;
		cout << "    Radiance: " << raytracer.scene.pointLights[i]->radiance.x << ", " << raytracer.scene.pointLights[i]->radiance.y << ", " << raytracer.scene.pointLights[i]->radiance.z << endl;
	}
}

void RenderCore::SetSkyData(const float3* pixels, const uint width, const uint height) {
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
void RenderCore::Render( const ViewPyramid& view, const Convergence converge )
{
	screen->Clear();

	int nx = screen->width;
	int ny = screen->height;

	float dx = 1.0f / (nx - 1);
	float dy = 1.0f / (ny - 1);

	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			float3 sx = x * dx * (view.p2 - view.p1);		// Screen x
			float3 sy = y * dy * (view.p3 - view.p1);		// Screen y
			float3 P = view.p1 + sx + sy;					// Point on screen
			float3 D = P - view.pos;						// Ray direction
			float3 c = raytracer.Color(view.pos, D, 1);		// Color vector

			uint color = ((int)(c.z * 255.0f) << 16) + ((int)(c.y * 255.0f) << 8) + (int)(c.x * 255.0f);
			screen->Plot(x, y, color);
		}
	}

	// copy pixel buffer to OpenGL render target texture
	glBindTexture(GL_TEXTURE_2D, targetTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown()
{
	delete screen;
}

// EOF