/* rendercore.h - Copyright 2019 Utrecht University

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

#pragma once

namespace lh2core
{

class RenderCore
{
public:
	// methods
	void Init();
	void SetTarget( GLTexture* target );
	void SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangles, const uint* alphaFlags = 0 );
	void SetTextures(const CoreTexDesc* tex, const int textureCount);
	void SetMaterials(CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount);
	void Render( const ViewPyramid& view, const Convergence converge, const float brightness, const float contrast );
	void Shutdown();
	// internal methods
private:
	// data members
	Bitmap* screen = 0;								// temporary storage of RenderCore output; will be copied to render target
	int targetTextureID = 0;						// ID of the target OpenGL texture
	vector<Mesh> meshes;							// mesh data storage
	RayTracer raytracer;
public:
	CoreStats coreStats;							// rendering statistics
};

} // namespace lh2core

// EOF