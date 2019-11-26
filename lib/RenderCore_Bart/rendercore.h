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

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Minimalistic mesh storage.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
class Mesh
{
public:
	float4* vertices = 0;							// vertex data received via SetGeometry
	int vcount = 0;									// vertex count
	CoreTri* triangles = 0;							// 'fat' triangle data
};

class Ray
{
public:
	Ray(const float3& o, const float3 d) { O = o; D = normalize(d); }
	const float3 origin() { return O; }
	const float3 direction() { return D; }
	const float3 point(float t) { return O + t * D; }

	bool IntersectsTriangle(const CoreTri& triangle, float& t) {
		float3 vertex0 = triangle.vertex0;
		float3 vertex1 = triangle.vertex1;
		float3 vertex2 = triangle.vertex2;
		float3 edge1, edge2, h, s, q;
		float a, f, u, v;
		edge1 = vertex1 - vertex0;
		edge2 = vertex2 - vertex0;
		h = cross(D, edge2);
		a = dot(edge1, h);
		if (a > -EPSILON && a < EPSILON)
			return false;    // This ray is parallel to this triangle.
		f = 1.0 / a;
		s = O - vertex0;
		u = f * dot(s, h);
		if (u < 0.0 || u > 1.0)
			return false;
		q = cross(s, edge1);
		v = f * dot(D, q);
		if (v < 0.0 || u + v > 1.0)
			return false;
		// At this stage we can compute t to find out where the intersection point is on the line.
		float tt = f * dot(edge2, q);
		if (tt > EPSILON && tt < 1 / EPSILON) // ray intersection
		{
			t = tt;
			return true;
		}
		else // This means that there is a line intersection but not a ray intersection.
			return false;
	}

private:
	float3 O;
	float3 D;
};

//  +-----------------------------------------------------------------------------+
//  |  RenderCore                                                                 |
//  |  Encapsulates device code.                                            LH2'19|
//  +-----------------------------------------------------------------------------+
class RenderCore
{
public:
	// methods
	void Init();
	void SetTarget( GLTexture* target );
	void SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangles, const uint* alphaFlags = 0 );
	void Render( const ViewPyramid& view, const Convergence converge, const float brightness, const float contrast );
	void Shutdown();
	// internal methods
private:
	// data members
	Bitmap* screen = 0;								// temporary storage of RenderCore output; will be copied to render target
	int targetTextureID = 0;						// ID of the target OpenGL texture
	vector<Mesh> meshes;							// mesh data storage
public:
	CoreStats coreStats;							// rendering statistics
};

} // namespace lh2core

// EOF