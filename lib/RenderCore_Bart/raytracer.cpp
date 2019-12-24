#include "core_settings.h"
#include <iostream>

uint ScaleColor(uint c, int scale)
{
	unsigned int rb = (((c & 0xff00ff) * scale) >> 8) & 0xff00ff;
	unsigned int g = (((c & 0xff00) * scale) >> 8) & 0xff00;
	return rb + g;
}

void Accumulator::Rebuild(int width, int height) {
	frame.clear();
	frame.resize(width * height, make_float3(0, 0, 0));
	frame_count = 0;
	w = width;
}

float3 Triangle::RandomPoint() {
	float a = Rand(1.0f), b = Rand(1.0f);
	if (a + b > 1.0f) { a = 1 - a; b = 1 - b; }
	return vertex0 + a * (vertex1 - vertex0) + b * (vertex2 - vertex0);
}

float3 RayTracer::Color(float3 O, float3 D, uint depth, bool outside) {
	float3 color = make_float3(0, 0, 0);
	if (depth <= 0) { return color; }
	D = normalize(D);

	Ray ray = Ray(O, D);
	float t = FLT_MAX;
	CoreTri triangle;

	// The ray hit the skydome, we don't do lighting
	if (!IntersectTopBVH(ray, triangle, t)) {
		float u = 1 + atan2(D.x, -D.z) * INVPI;
		float v = acos(D.y) * INVPI;

		unsigned long long width = round(u * scene.skyHeight);
		unsigned long long height = round(v * scene.skyHeight);

		return clamp(scene.skyDome[min(height * scene.skyHeight * 2 + width, scene.skyDome.size() - 1)], 0.0f, 1.0f);
	}

	float specularity = scene.matList[triangle.material]->specularity;
	float transmission = scene.matList[triangle.material]->transmission;

	// Normalize specularity and transmission if they sum > 1
	if (specularity + transmission > 1) {
		specularity /= specularity + transmission;
		transmission /= specularity + transmission;
	}

	float diffusion = 1.0f - specularity - transmission;
	float3 N = make_float3(triangle.Nx, triangle.Ny, triangle.Nz); // Normalized

	// Transmission
	if (transmission > 0.01f) {
		float H1 = 1.0f, H2 = scene.matList[triangle.material]->IOR;

		if (!outside) {		// If the ray is exiting a transmissive material
			swap(H1, H2);	// Swap the IOR
			N = -N;			// Invert the normal
		}

		float cosTi = dot(-D, N);
		float sin2Tt = (H1 / H2) * (H1 / H2) * (1.0f - (cosTi * cosTi));

		// Calculate using Schlick's approximation
		float fresnel;
		float R0 = ((H1 - H2) / (H1 + H2)) * ((H1 - H2) / (H1 + H2));
		if (H1 > H2 && sin2Tt > 1.0f) { // If there is TIR
			fresnel = 1.0f;

			// Testing purposes, colors area of TIR green
			// return make_float3(0, 1, 0);
		}
		else {
			float x = 1.0f - cosTi;
			fresnel = R0 + (1.0f - R0) * x * x * x * x * x;
		}

		specularity = specularity + transmission * fresnel;
		transmission = transmission * (1.0f - fresnel);

		float H = H1 / H2;
		float S = sqrt(1 - sin2Tt);

		color += transmission * Color(ray.point(t) - N * 2 * EPSILON, H * D + (H * cosTi - S) * N, depth - 1, !outside);
	}

	if (specularity > 0.01f) { color += specularity * Color(ray.point(t), D - 2 * (D * N) * N, depth - 1, outside); }
	if (diffusion > 0.01f) { color += diffusion * Illumination(scene.matList[triangle.material]->diffuse, ray.point(t)); }

	return color;
}

float3 RayTracer::ColorDebugBVH(float3 O, float3 D) {
	float3 color = make_float3(0, 1.0f, 0);
	D = normalize(D);

	Ray ray = Ray(O, D);
	CoreTri triangle;
	float t = FLT_MAX;
	int c = 0;

	if (IntersectTopBVH(ray, triangle, t, &c)) {
		// return make_float3(1, 1, 1);
	}

	if (c == 0) { return make_float3(0, 0, 0); } // No BVH intersection, black

	float delta = 0.004f;
	color += make_float3(c * delta, c * -delta, 0);

	return clamp(color, 0, 1);
}

float3 RayTracer::Illumination(float3 color, float3 O) {
	float3 light_color = make_float3(0, 0, 0);

	// Point lights
	for (int i = 0; i < scene.pointLights.size(); i++) {
		float3 direction = scene.pointLights[i]->position - O;
		Ray shadow_ray = Ray(O, direction);

		float t_light = length(fabs(direction));
		float t = FLT_MAX;
		CoreTri triangle;

		IntersectTopBVH(shadow_ray, triangle, t);
		if (t_light <= t) {
			light_color += scene.pointLights[i]->radiance / (t_light * t_light);
		}
	}

	// Monte Carlo area lighting
	int N = 1;
	for (int n = 0; n < N; n++) {
		uint i = RandomUInt() % scene.areaLights.size();
		float p = scene.areaLights[i]->area / total_light_area; // Probability of hitting this light

		float3 direction = scene.areaLights[i]->RandomPoint() - O;
		Ray shadow_ray = Ray(O, direction);

		float t_light = length(fabs(direction));
		float t = FLT_MAX;
		CoreTri triangle;

		IntersectTopBVH(shadow_ray, triangle, t);
		if (t_light <= t) {
			light_color += ((scene.areaLights[i]->radiance / (t_light * t_light)) / p) / N;
		}
	}

	return clamp(color * light_color, 0, 1);
}

void RayTracer::ConstructBVH() {
	// Fill the triangle pointer array
	triangle_pointers = new CoreTri[N];

	int idx = 0;
	for (Mesh mesh : scene.meshes) {
		for (int i = 0; i < mesh.vcount / 3; i++) {
			triangle_pointers[idx++] = mesh.triangles[i];
		}
	}

	// Build a BVH over every mesh
	pool = new BVH[2 * N - 1 + scene.meshes.size()];

	for (Mesh mesh : scene.meshes) {
		BVH& root = pool[pool_ptr];
		top_bvh.root_nodes.push_back(pool_ptr);	// Save the root node pointer in the top level BVH
		pool_ptr += 2;							// Skip node after root for alignment

		root.first = geometry_ptr;
		root.count = mesh.vcount / 3;

		geometry_ptr += mesh.vcount / 3;

		RecursiveSplitBVH(root);
	}

	UpdateBounds();
}

bool RayTracer::SplitBVH(BVH& bvh) {
	if (bvh.count < 4) { return false; } // Leaves have at least 2 primitives in them

	CoreTri* left_split = new CoreTri[bvh.count];
	CoreTri* right_split = new CoreTri[bvh.count];
	int left_count;
	
	// If there is value in splitting
	if (PartitionBinningSAH(bvh, left_split, right_split, left_count)) {
		// Update triangle pointer array
		for (int i = 0; i < left_count; i++) {
			triangle_pointers[bvh.first + i] = left_split[i];
		}

		for (int i = 0; i < bvh.count - left_count; i++) {
			triangle_pointers[bvh.first + left_count + i] = right_split[i];
		}

		// Split the BVH in two
		BVH& left = pool[pool_ptr];
		BVH& right = pool[pool_ptr + 1];

		left.first = bvh.first;
		left.count = left_count;

		right.first = bvh.first + left_count;
		right.count = bvh.count - left_count;

		bvh.count = 0;			// This node is not a leaf anymore
		bvh.first = pool_ptr;	// Point to its left child

		delete[] left_split;
		delete[] right_split;

		return true;
	}

	delete[] left_split;
	delete[] right_split;

	return false; // We did not split
}

void RayTracer::RecursiveSplitBVH(BVH& bvh) {
	if (SplitBVH(bvh)) {
		BVH& left = pool[pool_ptr++];
		BVH& right = pool[pool_ptr++];

		RecursiveSplitBVH(left);
		RecursiveSplitBVH(right);
	}
}

float RayTracer::SplitCost(CoreTri* primitives, int first, int count) {
	float3 min_b = make_float3(0, 0, 0);
	float3 max_b = make_float3(0, 0, 0);

	// Calculate AABB bounds
	for (int i = first; i < first + count; i++) {
		CoreTri& tri = primitives[i];
		float3 tri_min_bound = fminf(fminf(tri.vertex0, tri.vertex1), tri.vertex2);
		float3 tri_max_bound = fmaxf(fmaxf(tri.vertex0, tri.vertex1), tri.vertex2);

		min_b = fminf(min_b, tri_min_bound);
		max_b = fmaxf(max_b, tri_max_bound);
	}

	float3 dims = max_b - min_b; // Dimensions of the AABB
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z; // AABB surface area

	return count * area;
}

void RayTracer::UpdateBounds() {
	for (int i = pool_ptr - 1; i >= 0; i--) {
		// Unused BVH
		if (pool[i].count == -1) {
			continue;
		}

		// Node, inheret AABB from children
		if (pool[i].count == 0) {
			BVH& left = pool[pool[i].first];
			BVH& right = pool[pool[i].first + 1];

			float3 min_bound = fminf(left.min_bound, right.min_bound);
			float3 max_bound = fmaxf(left.max_bound, right.max_bound);

			pool[i].min_bound = min_bound;
			pool[i].max_bound = max_bound;

			continue;
		}

		// Leaf, calculate AABB from primitives
		float3 min_b = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
		float3 max_b = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		for (int j = pool[i].first; j < pool[i].first + pool[i].count; j++) {
			CoreTri& tri = triangle_pointers[j];

			float3 tri_min_bound = fminf(fminf(tri.vertex0, tri.vertex1), tri.vertex2);
			float3 tri_max_bound = fmaxf(fmaxf(tri.vertex0, tri.vertex1), tri.vertex2);

			min_b = fminf(min_b, tri_min_bound);
			max_b = fmaxf(max_b, tri_max_bound);
		}

		pool[i].min_bound = min_b; pool[i].max_bound = max_b;
	}

	// Update top level BVH bounds
	top_bvh.min_bound = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
	top_bvh.max_bound = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (int i = 0; i < top_bvh.root_nodes.size(); i++) {
		top_bvh.min_bound = fminf(top_bvh.min_bound, pool[top_bvh.root_nodes[i]].min_bound);
		top_bvh.max_bound = fmaxf(top_bvh.max_bound, pool[top_bvh.root_nodes[i]].max_bound);
	}
}

bool RayTracer::IntersectTriangle(const Ray& ray, const CoreTri& triangle, float& t) {
	float3 vertex0 = triangle.vertex0;
	float3 vertex1 = triangle.vertex1;
	float3 vertex2 = triangle.vertex2;
	float3 edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = cross(ray.D, edge2);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0 / a;
	s = ray.O - vertex0;
	u = f * dot(s, h);
	if (u < 0.0 || u > 1.0)
		return false;
	q = cross(s, edge1);

	v = f * dot(ray.D, q);
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

bool RayTracer::IntersectBVH(const Ray& ray, const BVH& bvh, float& t) {
	// Taken from: https://gamedev.stackexchange.com/a/18459

	float3 dir_frac = make_float3(0, 0, 0);

	// D is the unit direction of the ray
	dir_frac.x = 1.0f / ray.D.x;
	dir_frac.y = 1.0f / ray.D.y;
	dir_frac.z = 1.0f / ray.D.z;

	// O is the origin of ray
	float t1 = (bvh.min_bound.x - ray.O.x) * dir_frac.x;
	float t2 = (bvh.max_bound.x - ray.O.x) * dir_frac.x;
	float t3 = (bvh.min_bound.y - ray.O.y) * dir_frac.y;
	float t4 = (bvh.max_bound.y - ray.O.y) * dir_frac.y;
	float t5 = (bvh.min_bound.z - ray.O.z) * dir_frac.z;
	float t6 = (bvh.max_bound.z - ray.O.z) * dir_frac.z;

	float t_min = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
	float t_max = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

	// If tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
	if (t_max < 0) {
		t = t_max;
		return false;
	}

	// If tmin > tmax, ray doesn't intersect AABB
	if (t_min > t_max) {
		t = t_max;
		return false;
	}

	t = t_min;
	return true;
}

bool RayTracer::RecursiveIntersectBVH(const Ray& ray, const BVH& bvh, CoreTri& tri, float& t, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	float initial_t = t;
	float found_t = t;

	if (bvh.count > 0) { // If the node is a leaf
		for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
			if (IntersectTriangle(ray, triangle_pointers[i], found_t) && found_t < t) {
				t = found_t;
				tri = triangle_pointers[i];
			}
		}
	}
	else { // If the node is not a leaf
		float t_left = FLT_MAX;
		float t_right = FLT_MAX;

		// Only traverse children that are actually intersected
		if (IntersectBVH(ray, pool[bvh.first], t_left)) {
			if (IntersectBVH(ray, pool[bvh.first+1], t_right)) {
				if (t_left < t_right) { // Traverse the closest child first
					RecursiveIntersectBVH(ray, pool[bvh.first], tri, t, c);
					RecursiveIntersectBVH(ray, pool[bvh.first+1], tri, t, c);
				}
				else {
					RecursiveIntersectBVH(ray, pool[bvh.first+1], tri, t, c);
					RecursiveIntersectBVH(ray, pool[bvh.first], tri, t, c);
				}
			}
			else {
				RecursiveIntersectBVH(ray, pool[bvh.first], tri, t, c);
			}
		}
		else if (IntersectBVH(ray, pool[bvh.first+1], t_right)) {
			RecursiveIntersectBVH(ray, pool[bvh.first+1], tri, t, c);
		}
	}

	return t < initial_t; // If an intersection was found
}

bool RayTracer::IntersectTopBVH(const Ray& ray, CoreTri& tri, float& t, int* c) {
	// First check if we actually intersect the top level BVH
	float3 dir_frac = make_float3(0, 0, 0);

	// D is the unit direction of the ray
	dir_frac.x = 1.0f / ray.D.x;
	dir_frac.y = 1.0f / ray.D.y;
	dir_frac.z = 1.0f / ray.D.z;

	// O is the origin of ray
	float t1 = (top_bvh.min_bound.x - ray.O.x) * dir_frac.x;
	float t2 = (top_bvh.max_bound.x - ray.O.x) * dir_frac.x;
	float t3 = (top_bvh.min_bound.y - ray.O.y) * dir_frac.y;
	float t4 = (top_bvh.max_bound.y - ray.O.y) * dir_frac.y;
	float t5 = (top_bvh.min_bound.z - ray.O.z) * dir_frac.z;
	float t6 = (top_bvh.max_bound.z - ray.O.z) * dir_frac.z;

	float t_min = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
	float t_max = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

	if (t_max < 0) { return false; } // If tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
	if (t_min > t_max) { return false; } // If tmin > tmax, ray doesn't intersect AABB

	// Sort children based on the distance to their AABB
	// Children whose AABB is not intersected are disregarded
	vector<tuple<float, int>> traversal_order;

	for (int i = 0; i < top_bvh.root_nodes.size(); i++) {
		float aabb_t = FLT_MAX;
		if (IntersectBVH(ray, pool[top_bvh.root_nodes[i]], aabb_t)) {
			traversal_order.push_back(make_tuple(aabb_t, top_bvh.root_nodes[i]));
		}
	}

	sort(traversal_order.begin(), traversal_order.end());

	// Intersect children in order of proximity
	float initial_t = t;

	for (int i = 0; i < traversal_order.size(); i++) {
		if (get<0>(traversal_order[i]) < t) {
			RecursiveIntersectBVH(ray, pool[get<1>(traversal_order[i])], tri, t, c);
		}
	}

	return t < initial_t;
}

bool RayTracer::PartitionSAH(BVH& bvh, CoreTri* left, CoreTri* right, int& left_count) {
	float best_split_cost = FLT_MAX;
	float best_split_pos;
	char best_split_plane;

	left_count = 0;
	int right_count;

	// X split
	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = triangle_pointers[i];
		float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		left_count = 0;
		right_count = 0;

		for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
			CoreTri& tri2 = triangle_pointers[j];
			float3 tri2_center = (tri2.vertex0 + tri2.vertex1 + tri2.vertex2) / 3.0f;
			if (tri2_center.x + EPSILON < tri_center.x) {
				left[left_count++] = tri2;
			}
			else {
				right[right_count++] = tri2;
			}
		}

		float split_cost = SplitCost(left, 0, left_count) + SplitCost(right, 0, right_count);
		if (split_cost + EPSILON < best_split_cost) {
			best_split_cost = split_cost;
			best_split_plane = 'X';
			best_split_pos = tri_center.x;
		}
	}

	// Y split
	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = triangle_pointers[i];
		float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		left_count = 0;
		right_count = 0;

		for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
			CoreTri& tri2 = triangle_pointers[j];
			float3 tri2_center = (tri2.vertex0 + tri2.vertex1 + tri2.vertex2) / 3.0f;
			if (tri2_center.y + EPSILON < tri_center.y) {
				left[left_count++] = tri2;
			}
			else {
				right[right_count++] = tri2;
			}
		}

		float split_cost = SplitCost(left, 0, left_count) + SplitCost(right, 0, right_count);
		if (split_cost + EPSILON < best_split_cost) {
			best_split_cost = split_cost;
			best_split_plane = 'Y';
			best_split_pos = tri_center.y;
		}
	}

	// Z split
	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = triangle_pointers[i];
		float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		left_count = 0;
		right_count = 0;

		for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
			CoreTri& tri2 = triangle_pointers[j];
			float3 tri2_center = (tri2.vertex0 + tri2.vertex1 + tri2.vertex2) / 3.0f;
			if (tri2_center.z + EPSILON < tri_center.z) {
				left[left_count++] = tri2;
			}
			else {
				right[right_count++] = tri2;
			}
		}

		float split_cost = SplitCost(left, 0, left_count) + SplitCost(right, 0, right_count);
		if (split_cost + EPSILON < best_split_cost) {
			best_split_cost = split_cost;
			best_split_plane = 'Z';
			best_split_pos = tri_center.z;
		}
	}

	// If there is no value in splitting, don't split
	if (SplitCost(triangle_pointers, bvh.first, bvh.count) - EPSILON < best_split_cost) {
		return false;
	}

	// Define the best splits
	left_count = 0;
	right_count = 0;

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = triangle_pointers[i];
		float3 center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		// X split
		if (best_split_plane == 'X') {
			if (center.x < best_split_pos) {
				left[left_count++] = tri;
			}
			else {
				right[right_count++] = tri;
			}
		}

		// Y split
		if (best_split_plane == 'Y') {
			if (center.y < best_split_pos) {
				left[left_count++] = tri;
			}
			else {
				right[right_count++] = tri;
			}
		}

		// Z split
		if (best_split_plane == 'Z') {
			if (center.z < best_split_pos) {
				left[left_count++] = tri;
			}
			else {
				right[right_count++] = tri;
			}
		}
	}

	return true;
}

bool RayTracer::PartitionBinningSAH(BVH& bvh, CoreTri* left, CoreTri* right, int& left_count) {
	float base_cost = SplitCost(triangle_pointers, bvh.first, bvh.count);	// Cost of current BVH before splitting, according to SAH heuristic
	float3 c_min_bound = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);			// Triangle centroids AABB min bound
	float3 c_max_bound = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);			// Triangle centroids AABB max bound

	left_count = 0;
	int right_count;
	int bin_count = 8; // Must be larger than 1

	float best_split_cost = FLT_MAX;
	float best_split_pos;
	char best_split_plane;

	// Calculate the centroid AABB
	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = triangle_pointers[i];
		float3 center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		c_min_bound = fminf(c_min_bound, center);
		c_max_bound = fmaxf(c_max_bound, center);
	}

	// X split
	if (c_min_bound.x != c_max_bound.x) { // If all primitives have the same X position, don't split over X axis
		float interval = (c_max_bound.x - c_min_bound.x) / bin_count;
		for (int i = 0; i < bin_count; i++) {
			float pos = c_min_bound.x + i * interval;

			left_count = 0;
			right_count = 0;

			for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
				CoreTri& tri = triangle_pointers[j];
				float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

				if (tri_center.x <= pos) {
					left[left_count++] = tri;
				}
				else {
					right[right_count++] = tri;
				}
			}

			float split_cost = SplitCost(left, 0, left_count) + SplitCost(right, 0, right_count);
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = 'X';
				best_split_pos = pos;
			}
		}
	}

	// Y split
	if (c_min_bound.y != c_max_bound.y) {
		float interval = (c_max_bound.y - c_min_bound.y) / bin_count;
		for (int i = 0; i < bin_count; i++) {
			float pos = c_min_bound.y + i * interval;

			left_count = 0;
			right_count = 0;

			for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
				CoreTri& tri = triangle_pointers[j];
				float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

				if (tri_center.y <= pos) {
					left[left_count++] = tri;
				}
				else {
					right[right_count++] = tri;
				}
			}

			float split_cost = SplitCost(left, 0, left_count) + SplitCost(right, 0, right_count);
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = 'Y';
				best_split_pos = pos;
			}
		}
	}

	// Z split
	if (c_min_bound.z != c_max_bound.z) {
		float interval = (c_max_bound.z - c_min_bound.z) / bin_count;
		for (int i = 0; i < bin_count; i++) {
			float pos = c_min_bound.z + i * interval;

			left_count = 0;
			right_count = 0;

			for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
				CoreTri& tri = triangle_pointers[j];
				float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

				if (tri_center.z <= pos) {
					left[left_count++] = tri;
				}
				else {
					right[right_count++] = tri;
				}
			}

			float split_cost = SplitCost(left, 0, left_count) + SplitCost(right, 0, right_count);
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = 'Z';
				best_split_pos = pos;
			}
		}
	}

	// If there is no value in splitting, don't split
	if (base_cost - EPSILON < best_split_cost) {
		return false;
	}

	// Define the best splits
	left_count = 0;
	right_count = 0;

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = triangle_pointers[i];
		float3 center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		// X split
		if (best_split_plane == 'X') {
			if (center.x <= best_split_pos) {
				left[left_count++] = tri;
			}
			else {
				right[right_count++] = tri;
			}
		}

		// Y split
		if (best_split_plane == 'Y') {
			if (center.y <= best_split_pos) {
				left[left_count++] = tri;
			}
			else {
				right[right_count++] = tri;
			}
		}

		// Z split
		if (best_split_plane == 'Z') {
			if (center.z <= best_split_pos) {
				left[left_count++] = tri;
			}
			else {
				right[right_count++] = tri;
			}
		}
	}

	// Debug, if we somehow end up with all primitives on one side of the split
	if (left_count == 0 || right_count == 0) {
		cout << "Left count: " << left_count << endl;
		cout << "Right count: " << right_count << endl;
		cout << "Axis: " << best_split_plane << endl;
		cout << "Position: " << best_split_pos << endl;
		cout << "Min bounds: " << c_min_bound.x << ", " << c_min_bound.y << ", " << c_min_bound.z << endl;
		cout << "Max bounds: " << c_max_bound.x << ", " << c_max_bound.y << ", " << c_max_bound.z << endl;
		cout << "Base cost: " << base_cost - EPSILON << endl;
		cout << "Best cost: " << best_split_cost << endl;
		cout << "c_min_bound.z <= pos: " << ((c_min_bound.z <= best_split_pos) ? "true" : "false") << endl;
		cout << endl;
	}

	return true;
}

void RayTracer::PrintBVH(const BVH& bvh) {
	if (bvh.count > 0) {
		cout << "[" << bvh.count << "]";
	}
	else {
		cout << "[";
		PrintBVH(pool[bvh.first]);
		PrintBVH(pool[bvh.first+1]);
		cout << "]";
	}
}

Scene::~Scene()
{
	for (auto mat : matList) delete mat;
	for (auto pointLight : pointLights) delete pointLight;
	for (auto areaLight : areaLights) delete areaLight;
}

RayTracer::~RayTracer() {
	delete[] triangle_pointers;
	delete[] pool;
}

// -----------------------------------------------------------
// static data for the ray tracer
// -----------------------------------------------------------
Scene RayTracer::scene;

