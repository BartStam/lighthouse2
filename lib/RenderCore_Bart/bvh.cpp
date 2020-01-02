#include "core_settings.h"
#include <iostream>

/*
	BVH
*/

BVH::~BVH() {
	
}

bool BVH::IntersectAABB(const Ray& ray, const BVHNode& bvh, float& t) {
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

/*
	BVH2
*/

BVH2::BVH2(Mesh* m) : mesh(m) {
	N = mesh->vcount / 3;

	// Fill the triangle pointer array
	tri_indices = new int[N];

	for (int i = 0; i < N; i++) {
		tri_indices[i] = i;
	}

	pool = new BVHNode[2 * N];
	BVHNode& root = pool[0];
	
	root.first = 0;
	root.count = N;

	SubdivideRecursively(root);
	UpdateBounds();
}

BVH2::~BVH2() {
	delete[] pool;
	delete[] tri_indices;
}

float BVH2::SplitCost(const int* indices, int first, int count) {
	float3 min_b = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
	float3 max_b = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// Calculate AABB bounds
	for (int i = first; i < first + count; i++) {
		const CoreTri& tri = mesh->triangles[indices[i]];
		float3 tri_min_bound = fminf(fminf(tri.vertex0, tri.vertex1), tri.vertex2);
		float3 tri_max_bound = fmaxf(fmaxf(tri.vertex0, tri.vertex1), tri.vertex2);

		min_b = fminf(min_b, tri_min_bound);
		max_b = fmaxf(max_b, tri_max_bound);
	}

	float3 dims = max_b - min_b; // Dimensions of the AABB
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z; // AABB surface area

	return count * area;
}

bool BVH2::Partition(BVHNode& bvh, int* indices, int* counts) {
	int bin_count = 8; // Must be larger than 1

	int* left = new int[bvh.count];
	int* right = new int[bvh.count];

	float base_cost = SplitCost(tri_indices, bvh.first, bvh.count);	// Cost of current BVH before splitting, according to SAH heuristic
	float3 c_min_bound = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);			// Triangle centroids AABB min bound
	float3 c_max_bound = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);			// Triangle centroids AABB max bound

	float best_split_cost = FLT_MAX;
	float best_split_pos;
	char best_split_plane;

	// Calculate the centroid AABB
	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = mesh->triangles[tri_indices[i]];
		float3 center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		c_min_bound = fminf(c_min_bound, center);
		c_max_bound = fmaxf(c_max_bound, center);
	}

	// X split
	if (c_min_bound.x != c_max_bound.x) { // If all primitives have the same X position, don't split over X axis
		float interval = (c_max_bound.x - c_min_bound.x) / bin_count;
		for (int i = 0; i < bin_count; i++) {
			float pos = c_min_bound.x + i * interval;

			counts[0] = 0; // Left count
			counts[1] = 0; // Right counts

			for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
				CoreTri& tri = mesh->triangles[tri_indices[j]];
				float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

				if (tri_center.x <= pos) {
					left[counts[0]++] = tri_indices[j];
				}
				else {
					right[counts[1]++] = tri_indices[j];
				}
			}

			float split_cost = SplitCost(left, 0, counts[0]) + SplitCost(right, 0, counts[1]);
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

			counts[0] = 0; // Left count
			counts[1] = 0; // Right counts

			for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
				CoreTri& tri = mesh->triangles[tri_indices[j]];
				float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

				if (tri_center.y <= pos) {
					left[counts[0]++] = tri_indices[j];
				}
				else {
					right[counts[1]++] = tri_indices[j];
				}
			}

			float split_cost = SplitCost(left, 0, counts[0]) + SplitCost(right, 0, counts[1]);
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

			counts[0] = 0; // Left count
			counts[1] = 0; // Right counts

			for (int j = bvh.first; j < bvh.first + bvh.count; j++) {
				CoreTri& tri = mesh->triangles[tri_indices[j]];
				float3 tri_center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

				if (tri_center.z <= pos) {
					left[counts[0]++] = tri_indices[j];
				}
				else {
					right[counts[1]++] = tri_indices[j];
				}
			}

			float split_cost = SplitCost(left, 0, counts[0]) + SplitCost(right, 0, counts[1]);
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = 'Z';
				best_split_pos = pos;
			}
		}
	}

	// If there is no value in splitting, don't split
	if (base_cost - EPSILON < best_split_cost) {
		delete[] left;
		delete[] right;
		return false;
	}

	// Define the best splits
	counts[0] = 0; // Left count
	counts[1] = 0; // Right counts

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		CoreTri& tri = mesh->triangles[tri_indices[i]];
		float3 center = (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;

		// X split
		if (best_split_plane == 'X') {
			if (center.x <= best_split_pos) {
				left[counts[0]++] = tri_indices[i];
			}
			else {
				right[counts[1]++] = tri_indices[i];
			}
		}

		// Y split
		if (best_split_plane == 'Y') {
			if (center.y <= best_split_pos) {
				left[counts[0]++] = tri_indices[i];
			}
			else {
				right[counts[1]++] = tri_indices[i];
			}
		}

		// Z split
		if (best_split_plane == 'Z') {
			if (center.z <= best_split_pos) {
				left[counts[0]++] = tri_indices[i];
			}
			else {
				right[counts[1]++] = tri_indices[i];
			}
		}
	}

	// Merge left and right into one triangle array
	for (int i = 0; i < counts[0]; i++) {
		indices[i] = left[i];
	}

	for (int i = 0; i < counts[1]; i++) {
		indices[counts[0] + i] = right[i];
	}

	delete[] left;
	delete[] right;

	return true;
}

bool BVH2::Subdivide(BVHNode& bvh) {
	if (bvh.count < 4) { return false; } // Leaves have at least 2 primitives in them

	int* indices = new int[bvh.count];
	int counts[2] = {0};

	// If there is value in splitting
	if (Partition(bvh, indices, counts)) {
		// Update indices
		for (int i = 0; i < bvh.count; i++) {
			tri_indices[bvh.first + i] = indices[i];
		}

		// Split the BVH in two
		BVHNode& left = pool[pool_pointer];
		BVHNode& right = pool[pool_pointer + 1];

		left.first = bvh.first;
		left.count = counts[0];

		right.first = bvh.first + counts[0];
		right.count = counts[1];

		bvh.count = 0;				// This node is not a leaf anymore
		bvh.first = pool_pointer;	// Point to its left child

		delete[] indices;

		return true;
	}

	delete[] indices;

	return false; // We did not split
}

void BVH2::SubdivideRecursively(BVHNode& bvh) {
	if (Subdivide(bvh)) {
		BVHNode& left = pool[pool_pointer++];
		BVHNode& right = pool[pool_pointer++];

		SubdivideRecursively(left);
		SubdivideRecursively(right);
	}
}

void BVH2::UpdateBounds() {
	for (int i = pool_pointer - 1; i >= 0; i--) {
		// Unused BVH
		if (pool[i].count == -1) {
			continue;
		}

		// Node, inheret AABB from children
		if (pool[i].count == 0) {
			BVHNode& left = pool[pool[i].first];
			BVHNode& right = pool[pool[i].first + 1];

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
			CoreTri& tri = mesh->triangles[tri_indices[j]];

			float3 tri_min_bound = fminf(fminf(tri.vertex0, tri.vertex1), tri.vertex2);
			float3 tri_max_bound = fmaxf(fmaxf(tri.vertex0, tri.vertex1), tri.vertex2);

			min_b = fminf(min_b, tri_min_bound);
			max_b = fmaxf(max_b, tri_max_bound);
		}

		pool[i].min_bound = min_b; pool[i].max_bound = max_b;
	}
}

bool BVH2::Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	float initial_t = t;
	float found_t = t;

	if (bvh.count > 0) { // If the node is a leaf
		for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
			//cout << "Intersecting triangle at index: " << tri_indices[i] << endl;
			//cout << "Mesh total triangle count: " << (*mesh).vcount / 3 << endl;
			//cout << "Triangle vertex0.x position: " << (*mesh).triangles[tri_indices[i]].vertex0.x << endl;
			if (ray.IntersectTriangle((*mesh).triangles[tri_indices[i]], found_t) && found_t < t) {
				t = found_t;
				tri = mesh->triangles[tri_indices[i]];
			}
		}
	}
	else { // If the node is not a leaf
		float t_left = FLT_MAX;
		float t_right = FLT_MAX;

		// Only traverse children that are actually intersected
		if (IntersectAABB(ray, pool[bvh.first], t_left)) {
			if (IntersectAABB(ray, pool[bvh.first + 1], t_right)) {
				if (t_left < t_right) { // Traverse the closest child first
					Traverse(ray, pool[bvh.first], tri, t, c);
					Traverse(ray, pool[bvh.first + 1], tri, t, c);
				}
				else {
					Traverse(ray, pool[bvh.first + 1], tri, t, c);
					Traverse(ray, pool[bvh.first], tri, t, c);
				}
			}
			else {
				Traverse(ray, pool[bvh.first], tri, t, c);
			}
		}
		else if (IntersectAABB(ray, pool[bvh.first + 1], t_right)) {
			Traverse(ray, pool[bvh.first + 1], tri, t, c);
		}
	}

	return t < initial_t; // If an intersection was found
}

void BVH2::Print(BVHNode& bvh) {
	if (bvh.count > 0) {
		cout << "[" << bvh.count << "]";
	}
	else {
		cout << "[";
		Print(pool[bvh.first]);
		Print(pool[bvh.first + 1]);
		cout << "]";
	}
}

/*
	BVH4
*/

BVH4::BVH4(Mesh* m) : mesh(m) {
	N = mesh->vcount / 3;
}

BVH4::~BVH4() {
	delete[] pool;
	delete[] tri_indices;
}

float BVH4::SplitCost(const int* indices, int first, int count) {
	return 0.0f;
}

bool BVH4::Partition(BVHNode& bvh, int* indices, int* counts) {
	return false;
}

bool BVH4::Subdivide(BVHNode& bvh) {
	return false;
}

void BVH4::SubdivideRecursively(BVHNode& bvh) {

}

void BVH4::UpdateBounds() {

}

bool BVH4::Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c) {
	return false;
}

void BVH4::Print(BVHNode& bvh) {

}

/*
	TopLevelBVH
*/

TopLevelBVH::TopLevelBVH() {
	
}

TopLevelBVH::~TopLevelBVH() {
	for (auto bvh : bvh_vector) { delete bvh; }
}

float TopLevelBVH::SplitCost(const int* indices, int first, int count) {
	float3 min_b = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
	float3 max_b = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// Calculate AABB bounds
	for (int i = first; i < first + count; i++) {
		BVHNode& bvh_root = bvh_vector[indices[i]]->Root();
		min_b = fminf(min_b, bvh_root.min_bound);
		max_b = fmaxf(max_b, bvh_root.max_bound);
	}

	float3 dims = max_b - min_b; // Dimensions of the AABB
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z; // AABB surface area

	return count * area;
}

bool TopLevelBVH::Partition(BVHNode& bvh, int* indices, int* counts) {
	return false;
}

bool TopLevelBVH::Subdivide(BVHNode& bvh) {
	if (bvh.count < 4) { return false; } // Leaves have at least 2 mesh-level BVHs in them

	int* indices = new int[bvh.count];
	int counts[2] = { 0 };

	// If there is value in splitting
	if (Partition(bvh, indices, counts)) {
		// Update indices
		for (int i = 0; i < bvh.count; i++) {
			tri_indices[bvh.first + i] = indices[i];
		}

		// Split the BVH in two
		BVHNode& left = pool[pool_pointer];
		BVHNode& right = pool[pool_pointer + 1];

		left.first = bvh.first;
		left.count = counts[0];

		right.first = bvh.first + counts[0];
		right.count = counts[1];

		bvh.count = 0;				// This node is not a leaf anymore
		bvh.first = pool_pointer;	// Point to its left child

		delete[] indices;

		return true;
	}

	delete[] indices;

	return false; // We did not split
}

void TopLevelBVH::SubdivideRecursively(BVHNode& bvh) {
	if (Subdivide(bvh)) {
		BVHNode& left = pool[pool_pointer++];
		BVHNode& right = pool[pool_pointer++];

		SubdivideRecursively(left);
		SubdivideRecursively(right);
	}
}

void TopLevelBVH::UpdateBounds() {
	for (int i = pool_pointer - 1; i >= 0; i--) {
		// Unused BVH
		if (pool[i].count == -1) {
			continue;
		}

		// Node, inheret AABB from children
		if (pool[i].count == 0) {
			BVHNode& left = pool[pool[i].first];
			BVHNode& right = pool[pool[i].first + 1];

			float3 min_bound = fminf(left.min_bound, right.min_bound);
			float3 max_bound = fmaxf(left.max_bound, right.max_bound);

			pool[i].min_bound = min_bound;
			pool[i].max_bound = max_bound;

			continue;
		}

		// Leaf, calculate AABB from mesh-level BVHs
		float3 min_b = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
		float3 max_b = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		for (int j = pool[i].first; j < pool[i].first + pool[i].count; j++) {
			BVH* bvh = bvh_vector[tri_indices[j]];

			float3 bvh_min_bound = bvh->Root().min_bound;
			float3 bvh_max_bound = bvh->Root().max_bound;

			min_b = fminf(min_b, bvh_min_bound);
			max_b = fmaxf(max_b, bvh_max_bound);
		}

		pool[i].min_bound = min_b; pool[i].max_bound = max_b;
	}
}

bool TopLevelBVH::Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	float initial_t = t;

	if (bvh.count > 0) { // If the node is a leaf
		for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
			bvh_vector[tri_indices[i]]->Traverse(ray, bvh_vector[tri_indices[i]]->Root(), tri, t, c); // Continue traversing the mesh-level BVH
		}
	}
	else { // If the node is not a leaf
		float t_left = FLT_MAX;
		float t_right = FLT_MAX;

		// Only traverse children that are actually intersected
		if (IntersectAABB(ray, pool[bvh.first], t_left)) {
			if (IntersectAABB(ray, pool[bvh.first + 1], t_right)) {
				if (t_left < t_right) { // Traverse the closest child first
					Traverse(ray, pool[bvh.first], tri, t, c);
					Traverse(ray, pool[bvh.first + 1], tri, t, c);
				}
				else {
					Traverse(ray, pool[bvh.first + 1], tri, t, c);
					Traverse(ray, pool[bvh.first], tri, t, c);
				}
			}
			else {
				Traverse(ray, pool[bvh.first], tri, t, c);
			}
		}
		else if (IntersectAABB(ray, pool[bvh.first + 1], t_right)) {
			Traverse(ray, pool[bvh.first + 1], tri, t, c);
		}
	}

	return t < initial_t; // If an intersection was found
}

void TopLevelBVH::Print(BVHNode& bvh) {
	if (bvh.count > 0) {
		cout << "[" << bvh.count << "]";
	}
	else {
		cout << "[";
		Print(pool[bvh.first]);
		Print(pool[bvh.first + 1]);
		cout << "]";
	}
}

void TopLevelBVH::AddBVH(BVH* bvh, bool rebuild) {
	bvh_vector.push_back(bvh);
	if (rebuild) { Rebuild(); }
}

void TopLevelBVH::Rebuild() {
	N = bvh_vector.size();

	delete[] pool;
	delete[] tri_indices;

	tri_indices = new int[N];
	for (int i = 0; i < N; i++) {
		tri_indices[i] = i;
	}

	pool = new BVHNode[2 * N];
	BVHNode& root = pool[0];

	root.first = 0;
	root.count = N;

	UpdateBounds();
}