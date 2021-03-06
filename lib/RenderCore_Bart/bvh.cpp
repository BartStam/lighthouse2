#include "core_settings.h"

/*
	BVH
*/

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
}

BVH2::~BVH2() {
	delete[] pool;
	delete[] tri_indices;
}

void BVH2::Rebuild() {
	if (pool_pointer != -1) { delete[] tri_indices; } // Only clean up if this is not the first time we build the BVH

	tri_indices = new int[N]; // Assume triangle count does not change
	for (int i = 0; i < N; i++) {
		tri_indices[i] = i;
	}

	pool = new BVHNode[2 * N];
	BVHNode& root = pool[0];
	pool_pointer = 2;

	root.first = 0;
	root.count = N;

	SubdivideRecursively(root);
	UpdateBounds();
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

bool BVH2::Partition(BVHNode& bvh, int* indices, int* counts, int bins) {
	int* left = new int[bvh.count];
	int* right = new int[bvh.count];

	float base_cost = SplitCost(tri_indices, bvh.first, bvh.count);	// Cost of current BVH before splitting, according to SAH heuristic
	float best_split_cost = FLT_MAX;
	float best_split_pos;
	int best_split_plane;

	// Calculate the centroid AABB
	float3 c_min_bound = make_float3(FLT_MAX);
	float3 c_max_bound = make_float3(-FLT_MAX);

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		c_min_bound = fminf(c_min_bound, mesh->tri_centers[tri_indices[i]]);
		c_max_bound = fmaxf(c_max_bound, mesh->tri_centers[tri_indices[i]]);
	}

	for (int i = 0; i < 3; i++) { // X, Y, Z
		if (c_min_bound.get(i) == c_max_bound.get(i)) { continue; } // If all primitives have the same position on this plane, don't split
			float interval = (c_max_bound.get(i) - c_min_bound.get(i)) / bins;
			for (int j = 0; j < bins; j++) {
				float pos = c_min_bound.get(i) + j * interval;

				counts[0] = 0; // Left count
				counts[1] = 0; // Right counts

				for (int k = bvh.first; k < bvh.first + bvh.count; k++) {
					float3 tri_center = mesh->tri_centers[tri_indices[k]];

					if (tri_center.get(i) <= pos) {
						left[counts[0]++] = tri_indices[k];
					}
					else {
						right[counts[1]++] = tri_indices[k];
					}
				}

				float split_cost = SplitCost(left, 0, counts[0]) + SplitCost(right, 0, counts[1]);
				if (split_cost + EPSILON < best_split_cost) {
					best_split_cost = split_cost;
					best_split_plane = i;
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

	// Perform the split
	counts[0] = 0; // Left count
	counts[1] = 0; // Right counts

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		float3 center = mesh->tri_centers[tri_indices[i]];

		if (center.get(best_split_plane) <= best_split_pos) {
			left[counts[0]++] = tri_indices[i];
		}
		else {
			right[counts[1]++] = tri_indices[i];
		}
	}

	// Merge left and right into one indices array
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

		// Leaf, (re)calculate AABB from primitives
		pool[i].min_bound = make_float3(FLT_MAX);
		pool[i].max_bound = make_float3(-FLT_MAX);

		for (int j = pool[i].first; j < pool[i].first + pool[i].count; j++) {
			pool[i].min_bound = fminf(pool[i].min_bound, mesh->tri_min_bounds[tri_indices[j]]);
			pool[i].max_bound = fmaxf(pool[i].max_bound, mesh->tri_max_bounds[tri_indices[j]]);
		}
	}
}

bool BVH2::Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections
	
	BVHNode& bvh = pool[pool_index];

	float initial_t = t;
	float found_t = t;

	if (bvh.count > 0) { // If the node is a leaf
		for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
			if (ray.IntersectTriangle((*mesh).triangles[tri_indices[i]], found_t) && found_t < t) {
				t = found_t;
				CoreTri& tri = mesh->triangles[tri_indices[i]];
				material = tri.material;
				N = make_float3(tri.Nx, tri.Ny, tri.Nz);
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
					Traverse(ray, material, N, t, bvh.first, c);
					Traverse(ray, material, N, t, bvh.first + 1, c);
				}
				else {
					Traverse(ray, material, N, t, bvh.first + 1, c);
					Traverse(ray, material, N, t, bvh.first, c);
				}
			}
			else {
				Traverse(ray, material, N, t, bvh.first, c);
			}
		}
		else if (IntersectAABB(ray, pool[bvh.first + 1], t_right)) {
			Traverse(ray, material, N, t, bvh.first + 1, c);
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
	BVH4 (not yet implemented)
*/

BVH4::BVH4(Mesh* m) : mesh(m) {
	N = mesh->vcount / 3;
}

BVH4::~BVH4() {
	delete[] pool;
	delete[] tri_indices;
}

void BVH4::Rebuild() {

}

float BVH4::SplitCost(const int* indices, int first, int count) {
	return 0.0f;
}

bool BVH4::Partition(BVHNode& bvh, int* indices, int* counts, int bins) {
	return false;
}

bool BVH4::Subdivide(BVHNode& bvh) {
	return false;
}

void BVH4::SubdivideRecursively(BVHNode& bvh) {

}

void BVH4::UpdateBounds() {

}

bool BVH4::Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index, int* c) {
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
	
}

void TopLevelBVH::Rebuild() {
	N = instance_vector.size();

	// Only clean up if this is not the first time we build the BVH
	if (pool_pointer != -1) {
		delete[] pool;
		delete[] tri_indices;
	}

	tri_indices = new int[N];
	for (int i = 0; i < N; i++) {
		tri_indices[i] = i;
	}

	pool = new BVHNode[2 * N];
	BVHNode& root = pool[0];
	pool_pointer = 2;

	root.first = 0;
	root.count = N;

	SubdivideRecursively(root);
	UpdateBounds();
}

float TopLevelBVH::SplitCost(const int* indices, int first, int count) {
	float3 min_b = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
	float3 max_b = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// Calculate AABB bounds
	for (int i = first; i < first + count; i++) {
		const BVHNode& bvh_root = instance_vector[indices[i]]->mesh->bvh->Root();
		min_b = fminf(min_b, bvh_root.min_bound);
		max_b = fmaxf(max_b, bvh_root.max_bound);
	}

	float3 dims = max_b - min_b; // Dimensions of the AABB
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z; // AABB surface area

	return count * area;
}

bool TopLevelBVH::Partition(BVHNode& bvh, int* indices, int* counts, int bins) {
	int* left = new int[bvh.count];
	int* right = new int[bvh.count];

	float base_cost = SplitCost(tri_indices, bvh.first, bvh.count);	// Cost of current BVH before splitting, according to SAH heuristic
	float best_split_cost = FLT_MAX;
	float best_split_pos;
	int best_split_plane;

	// Calculate the centroid AABB
	float3 c_min_bound = make_float3(FLT_MAX);
	float3 c_max_bound = make_float3(-FLT_MAX);

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		float3 center = instance_vector[tri_indices[i]]->mesh->aabb_center;
		center = instance_vector[tri_indices[i]]->transform.TransformPoint(center);

		c_min_bound = fminf(c_min_bound, center);
		c_max_bound = fmaxf(c_max_bound, center);
	}

	// Search for the best split using binning
	for (int i = 0; i < 3; i++) { // X, Y, Z
		if (c_min_bound.get(i) == c_max_bound.get(i)) { continue; } // If all primitives have the same position on this plane, don't split

		float interval = (c_max_bound.get(i) - c_min_bound.get(i)) / bins;

		for (int j = 0; j < bins; j++) {
			float pos = c_min_bound.get(i) + j * interval;

			counts[0] = 0; // Left count
			counts[1] = 0; // Right counts

			for (int k = bvh.first; k < bvh.first + bvh.count; k++) {
				float3 center = instance_vector[tri_indices[k]]->mesh->aabb_center;
				center = instance_vector[tri_indices[k]]->transform.TransformPoint(center);

				if (center.get(i) <= pos) {
					left[counts[0]++] = tri_indices[k];
				}
				else {
					right[counts[1]++] = tri_indices[k];
				}
			}

			float split_cost = SplitCost(left, 0, counts[0]) + SplitCost(right, 0, counts[1]);
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
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

	// Perform the split
	counts[0] = 0; // Left count
	counts[1] = 0; // Right counts

	for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
		float3 center = instance_vector[tri_indices[i]]->mesh->aabb_center;
		center = instance_vector[tri_indices[i]]->transform.TransformPoint(center);

		if (center.get(best_split_plane) <= best_split_pos) {
			left[counts[0]++] = tri_indices[i];
		}
		else {
			right[counts[1]++] = tri_indices[i];
		}
	}

	// Merge left and right into one indices array
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

bool TopLevelBVH::Subdivide(BVHNode& bvh) {
	if (bvh.count < 4) { return false; } // Leaves have at least 2 instance-level BVHs in them

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

		// Leaf, calculate AABB from instance-level BVHs
		float3 min_b = make_float3(FLT_MAX);
		float3 max_b = make_float3(-FLT_MAX);

		for (int j = pool[i].first; j < pool[i].first + pool[i].count; j++) {
			float3 mesh_minb = instance_vector[tri_indices[j]]->mesh->aabb_min_bound;
			float3 mesh_maxb = instance_vector[tri_indices[j]]->mesh->aabb_max_bound;

			// Transform the mesh AABB bounds according to the instance transform
			mesh_minb = instance_vector[tri_indices[j]]->transform.TransformPoint(mesh_minb);
			mesh_maxb = instance_vector[tri_indices[j]]->transform.TransformPoint(mesh_maxb);

			min_b = fminf(min_b, mesh_minb);
			max_b = fmaxf(max_b, mesh_maxb);
		}

		pool[i].min_bound = min_b; pool[i].max_bound = max_b;
	}
}

bool TopLevelBVH::Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	BVHNode& bvh = pool[pool_index];
	
	float initial_t = t;
	float found_t = t;

	if (bvh.count > 0) { // If the node is a leaf
		for (int i = bvh.first; i < bvh.first + bvh.count; i++) {
			float t_aabb = FLT_MAX;

			// We're entering an instance-level BVH, so we transform the ray by the inverse of its local transform
			Ray transformed_ray = ray;
			mat4 inv_transform = instance_vector[tri_indices[i]]->inv_transform;
			transformed_ray.O = inv_transform.TransformPoint(transformed_ray.O);
			transformed_ray.D = normalize(inv_transform.TransformVector(transformed_ray.D));

			if (IntersectAABB(transformed_ray, instance_vector[tri_indices[i]]->mesh->bvh->Root(), t_aabb) && t_aabb < t) {
				instance_vector[tri_indices[i]]->mesh->bvh->Traverse(transformed_ray, material, N, t, 0, c); // Continue traversing the instance-level BVH

				// If we find a closer intersection than what we previously found
				if (t < found_t) {
					found_t = t;
					N = normalize(instance_vector[tri_indices[i]]->transform.TransformVector(N)); // Make sure we return the transformed normal (in case of rotation)
				}
			}
		}
	}
	else { // If the node is not a leaf
		float t_left = FLT_MAX;
		float t_right = FLT_MAX;

		// Only traverse children that are actually intersected
		if (IntersectAABB(ray, pool[bvh.first], t_left) && t_left < t) {
			if (IntersectAABB(ray, pool[bvh.first + 1], t_right) && t_right < t) {
				if (t_left < t_right) { // Traverse the closest child first
					Traverse(ray, material, N, t, bvh.first, c);
					Traverse(ray, material, N, t, bvh.first + 1, c);
				}
				else {
					Traverse(ray, material, N, t, bvh.first + 1, c);
					Traverse(ray, material, N, t, bvh.first, c);
				}
			}
			else {
				Traverse(ray, material, N, t, bvh.first, c);
			}
		}
		else if (IntersectAABB(ray, pool[bvh.first + 1], t_right) && t_right < t) {
			Traverse(ray, material, N, t, bvh.first + 1, c);
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

void TopLevelBVH::AddInstance(Instance* instance) {
	instance_vector.push_back(instance);
}
