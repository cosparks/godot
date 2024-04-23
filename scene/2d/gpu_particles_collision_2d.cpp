/**************************************************************************/
/*  gpu_particles_collision_2d.cpp                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "gpu_particles_collision_2d.h"

#include "core/object/worker_thread_pool.h"
#include "scene/2d/camera_2d.h"
#include "scene/main/viewport.h"

// cosparks TODO: remove stubs

void update_gizmos() {
	return;
}

void set_base(RID collision) {
	return;
}

// end cosparks TODO

void GPUParticlesCollision2D::set_cull_mask(uint32_t p_cull_mask) {
	cull_mask = p_cull_mask;
	RS::get_singleton()->particles_collision_set_cull_mask(collision, p_cull_mask);
}

uint32_t GPUParticlesCollision2D::get_cull_mask() const {
	return cull_mask;
}

void GPUParticlesCollision2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cull_mask", "mask"), &GPUParticlesCollision2D::set_cull_mask);
	ClassDB::bind_method(D_METHOD("get_cull_mask"), &GPUParticlesCollision2D::get_cull_mask);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "cull_mask", PROPERTY_HINT_LAYERS_3D_RENDER), "set_cull_mask", "get_cull_mask");
}

GPUParticlesCollision2D::GPUParticlesCollision2D(RS::ParticlesCollisionType p_type) {
	collision = RS::get_singleton()->particles_collision_create();
	RS::get_singleton()->particles_collision_set_collision_type(collision, p_type);
	set_base(collision);
}

GPUParticlesCollision2D::~GPUParticlesCollision2D() {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	RS::get_singleton()->free(collision);
}

/////////////////////////////////

void GPUParticlesCollisionCircle2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &GPUParticlesCollisionCircle2D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &GPUParticlesCollisionCircle2D::get_radius);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.01,1024,0.01,or_greater,suffix:m"), "set_radius", "get_radius");
}

void GPUParticlesCollisionCircle2D::set_radius(real_t p_radius) {
	radius = p_radius;
	RS::get_singleton()->particles_collision_set_sphere_radius(_get_collision(), radius);
	update_gizmos();
}

real_t GPUParticlesCollisionCircle2D::get_radius() const {
	return radius;
}

GPUParticlesCollisionCircle2D::GPUParticlesCollisionCircle2D() :
	GPUParticlesCollision2D(RS::PARTICLES_COLLISION_TYPE_SPHERE_COLLIDE) {
}

GPUParticlesCollisionCircle2D::~GPUParticlesCollisionCircle2D() {
}

///////////////////////////

void GPUParticlesCollisionRect2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &GPUParticlesCollisionRect2D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &GPUParticlesCollisionRect2D::get_size);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_RANGE, "0.01,1024,0.01,or_greater,suffix:m"), "set_size", "get_size");
}

void GPUParticlesCollisionRect2D::set_size(const Vector3 &p_size) {
	size = p_size;
	RS::get_singleton()->particles_collision_set_box_extents(_get_collision(), size / 2);
	update_gizmos();
}

Vector3 GPUParticlesCollisionRect2D::get_size() const {
	return size;
}


GPUParticlesCollisionRect2D::GPUParticlesCollisionRect2D() :
		GPUParticlesCollision2D(RS::PARTICLES_COLLISION_TYPE_BOX_COLLIDE) {
}

GPUParticlesCollisionRect2D::~GPUParticlesCollisionRect2D() {
}

///////////////////////////////
///////////////////////////

void GPUParticlesCollisionSDF2D::_find_meshes(const AABB &p_aabb, Node *p_at_node, List<PlotMesh> &plot_meshes) {
	// cosparks TODO
}

uint32_t GPUParticlesCollisionSDF2D::_create_bvh(LocalVector<BVH> &bvh_tree, FacePos *p_faces, uint32_t p_face_count, const Face3 *p_triangles, float p_thickness) {
	if (p_face_count == 1) {
		return BVH::LEAF_BIT | p_faces[0].index;
	}

	uint32_t index = bvh_tree.size();
	{
		BVH bvh;

		for (uint32_t i = 0; i < p_face_count; i++) {
			const Face3 &f = p_triangles[p_faces[i].index];
			AABB aabb(f.vertex[0], Vector3());
			aabb.expand_to(f.vertex[1]);
			aabb.expand_to(f.vertex[2]);
			if (p_thickness > 0.0) {
				Vector3 normal = p_triangles[p_faces[i].index].get_plane().normal;
				aabb.expand_to(f.vertex[0] - normal * p_thickness);
				aabb.expand_to(f.vertex[1] - normal * p_thickness);
				aabb.expand_to(f.vertex[2] - normal * p_thickness);
			}
			if (i == 0) {
				bvh.bounds = aabb;
			} else {
				bvh.bounds.merge_with(aabb);
			}
		}
		bvh_tree.push_back(bvh);
	}

	uint32_t middle = p_face_count / 2;

	SortArray<FacePos, FaceSort> s;
	s.compare.axis = bvh_tree[index].bounds.get_longest_axis_index();
	s.sort(p_faces, p_face_count);

	uint32_t left = _create_bvh(bvh_tree, p_faces, middle, p_triangles, p_thickness);
	uint32_t right = _create_bvh(bvh_tree, p_faces + middle, p_face_count - middle, p_triangles, p_thickness);

	bvh_tree[index].children[0] = left;
	bvh_tree[index].children[1] = right;

	return index;
}

static _FORCE_INLINE_ real_t Vector3_dot2(const Vector3 &p_vec3) {
	return p_vec3.dot(p_vec3);
}

void GPUParticlesCollisionSDF2D::_find_closest_distance(const Vector3 &p_pos, const BVH *p_bvh, uint32_t p_bvh_cell, const Face3 *p_triangles, float p_thickness, float &r_closest_distance) {
	if (p_bvh_cell & BVH::LEAF_BIT) {
		p_bvh_cell &= BVH::LEAF_MASK; //remove bit

		Vector3 point = p_pos;
		Plane p = p_triangles[p_bvh_cell].get_plane();
		float d = p.distance_to(point);
		float inside_d = 1e20;
		if (d < 0 && d > -p_thickness) {
			//inside planes, do this in 2D

			Vector3 x_axis = (p_triangles[p_bvh_cell].vertex[0] - p_triangles[p_bvh_cell].vertex[1]).normalized();
			Vector3 y_axis = p.normal.cross(x_axis).normalized();

			Vector2 points[3];
			for (int i = 0; i < 3; i++) {
				points[i] = Vector2(x_axis.dot(p_triangles[p_bvh_cell].vertex[i]), y_axis.dot(p_triangles[p_bvh_cell].vertex[i]));
			}

			Vector2 p2d = Vector2(x_axis.dot(point), y_axis.dot(point));

			{
				// https://www.shadertoy.com/view/XsXSz4

				Vector2 e0 = points[1] - points[0];
				Vector2 e1 = points[2] - points[1];
				Vector2 e2 = points[0] - points[2];

				Vector2 v0 = p2d - points[0];
				Vector2 v1 = p2d - points[1];
				Vector2 v2 = p2d - points[2];

				Vector2 pq0 = v0 - e0 * CLAMP(v0.dot(e0) / e0.dot(e0), 0.0, 1.0);
				Vector2 pq1 = v1 - e1 * CLAMP(v1.dot(e1) / e1.dot(e1), 0.0, 1.0);
				Vector2 pq2 = v2 - e2 * CLAMP(v2.dot(e2) / e2.dot(e2), 0.0, 1.0);

				float s = SIGN(e0.x * e2.y - e0.y * e2.x);
				Vector2 d2 = Vector2(pq0.dot(pq0), s * (v0.x * e0.y - v0.y * e0.x)).min(Vector2(pq1.dot(pq1), s * (v1.x * e1.y - v1.y * e1.x))).min(Vector2(pq2.dot(pq2), s * (v2.x * e2.y - v2.y * e2.x)));

				inside_d = -Math::sqrt(d2.x) * SIGN(d2.y);
			}

			//make sure distance to planes is not shorter if inside
			if (inside_d < 0) {
				inside_d = MAX(inside_d, d);
				inside_d = MAX(inside_d, -(p_thickness + d));
			}

			r_closest_distance = MIN(r_closest_distance, inside_d);
		} else {
			if (d < 0) {
				point -= p.normal * p_thickness; //flatten
			}

			// https://iquilezles.org/www/articles/distfunctions/distfunctions.htm
			Vector3 a = p_triangles[p_bvh_cell].vertex[0];
			Vector3 b = p_triangles[p_bvh_cell].vertex[1];
			Vector3 c = p_triangles[p_bvh_cell].vertex[2];

			Vector3 ba = b - a;
			Vector3 pa = point - a;
			Vector3 cb = c - b;
			Vector3 pb = point - b;
			Vector3 ac = a - c;
			Vector3 pc = point - c;
			Vector3 nor = ba.cross(ac);

			inside_d = Math::sqrt(
					(SIGN(ba.cross(nor).dot(pa)) + SIGN(cb.cross(nor).dot(pb)) + SIGN(ac.cross(nor).dot(pc)) < 2.0)
							? MIN(MIN(
										  Vector3_dot2(ba * CLAMP(ba.dot(pa) / Vector3_dot2(ba), 0.0, 1.0) - pa),
										  Vector3_dot2(cb * CLAMP(cb.dot(pb) / Vector3_dot2(cb), 0.0, 1.0) - pb)),
									  Vector3_dot2(ac * CLAMP(ac.dot(pc) / Vector3_dot2(ac), 0.0, 1.0) - pc))
							: nor.dot(pa) * nor.dot(pa) / Vector3_dot2(nor));

			r_closest_distance = MIN(r_closest_distance, inside_d);
		}

	} else {
		bool pass = true;
		if (!p_bvh[p_bvh_cell].bounds.has_point(p_pos)) {
			//outside, find closest point
			Vector3 he = p_bvh[p_bvh_cell].bounds.size * 0.5;
			Vector3 center = p_bvh[p_bvh_cell].bounds.position + he;

			Vector3 rel = (p_pos - center).abs();
			Vector3 closest = rel.min(he);
			float d = rel.distance_to(closest);

			if (d >= r_closest_distance) {
				pass = false; //already closer than this aabb, discard
			}
		}

		if (pass) {
			_find_closest_distance(p_pos, p_bvh, p_bvh[p_bvh_cell].children[0], p_triangles, p_thickness, r_closest_distance);
			_find_closest_distance(p_pos, p_bvh, p_bvh[p_bvh_cell].children[1], p_triangles, p_thickness, r_closest_distance);
		}
	}
}

void GPUParticlesCollisionSDF2D::_compute_sdf_z(uint32_t p_z, ComputeSDFParams *params) {
	int32_t z_ofs = p_z * params->size.y * params->size.x;
	for (int32_t y = 0; y < params->size.y; y++) {
		int32_t y_ofs = z_ofs + y * params->size.x;
		for (int32_t x = 0; x < params->size.x; x++) {
			int32_t x_ofs = y_ofs + x;
			float &cell = params->cells[x_ofs];

			Vector3 pos = params->cell_offset + Vector3(x, y, p_z) * params->cell_size;

			cell = 1e20;

			_find_closest_distance(pos, params->bvh, 0, params->triangles, params->thickness, cell);
		}
	}
}

void GPUParticlesCollisionSDF2D::_compute_sdf(ComputeSDFParams *params) {
	WorkerThreadPool::GroupID group_task = WorkerThreadPool::get_singleton()->add_template_group_task(this, &GPUParticlesCollisionSDF2D::_compute_sdf_z, params, params->size.z);
	while (!WorkerThreadPool::get_singleton()->is_group_task_completed(group_task)) {
		OS::get_singleton()->delay_usec(10000);
		if (bake_step_function) {
			bake_step_function(WorkerThreadPool::get_singleton()->get_group_processed_element_count(group_task) * 100 / params->size.z, "Baking SDF");
		}
	}
	WorkerThreadPool::get_singleton()->wait_for_group_task_completion(group_task);
}

Vector3i GPUParticlesCollisionSDF2D::get_estimated_cell_size() const {
	static const int subdivs[RESOLUTION_MAX] = { 16, 32, 64, 128, 256, 512 };
	int subdiv = subdivs[get_resolution()];

	AABB aabb(-size / 2, size);

	float cell_size = aabb.get_longest_axis_size() / float(subdiv);

	Vector3i sdf_size = Vector3i(aabb.size / cell_size);
	sdf_size = sdf_size.max(Vector3i(1, 1, 1));
	return sdf_size;
}

Ref<Image> GPUParticlesCollisionSDF2D::bake() {
	static const int subdivs[RESOLUTION_MAX] = { 16, 32, 64, 128, 256, 512 };
	int subdiv = subdivs[get_resolution()];

	AABB aabb(-size / 2, size);

	float cell_size = aabb.get_longest_axis_size() / float(subdiv);

	Vector3i sdf_size = Vector3i(aabb.size / cell_size);
	sdf_size = sdf_size.max(Vector3i(1, 1, 1));

	if (bake_begin_function) {
		bake_begin_function(100);
	}

	aabb.size = Vector3(sdf_size) * cell_size;

	List<PlotMesh> plot_meshes;
	_find_meshes(aabb, get_parent(), plot_meshes);

	LocalVector<Face3> faces;

	if (bake_step_function) {
		bake_step_function(0, "Finding Meshes");
	}

	for (const PlotMesh &pm : plot_meshes) {
		for (int i = 0; i < pm.mesh->get_surface_count(); i++) {
			if (pm.mesh->surface_get_primitive_type(i) != Mesh::PRIMITIVE_TRIANGLES) {
				continue; //only triangles
			}

			Array a = pm.mesh->surface_get_arrays(i);

			Vector<Vector3> vertices = a[Mesh::ARRAY_VERTEX];
			const Vector3 *vr = vertices.ptr();
			Vector<int> index = a[Mesh::ARRAY_INDEX];

			if (index.size()) {
				int facecount = index.size() / 3;
				const int *ir = index.ptr();

				for (int j = 0; j < facecount; j++) {
					Face3 face;

					for (int k = 0; k < 3; k++) {
						face.vertex[k] = pm.local_xform.xform(vr[ir[j * 3 + k]]);
					}

					//test against original bounds
					if (!Geometry3D::triangle_box_overlap(aabb.get_center(), aabb.size * 0.5, face.vertex)) {
						continue;
					}

					faces.push_back(face);
				}

			} else {
				int facecount = vertices.size() / 3;

				for (int j = 0; j < facecount; j++) {
					Face3 face;

					for (int k = 0; k < 3; k++) {
						face.vertex[k] = pm.local_xform.xform(vr[j * 3 + k]);
					}

					//test against original bounds
					if (!Geometry3D::triangle_box_overlap(aabb.get_center(), aabb.size * 0.5, face.vertex)) {
						continue;
					}

					faces.push_back(face);
				}
			}
		}
	}

	//compute bvh
	if (faces.size() <= 1) {
		ERR_PRINT("No faces detected during GPUParticlesCollisionSDF2D bake. Check whether there are visible meshes matching the bake mask within its extents.");
		if (bake_end_function) {
			bake_end_function();
		}
		return Ref<Image>();
	}

	LocalVector<FacePos> face_pos;

	face_pos.resize(faces.size());

	float th = cell_size * thickness;

	for (uint32_t i = 0; i < faces.size(); i++) {
		face_pos[i].index = i;
		face_pos[i].center = (faces[i].vertex[0] + faces[i].vertex[1] + faces[i].vertex[2]) / 2;
		if (th > 0.0) {
			face_pos[i].center -= faces[i].get_plane().normal * th * 0.5;
		}
	}

	if (bake_step_function) {
		bake_step_function(0, "Creating BVH");
	}

	LocalVector<BVH> bvh;

	_create_bvh(bvh, face_pos.ptr(), face_pos.size(), faces.ptr(), th);

	Vector<uint8_t> cells_data;
	cells_data.resize(sdf_size.z * sdf_size.y * sdf_size.x * (int)sizeof(float));

	if (bake_step_function) {
		bake_step_function(0, "Baking SDF");
	}

	ComputeSDFParams params;
	params.cells = (float *)cells_data.ptrw();
	params.size = sdf_size;
	params.cell_size = cell_size;
	params.cell_offset = aabb.position + Vector3(cell_size * 0.5, cell_size * 0.5, cell_size * 0.5);
	params.bvh = bvh.ptr();
	params.triangles = faces.ptr();
	params.thickness = th;
	_compute_sdf(&params);

	Ref<Image> ret = Image::create_from_data(sdf_size.x, sdf_size.y * sdf_size.z, false, Image::FORMAT_RF, cells_data);
	ret->convert(Image::FORMAT_RH); //convert to half, save space
	ret->set_meta("depth", sdf_size.z); //hack, make sure to add to the docs of this function

	if (bake_end_function) {
		bake_end_function();
	}

	return ret;
}

PackedStringArray GPUParticlesCollisionSDF2D::get_configuration_warnings() const {
	PackedStringArray warnings = Node::get_configuration_warnings();

	if (bake_mask == 0) {
		warnings.push_back(RTR("The Bake Mask has no bits enabled, which means baking will not produce any collision for this GPUParticlesCollisionSDF2D.\nTo resolve this, enable at least one bit in the Bake Mask property."));
	}

	return warnings;
}

void GPUParticlesCollisionSDF2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &GPUParticlesCollisionSDF2D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &GPUParticlesCollisionSDF2D::get_size);

	ClassDB::bind_method(D_METHOD("set_resolution", "resolution"), &GPUParticlesCollisionSDF2D::set_resolution);
	ClassDB::bind_method(D_METHOD("get_resolution"), &GPUParticlesCollisionSDF2D::get_resolution);

	ClassDB::bind_method(D_METHOD("set_texture", "texture"), &GPUParticlesCollisionSDF2D::set_texture);
	ClassDB::bind_method(D_METHOD("get_texture"), &GPUParticlesCollisionSDF2D::get_texture);

	ClassDB::bind_method(D_METHOD("set_thickness", "thickness"), &GPUParticlesCollisionSDF2D::set_thickness);
	ClassDB::bind_method(D_METHOD("get_thickness"), &GPUParticlesCollisionSDF2D::get_thickness);

	ClassDB::bind_method(D_METHOD("set_bake_mask", "mask"), &GPUParticlesCollisionSDF2D::set_bake_mask);
	ClassDB::bind_method(D_METHOD("get_bake_mask"), &GPUParticlesCollisionSDF2D::get_bake_mask);
	ClassDB::bind_method(D_METHOD("set_bake_mask_value", "layer_number", "value"), &GPUParticlesCollisionSDF2D::set_bake_mask_value);
	ClassDB::bind_method(D_METHOD("get_bake_mask_value", "layer_number"), &GPUParticlesCollisionSDF2D::get_bake_mask_value);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_RANGE, "0.01,1024,0.01,or_greater,suffix:m"), "set_size", "get_size");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "resolution", PROPERTY_HINT_ENUM, "16,32,64,128,256,512"), "set_resolution", "get_resolution");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "thickness", PROPERTY_HINT_RANGE, "0.0,2.0,0.01,suffix:m"), "set_thickness", "get_thickness");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "bake_mask", PROPERTY_HINT_LAYERS_3D_RENDER), "set_bake_mask", "get_bake_mask");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "texture", PROPERTY_HINT_RESOURCE_TYPE, "Texture3D"), "set_texture", "get_texture");

	BIND_ENUM_CONSTANT(RESOLUTION_16);
	BIND_ENUM_CONSTANT(RESOLUTION_32);
	BIND_ENUM_CONSTANT(RESOLUTION_64);
	BIND_ENUM_CONSTANT(RESOLUTION_128);
	BIND_ENUM_CONSTANT(RESOLUTION_256);
	BIND_ENUM_CONSTANT(RESOLUTION_512);
	BIND_ENUM_CONSTANT(RESOLUTION_MAX);
}

void GPUParticlesCollisionSDF2D::set_thickness(float p_thickness) {
	thickness = p_thickness;
}

float GPUParticlesCollisionSDF2D::get_thickness() const {
	return thickness;
}

void GPUParticlesCollisionSDF2D::set_size(const Vector3 &p_size) {
	size = p_size;
	RS::get_singleton()->particles_collision_set_box_extents(_get_collision(), size / 2);
	update_gizmos();
}

Vector3 GPUParticlesCollisionSDF2D::get_size() const {
	return size;
}

void GPUParticlesCollisionSDF2D::set_resolution(Resolution p_resolution) {
	resolution = p_resolution;
	update_gizmos();
}

GPUParticlesCollisionSDF2D::Resolution GPUParticlesCollisionSDF2D::get_resolution() const {
	return resolution;
}

void GPUParticlesCollisionSDF2D::set_bake_mask(uint32_t p_mask) {
	bake_mask = p_mask;
	update_configuration_warnings();
}

uint32_t GPUParticlesCollisionSDF2D::get_bake_mask() const {
	return bake_mask;
}

void GPUParticlesCollisionSDF2D::set_bake_mask_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1 || p_layer_number > 20, vformat("The render layer number (%d) must be between 1 and 20 (inclusive).", p_layer_number));
	uint32_t mask = get_bake_mask();
	if (p_value) {
		mask |= 1 << (p_layer_number - 1);
	} else {
		mask &= ~(1 << (p_layer_number - 1));
	}
	set_bake_mask(mask);
}

bool GPUParticlesCollisionSDF2D::get_bake_mask_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1 || p_layer_number > 20, false, vformat("The render layer number (%d) must be between 1 and 20 (inclusive).", p_layer_number));
	return bake_mask & (1 << (p_layer_number - 1));
}

void GPUParticlesCollisionSDF2D::set_texture(const Ref<Texture3D> &p_texture) {
	texture = p_texture;
	RID tex = texture.is_valid() ? texture->get_rid() : RID();
	RS::get_singleton()->particles_collision_set_field_texture(_get_collision(), tex);
}

Ref<Texture3D> GPUParticlesCollisionSDF2D::get_texture() const {
	return texture;
}

GPUParticlesCollisionSDF2D::BakeBeginFunc GPUParticlesCollisionSDF2D::bake_begin_function = nullptr;
GPUParticlesCollisionSDF2D::BakeStepFunc GPUParticlesCollisionSDF2D::bake_step_function = nullptr;
GPUParticlesCollisionSDF2D::BakeEndFunc GPUParticlesCollisionSDF2D::bake_end_function = nullptr;

GPUParticlesCollisionSDF2D::GPUParticlesCollisionSDF2D() :
		GPUParticlesCollision2D(RS::PARTICLES_COLLISION_TYPE_SDF_COLLIDE) {
}

GPUParticlesCollisionSDF2D::~GPUParticlesCollisionSDF2D() {
}

////////////////////////////
////////////////////////////

void GPUParticlesAttractor2D::set_cull_mask(uint32_t p_cull_mask) {
	cull_mask = p_cull_mask;
	RS::get_singleton()->particles_collision_set_cull_mask(collision, p_cull_mask);
}

uint32_t GPUParticlesAttractor2D::get_cull_mask() const {
	return cull_mask;
}

void GPUParticlesAttractor2D::set_strength(real_t p_strength) {
	strength = p_strength;
	RS::get_singleton()->particles_collision_set_attractor_strength(collision, p_strength);
}

real_t GPUParticlesAttractor2D::get_strength() const {
	return strength;
}

void GPUParticlesAttractor2D::set_attenuation(real_t p_attenuation) {
	attenuation = p_attenuation;
	RS::get_singleton()->particles_collision_set_attractor_attenuation(collision, p_attenuation);
}

real_t GPUParticlesAttractor2D::get_attenuation() const {
	return attenuation;
}

void GPUParticlesAttractor2D::set_directionality(real_t p_directionality) {
	directionality = p_directionality;
	RS::get_singleton()->particles_collision_set_attractor_directionality(collision, p_directionality);
	update_gizmos();
}

real_t GPUParticlesAttractor2D::get_directionality() const {
	return directionality;
}

void GPUParticlesAttractor2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cull_mask", "mask"), &GPUParticlesAttractor2D::set_cull_mask);
	ClassDB::bind_method(D_METHOD("get_cull_mask"), &GPUParticlesAttractor2D::get_cull_mask);

	ClassDB::bind_method(D_METHOD("set_strength", "strength"), &GPUParticlesAttractor2D::set_strength);
	ClassDB::bind_method(D_METHOD("get_strength"), &GPUParticlesAttractor2D::get_strength);

	ClassDB::bind_method(D_METHOD("set_attenuation", "attenuation"), &GPUParticlesAttractor2D::set_attenuation);
	ClassDB::bind_method(D_METHOD("get_attenuation"), &GPUParticlesAttractor2D::get_attenuation);

	ClassDB::bind_method(D_METHOD("set_directionality", "amount"), &GPUParticlesAttractor2D::set_directionality);
	ClassDB::bind_method(D_METHOD("get_directionality"), &GPUParticlesAttractor2D::get_directionality);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "strength", PROPERTY_HINT_RANGE, "-128,128,0.01,or_greater,or_less"), "set_strength", "get_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "attenuation", PROPERTY_HINT_EXP_EASING, "0,8,0.01"), "set_attenuation", "get_attenuation");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "directionality", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_directionality", "get_directionality");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "cull_mask", PROPERTY_HINT_LAYERS_3D_RENDER), "set_cull_mask", "get_cull_mask");
}

GPUParticlesAttractor2D::GPUParticlesAttractor2D(RS::ParticlesCollisionType p_type) {
	collision = RS::get_singleton()->particles_collision_create();
	RS::get_singleton()->particles_collision_set_collision_type(collision, p_type);
	set_base(collision);
}
GPUParticlesAttractor2D::~GPUParticlesAttractor2D() {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	RS::get_singleton()->free(collision);
}

/////////////////////////////////

void GPUParticlesAttractorCircle2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &GPUParticlesAttractorCircle2D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &GPUParticlesAttractorCircle2D::get_radius);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.01,1024,0.01,or_greater,suffix:m"), "set_radius", "get_radius");
}

void GPUParticlesAttractorCircle2D::set_radius(real_t p_radius) {
	radius = p_radius;
	RS::get_singleton()->particles_collision_set_sphere_radius(_get_collision(), radius);
	update_gizmos();
}

real_t GPUParticlesAttractorCircle2D::get_radius() const {
	return radius;
}

GPUParticlesAttractorCircle2D::GPUParticlesAttractorCircle2D() :
		GPUParticlesAttractor2D(RS::PARTICLES_COLLISION_TYPE_SPHERE_ATTRACT) {
}

GPUParticlesAttractorCircle2D::~GPUParticlesAttractorCircle2D() {
}

///////////////////////////

void GPUParticlesAttractorRect2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &GPUParticlesAttractorRect2D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &GPUParticlesAttractorRect2D::get_size);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_RANGE, "0.01,1024,0.01,or_greater,suffix:m"), "set_size", "get_size");
}

void GPUParticlesAttractorRect2D::set_size(const Vector2 &p_size) {
	size = p_size;
	//RS::get_singleton()->particles_collision_set_box_extents(_get_collision(), size / 2);
	update_gizmos();
}

Vector2 GPUParticlesAttractorRect2D::get_size() const {
	return size;
}

GPUParticlesAttractorRect2D::GPUParticlesAttractorRect2D() :
		GPUParticlesAttractor2D(RS::PARTICLES_COLLISION_TYPE_BOX_ATTRACT) {
}

GPUParticlesAttractorRect2D::~GPUParticlesAttractorRect2D() {
}
