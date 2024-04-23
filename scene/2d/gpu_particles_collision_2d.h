/**************************************************************************/
/*  gpu_particles_collision_2d.h                                          */
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

#ifndef GPU_PARTICLES_COLLISION_3D_H
#define GPU_PARTICLES_COLLISION_3D_H

#include "core/templates/local_vector.h"
#include "scene/2d/node_2d.h"

class GPUParticlesCollision2D : public Node2D {
	GDCLASS(GPUParticlesCollision2D, Node2D);

	uint32_t cull_mask = 0xFFFFFFFF;
	RID collision;

protected:
	_FORCE_INLINE_ RID _get_collision() { return collision; }
	static void _bind_methods();

	GPUParticlesCollision2D(RS::ParticlesCollisionType p_type);

public:
	void set_cull_mask(uint32_t p_cull_mask);
	uint32_t get_cull_mask() const;

	~GPUParticlesCollision2D();
};

class GPUParticlesCollisionCircle2D : public GPUParticlesCollision2D {
	GDCLASS(GPUParticlesCollisionCircle2D, GPUParticlesCollision2D);

	real_t radius = 1.0;

protected:
	static void _bind_methods();

public:
	void set_radius(real_t p_radius);
	real_t get_radius() const;

	GPUParticlesCollisionCircle2D();
	~GPUParticlesCollisionCircle2D();
};

class GPUParticlesCollisionRect2D : public GPUParticlesCollision2D {
	GDCLASS(GPUParticlesCollisionRect2D, GPUParticlesCollision2D);

	Vector3 size = Vector3(2, 2, 2);

protected:
	static void _bind_methods();

public:
	void set_size(const Vector3 &p_size);
	Vector3 get_size() const;

	GPUParticlesCollisionRect2D();
	~GPUParticlesCollisionRect2D();
};

class GPUParticlesCollisionSDF2D : public GPUParticlesCollision2D {
	GDCLASS(GPUParticlesCollisionSDF2D, GPUParticlesCollision2D);

public:
	enum Resolution {
		RESOLUTION_16,
		RESOLUTION_32,
		RESOLUTION_64,
		RESOLUTION_128,
		RESOLUTION_256,
		RESOLUTION_512,
		RESOLUTION_MAX,
	};

	typedef void (*BakeBeginFunc)(int);
	typedef void (*BakeStepFunc)(int, const String &);
	typedef void (*BakeEndFunc)();

private:
	Vector3 size = Vector3(2, 2, 2);
	Resolution resolution = RESOLUTION_64;
	uint32_t bake_mask = 0xFFFFFFFF;
	Ref<Texture3D> texture;
	float thickness = 1.0;

	struct PlotMesh {
		Ref<Mesh> mesh;
		Transform3D local_xform;
	};

	void _find_meshes(const AABB &p_aabb, Node *p_at_node, List<PlotMesh> &plot_meshes);

	struct BVH {
		enum {
			LEAF_BIT = 1 << 30,
			LEAF_MASK = LEAF_BIT - 1
		};
		AABB bounds;
		uint32_t children[2] = {};
	};

	struct FacePos {
		Vector3 center;
		uint32_t index = 0;
	};

	struct FaceSort {
		uint32_t axis = 0;
		bool operator()(const FacePos &p_left, const FacePos &p_right) const {
			return p_left.center[axis] < p_right.center[axis];
		}
	};

	uint32_t _create_bvh(LocalVector<BVH> &bvh_tree, FacePos *p_faces, uint32_t p_face_count, const Face3 *p_triangles, float p_thickness);

	struct ComputeSDFParams {
		float *cells = nullptr;
		Vector3i size;
		float cell_size = 0.0;
		Vector3 cell_offset;
		const BVH *bvh = nullptr;
		const Face3 *triangles = nullptr;
		float thickness = 0.0;
	};

	void _find_closest_distance(const Vector3 &p_pos, const BVH *p_bvh, uint32_t p_bvh_cell, const Face3 *p_triangles, float p_thickness, float &r_closest_distance);
	void _compute_sdf_z(uint32_t p_z, ComputeSDFParams *params);
	void _compute_sdf(ComputeSDFParams *params);

protected:
	static void _bind_methods();

public:
	virtual PackedStringArray get_configuration_warnings() const override;

	void set_thickness(float p_thickness);
	float get_thickness() const;

	void set_size(const Vector3 &p_size);
	Vector3 get_size() const;

	void set_resolution(Resolution p_resolution);
	Resolution get_resolution() const;

	void set_bake_mask(uint32_t p_mask);
	uint32_t get_bake_mask() const;

	void set_bake_mask_value(int p_layer_number, bool p_enable);
	bool get_bake_mask_value(int p_layer_number) const;

	void set_texture(const Ref<Texture3D> &p_texture);
	Ref<Texture3D> get_texture() const;

	Vector3i get_estimated_cell_size() const;
	Ref<Image> bake();

	static BakeBeginFunc bake_begin_function;
	static BakeStepFunc bake_step_function;
	static BakeEndFunc bake_end_function;

	GPUParticlesCollisionSDF2D();
	~GPUParticlesCollisionSDF2D();
};

VARIANT_ENUM_CAST(GPUParticlesCollisionSDF2D::Resolution)

class GPUParticlesAttractor2D : public Node2D {
	GDCLASS(GPUParticlesAttractor2D, Node2D);

	uint32_t cull_mask = 0xFFFFFFFF;
	RID collision;
	real_t strength = 1.0;
	real_t attenuation = 1.0;
	real_t directionality = 0.0;

protected:
	_FORCE_INLINE_ RID _get_collision() { return collision; }
	static void _bind_methods();

	GPUParticlesAttractor2D(RS::ParticlesCollisionType p_type);

public:
	void set_cull_mask(uint32_t p_cull_mask);
	uint32_t get_cull_mask() const;

	void set_strength(real_t p_strength);
	real_t get_strength() const;

	void set_attenuation(real_t p_attenuation);
	real_t get_attenuation() const;

	void set_directionality(real_t p_directionality);
	real_t get_directionality() const;

	~GPUParticlesAttractor2D();
};

class GPUParticlesAttractorCircle2D : public GPUParticlesAttractor2D {
	GDCLASS(GPUParticlesAttractorCircle2D, GPUParticlesAttractor2D);

	real_t radius = 1.0;

protected:
	static void _bind_methods();

public:
	void set_radius(real_t p_radius);
	real_t get_radius() const;

	GPUParticlesAttractorCircle2D();
	~GPUParticlesAttractorCircle2D();
};

class GPUParticlesAttractorRect2D : public GPUParticlesAttractor2D {
	GDCLASS(GPUParticlesAttractorRect2D, GPUParticlesAttractor2D);

	Vector2 size = Vector2(2, 2);

protected:
	static void _bind_methods();

public:
	void set_size(const Vector2 &p_size);
	Vector2 get_size() const;

	GPUParticlesAttractorRect2D();
	~GPUParticlesAttractorRect2D();
};

#endif // GPU_PARTICLES_COLLISION_3D_H
