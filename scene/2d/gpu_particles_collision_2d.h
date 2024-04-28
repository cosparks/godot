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

#ifndef GPU_PARTICLES_COLLISION_2D_H
#define GPU_PARTICLES_COLLISION_2D_H

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

#endif // GPU_PARTICLES_COLLISION_2D_H
