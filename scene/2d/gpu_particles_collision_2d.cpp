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

	ADD_PROPERTY(PropertyInfo(Variant::INT, "cull_mask", PROPERTY_HINT_LAYERS_2D_RENDER), "set_cull_mask", "get_cull_mask");
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
