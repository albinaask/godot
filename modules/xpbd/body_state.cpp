#include "body_state.h"
#include "servers/physics_server_3d.h"
#include "space_state.h"
#include "core/object/object.h"

XPBDDirectBodyState3D::XPBDDirectBodyState3D() {}
XPBDDirectBodyState3D::~XPBDDirectBodyState3D(){}

void XPBDDirectBodyState3D::set_linear_velocity(const Vector3 &p_velocity) {return;}
Vector3 XPBDDirectBodyState3D::get_linear_velocity() const {return Vector3();}

Vector3 XPBDDirectBodyState3D::get_total_gravity() const{return Vector3();}
real_t XPBDDirectBodyState3D::get_total_angular_damp() const{return 0;}
real_t XPBDDirectBodyState3D::get_total_linear_damp() const{return 0;}
Vector3 XPBDDirectBodyState3D::get_center_of_mass() const {return Vector3();}
Basis XPBDDirectBodyState3D::get_principal_inertia_axes() const {return Basis();}
real_t XPBDDirectBodyState3D::get_inverse_mass() const {return 0;} // get the mass
Vector3 XPBDDirectBodyState3D::get_inverse_inertia() const {return Vector3();} // get density of this body space
Basis XPBDDirectBodyState3D::get_inverse_inertia_tensor() const {return Basis();} // get density of this body space
bool XPBDDirectBodyState3D::is_sleeping() const {return false;}
int XPBDDirectBodyState3D::get_contact_count() const {return false;}
real_t XPBDDirectBodyState3D::get_step() const {return 0;}

Vector3 XPBDDirectBodyState3D::get_contact_local_position(int p_contact_index) const{return Vector3();}
Vector3 XPBDDirectBodyState3D::get_contact_local_normal(int p_contact_index) const{return Vector3();}
real_t XPBDDirectBodyState3D::get_contact_impulse(int p_contact_index) const{return 0;}
int XPBDDirectBodyState3D::get_contact_local_shape(int p_contact_index) const{return 0;}

RID XPBDDirectBodyState3D::get_contact_collider(int p_contact_index) const{return RID();}
Vector3 XPBDDirectBodyState3D::get_contact_collider_position(int p_contact_index) const{return Vector3();}
ObjectID XPBDDirectBodyState3D::get_contact_collider_id(int p_contact_index) const{return ObjectID();}
int XPBDDirectBodyState3D::get_contact_collider_shape(int p_contact_index) const{return 0;}
Vector3 XPBDDirectBodyState3D::get_contact_collider_velocity_at_position(int p_contact_index) const{return Vector3();}

void XPBDDirectBodyState3D::set_angular_velocity(const Vector3 &p_velocity) {return;}
Vector3 XPBDDirectBodyState3D::get_angular_velocity() const {return Vector3();}

void XPBDDirectBodyState3D::set_transform(const Transform &p_transform) {return;}
Transform XPBDDirectBodyState3D::get_transform() const {return Transform();}

void XPBDDirectBodyState3D::add_central_force(const Vector3 &p_force) {return;}
void XPBDDirectBodyState3D::apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position) {return;}
void XPBDDirectBodyState3D::add_torque(const Vector3 &p_torque) {return;}
void XPBDDirectBodyState3D::apply_central_impulse(const Vector3 &p_impulse) {return;}
void XPBDDirectBodyState3D::apply_torque_impulse(const Vector3 &p_impulse) {return;}
void XPBDDirectBodyState3D::add_force(const Vector3 &p_force, const Vector3 &p_position) {}

void XPBDDirectBodyState3D::set_sleep_state(bool p_sleep) {return;}

Object *XPBDDirectBodyState3D::get_contact_collider_object(int p_contact_idx) const {return memnew(Object);}

void XPBDDirectBodyState3D::integrate_forces() {return;}

PhysicsDirectSpaceState3D *XPBDDirectBodyState3D::get_space_state() {return &space;}