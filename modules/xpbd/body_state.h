#ifndef XPBD_DIRECT_BODY_STATE_H
#define XPBD_DIRECT_BODY_STATE_H

#include "servers/physics_server_3d.h"
#include "space_state.h"
class XPBDDirectBodyState3D : public PhysicsDirectBodyState3D {
    GDCLASS(XPBDDirectBodyState3D, PhysicsDirectBodyState3D);

private:
    XPBDDirectSpaceState3D space;

public:

    XPBDDirectBodyState3D();
	~XPBDDirectBodyState3D();

    virtual Vector3 get_total_gravity() const override;
	virtual real_t get_total_angular_damp() const override;
	virtual real_t get_total_linear_damp() const override;

	virtual Vector3 get_center_of_mass() const override;
	virtual Basis get_principal_inertia_axes() const override;
	virtual real_t get_inverse_mass() const override; // get the mass
	virtual Vector3 get_inverse_inertia() const override; // get density of this body space
	virtual Basis get_inverse_inertia_tensor() const override; // get density of this body space

	virtual void set_linear_velocity(const Vector3 &p_velocity) override;
	virtual Vector3 get_linear_velocity() const override;

	virtual void set_angular_velocity(const Vector3 &p_velocity) override;
	virtual Vector3 get_angular_velocity() const override;

	virtual void set_transform(const Transform &p_transform) override;
	virtual Transform get_transform() const override;

	virtual void add_central_force(const Vector3 &p_force) override;
	virtual void add_force(const Vector3 &p_force, const Vector3 &p_position) override;
	virtual void add_torque(const Vector3 &p_torque) override;
	virtual void apply_central_impulse(const Vector3 &p_impulse) override;
	virtual void apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position) override;
	virtual void apply_torque_impulse(const Vector3 &p_impulse) override;

	virtual void set_sleep_state(bool p_sleep) override;
	virtual bool is_sleeping() const override;

	virtual int get_contact_count() const override;

	virtual Vector3 get_contact_local_position(int p_contact_idx) const override;
	virtual Vector3 get_contact_local_normal(int p_contact_idx) const override;
	virtual real_t get_contact_impulse(int p_contact_idx) const override;
	virtual int get_contact_local_shape(int p_contact_idx) const override;

	virtual RID get_contact_collider(int p_contact_idx) const override;
	virtual Vector3 get_contact_collider_position(int p_contact_idx) const override;
	virtual ObjectID get_contact_collider_id(int p_contact_idx) const override;
	virtual Object *get_contact_collider_object(int p_contact_idx) const override;
	virtual int get_contact_collider_shape(int p_contact_idx) const override;
	virtual Vector3 get_contact_collider_velocity_at_position(int p_contact_idx) const override;

	virtual real_t get_step() const override;
	virtual void integrate_forces() override;

	virtual PhysicsDirectSpaceState3D *get_space_state() override;
};
#endif