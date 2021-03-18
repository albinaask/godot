#ifndef XPBD_PHYSICS_SERVER_H
#define XPBD_PHYSICS_SERVER_H

#include "core/templates/rid.h"
#include "servers/physics_server_3d.h"

class XPBDPhysicsServer3D : public PhysicsServer3D {
	GDCLASS(XPBDPhysicsServer3D, PhysicsServer3D);

	bool active = true;

public:
	XPBDPhysicsServer3D();
	~XPBDPhysicsServer3D();
	
	virtual RID plane_shape_create() override;
	virtual RID ray_shape_create() override;
	virtual RID sphere_shape_create() override;
	virtual RID box_shape_create() override;
	virtual RID capsule_shape_create() override;
	virtual RID cylinder_shape_create() override;
	virtual RID convex_polygon_shape_create() override;
	virtual RID concave_polygon_shape_create() override;
	virtual RID heightmap_shape_create() override;
	virtual RID custom_shape_create() override;

	virtual void shape_set_data(RID p_shape, const Variant &p_data) override;
	virtual void shape_set_custom_solver_bias(RID p_shape, real_t p_bias) override;

	virtual ShapeType shape_get_type(RID p_shape) const override;
	virtual Variant shape_get_data(RID p_shape) const override;

	virtual void shape_set_margin(RID p_shape, real_t p_margin) override;
	virtual real_t shape_get_margin(RID p_shape) const override;

	virtual real_t shape_get_custom_solver_bias(RID p_shape) const override;

	// SPACE API

	virtual RID space_create() override;
	virtual void space_set_active(RID p_space, bool p_active) override;
	virtual bool space_is_active(RID p_space) const override;

	/// Not supported
	virtual void space_set_param(RID p_space, SpaceParameter p_param, real_t p_value) override;
	/// Not supported
	virtual real_t space_get_param(RID p_space, SpaceParameter p_param) const override;

	virtual PhysicsDirectSpaceState3D *space_get_direct_state(RID p_space) override;

	virtual void space_set_debug_contacts(RID p_space, int p_max_contacts) override;
	virtual Vector<Vector3> space_get_contacts(RID p_space) const override;
	virtual int space_get_contact_count(RID p_space) const override;


	// area API

	virtual RID area_create() override;

	virtual void area_set_space(RID p_area, RID p_space) override;

	virtual RID area_get_space(RID p_area) const override;

	virtual void area_set_space_override_mode(RID p_area, AreaSpaceOverrideMode p_mode) override;
	virtual AreaSpaceOverrideMode area_get_space_override_mode(RID p_area) const override;

	virtual void area_add_shape(RID p_area, RID p_shape, const Transform &p_transform = Transform(), bool p_disabled = false) override;
	virtual void area_set_shape(RID p_area, int p_shape_idx, RID p_shape) override;
	virtual void area_set_shape_transform(RID p_area, int p_shape_idx, const Transform &p_transform) override;
	virtual int area_get_shape_count(RID p_area) const override;
	virtual RID area_get_shape(RID p_area, int p_shape_idx) const override;
	virtual Transform area_get_shape_transform(RID p_area, int p_shape_idx) const override;
	virtual void area_remove_shape(RID p_area, int p_shape_idx) override;
	virtual void area_clear_shapes(RID p_area) override;
	virtual void area_set_shape_disabled(RID p_area, int p_shape_idx, bool p_disabled) override;
	virtual void area_attach_object_instance_id(RID p_area, ObjectID p_id) override;
	virtual ObjectID area_get_object_instance_id(RID p_area) const override;

	virtual void area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value) override;
	virtual Variant area_get_param(RID p_area, AreaParameter p_param) const override;

	virtual void area_set_transform(RID p_area, const Transform &p_transform) override;
	virtual Transform area_get_transform(RID p_area) const override;

	virtual void area_set_collision_mask(RID p_area, uint32_t p_mask) override;
	virtual void area_set_collision_layer(RID p_area, uint32_t p_layer) override;

	virtual void area_set_monitorable(RID p_area, bool p_monitorable) override;
	virtual void area_set_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method) override;
	virtual void area_set_area_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method) override;
	virtual void area_set_ray_pickable(RID p_area, bool p_enable) override;
	//virtual bool area_is_ray_pickable(RID p_area) const override;

	virtual RID body_create() override;
	virtual RID body_create(BodyMode p_mode = BODY_MODE_RIGID, bool p_init_sleeping = false);

	virtual void body_set_space(RID p_body, RID p_space) override;
	virtual RID body_get_space(RID p_body) const override;

	virtual void body_set_mode(RID p_body, BodyMode p_mode) override;
	virtual BodyMode body_get_mode(RID p_body) const override;

	virtual void body_add_shape(RID p_body, RID p_shape, const Transform &p_transform = Transform(), bool p_disabled = false) override;
	// Not supported, Please remove and add new shape
	virtual void body_set_shape(RID p_body, int p_shape_idx, RID p_shape) override;
	virtual void body_set_shape_transform(RID p_body, int p_shape_idx, const Transform &p_transform) override;

	virtual int body_get_shape_count(RID p_body) const override;
	virtual RID body_get_shape(RID p_body, int p_shape_idx) const override;
	virtual Transform body_get_shape_transform(RID p_body, int p_shape_idx) const override;

	virtual void body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled) override;

	virtual void body_remove_shape(RID p_body, int p_shape_idx) override;
	virtual void body_clear_shapes(RID p_body) override;

	// Used for Rigid and Soft Bodies
	virtual void body_attach_object_instance_id(RID p_body, ObjectID p_id) override;
	virtual ObjectID body_get_object_instance_id(RID p_body) const override;

	virtual void body_set_enable_continuous_collision_detection(RID p_body, bool p_enable) override;
	virtual bool body_is_continuous_collision_detection_enabled(RID p_body) const override;

	virtual void body_set_collision_layer(RID p_body, uint32_t p_layer) override;
	virtual uint32_t body_get_collision_layer(RID p_body) const override;

	virtual void body_set_collision_mask(RID p_body, uint32_t p_mask) override;
	virtual uint32_t body_get_collision_mask(RID p_body) const override;

	/// This is not supported by physics server
	virtual void body_set_user_flags(RID p_body, uint32_t p_flags) override;
	/// This is not supported by physics server
	virtual uint32_t body_get_user_flags(RID p_body) const override;

	virtual void body_set_param(RID p_body, BodyParameter p_param, real_t p_value) override;
	virtual real_t body_get_param(RID p_body, BodyParameter p_param) const override;

	virtual void body_set_kinematic_safe_margin(RID p_body, real_t p_margin) override;
	virtual real_t body_get_kinematic_safe_margin(RID p_body) const override;

	virtual void body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) override;
	virtual Variant body_get_state(RID p_body, BodyState p_state) const override;

	virtual void body_set_applied_force(RID p_body, const Vector3 &p_force) override;
	virtual Vector3 body_get_applied_force(RID p_body) const override;

	virtual void body_set_applied_torque(RID p_body, const Vector3 &p_torque) override;
	virtual Vector3 body_get_applied_torque(RID p_body) const override;

	virtual void body_add_central_force(RID p_body, const Vector3 &p_force) override;
	virtual void body_add_force(RID p_body, const Vector3 &p_force, const Vector3 &p_position = Vector3()) override;
	virtual void body_add_torque(RID p_body, const Vector3 &p_torque) override;

	virtual void body_apply_central_impulse(RID p_body, const Vector3 &p_impulse) override;
	virtual void body_apply_impulse(RID p_body, const Vector3 &p_impulse, const Vector3 &p_position = Vector3()) override;
	virtual void body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse) override;
	virtual void body_set_axis_velocity(RID p_body, const Vector3 &p_axis_velocity) override;

	virtual void body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock) override;
	virtual bool body_is_axis_locked(RID p_body, BodyAxis p_axis) const override;

	virtual void body_add_collision_exception(RID p_body, RID p_body_b) override;
	virtual void body_remove_collision_exception(RID p_body, RID p_body_b) override;
	virtual void body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) override;

	virtual void body_set_max_contacts_reported(RID p_body, int p_contacts) override;
	virtual int body_get_max_contacts_reported(RID p_body) const override;

	virtual void body_set_contacts_reported_depth_threshold(RID p_body, real_t p_threshold) override;
	virtual real_t body_get_contacts_reported_depth_threshold(RID p_body) const override;

	virtual void body_set_omit_force_integration(RID p_body, bool p_omit) override;
	virtual bool body_is_omitting_force_integration(RID p_body) const override;

	virtual void body_set_force_integration_callback(RID p_body, Object *p_receiver, const StringName &p_method, const Variant &p_udata = Variant()) override;

	virtual void body_set_ray_pickable(RID p_body, bool p_enable) override;
	//virtual bool body_is_ray_pickable(RID p_body) const override;

	// this function only works on physics process, errors and returns null otherwise
	virtual PhysicsDirectBodyState3D *body_get_direct_state(RID p_body) override;

	virtual bool body_test_motion(RID p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, MotionResult *r_result = nullptr, bool p_exclude_raycast_shapes = true) override;
	virtual int body_test_ray_separation(RID p_body, const Transform &p_transform, bool p_infinite_inertia, Vector3 &r_recover_motion, SeparationResult *r_results, int p_result_max, real_t p_margin = 0.001) override;

	/*SOFT BODY API */

	virtual RID soft_body_create() override;

	virtual void soft_body_update_rendering_server(RID p_body, class SoftBodyRenderingServerHandler *p_rendering_server_handler) override;

	virtual void soft_body_set_space(RID p_body, RID p_space) override;
	virtual RID soft_body_get_space(RID p_body) const override;

	virtual void soft_body_set_mesh(RID p_body, const REF &p_mesh) override;

	virtual void soft_body_set_collision_layer(RID p_body, uint32_t p_layer) override;
	virtual uint32_t soft_body_get_collision_layer(RID p_body) const override;

	virtual void soft_body_set_collision_mask(RID p_body, uint32_t p_mask) override;
	virtual uint32_t soft_body_get_collision_mask(RID p_body) const override;

	virtual void soft_body_add_collision_exception(RID p_body, RID p_body_b) override;
	virtual void soft_body_remove_collision_exception(RID p_body, RID p_body_b) override;
	virtual void soft_body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) override;

	virtual void soft_body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) override;
	virtual Variant soft_body_get_state(RID p_body, BodyState p_state) const override;

	/// Special function. This function has bad performance
	virtual void soft_body_set_transform(RID p_body, const Transform &p_transform) override;
	virtual Vector3 soft_body_get_vertex_position(RID p_body, int vertex_index) const override;

	virtual void soft_body_set_ray_pickable(RID p_body, bool p_enable) override;
	virtual bool soft_body_is_ray_pickable(RID p_body);

	virtual void soft_body_set_simulation_precision(RID p_body, int p_simulation_precision) override;
	virtual int soft_body_get_simulation_precision(RID p_body) const override;

	virtual void soft_body_set_total_mass(RID p_body, real_t p_total_mass) override;
	virtual real_t soft_body_get_total_mass(RID p_body) const override;

	virtual void soft_body_set_linear_stiffness(RID p_body, real_t p_stiffness) override;
	virtual real_t soft_body_get_linear_stiffness(RID p_body) const override;

	virtual void soft_body_set_angular_stiffness(RID p_body, real_t p_stiffness) override;
	virtual real_t soft_body_get_angular_stiffness(RID p_body) const override;

	virtual void soft_body_set_volume_stiffness(RID p_body, real_t p_stiffness) override;
	virtual real_t soft_body_get_volume_stiffness(RID p_body) const override;

	virtual void soft_body_set_pressure_coefficient(RID p_body, real_t p_pressure_coefficient) override;
	virtual real_t soft_body_get_pressure_coefficient(RID p_body) const override ;

	virtual void soft_body_set_pose_matching_coefficient(RID p_body, real_t p_pose_matching_coefficient) override;
	virtual real_t soft_body_get_pose_matching_coefficient(RID p_body) const override;

	virtual void soft_body_set_damping_coefficient(RID p_body, real_t p_damping_coefficient) override;
	virtual real_t soft_body_get_damping_coefficient(RID p_body) const override;

	virtual void soft_body_set_drag_coefficient(RID p_body, real_t p_drag_coefficient) override;
	virtual real_t soft_body_get_drag_coefficient(RID p_body) const override;

	virtual void soft_body_move_point(RID p_body, int p_point_index, const Vector3 &p_global_position) override;
	virtual Vector3 soft_body_get_point_global_position(RID p_body, int p_point_index) const override;

	virtual Vector3 soft_body_get_point_offset(RID p_body, int p_point_index) const override;

	virtual void soft_body_remove_all_pinned_points(RID p_body) override;
	virtual void soft_body_pin_point(RID p_body, int p_point_index, bool p_pin) override;
	virtual bool soft_body_is_point_pinned(RID p_body, int p_point_index) const override;

	//JOINT API

	virtual RID joint_create() override;
	virtual void joint_clear(RID p_joint) override;

	virtual JointType joint_get_type(RID p_joint) const override;

	virtual void joint_set_solver_priority(RID p_joint, int p_priority) override;
	virtual int joint_get_solver_priority(RID p_joint) const override;

	virtual void joint_disable_collisions_between_bodies(RID p_joint, const bool p_disable) override;
	virtual bool joint_is_disabled_collisions_between_bodies(RID p_joint) const override;

	

	virtual void pin_joint_set_param(RID p_joint, PinJointParam p_param, real_t p_value) override;
	virtual real_t pin_joint_get_param(RID p_joint, PinJointParam p_param) const override;

	virtual void pin_joint_set_local_a(RID p_joint, const Vector3 &p_A) override;
	virtual Vector3 pin_joint_get_local_a(RID p_joint) const override;

	virtual void pin_joint_set_local_b(RID p_joint, const Vector3 &p_B) override;
	virtual Vector3 pin_joint_get_local_b(RID p_joint) const override;

	virtual void hinge_joint_set_param(RID p_joint, HingeJointParam p_param, real_t p_value) override;
	virtual real_t hinge_joint_get_param(RID p_joint, HingeJointParam p_param) const override;

	virtual void hinge_joint_set_flag(RID p_joint, HingeJointFlag p_flag, bool p_value) override;
	virtual bool hinge_joint_get_flag(RID p_joint, HingeJointFlag p_flag) const override;

	virtual void slider_joint_set_param(RID p_joint, SliderJointParam p_param, real_t p_value) override;
	virtual real_t slider_joint_get_param(RID p_joint, SliderJointParam p_param) const override;	

	virtual void cone_twist_joint_set_param(RID p_joint, ConeTwistJointParam p_param, real_t p_value) override;
	virtual real_t cone_twist_joint_get_param(RID p_joint, ConeTwistJointParam p_param) const override;

	virtual void generic_6dof_joint_set_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param, real_t p_value) override;
	virtual real_t generic_6dof_joint_get_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param)const override;

	virtual void generic_6dof_joint_set_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag, bool p_enable) override;
	virtual bool generic_6dof_joint_get_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag) const override;

	virtual RID joint_create_slider(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) const;
	virtual RID joint_create_cone_twist(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) const;
	virtual RID joint_create_generic_6dof(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B)const;
	virtual RID joint_create_hinge(RID p_body_A, const Transform &p_hinge_A, RID p_body_B, const Transform &p_hinge_B) const;
	virtual RID joint_create_hinge_simple(RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B) const;
	virtual RID joint_create_pin(RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B) const;

	virtual void joint_make_pin(RID p_joint, RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B)override;
	virtual void joint_make_hinge(RID p_joint, RID p_body_A, const Transform &p_hinge_A, RID p_body_B, const Transform &p_hinge_B) override;
	virtual void joint_make_hinge_simple(RID p_joint, RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, 
								const Vector3 &p_pivot_B, const Vector3 &p_axis_B) override;
	virtual void joint_make_slider(RID p_joint, RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) override;
	virtual void joint_make_cone_twist(RID p_joint, RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) override;
	virtual void joint_make_generic_6dof(RID p_joint, RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) override;


	//MISC

	virtual void free(RID p_rid) override;

	virtual void set_active(bool p_active) override {
		active = p_active;
	}

	bool isActive() {
		return active;
	}

	virtual void init() override;
	virtual void step(real_t p_deltaTime) override;
	virtual void sync() override;
	virtual void flush_queries() override;
	virtual void end_sync() override;
	virtual void finish() override;

	virtual bool is_flushing_queries() const override;

	virtual int get_process_info(ProcessInfo p_info) override;


};
#endif