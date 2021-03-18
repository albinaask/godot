#include "register_types.h"
#include "core/templates/rid.h"
#include "servers/physics_server_3d.h"
#include "xpbd_physics_server.h"

const RID zero_RID;

XPBDPhysicsServer3D::XPBDPhysicsServer3D() : 
        PhysicsServer3D() {}

XPBDPhysicsServer3D::~XPBDPhysicsServer3D() {}
	
    //shape API
	
RID XPBDPhysicsServer3D::plane_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::ray_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::sphere_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::box_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::capsule_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::cylinder_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::convex_polygon_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::concave_polygon_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::heightmap_shape_create(){return zero_RID;}
RID XPBDPhysicsServer3D::custom_shape_create(){return zero_RID;}

void XPBDPhysicsServer3D::shape_set_data(RID p_shape, const Variant &p_data){}

void XPBDPhysicsServer3D::shape_set_custom_solver_bias(RID p_shape, real_t p_bias){}

PhysicsServer3D::ShapeType XPBDPhysicsServer3D::shape_get_type(RID p_shape) const{return PhysicsServer3D::ShapeType::SHAPE_BOX;}
Variant XPBDPhysicsServer3D::shape_get_data(RID p_shape) const{return 0;}

void XPBDPhysicsServer3D::shape_set_margin(RID p_shape, real_t p_margin){return;}
real_t XPBDPhysicsServer3D::shape_get_margin(RID p_shape) const{return 0;}

real_t XPBDPhysicsServer3D::shape_get_custom_solver_bias(RID p_shape) const{return 0;}

//space API

RID XPBDPhysicsServer3D::space_create(){return zero_RID;}
void XPBDPhysicsServer3D::space_set_active(RID p_space, bool p_active){}
bool XPBDPhysicsServer3D::space_is_active(RID p_space) const{return false;}

void XPBDPhysicsServer3D::space_set_param(RID p_space, SpaceParameter p_param, real_t p_value){return;}
real_t XPBDPhysicsServer3D::space_get_param(RID p_space, SpaceParameter p_param) const{return 0;}

// this function only works on physics process, errors and returns null otherwise
PhysicsDirectSpaceState3D *space_get_direct_state(RID p_space){return 0;}

void XPBDPhysicsServer3D::space_set_debug_contacts(RID p_space, int p_max_contacts){return;}
Vector<Vector3> XPBDPhysicsServer3D::space_get_contacts(RID p_space) const{return Vector<Vector3>();}
int XPBDPhysicsServer3D::space_get_contact_count(RID p_space) const{return 0;}

//area API
RID XPBDPhysicsServer3D::area_create(){return zero_RID;}

void XPBDPhysicsServer3D::area_set_space(RID p_area, RID p_space){return;}
RID XPBDPhysicsServer3D::area_get_space(RID p_area) const{return zero_RID;}

void XPBDPhysicsServer3D::area_set_space_override_mode(RID p_area, AreaSpaceOverrideMode p_mode){return;}
PhysicsServer3D::AreaSpaceOverrideMode XPBDPhysicsServer3D::area_get_space_override_mode(RID p_area) const{return PhysicsServer3D::AreaSpaceOverrideMode();}

void XPBDPhysicsServer3D::area_add_shape(RID p_area, RID p_shape, const Transform &p_transform, bool p_disabled){return;}
void XPBDPhysicsServer3D::area_set_shape(RID p_area, int p_shape_idx, RID p_shape){return;}
void XPBDPhysicsServer3D::area_set_shape_transform(RID p_area, int p_shape_idx, const Transform &p_transform){return;}

int XPBDPhysicsServer3D::area_get_shape_count(RID p_area) const{return 0;}
RID XPBDPhysicsServer3D::area_get_shape(RID p_area, int p_shape_idx) const{return zero_RID;}
Transform XPBDPhysicsServer3D::area_get_shape_transform(RID p_area, int p_shape_idx) const{return Transform();}

void XPBDPhysicsServer3D::area_remove_shape(RID p_area, int p_shape_idx){return;}
void XPBDPhysicsServer3D::area_clear_shapes(RID p_area){return;}

void XPBDPhysicsServer3D::area_set_shape_disabled(RID p_area, int p_shape_idx, bool p_disabled){return;}

void XPBDPhysicsServer3D::area_attach_object_instance_id(RID p_area, ObjectID p_id){return;}
ObjectID XPBDPhysicsServer3D::area_get_object_instance_id(RID p_area) const{return ObjectID();}

void XPBDPhysicsServer3D::area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value){return;}
void XPBDPhysicsServer3D::area_set_transform(RID p_area, const Transform &p_transform){return;}

Variant XPBDPhysicsServer3D::area_get_param(RID p_parea, AreaParameter p_param) const{return 0;}
Transform XPBDPhysicsServer3D::area_get_transform(RID p_area) const{return Transform();}

void XPBDPhysicsServer3D::area_set_collision_mask(RID p_area, uint32_t p_mask){return;}
void XPBDPhysicsServer3D::area_set_collision_layer(RID p_area, uint32_t p_layer){return;}

void XPBDPhysicsServer3D::area_set_monitorable(RID p_area, bool p_monitorable){return;}

void XPBDPhysicsServer3D::area_set_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method){return;}
void XPBDPhysicsServer3D::area_set_area_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method){return;}

void XPBDPhysicsServer3D::area_set_ray_pickable(RID p_area, bool p_enable){return;}

// BODY API
RID XPBDPhysicsServer3D::body_create(BodyMode p_mode, bool p_init_sleeping){return zero_RID;}

void XPBDPhysicsServer3D::body_set_space(RID p_body, RID p_space){return;}
RID XPBDPhysicsServer3D::body_get_space(RID p_body) const{return zero_RID;}

void XPBDPhysicsServer3D::body_set_mode(RID p_body, BodyMode p_mode){return;}
PhysicsServer3D::BodyMode XPBDPhysicsServer3D::body_get_mode(RID p_body) const{return PhysicsServer3D::BodyMode::BODY_MODE_RIGID;}

void XPBDPhysicsServer3D::body_add_shape(RID p_body, RID p_shape, const Transform &p_transform, bool p_disabled){return;}
void XPBDPhysicsServer3D::body_set_shape(RID p_body, int p_shape_idx, RID p_shape){return;}
void XPBDPhysicsServer3D::body_set_shape_transform(RID p_body, int p_shape_idx, const Transform &p_transform){return;}

int XPBDPhysicsServer3D::body_get_shape_count(RID p_body) const{return 0;}
RID XPBDPhysicsServer3D::body_get_shape(RID p_body, int p_shape_idx) const{return zero_RID;}
Transform XPBDPhysicsServer3D::body_get_shape_transform(RID p_body, int p_shape_idx) const{return Transform();}

void XPBDPhysicsServer3D::body_remove_shape(RID p_body, int p_shape_idx){return;}
void XPBDPhysicsServer3D::body_clear_shapes(RID p_body){return;}

void XPBDPhysicsServer3D::body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled){return;}

void XPBDPhysicsServer3D::body_attach_object_instance_id(RID p_body, ObjectID p_id){return;}
ObjectID XPBDPhysicsServer3D::body_get_object_instance_id(RID p_body) const{return ObjectID();}

void XPBDPhysicsServer3D::body_set_enable_continuous_collision_detection(RID p_body, bool p_enable){return;}
bool XPBDPhysicsServer3D::body_is_continuous_collision_detection_enabled(RID p_body) const{return false;}

void XPBDPhysicsServer3D::body_set_collision_layer(RID p_body, uint32_t p_layer){return;}
uint32_t XPBDPhysicsServer3D::body_get_collision_layer(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::body_set_collision_mask(RID p_body, uint32_t p_mask){return;}
uint32_t XPBDPhysicsServer3D::body_get_collision_mask(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::body_set_user_flags(RID p_body, uint32_t p_flags){return;}
uint32_t XPBDPhysicsServer3D::body_get_user_flags(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::body_set_param(RID p_body, BodyParameter p_param, real_t p_value){return;}
real_t XPBDPhysicsServer3D::body_get_param(RID p_body, BodyParameter p_param) const{return 0;}

void XPBDPhysicsServer3D::body_set_kinematic_safe_margin(RID p_body, real_t p_margin){return;}
real_t XPBDPhysicsServer3D::body_get_kinematic_safe_margin(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::body_set_state(RID p_body, BodyState p_state, const Variant &p_variant){return;}
Variant XPBDPhysicsServer3D::body_get_state(RID p_body, BodyState p_state) const{return 0;}

void XPBDPhysicsServer3D::body_set_applied_force(RID p_body, const Vector3 &p_force){return;}
Vector3 XPBDPhysicsServer3D::body_get_applied_force(RID p_body) const{return Vector3();}

void XPBDPhysicsServer3D::body_set_applied_torque(RID p_body, const Vector3 &p_torque){return;}
Vector3 XPBDPhysicsServer3D::body_get_applied_torque(RID p_body) const{return Vector3();}

void XPBDPhysicsServer3D::body_add_central_force(RID p_body, const Vector3 &p_force){return;}
void XPBDPhysicsServer3D::body_add_force(RID p_body, const Vector3 &p_force, const Vector3 &p_position){return;}
void XPBDPhysicsServer3D::body_add_torque(RID p_body, const Vector3 &p_torque){return;}

void XPBDPhysicsServer3D::body_apply_central_impulse(RID p_body, const Vector3 &p_impulse){return;}
void XPBDPhysicsServer3D::body_apply_impulse(RID p_body, const Vector3 &p_impulse, const Vector3 &p_position){return;}
void XPBDPhysicsServer3D::body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse){return;}
void XPBDPhysicsServer3D::body_set_axis_velocity(RID p_body, const Vector3 &p_axis_velocity){return;}

void XPBDPhysicsServer3D::body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock){return;}
bool XPBDPhysicsServer3D::body_is_axis_locked(RID p_body, BodyAxis p_axis) const{return false;}

void XPBDPhysicsServer3D::body_add_collision_exception(RID p_body, RID p_body_b){return;}
void XPBDPhysicsServer3D::body_remove_collision_exception(RID p_body, RID p_body_b){return;}
void XPBDPhysicsServer3D::body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions){return;}

void XPBDPhysicsServer3D::body_set_max_contacts_reported(RID p_body, int p_contacts){return;}
int XPBDPhysicsServer3D::body_get_max_contacts_reported(RID p_body) const{return 0;}

//missing remove
void XPBDPhysicsServer3D::body_set_contacts_reported_depth_threshold(RID p_body, real_t p_threshold){return;}
real_t XPBDPhysicsServer3D::body_get_contacts_reported_depth_threshold(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::body_set_omit_force_integration(RID p_body, bool p_omit){return;}
bool XPBDPhysicsServer3D::body_is_omitting_force_integration(RID p_body) const{return false;}

void XPBDPhysicsServer3D::body_set_force_integration_callback(RID p_body, Object *p_receiver, const StringName &p_method, const Variant &p_udata){return;}

void XPBDPhysicsServer3D::body_set_ray_pickable(RID p_body, bool p_enable){return;}

// this function only works on physics process, errors and returns null otherwise
PhysicsDirectBodyState3D *body_get_direct_state(RID p_body){return 0;}

bool XPBDPhysicsServer3D::body_test_motion(RID p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, MotionResult *r_result, bool p_exclude_raycast_shapes){return 0;}

//soft body API

RID XPBDPhysicsServer3D::soft_body_create(){return zero_RID;}

void XPBDPhysicsServer3D::soft_body_update_rendering_server(RID p_body, class SoftBodyRenderingServerHandler *p_rendering_server_handler){return;}

void XPBDPhysicsServer3D::soft_body_set_space(RID p_body, RID p_space){return;}
RID XPBDPhysicsServer3D::soft_body_get_space(RID p_body) const{return zero_RID;}

void XPBDPhysicsServer3D::soft_body_set_mesh(RID p_body, const REF &p_mesh){return;}

void XPBDPhysicsServer3D::soft_body_set_collision_layer(RID p_body, uint32_t p_layer){return;}
uint32_t XPBDPhysicsServer3D::soft_body_get_collision_layer(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_collision_mask(RID p_body, uint32_t p_mask){return;}
uint32_t XPBDPhysicsServer3D::soft_body_get_collision_mask(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_add_collision_exception(RID p_body, RID p_body_b){return;}
void XPBDPhysicsServer3D::soft_body_remove_collision_exception(RID p_body, RID p_body_b){return;}
void XPBDPhysicsServer3D::soft_body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions){return;}

void XPBDPhysicsServer3D::soft_body_set_state(RID p_body, BodyState p_state, const Variant &p_variant){return;}
Variant XPBDPhysicsServer3D::soft_body_get_state(RID p_body, BodyState p_state) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_transform(RID p_body, const Transform &p_transform){return;}
Vector3 XPBDPhysicsServer3D::soft_body_get_vertex_position(RID p_body, int vertex_index) const{return Vector3();}

void XPBDPhysicsServer3D::soft_body_set_ray_pickable(RID p_body, bool p_enable){return;}

void XPBDPhysicsServer3D::soft_body_set_simulation_precision(RID p_body, int p_simulation_precision){return;}
int XPBDPhysicsServer3D::soft_body_get_simulation_precision(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_total_mass(RID p_body, real_t p_total_mass){return;}
real_t XPBDPhysicsServer3D::soft_body_get_total_mass(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_linear_stiffness(RID p_body, real_t p_stiffness){return;}
real_t XPBDPhysicsServer3D::soft_body_get_linear_stiffness(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_angular_stiffness(RID p_body, real_t p_stiffness){return;}
real_t XPBDPhysicsServer3D::soft_body_get_angular_stiffness(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_volume_stiffness(RID p_body, real_t p_stiffness){return;}
real_t XPBDPhysicsServer3D::soft_body_get_volume_stiffness(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_pressure_coefficient(RID p_body, real_t p_pressure_coefficient){return;}
real_t XPBDPhysicsServer3D::soft_body_get_pressure_coefficient(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_pose_matching_coefficient(RID p_body, real_t p_pose_matching_coefficient){return;}
real_t XPBDPhysicsServer3D::soft_body_get_pose_matching_coefficient(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_damping_coefficient(RID p_body, real_t p_damping_coefficient){return;}
real_t XPBDPhysicsServer3D::soft_body_get_damping_coefficient(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_set_drag_coefficient(RID p_body, real_t p_drag_coefficient){return;}
real_t XPBDPhysicsServer3D::soft_body_get_drag_coefficient(RID p_body) const{return 0;}

void XPBDPhysicsServer3D::soft_body_move_point(RID p_body, int p_point_index, const Vector3 &p_global_position){return;}
Vector3 XPBDPhysicsServer3D::soft_body_get_point_global_position(RID p_body, int p_point_index) const{return Vector3();}

Vector3 XPBDPhysicsServer3D::soft_body_get_point_offset(RID p_body, int p_point_index) const{return Vector3();}

void XPBDPhysicsServer3D::soft_body_remove_all_pinned_points(RID p_body){return;}
void XPBDPhysicsServer3D::soft_body_pin_point(RID p_body, int p_point_index, bool p_pin){return;}
bool XPBDPhysicsServer3D::soft_body_is_point_pinned(RID p_body, int p_point_index) const{return false;}


//Joint API
RID XPBDPhysicsServer3D::joint_create(){return zero_RID;}

void XPBDPhysicsServer3D::joint_clear(RID p_joint){return;}

PhysicsServer3D::JointType XPBDPhysicsServer3D::joint_get_type(RID p_joint) const{return PhysicsServer3D::JointType::JOINT_TYPE_6DOF;}

void XPBDPhysicsServer3D::joint_set_solver_priority(RID p_joint, int p_priority){return;}
int XPBDPhysicsServer3D::joint_get_solver_priority(RID p_joint) const{return 0;}

void XPBDPhysicsServer3D::joint_disable_collisions_between_bodies(RID p_joint, const bool p_disable){return;}
bool XPBDPhysicsServer3D::joint_is_disabled_collisions_between_bodies(RID p_joint) const{return 0;}

void XPBDPhysicsServer3D::joint_make_pin(RID p_joint, RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B){return;}

void XPBDPhysicsServer3D::pin_joint_set_param(RID p_joint, PinJointParam p_param, real_t p_value){return;}
real_t XPBDPhysicsServer3D::pin_joint_get_param(RID p_joint, PinJointParam p_param) const{return 0;}

void XPBDPhysicsServer3D::pin_joint_set_local_a(RID p_joint, const Vector3 &p_A){return;}
Vector3 XPBDPhysicsServer3D::pin_joint_get_local_a(RID p_joint) const{return Vector3();}

void XPBDPhysicsServer3D::pin_joint_set_local_b(RID p_joint, const Vector3 &p_B){return;}
Vector3 XPBDPhysicsServer3D::pin_joint_get_local_b(RID p_joint) const{return Vector3();}

void XPBDPhysicsServer3D::joint_make_hinge(RID p_joint, RID p_body_A, const Transform &p_hinge_A, RID p_body_B, const Transform &p_hinge_B){return;}
void XPBDPhysicsServer3D::joint_make_hinge_simple(RID p_joint, RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B){return;}

void XPBDPhysicsServer3D::hinge_joint_set_param(RID p_joint, HingeJointParam p_param, real_t p_value){return;}
real_t XPBDPhysicsServer3D::hinge_joint_get_param(RID p_joint, HingeJointParam p_param) const{return 0;}

void XPBDPhysicsServer3D::hinge_joint_set_flag(RID p_joint, HingeJointFlag p_flag, bool p_value){return;}
bool XPBDPhysicsServer3D::hinge_joint_get_flag(RID p_joint, HingeJointFlag p_flag) const{return false;}

void XPBDPhysicsServer3D::joint_make_slider(RID p_joint, RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B){return;} //reference frame is A

void XPBDPhysicsServer3D::slider_joint_set_param(RID p_joint, SliderJointParam p_param, real_t p_value){return;}
real_t XPBDPhysicsServer3D::slider_joint_get_param(RID p_joint, SliderJointParam p_param) const{return 0;}

void XPBDPhysicsServer3D::joint_make_cone_twist(RID p_joint, RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B){return;} //reference frame is A

void XPBDPhysicsServer3D::cone_twist_joint_set_param(RID p_joint, ConeTwistJointParam p_param, real_t p_value){return;}
real_t XPBDPhysicsServer3D::cone_twist_joint_get_param(RID p_joint, ConeTwistJointParam p_param) const{return 0;}

void XPBDPhysicsServer3D::joint_make_generic_6dof(RID p_joint, RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B){return;} //reference frame is A

void XPBDPhysicsServer3D::generic_6dof_joint_set_param(RID p_joint, Vector3::Axis, G6DOFJointAxisParam p_param, real_t p_value){return ;}
real_t XPBDPhysicsServer3D::generic_6dof_joint_get_param(RID p_joint, Vector3::Axis, G6DOFJointAxisParam p_param)const{return 0;}

void XPBDPhysicsServer3D::generic_6dof_joint_set_flag(RID p_joint, Vector3::Axis, G6DOFJointAxisFlag p_flag, bool p_enable){return;}
bool XPBDPhysicsServer3D::generic_6dof_joint_get_flag(RID p_joint, Vector3::Axis, G6DOFJointAxisFlag p_flag)const{return false;}

void XPBDPhysicsServer3D::free(RID p_rid){return;}

void XPBDPhysicsServer3D::init(){return;}
void XPBDPhysicsServer3D::step(real_t p_step){return;}
void XPBDPhysicsServer3D::sync(){return;}
void XPBDPhysicsServer3D::flush_queries(){return;}
void XPBDPhysicsServer3D::end_sync(){return;}
void XPBDPhysicsServer3D::finish(){return;}

bool XPBDPhysicsServer3D::is_flushing_queries() const{return 0;}

int XPBDPhysicsServer3D::get_process_info(ProcessInfo p_info){return 0;}


