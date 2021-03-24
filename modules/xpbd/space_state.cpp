#include "space_state.h"

XPBDDirectSpaceState3D::XPBDDirectSpaceState3D() : PhysicsDirectSpaceState3D(){}
XPBDDirectSpaceState3D::~XPBDDirectSpaceState3D(){}

int XPBDDirectSpaceState3D::intersect_point(const Vector3 &p_point, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false){return 0;}

bool XPBDDirectSpaceState3D::intersect_ray(const Vector3 &p_from, const Vector3 &p_to, RayResult &r_result, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false, bool p_pick_ray = false){return 0;}

int XPBDDirectSpaceState3D::intersect_shape(const RID &p_shape, const Transform &p_xform, real_t p_margin, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false){return 0;}

bool XPBDDirectSpaceState3D::cast_motion(const RID &p_shape, const Transform &p_xform, const Vector3 &p_motion, real_t p_margin, real_t &p_closest_safe, real_t &p_closest_unsafe, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false, ShapeRestInfo *r_info = nullptr){return 0;}

bool XPBDDirectSpaceState3D::collide_shape(RID p_shape, const Transform &p_shape_xform, real_t p_margin, Vector3 *r_results, int p_result_max, int &r_result_count, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false){return 0;}

bool XPBDDirectSpaceState3D::rest_info(RID p_shape, const Transform &p_shape_xform, real_t p_margin, ShapeRestInfo *r_info, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false){return 0;}

Vector3 XPBDDirectSpaceState3D::get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const {return Vector3();}
