#ifndef XPBD_SPACE_STATE
#define XPBD_SPACE_STATE

#include "servers/physics_server_3d.h"
#include "core/templates/rid.h"

class XPBDDirectSpaceState3D : public PhysicsDirectSpaceState3D{
    GDCLASS(XPBDDirectSpaceState3D, PhysicsDirectSpaceState3D);
    
public:

    XPBDDirectSpaceState3D();
	~XPBDDirectSpaceState3D();

    virtual int intersect_point(const Vector3 &p_point, ShapeResult *r_results, int p_result_max, 
                                const Set<RID> &p_exclude, uint32_t p_collision_mask, 
                                bool p_collide_with_bodies, bool p_collide_with_areas)override;

    virtual bool intersect_ray(const Vector3 &p_from, const Vector3 &p_to, RayResult &r_result,
                                const Set<RID> &p_exclude, uint32_t p_collision_mask,
                                bool p_collide_with_bodies, bool p_collide_with_areas, 
                                bool p_pick_ray)override;

	virtual int intersect_shape(const RID &p_shape, const Transform &p_xform, real_t p_margin,
                                ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, 
                                uint32_t p_collision_mask, bool p_collide_with_bodies, 
                                bool p_collide_with_areas)override;

    virtual bool cast_motion(const RID &p_shape, const Transform &p_xform, const Vector3 &p_motion,
                                real_t p_margin, real_t &p_closest_safe, real_t &p_closest_unsafe, 
                                const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, 
                                bool p_collide_with_areas, ShapeRestInfo *r_info)override;

	virtual bool collide_shape(RID p_shape, const Transform &p_shape_xform, real_t p_margin, Vector3 *r_results, 
                                int p_result_max, int &r_result_count, const Set<RID> &p_exclude, 
                                uint32_t p_collision_mask, bool p_collide_with_bodies, 
                                bool p_collide_with_areas)override;

	virtual bool rest_info(RID p_shape, const Transform &p_shape_xform, real_t p_margin, ShapeRestInfo *r_info, 
                                const Set<RID> &p_exclude, uint32_t p_collision_mask, 
                                bool p_collide_with_bodies, bool p_collide_with_areas)override;

	virtual Vector3 get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const override;
};
#endif