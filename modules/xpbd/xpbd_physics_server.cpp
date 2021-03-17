#include "register_types.h"
#include "core/templates/rid.h"
#include "servers/physics_server_3d.h"

XPBDPhysicsServer3D::XPBDPhysicsServer3D() : 
        PhysicsServer3D() {}

XPBDPhysicsServer3D::~XPBDPhysicsServer3D() {}


RID XPBDPhysicsServer3D::plane_shape_create(){
    return 0;
}

    RID plane_shape_create() = 0;
	RID ray_shape_create() = 0;
	RID sphere_shape_create() = 0;
	RID box_shape_create() = 0;
	RID capsule_shape_create() = 0;
	RID cylinder_shape_create() = 0;
	RID convex_polygon_shape_create() = 0;
	RID concave_polygon_shape_create() = 0;
	RID heightmap_shape_create() = 0;
	RID custom_shape_create() = 0;

	void shape_set_data(RID p_shape, const Variant &p_data) = 0;
	void shape_set_custom_solver_bias(RID p_shape, real_t p_bias) = 0;

	ShapeType shape_get_type(RID p_shape) const = 0;
	Variant shape_get_data(RID p_shape) const = 0;

	void shape_set_margin(RID p_shape, real_t p_margin) = 0;
	real_t shape_get_margin(RID p_shape) const = 0;

	real_t shape_get_custom_solver_bias(RID p_shape) const = 0;
