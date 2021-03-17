#ifndef XPBD_PHYSICS_SERVER_H
#define XPBD_PHYSICS_SERVER_H

#include "core/templates/rid.h"
#include "servers/physics_server_3d.h"

class XPBDPhysicsServer3D : public PhysicsServer3D {
	GDCLASS(XPBDPhysicsServer3D, PhysicsServer3D);
};

public:
	BulletPhysicsServer3D();
	~BulletPhysicsServer3D();

#endif
