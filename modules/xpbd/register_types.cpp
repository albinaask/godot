#include "register_types.h"

#include "core/class_db.h"
//#include "core/project_settings.h"

/**
	@author albinaask
*/

#ifndef _3D_DISABLED
PhysicsServer3D *_createXPBDPhysicsCallback() {
	return memnew(XPBDPhysicsServer3D);
}
#endif

void register_xpbd_types() {
#ifndef _3D_DISABLED
	PhysicsServer3DManager::register_server("XPBD", &_createXPBDPhysicsCallback);

	//PhysicsServer3DManager::set_default_server("Bullet", 1);
	//GLOBAL_DEF("physics/3d/active_soft_world", true);
	//ProjectSettings::get_singleton()->set_custom_property_info("physics/3d/active_soft_world", PropertyInfo(Variant::BOOL, "physics/3d/active_soft_world"));
#endif
}

void unregister_xpbd_types() {
}






