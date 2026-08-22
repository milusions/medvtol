def gazebo_to_px4_ned(gz_x, gz_y, gz_z,spawn_x=0.0, spawn_y=0.0, spawn_z=0.0):
        """Converts Gazebo ENU (world frame) to PX4 NED (local frame)."""
        # 1. Apply translation offset
        local_enu_x = gz_x - spawn_x
        local_enu_y = gz_y - spawn_y
        local_enu_z = gz_z - spawn_z
        
        # 2. Apply ENU to NED rotation matrix
        px4_x = local_enu_y       # North is Gazebo Y
        px4_y = local_enu_x       # East is Gazebo X
        px4_z = -local_enu_z      # Down is inverted Gazebo Z
        
        return px4_x, px4_y, px4_z

def px4_ned_to_gazebo(px4_x, px4_y, px4_z,spawn_x=0.0, spawn_y=0.0, spawn_z=0.0):
        """Converts PX4 NED (local frame) back to Gazebo ENU (world frame)."""
        # 1. Apply NED to ENU rotation
        local_enu_x = px4_y
        local_enu_y = px4_x
        local_enu_z = -px4_z
        
        # 2. Re-apply translation offset
        gz_x = local_enu_x + spawn_x
        gz_y = local_enu_y + spawn_y
        gz_z = local_enu_z + spawn_z
        
        return gz_x, gz_y, gz_z