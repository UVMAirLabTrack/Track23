    marker_msg.pose = self.pose
    marker_msg.mesh_resource = self.marker_path

    marker_msg.scale.x = self.marker_data.get('Scale_x', 1.0)
    marker_msg.scale.y = self.marker_data.get('Scale_y', 1.0)
    marker_msg.scale.z = self.marker_data.get('Scale_z', 1.0)
    change_color = self.marker_data.get('Change_Color', False)
    if change_color:
        marker_msg.color.r = self.marker_data.get('Color_r', 1.0)
        marker_msg.color.g = self.marker_data.get('Color_g', 1.0)
        marker_msg.color.b = self.marker_data.get('Color_b', 1.0)
        marker_msg.color.a = self.marker_data.get('Color_a', 1.0)
        
    marker_msg.lifetime.sec = int(self.marker_data.get('Lifetime_sec', 1))
    marker_msg.frame_locked = bool(self.marker_data.get('Frame_locked', False))
    marker_msg.mesh_use_embedded_materials = bool(self.marker_data.get('Mesh_use_embedded_materials', False))
    marker_msg.header.frame_id = self.marker_data.get('Frame_id', "map")
