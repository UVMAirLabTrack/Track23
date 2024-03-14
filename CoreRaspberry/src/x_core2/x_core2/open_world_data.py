import os

def world_select(file_path):
    selected_world = None
    world_pose = None

    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()

            if len(parts) > 1 and 'y' in parts[1]:
                selected_world = f"{parts[0]}.dae"
                world_pose = f"{parts[0]}.txt"
                world_marker = f"{parts[0]}_markers.txt"
                break  # Exit the loop after the first match

    return selected_world, world_pose, world_marker

def find_world_path():

    script_path = os.path.dirname(os.path.abspath(__file__))
    parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    world_ctrl = os.path.join(parent_folder, 'worlds', 'world_select.txt')
    world_file, world_pose,world_marker = world_select(world_ctrl)
    world_path = os.path.join(parent_folder, 'worlds', f'{world_file}')
    

    return world_path

def find_pose_path():

    script_path = os.path.dirname(os.path.abspath(__file__))
    parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    world_ctrl = os.path.join(parent_folder, 'worlds', 'world_select.txt')
    world_file, world_pose,world_marker = world_select(world_ctrl)
    
    world_pose_path = os.path.join(parent_folder, 'worlds', f'{world_pose}')

    return world_pose_path

def find_marker_path():

    script_path = os.path.dirname(os.path.abspath(__file__))
    parent_folder = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    world_ctrl = os.path.join(parent_folder, 'worlds', 'world_select.txt')
    world_file, world_pose,world_marker = world_select(world_ctrl)
    world_marker_path = os.path.join(parent_folder, 'worlds', f'{world_marker}')

    return world_marker_path

def main(args=None):
    pass
if __name__ == '__main__':
    main()
