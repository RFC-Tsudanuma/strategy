import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Get the source directory dynamically
    # The launch file is in install/share, but we need the source directory
    current_file = os.path.abspath(__file__)
    
    # Check if we're running from install or source directory
    if 'install' in current_file:
        # Running from install directory, need to find source
        # Go up to workspace root and then to src/strategy
        parts = current_file.split(os.sep)
        ws_index = parts.index('install')
        ws_root = os.sep.join(parts[:ws_index])
        package_dir = os.path.join(ws_root, 'src', 'strategy')
    else:
        # Running from source directory
        package_dir = os.path.dirname(os.path.dirname(current_file))
    
    venv_activate = os.path.join(package_dir, '.venv', 'bin', 'activate')
    main_script = os.path.join(package_dir, 'scripts', 'main.py')
    
    cmd = [
        'bash', '-c',
        f'source {venv_activate} && python {main_script}'
    ]
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=cmd,
            name='main',
            output='screen',
        )
    ])