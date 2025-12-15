from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Main Controller (Output Visible)
        Node(
            package='dealer_core',
            executable='main',
            name='dealer_core_node',
            output='screen',  # Keeps this visible
            emulate_tty=True,
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),
        
        # 2. MQTT / NETPIE Bridge (Output Hidden)
        Node(
            package='mqtt_node',
            executable='netpie_room_config_node',
            name='netpie_node',
            output='log'  # Hides output from terminal
        ),
        
        # 3. IR Sensor / Angle Detector (Output Hidden)
        Node(
            package='ir_reader',
            executable='angle_detect', 
            name='ir_reader_node',
            output='log'
        ),

        # 4. LCD Display Node (Output Hidden)
        Node(
            package='lcd_node',
            executable='lcd_node',
            name='lcd_node',
            output='log'
        ),

        # 5. Button Node (Output Hidden)
        Node(
            package='lcd_node',
            executable='button_push',
            name='button',
            output='log'
        ),

        # 6. Rotator Node [ADDED]
        Node(
            package='rotator',           # Assuming package name is 'rotator'
            executable='rotator_node',   # Matches 'rotator_node = ...'
            name='rotator_node',
            output='log'
        ),

        # 7. Card Spreader Node [ADDED]
        Node(
            package='motor_controller',      # Assuming package name is 'motor_controller'
            executable='card_spreader_node', # Matches 'card_spreader_node = ...'
            name='card_spreader_node',
            output='log'
        )
    ])