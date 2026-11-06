"""Optional MicroXRCEAgent entrypoint; reuse an already running agent when available."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, FindExecutable


def build(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    transport = arg('transport')
    executable = FindExecutable(name=arg('executable')).perform(context)
    if transport == 'serial':
        command = [executable, 'serial', '--dev', arg('device'), '-b', str(int(arg('baudrate')))]
    elif transport == 'udp4':
        port = int(arg('port'))
        if not 0 < port < 65536: raise ValueError('port must be in 1..65535')
        command = [executable, 'udp4', '-p', str(port)]
    else:
        raise ValueError('transport must be serial or udp4')
    return [ExecuteProcess(cmd=command, name='px4_dds_agent', output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='udp4'),
        DeclareLaunchArgument('port', default_value='8888'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('executable', default_value='MicroXRCEAgent'),
        OpaqueFunction(function=build),
    ])
