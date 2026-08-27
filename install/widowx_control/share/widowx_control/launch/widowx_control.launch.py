from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    home_on_start = LaunchConfiguration("home_on_start")
    touch_port = LaunchConfiguration("touch_port")
    sensor = LaunchConfiguration("sensor")
    poses_file = LaunchConfiguration("poses_file")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="Porta serial do ArbotiX-M"),
        DeclareLaunchArgument(
            "home_on_start", default_value="true",
            description="Ao conectar, vai direto para a home pose e recusa "
                        "qualquer outro movimento ate chegar"),
        DeclareLaunchArgument(
            "touch_port", default_value="",
            description="Porta serial do touch sensor STM32 (aba Sensores). "
                        "Vazio = auto-detect da CDC nativa."),
        DeclareLaunchArgument(
            "sensor", default_value="5",
            description="Grade do touch sensor: 4 (4x4, com linha TOTAL) ou "
                        "5 (5x5, com os cuneiformes)"),
        DeclareLaunchArgument(
            "poses_file", default_value="",
            description="JSON de poses e movimentos do teach pendant. "
                        "Vazio = ~/.widowx_poses.json"),
        Node(
            package="widowx_control",
            executable="widowx_driver",
            name="widowx_driver",
            # ParameterValue com value_type: o argumento chega como TEXTO
            # ("true"), e um parametro declarado como bool recusa o override
            # com erro de tipo.
            parameters=[{"port": port,
                         "home_on_start": ParameterValue(home_on_start,
                                                         value_type=bool)}],
            output="screen",
        ),
        Node(
            package="widowx_control",
            executable="widowx_gui",
            name="widowx_gui",
            # Todos com value_type explicito: os argumentos de launch sao
            # TEXTO e o launch_ros adivinha o tipo do conteudo ("5" vira
            # inteiro, "" fica texto). Adivinhacao que discorde do
            # declare_parameter do no derruba o processo na largada - foi o
            # que aconteceu com `sensor` em 27/08/2026.
            parameters=[{
                "touch_port": ParameterValue(touch_port, value_type=str),
                "sensor": ParameterValue(sensor, value_type=int),
                "poses_file": ParameterValue(poses_file, value_type=str),
            }],
            output="screen",
        ),
    ])
