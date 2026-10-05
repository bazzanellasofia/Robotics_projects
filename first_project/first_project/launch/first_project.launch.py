
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def get_bags_directory():
    current_directory = os.path.dirname(__file__)

    while True:
        candidate_directory = os.path.join(current_directory, 'bags')
        if os.path.isdir(candidate_directory):
            return candidate_directory

        parent_directory = os.path.join(current_directory, '..')
        if parent_directory == current_directory:
            break

        current_directory = parent_directory

    return os.path.join(os.path.dirname(__file__), '..', 'bags')


def get_valid_bags(bags_directory):
    if not os.path.isdir(bags_directory):
        return []

    return sorted(
        entry
        for entry in os.listdir(bags_directory)
        if not entry.startswith('.') and os.path.isdir(os.path.join(bags_directory, entry))
    )


def generate_launch_description():
    bags_directory = get_bags_directory()
    valid_bags = get_valid_bags(bags_directory)

    if not valid_bags:
        raise RuntimeError(f"Nessuna bag trovata nella cartella: {bags_directory}")

    print(
        "Scrivere nel terminale il nome della bag da cui si vogliono ricevere i dati "
        f"tra i seguenti: {' '.join(valid_bags)}"
    )

    selected_bag = ""
    while selected_bag not in valid_bags:
        try:
            selected_bag = input("Bag selezionata: ").strip()
        except EOFError:
            selected_bag = valid_bags[0]
        if selected_bag not in valid_bags:
            print(f"Valore non valido. Inserire una tra: {' '.join(valid_bags)}")

    bag_path = os.path.join("..", "bags", selected_bag)

    rviz_config = os.path.join(
        get_package_share_directory('first_project'),
        'rviz',
        'first_project.rviz'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
            output='screen',
        ),

        Node(
            package='first_project',
            executable='odometer',
            name='odometer',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='first_project',
            executable='tf_error',
            name='tf_error',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['bash', '-lc', "pkill -f 'first_project.*(odometer|tf_error)' || true"],
                        output='screen'
                    )
                ]
            )
        ),
    ])