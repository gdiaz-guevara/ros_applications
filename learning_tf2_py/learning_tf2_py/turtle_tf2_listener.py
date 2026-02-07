import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


# -----------------------------
# VARIABLES GLOBALES
# -----------------------------
node = None
tf_buffer = None
publisher = None
spawner = None

turtle_spawning_service_ready = False
turtle_spawned = False
spawn_future = None

target_frame = None


# -----------------------------
# TIMER CALLBACK
# -----------------------------
def on_timer():
    global turtle_spawning_service_ready
    global turtle_spawned
    global spawn_future

    from_frame = target_frame
    to_frame = 'turtle2'

    # 1. Esperar a que el servicio esté listo
    if not turtle_spawning_service_ready:
        if spawner.service_is_ready():
            request = Spawn.Request()
            request.name = 'turtle2'
            request.x = float(4)
            request.y = float(2)
            request.theta = float(0)

            spawn_future = spawner.call_async(request)
            turtle_spawning_service_ready = True
            node.get_logger().info('Spawn request sent')
        else:
            node.get_logger().info('Spawn service not ready')
        return

    # 2. Esperar a que turtle2 exista
    if not turtle_spawned:
        if spawn_future.done():
            result = spawn_future.result()
            node.get_logger().info(f'Successfully spawned {result.name}')
            turtle_spawned = True
        else:
            node.get_logger().info('Spawn in progress...')
        return

    # 3. Ya existe turtle2 → calcular transform
    try:
        t = tf_buffer.lookup_transform(
            to_frame,
            from_frame,
            rclpy.time.Time()
        )
    except TransformException as ex:
        node.get_logger().info(
            f'Could not transform {to_frame} to {from_frame}: {ex}'
        )
        return

    # 4. Publicar velocidades
    msg = Twist()

    msg.angular.z = 1.0 * math.atan2(
        t.transform.translation.y,
        t.transform.translation.x
    )

    msg.linear.x = 0.5 * math.sqrt(
        t.transform.translation.x ** 2 +
        t.transform.translation.y ** 2
    )

    publisher.publish(msg)


# -----------------------------
# MAIN
# -----------------------------
def main(args=None):
    global node
    global tf_buffer
    global publisher
    global spawner
    global target_frame

    rclpy.init(args=args)

    node = Node('turtle_tf2_frame_listener')

    # Parámetro: qué frame seguir
    target_frame = node.declare_parameter(
        'target_frame', 'turtle1'
    ).get_parameter_value().string_value

    # TF2
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # Publisher
    publisher = node.create_publisher(Twist, 'turtle2/cmd_vel', 1)

    # Servicio spawn
    spawner = node.create_client(Spawn, 'spawn')

    # Timer
    node.create_timer(1.0, on_timer)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
