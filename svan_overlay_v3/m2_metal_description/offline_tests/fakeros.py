"""fakeros.py -- a tiny stand-in for rclpy + the few message types svan_walk_controller.py uses, so the REAL node file can be
imported and driven offline (by test_node_mj.py) with MuJoCo providing /joint_states, /ground_truth/odom and /scan.
It runs callbacks one after another in a single thread: it tests the node's logic, NOT ROS 2 middleware, QoS or threading."""
import sys
import types


class M:                                           # generic message: attributes spring into existence
    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        v = M(); object.__setattr__(self, k, v); return v


def _msg(_cls, **lists):
    def init(self):
        for k, v in lists.items():
            setattr(self, k, v() if callable(v) else v)
    return type(_cls, (M,), {"__init__": init})


Twist = _msg("Twist"); PoseStamped = _msg("PoseStamped"); TransformStamped = _msg("TransformStamped")
Odometry = _msg("Odometry"); Path = _msg("Path", poses=list); OccupancyGrid = _msg("OccupancyGrid", data=list)
JointState = _msg("JointState", name=list, position=list, velocity=list, effort=list)
LaserScan = _msg("LaserScan", ranges=list); Float64MultiArray = _msg("Float64MultiArray", data=list)


class Empty:
    class Request:
        pass


class _Log:
    def __init__(self, sink):
        self.sink = sink

    def _w(self, lvl):
        return lambda msg, **kw: self.sink.append((lvl, msg))

    def __getattr__(self, k):
        return self._w(k)


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic, bus):
        self.topic, self.bus = topic, bus

    def publish(self, msg):
        self.bus.published.setdefault(self.topic, []).append(msg)
        self.bus.last[self.topic] = msg


class Bus:
    def __init__(self):
        self.subs = {}; self.timers = []; self.published = {}; self.last = {}; self.logs = []; self.overrides = {}


BUS = Bus()


class Node:
    def __init__(self, name):
        self._params = {}

    def declare_parameter(self, name, default=None, descriptor=None):
        self._params[name] = BUS.overrides.get(name, default)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def get_logger(self):
        return _Log(BUS.logs)

    def create_publisher(self, typ, topic, qos):
        return _Pub(topic, BUS)

    def create_subscription(self, typ, topic, cb, qos, callback_group=None):
        BUS.subs.setdefault(topic, []).append(cb)

    def create_timer(self, period, cb, callback_group=None):
        BUS.timers.append([period, cb, None])

    def create_client(self, typ, name):
        return types.SimpleNamespace(service_is_ready=lambda: False, call_async=lambda r: None)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: M()))

    def destroy_node(self):
        pass


def install():
    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    rclpy = mod("rclpy", init=lambda *a, **k: None, shutdown=lambda: None, ok=lambda: True)
    rclpy.node = mod("rclpy.node", Node=Node)
    rclpy.qos = mod("rclpy.qos", DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1), HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
                    QoSProfile=lambda **k: k, ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2), qos_profile_sensor_data={})
    rclpy.callback_groups = mod("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=lambda: object())
    rclpy.executors = mod("rclpy.executors", MultiThreadedExecutor=lambda **k: None)
    mod("rclpy.node", Node=Node)
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Twist, PoseStamped=PoseStamped, TransformStamped=TransformStamped)
    mod("nav_msgs"); mod("nav_msgs.msg", Odometry=Odometry, Path=Path, OccupancyGrid=OccupancyGrid)
    mod("sensor_msgs"); mod("sensor_msgs.msg", JointState=JointState, LaserScan=LaserScan)
    mod("std_msgs"); mod("std_msgs.msg", Float64MultiArray=Float64MultiArray)
    mod("std_srvs"); mod("std_srvs.srv", Empty=Empty)
    mod("rcl_interfaces"); mod("rcl_interfaces.msg", ParameterDescriptor=lambda **k: None)
    mod("tf2_ros", TransformBroadcaster=lambda node: types.SimpleNamespace(sendTransform=lambda tf: BUS.published.setdefault("/tf", []).append(tf)))
