"""노드별 시각화가 공유하는 RViz 마커 생성과 드론 색상."""

from visualization_msgs.msg import Marker

COLORS = (
    (0.0, 0.85, 1.0), (1.0, 0.5, 0.05), (0.75, 0.3, 1.0), (1.0, 0.9, 0.1),
    (0.2, 0.9, 0.35), (1.0, 0.25, 0.4), (0.3, 0.5, 1.0),
)


def marker(node, frame, namespace, index, kind, xyz, color, scale):
    m = Marker()
    m.header.frame_id, m.header.stamp = frame, node.get_clock().now().to_msg()
    m.ns, m.id, m.type = namespace, index, kind
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
    m.scale.x, m.scale.y, m.scale.z = map(float, scale)
    m.color.r, m.color.g, m.color.b = map(float, color)
    m.color.a = 1.0
    return m

