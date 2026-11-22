# 对应现在仓库：follow_control.py 的 POST /follow + seq 幂等 + 状态轮询
# Action 服务器免费送的四样东西，你现在是手搓的：
#   ① 派任务（goal） ② 拒/放（goal_callback） ③ 中途取消（cancel_callback）
#   ④ 持续回报进度（feedback）+ 结束给结论（result）
# ⚠️ 语法检查过，未编译未运行；follow_msgs 还没真编译出来，import 不了。

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from follow_msgs.action import FollowPerson


class SessionNode(Node):
    def __init__(self):
        super().__init__('follow_session')
        self.declare_parameter('default_desired_distance_m', 1.20)
        self.declare_parameter('default_timeout_s', 30.0)

        self._current_goal = None          # 同一时刻只允许一个跟随任务

        self._server = ActionServer(
            self,
            FollowPerson,
            'follow',                    # → /follow_session/follow
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )
        self.get_logger().info('FOLLOW_SESSION_READY')

    def _on_goal(self, request_meta):
        # 这一段等价于你现在 POST 接口里的"会话冲突判断"。
        # 拒绝要返回 REJECT 而不是抛异常 —— 客户端拿到的是结构化的拒绝原因。
        if self._current_goal is not None:
            self.get_logger().warn('已有跟随任务，拒绝新目标')
            return GoalResponse.REJECT
        if request_meta.goal.mode not in ('person', 'point'):
            return GoalResponse.REJECT
        self.get_logger().info('接受目标: mode=%s' % request_meta.goal.mode)
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        # 取消是"请求"不是"命令"：这里可以拒绝，但跟随任务没理由拒绝。
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        self._current_goal = goal_handle
        distance = goal.desired_distance_m or self.get_parameter(
            'default_desired_distance_m').value

        result = FollowPerson.Result()
        tick = 0
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.succeeded = False
                    result.reason = '用户取消'
                    return result

                # 丢失判定 —— 对应你现在的 lost_timeout
                # 真实实现会在这里读 motion 的状态话题；省略成演示
                if tick > self.get_parameter('default_timeout_s').value * 10:
                    goal_handle.abort()
                    result.succeeded = False
                    result.reason = '目标丢失超时'
                    return result

                # 注意：execute 回调本身已经跑在 executor 线程里，绝对不要再 spin 自己。
                # 这是 Action 写法里最容易死锁的一处。
                fb = FollowPerson.Feedback()
                fb.status_text = '跟手中 d=%.2f' % distance
                goal_handle.publish_feedback(fb)

                tick += 1
                time.sleep(0.1)
        finally:
            self._current_goal = None

        goal_handle.succeed()
        result.succeeded = True
        result.reason = '任务时间到'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SessionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
