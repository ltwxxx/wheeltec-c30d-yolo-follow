# 对应现在仓库：follow_control.py / edge_bridge.py 里传递的 track dict
# 一个被跟踪的人。框是归一化坐标（0~1），所以换分辨率不用换算。
# 注意 msg 文件里 # 后面是注释，会跟着生成到各语言的文档里。

std_msgs/Header header        # 时间戳 + 坐标系，别省：下游要靠它算数据有多旧

int32 track_id                # 对应 dict 里的 track_id，跨帧追踪用的编号
float32 confidence            # 0~1，对应 dict 里的 score

# 归一化包围盒，对应 dict 里的 bbox
float32 x_norm
float32 y_norm
float32 w_norm
float32 h_norm

# 距离与方位，对应 dict 里的 distance_m / center_x_ratio
float32 distance_m            # 雷达补出来的真实距离；没补上时是 -1
bool distance_from_lidar      # 上面这个数到底来自雷达还是估算 —— 安全层要看这个
float32 center_x_ratio        # 框中心在画面里的横向位置，0.5 = 正中间
