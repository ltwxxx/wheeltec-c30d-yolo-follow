# 对应现在仓库：edge_bridge.py 一次 HTTP POST 发出去的那一整包 tracks
# 业界习惯：单帧多目标用数组消息一次发完，不搞"一个目标一条消息"。

std_msgs/Header header
Track[] tracks                # 本帧所有候选目标
int32 primary_track_id        # 本帧认定要跟的那个；-1 = 没找到
string[] reid_feature_ids     # 对应 dict 里的 reid_ids，仅观测用，不参与控制
