// catch ball step
// 调整机械臂位置，让爪子的摄像头对准球
// 判断目标球与底盘之间有没有其他颜色的球
// 如果有，闭合爪子的一边，下压，并迅速张开 -- 用于将其他颜色的球弹开
// 没有阻挡：向前靠近直到目标位置
// 执行抓取动作
// 抓取成功：调整机械臂位置，返回成功