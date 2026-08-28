class SimplePID:
    """离散位置式 PID 控制器，带积分限幅防 windup"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 integral_max=1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.output_max = output_max
        self.targetpoint = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0

    def compute(self, target: float, current: float) -> float:
        """计算 PID 输出，对标 C++ 版 PID::compute()"""
        error = target - current
        self.integral += error
        # 积分限幅防止 windup
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        self.derivative = error - self.prev_error
        self.targetpoint = (self.kp * error +
                           self.ki * self.integral +
                           self.kd * self.derivative)
        # 输出限幅
        self.targetpoint = max(-self.output_max, min(self.output_max, self.targetpoint))
        self.prev_error = error
        return self.targetpoint

    def reset(self):
        """重置所有累积状态"""
        self.targetpoint = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0

    def set_pid(self, kp, ki, kd):
        """动态设置 PID 参数"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # 参数变化时重置积分，避免突变冲击
        self.integral = 0.0
