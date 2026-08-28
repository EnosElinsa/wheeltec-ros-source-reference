#include <cmath>
#include <cstdint>

constexpr float RAD2DEG = 57.2957795f;
constexpr float Link_B  = 0.1400f;  // 杆 B 长度
constexpr float Link_C  = 0.1600f;  // 杆 C 长度

constexpr float AngleA_Min = -1.9f;
constexpr float AngleA_Max =  1.9f;
constexpr float AngleB_Min = -0.1700f;
constexpr float AngleB_Max =  1.5708f;
constexpr float AngleC_Min = -1.1790f;
constexpr float AngleC_Max =  0.7273f;

constexpr float PI = 3.14159265358979323846f;

enum DistanceComparison : uint8_t {
    DISTANCE_LESS_OR_EQUAL = 0,
    DISTANCE_GREATER_OR_EQUAL
};

enum Kinematic_Mode : uint8_t {
    Forward_Kinematic = 0,
    Inverse_Kinematic
};

enum MoveDirection : uint8_t {
    MOVE_UP = 0,   // 末端在 +Y 方向移动
    MOVE_DOWN,     // 末端在 -Y 方向移动
    MOVE_LEFT,     // 末端在 -X 方向移动
    MOVE_RIGHT     // 末端在 +X 方向移动
};

struct ArmWorkspaceArc_t {
    float center_x;
    float center_y;
    float radius;
    float theta_start;
    float theta_end;
    DistanceComparison distance_mode;
};

struct Arm_t {
    Kinematic_Mode kinematic_mode;
    float   joint_max[3];
    float   joint_min[3];
    float   joint_rad[3];   // 直接通过串口发给 STM32
    uint8_t gripper;        // 
    float   ee_x;           // 末端 X 坐标（米）
    float   ee_y;           // 末端 Y 坐标（米）
};

// 工作空间弧线定义（与原来的 g_workspace_arcs 一致）
static const ArmWorkspaceArc_t g_workspace_arcs[] = {
    { 0.0f,   0.14f,  0.16f,  -1.3f,  0.7f,  DISTANCE_GREATER_OR_EQUAL },
    { 0.0f,   0.0f,   0.0447f,-0.65f,-0.25f, DISTANCE_GREATER_OR_EQUAL },
    { 0.0f,   0.0f,   0.26f, -0.65f, 1.1f,  DISTANCE_LESS_OR_EQUAL    },
    { 0.0f,  -0.16f,  0.14f, -0.150f, 1.3f,  DISTANCE_GREATER_OR_EQUAL },
    { 0.138f,-0.0237f,0.16f, -1.57f, -1.0f,  DISTANCE_LESS_OR_EQUAL    },
};

void Arm2DOF_Init(Arm_t *arm) {
    if (!arm) return;
    arm->kinematic_mode = Inverse_Kinematic;

    arm->joint_min[0] = AngleA_Min;
    arm->joint_max[0] = AngleA_Max;
    arm->joint_min[1] = AngleB_Min;
    arm->joint_max[1] = AngleB_Max;
    arm->joint_min[2] = AngleC_Min + PI / 2.0f;
    arm->joint_max[2] = AngleC_Max + PI / 2.0f;

    arm->ee_x = 0.062f;
    arm->ee_y = -0.008f;

    arm->joint_rad[0] = 0.0f;
    arm->joint_rad[1] = 1.570796f;
    arm->joint_rad[2] = 0.391797f;
    arm->gripper      = 0;
}

// 工作空间判断
uint8_t isPointInsideArcs(float x, float y) {
    uint8_t arc_count = (y > 0.0f) ? 4 : 5;
    for (uint8_t i = 0; i < arc_count; ++i) {
        float dx = x - g_workspace_arcs[i].center_x;
        float dy = y - g_workspace_arcs[i].center_y;
        float d2 = dx * dx + dy * dy;
        float r2 = g_workspace_arcs[i].radius * g_workspace_arcs[i].radius;
        uint8_t within = 0;
        if (g_workspace_arcs[i].distance_mode == DISTANCE_LESS_OR_EQUAL)
            within = (d2 <= r2);
        else if (g_workspace_arcs[i].distance_mode == DISTANCE_GREATER_OR_EQUAL)
            within = (d2 >= r2);
        if (!within) return 0;
    }
    return 1;
}

// 正运动学
void Arm2DOF_Forward(const Arm_t * /*arm*/, float joint_b, float joint_c,
                     float *out_x, float *out_y) {
    float xb = Link_B * std::cos(joint_b);
    float yb = Link_B * std::sin(joint_b);
    float xc = xb + Link_C * std::cos(joint_c - PI + joint_b);
    float yc = yb + Link_C * std::sin(joint_c - PI + joint_b);
    if (out_x) *out_x = xc;
    if (out_y) *out_y = yc;
}

// 关节限位
uint8_t Arm2DOF_CheckJointLimits(const Arm_t *arm,
                                 float joint_b, float joint_c) {
    if (!arm) return 0;
    if (joint_b < arm->joint_min[1] || joint_b > arm->joint_max[1]) return 0;
    if (joint_c < arm->joint_min[2] || joint_c > arm->joint_max[2]) return 0;
    if (joint_b + joint_c < PI / 2.0f) return 0;
    return 1;
}

// 逆运动学
uint8_t Arm2DOF_Inverse(const Arm_t *arm, float x, float y,
                        float *out_b, float *out_c) {
    if (!arm) return 0;
    float r3 = std::sqrt(x * x + y * y);
    float cos_b1 = (Link_B * Link_B + r3 * r3 - Link_C * Link_C) /
                   (2.0f * Link_B * r3);
    float b1 = std::acos(cos_b1);
    float b2 = std::atan2(y, x);
    float b  = b1 + b2;
    float cos_c = (Link_B * Link_B + Link_C * Link_C - r3 * r3) /
                  (2.0f * Link_B * Link_C);
    float c = std::acos(cos_c);
    if (!Arm2DOF_CheckJointLimits(arm, b, c)) return 0;
    if (out_b) *out_b = b;
    if (out_c) *out_c = c;
    return 1;
}

// 设置末端目标位姿（位置），带工作空间和关节限制
uint8_t Arm2DOF_SetTargetEndEffector(Arm_t *arm, float x, float y) {
    if (!arm) return 0;
    if (!isPointInsideArcs(x, y)) return 0;
    float b, c;
    if (!Arm2DOF_Inverse(arm, x, y, &b, &c)) return 0;
    arm->joint_rad[1] = b;
    arm->joint_rad[2] = c;
    arm->ee_x = x;
    arm->ee_y = y;
    return 1;
}

// 设置目标关节角
uint8_t Arm2DOF_SetTargetJoint(Arm_t *arm, float joint_b, float joint_c) {
    if (!arm) return 0;
    float x, y;
    if (!Arm2DOF_CheckJointLimits(arm, joint_b, joint_c)) return 0;
    Arm2DOF_Forward(arm, joint_b, joint_c, &x, &y);
    if (!isPointInsideArcs(x, y)) return 0;
    arm->joint_rad[1] = joint_b;
    arm->joint_rad[2] = joint_c;
    arm->ee_x = x;
    arm->ee_y = y;
    return 1;
}

// 新增：相对移动末端的逆解接口
// dir 为 MOVE_UP/DOWN/LEFT/RIGHT，distance 为在该方向上的偏移米数
uint8_t Arm2DOF_MoveEndEffectorRelative(Arm_t *arm,
                                        MoveDirection dir,
                                        float distance) {
    if (!arm) return 0;
    if (distance == 0.0f) return 1;

    float dx = 0.0f;
    float dy = 0.0f;
    switch (dir) {
    case MOVE_UP:    dy =  distance; break;
    case MOVE_DOWN:  dy = -distance; break;
    case MOVE_LEFT:  dx = -distance; break;
    case MOVE_RIGHT: dx =  distance; break;
    default: return 0;
    }

    float new_x = arm->ee_x + dx;
    float new_y = arm->ee_y + dy;
    return Arm2DOF_SetTargetEndEffector(arm, new_x, new_y);
}

