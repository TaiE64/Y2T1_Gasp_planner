import pybullet as p
import time
#p.connect(p.UDP,"192.168.86.100")
import pybullet_data
#----------------------------------------------------------------------

#使用共享内存模式连接到 PyBullet 仿真环境。如果连接失败，回退到 GUI 模式。
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)
  
#----------------------------------------------------------------------

#setAdditionalSearchPath：设定搜索路径，用于加载资源文件（如 URDF 模型）。
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation()
#disable rendering during loading makes it much faster
#禁用可视化渲染提高加载速度，完成后再启用。
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

#----------------------------------------------------------------------

#根据代码中的 p.setAdditionalSearchPath(pybullet_data.getDataPath())，
# pr2_gripper.urdf 文件在 PyBullet 提供的默认数据路径中
#加载 URDF 模型文件，PR2 Gripper 放置在仿真世界坐标 (0.5, 0.3, 0.7)。
objects = [
    p.loadURDF("pr2_gripper.urdf", 0.500000, 0.300006, 0.700000, -0.000000, -0.000000, -0.000031,
               1.000000)
]
#Robot ID
pr2_gripper = objects[0]

#----------------------------------------------------------------------

#初始化关节位置，分别设置机械手的几个关节到指定角度。
jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
for jointIndex in range(p.getNumJoints(pr2_gripper)):
  p.resetJointState(pr2_gripper, jointIndex, jointPositions[jointIndex])

pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0.2, 0, 0],
                             [0.500000, 0.300006, 0.700000])
print("pr2_cid")
print(pr2_cid)
#p.changeConstraint(pr2_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

#----------------------------------------------------------------------

#启用可视化渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

#设置重力 -10m/s²（沿 z 轴），开启实时仿真。
#有约束，没用
# p.setGravity(0.000000, 0.000000, 0.000000)
# p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

done = False
while (1):
    # p.setGravity(0, 0, -10)
    #用户通过输入触发机械手张开或闭合。
    input("enter to close the gripper")
    #控制关节，分别设置 0 和 2 号关节（左右夹爪）的位置、速度和力。
    p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, 
                                targetPosition=0.7, maxVelocity=1,force=1)
    p.setJointMotorControl2(pr2_gripper, 2, p.POSITION_CONTROL, 
                                targetPosition=0.7, maxVelocity=1,force=1)
    input("enter to open the gripper")
    p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, 
                                targetPosition=0.0, maxVelocity=1,force=1)
    p.setJointMotorControl2(pr2_gripper, 2, p.POSITION_CONTROL, 
                                targetPosition=0.0, maxVelocity=1,force=1)
#断开连接
p.disconnect()
#----------------------------------------------------------------------

"""
  这段代码的作用是创建一个 **固定约束（Fixed Constraint）**，将 **`pr2_gripper`** 机械手模型固定在场景中的指定位置，从而防止其移动或旋转。下面对代码进行逐步分析和解释：

---

### **代码拆解分析**

```python
pr2_cid = p.createConstraint(
    pr2_gripper,          # 父物体 ID (pr2_gripper)
    -1,                   # 父物体的基座（-1 表示基座）
    -1,                   # 子物体 ID (-1 表示世界坐标系)
    -1,                   # 子物体的基座（-1 表示基座）
    p.JOINT_FIXED,        # 约束类型：固定约束
    [0, 0, 0],            # 约束的旋转轴（固定约束不需要轴，任意值都可以）
    [0.2, 0, 0],          # 父物体上的约束点位置（相对于父物体坐标系）
    [0.500000, 0.300006, 0.700000]  # 子物体上的约束点位置（相对于子物体坐标系/世界坐标系）
)
```
---
### **具体解释**

#### 1. **`pr2_gripper`**
- **`pr2_gripper`** 是通过 **`p.loadURDF`** 加载的 PR2 机械手模型的唯一 ID。
- 该参数表明固定约束的父物体是 PR2 机械手。

#### 2. **`-1` 表示基座**
- **`-1`** 用于指定父物体和子物体的基座（base link）。  
  - 父物体的基座：`parentLinkIndex = -1`。
  - 子物体的基座：`childLinkIndex = -1`。
- 在这里，父物体是 PR2 机械手，而子物体是 **世界坐标系**。

#### 3. **`p.JOINT_FIXED`**
- **固定约束**（`JOINT_FIXED`）的作用是将两个物体“固定”在一起，彼此之间没有相对运动。
- 在本例中，PR2 机械手被固定在世界坐标系中。

#### 4. **`[0, 0, 0]`（jointAxis）**
- 对于固定约束，旋转轴参数没有实际意义，可以设置为 `[0, 0, 0]`。

#### 5. **`[0.2, 0, 0]`（parentFramePosition）**
- 约束点在 **父物体（PR2 机械手）** 上的相对位置。
- 表示约束点位于 PR2 机械手本地坐标系的 `(0.2, 0, 0)`。

#### 6. **`[0.500000, 0.300006, 0.700000]`（childFramePosition）**
- 约束点在 **子物体（世界坐标系）** 上的相对位置。
- 表示约束点在世界坐标系中的 `(0.5, 0.3, 0.7)`。

#### 7. **返回值 `pr2_cid`**
- **`p.createConstraint`** 返回一个唯一的约束 ID（例如 `pr2_cid`），用于标识该约束。
- 这个 ID 可以用来修改或移除约束：
  ```python
  p.removeConstraint(pr2_cid)  # 删除约束
  ```

#### 8. **打印约束 ID**
```python
print("pr2_cid")
print(pr2_cid)
```
- 打印创建的约束 ID（`pr2_cid`），便于调试或后续操作。

---

### **总结：这段代码的作用**
1. **固定 PR2 机械手的位置**
   - 将 PR2 机械手模型的某一点（`[0.2, 0, 0]`）与世界坐标系中的某点（`[0.5, 0.3, 0.7]`）固定连接。
   - 防止 PR2 机械手在仿真中漂移或发生未定义的运动。

2. **用途**
   - 用于仿真调试：在某些场景中，不希望机械手因为重力或外力发生移动。
   - 可作为初始状态：在仿真启动时，将模型固定在某个位置，方便进一步操作。

---

### **拓展示例**
#### 修改约束
可以通过 **`p.changeConstraint`** 修改约束，例如调整位置：
```python
p.changeConstraint(pr2_cid, childFramePosition=[0.6, 0.4, 0.8])
```

#### 移除约束
当不需要约束时，可以将其移除：
```python
p.removeConstraint(pr2_cid)
```
  """