using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.PrivateImplementationDetails;

namespace rrtRobot
{
    public class RobotKinemetix
    {
        public static double M_PI = 3.1415926;

        /*  TxRobotPostureGenerate(TxRobot robot, ArrayList Solutions, double Gun_openning)
         * Update the robot's pose based on the results of the robot inverse kinematics calculation:
         * 1.  TxRobot robot-Which robot's posture needs updating;
         * 
         * 2. Solutions-an ArrayList, stores all inverse kinematic solutions for the robot poses. 
         * The robot pose is updated using the first inverse kinematic solution. 
         * This update is to verify the collision detection results between the robot flange's installed tooling and the environment; 
         * it is unrelated to path smoothing. The optimal solution will be selected in the path optimization function.
         * 
         * 3. Gun_Opening- The Gun_Opening parameter stores the numerical values for the axis mounted on the robot flange, such as Server Weld Gun;
         * 
         * 根据机器人逆向学运算的结果,更新机器人的姿态:
         * 1. robot指的是哪个机器人的姿态需要更新;
         * 2. Solutions 作为ArrayList 数据,存储了所有的机器人姿态的逆向解,
         * 这里机器人姿态的更新是按照第一个逆向解进行姿态更新, 
         * 这里更新机器人姿态的目的是为了验证机器人法兰盘安装Tooling的碰撞检测结果, 
         * 与路径的平滑无关,在路径的优化函数中,会挑选出最优解;
         * 3. Gun_Opening 参数存储了法兰盘安装轴的数值例如焊钳
         */

        public static void TxRobotPostureGenerate(TxRobot robot, ArrayList Solutions, double Gun_openning)
        {

            robot.CurrentPose = (TxPoseData)Solutions[0];

            ITxDevice tSelectObject = robot.MountedTools[0] as ITxDevice;

            ArrayList ToolJointValue = new ArrayList();

            ToolJointValue.Add(Gun_openning);
            TxPoseData txToolPoseData = tSelectObject.CurrentPose;

            txToolPoseData.JointValues = ToolJointValue;

            tSelectObject.CurrentPose = txToolPoseData;
            ToolJointValue.Clear();
            txToolPoseData.Dispose();

            ToolJointValue = null;
            txToolPoseData = null;


        }

        /* public static ArrayList robotInverseCal(TxRobot robot, point target)
         * Robot inverse kinematics calculations are performed using the built-in functions of TECNOMATIX. 
         * Other inverse kinematics functions (custom-defined) can also be substituted.
         * 通过TECNOMATIX自带的函数进行机器人逆向学计算，这里也可以更换成其他的逆向学函数(自定义的)
         */
        public static ArrayList robotInverseCal(TxRobot robot, point target)
        {

            TxRobotInverseData txRobotInverseData = new TxRobotInverseData();
            TxTransformation robotTarget = new TxTransformation();
            TxVector rotation = new TxVector();
            rotation.X = target.rx;
            rotation.Y = target.ry;
            rotation.Z = target.rz;


            TxTransformation txTransformation = new TxTransformation();

            txTransformation.RotationRPY_XYZ = rotation;
            txTransformation.Translation = new TxVector(target.x, target.y, target.z);

            txRobotInverseData.Destination = txTransformation;


            ArrayList solustions = robot.CalcInverseSolutions(txRobotInverseData);

            return solustions;

        }

        public static void DisposeTxposureData(ArrayList Solutions)
        {
            if (Solutions.Count == 0) return;

            for(int i=0;i<Solutions.Count;i++)
            {

                (Solutions[i] as TxPoseData).Dispose();
            }

        }
        /* public static List<double> MatrixRotate(double rx1, double ry1, double rz1, double rx2, double ry2, double rz2, bool rotate_WorldorSelf)
         * 1. 输入值(rx1, ry1, rz1) 定义了第一个姿态的旋转矢量，
         * (rx2, ry2, rz2)定义了目标姿态的旋转矢量，
         * rotate_WorldorSelf 如果为true就是绕着世界坐标进行旋转，false为绕着自身的旋转轴进行旋转
         * 2. 如果是绕着世界坐标系进行旋转为rot2的逆矩阵右乘rot1,如果是绕着自身坐标系进行旋转为rot2的逆矩阵左乘rot1
         * 3. 返回的数值result为旋转角度；
         * 
         * 1. The input values (rx1, ry1, rz1) define the rotation vector for the initial pose, 
         * and (rx2, ry2, rz2) define the rotation vector for the target pose. 
         * rotate_WorldorSelf, when true, indicates rotation about the world coordinate system; when false, it indicates rotation about the robot's own axes
         * 2. If the rotation is about the world coordinate system, the inverse of the target rotation matrix (rot2⁻¹) is right-multiplied by the initial rotation matrix (rot1) (rot1 * rot2⁻¹).
         * If the rotation is about the robot's own axes, the inverse of the target rotation matrix (rot2⁻¹) is left-multiplied by the initial rotation matrix (rot1) (rot2⁻¹ * rot1).
         * 3. The returned value, result, represents the rotation angle.
         * 
         */
        public static List<double> MatrixRotate(double rx1, double ry1, double rz1, double rx2, double ry2, double rz2, bool rotate_WorldorSelf)
        {
            List<double> result = new List<double>();


            TxTransformation rot1 = new TxTransformation(new TxVector(rx1, ry1, rz1), TxTransformation.TxRotationType.RPY_XYZ);


            TxTransformation rot2 = new TxTransformation(new TxVector(rx2, ry2, rz2), TxTransformation.TxRotationType.RPY_XYZ);
            TxTransformation rot = new TxTransformation();
            if (rotate_WorldorSelf)
            {
                rot = TxTransformation.Multiply(rot1, rot2.Inverse);

            }
            else
                rot = TxTransformation.Multiply(rot2.Inverse, rot1);



            result.Add(rot.RotationRPY_XYZ.X);
            result.Add(rot.RotationRPY_XYZ.Y);
            result.Add(rot.RotationRPY_XYZ.Z);

            return result;
        }

        /* public static List<double> RotationVector(double RX, double RY, double RZ, double rx, double ry, double rz, bool rotate_WorldorSelf)
         * 定义了初始坐标系(RX,RY,RZ)经过旋转角度(rx, ry, rz)后到达的目标坐标系的旋转矢量；
         * 1. (RX,RY,RZ)定义了起始坐标系的旋转矢量；
         * 2. (rx, ry, rz)定义了绕着各个坐标轴的旋转角度；
         * 3. rotate_WorldorSelf 如果为true就是绕着世界坐标进行旋转，false为绕着自身的旋转轴进行旋转
         * 4. 输出List<double> 是经过旋转后的位置坐标系的旋转矢量
         * This defines the rotation vector of a target coordinate system resulting from rotating an initial coordinate system (defined by its rotation vector) by specified angles around specific axes
         * 1. Initial Coordinate System: (RX, RY, RZ) defines the rotation vector of the initial coordinate system.
         * 2. Rotation Angles: (rx, ry, rz) define the rotation angles around the X, Y, and Z axes, respectively
         * 3. Rotation Reference Frame: rotate_WorldorSelf: If true, the rotation is performed around the world coordinate system's axes. If false, the rotation is performed around the axes of the initial coordinate system itself.
         * 4. Output: The output, List<double>, represents the rotation vector of the resulting (target) coordinate system after the rotation.
         */
        public static List<double> RotationVector(double RX, double RY, double RZ, double rx, double ry, double rz, bool rotate_WorldorSelf)
        {
            List<double> result = new List<double>();

            TxTransformation R_Original = new TxTransformation(new TxVector(RX, RY, RZ), TxTransformation.TxRotationType.RPY_XYZ);


            TxTransformation rotationMatrix = new TxTransformation(new TxVector(rx, ry, rz), TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation rot = new TxTransformation();
            if (rotate_WorldorSelf)
            {
                rot = TxTransformation.Multiply(rotationMatrix, R_Original);

            }
            else
                rot = TxTransformation.Multiply(R_Original, rotationMatrix);



            result.Add(rot.RotationRPY_XYZ.X);
            result.Add(rot.RotationRPY_XYZ.Y);
            result.Add(rot.RotationRPY_XYZ.Z);

            return result;

        }

        /*public static List<double> TranslateVec(double[] vecframe, double[] translate, bool worldorself)
         * 在函数RotationVector()的基础上增加了平移；
         * 1. R_VecFrame定义了义了起始坐标系的六个自由度；
         * 2. R_translate定义了染着坐标系的平移及旋转操作的六个自由度；
         * 3. rotate_WorldorSelf 如果为true就是绕着世界坐标进行旋转，false为绕着自身的旋转轴进行旋转
         * 4. 输出List<double> 是经过平移和旋转后的位置坐标系的坐标值
         * 
         * This function, RotationVector(), has been extended to include translation in addition to rotation.
         * 1. Initial Pose: R_VecFrame defines the six degrees of freedom (6DOF) of the initial coordinate system (position and orientation).
         * 2. Transformation: R_translate defines the six degrees of freedom (6DOF) of the transformation applied to the initial coordinate system. This transformation includes both translation and rotation.
         * 3. Rotation Reference Frame: rotate_WorldorSelf: If true, the rotation component of the transformation R_translate is performed around the world coordinate system's axes. If false, the rotation is performed around the axes of the initial coordinate system (R_VecFrame).
         * 4. Output: The output, List<double>, represents the six degrees of freedom (position and orientation) of the resulting coordinate system after applying both the translation and rotation defined in R_translate.
         */
        public static List<double> TranslateVec(double[] vecframe, double[] translate, bool worldorself)
        {
            List<double> result = new List<double>();

            TxTransformation R_VecFrame = new TxTransformation(new TxVector(vecframe[3], vecframe[4], vecframe[5]), // for translation
                new TxVector(vecframe[0], vecframe[1], vecframe[2]), // for rotation
                TxTransformation.TxRotationType.RPY_XYZ);


            TxTransformation R_translate = new TxTransformation(new TxVector(translate[0], translate[1], translate[2]), // for translation
               new TxVector(0, 0, 0), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);
            TxTransformation rot = new TxTransformation();
            if (worldorself)
            {
                rot = TxTransformation.Multiply(R_translate, R_VecFrame);

            }
            else
                rot = TxTransformation.Multiply(R_VecFrame, R_translate);


            result.Add(rot.Translation.X);
            result.Add(rot.Translation.Y);
            result.Add(rot.Translation.Z);

            return result;
        }

        public static char GetNextChar(char current)
        {
            switch (current)
            {
                case 'X':
                    return 'Y';
                case 'Y':
                    return 'Z';
                case 'Z':
                    return 'X';
                default:
                    return current; // 默认返回 'x'，以防输入不在预期范围内
            }
        }
        /*static List<double> spotagainstCollisonSrc(point p, point p_robot, ref char robDir, bool direction)
         *  spotagainstCollisonSrc函数的作用是根据机器人的基坐标系以及UI上面选择的逃离方向来确定机器人的工具按照坐标点哪个方向进行逃离；
         *  当rrt_connect 计算1000次之后仍未找到合适的路径的时候，会在起始点和终止点之间生成过渡点，重新计算起始点到过渡点，以及过渡点到终止点的轨迹；
         * 1. point p 定义的起始点与终止点之间的过渡点，这个过渡点可能碰撞检测失效，所以需要进行过渡点的移动；
         * 2. point p_robot 定义了机器人的基坐标系；
         * 3. robchar 定义了用于在UI界面上选择的坐标轴，是基于机器人的基坐标系来选择的；
         * 4. 计算过程是将p点投影在p_robot坐标系下面的坐标值tx_Project（以p_robot为零位坐标系获取p的坐标值）
         * 5. 计算点tx_Project与UI界面上选择的坐标轴的夹角，并输出夹角最小的点p的坐标轴，则机器人的工具就按照这个坐标轴的方向进行逃离碰撞检测清单；
         * 
         * The spotagainstCollisonSrc function determines the escape direction for the robot's end-effector (tool) to avoid collisions, based on the robot's base coordinate system and the escape direction selected in the user interface (UI).
         * When the RRT-Connect algorithm fails to find a collision-free path after 1000 iterations, an intermediate point is generated between the start and end points. 
         * The algorithm then recalculates paths from the start point to the intermediate point, and from the intermediate point to the end point.
         * 1. Intermediate Point: point p represents this intermediate point, which might be in collision. Therefore, it needs to be moved to a collision-free location.
         * 2. Robot Base Coordinate System: point p_robot defines the origin of the robot's base coordinate system.
         * 3. Escape Axis Selection: robchar represents the axis (X, Y, or Z) selected in the UI. This selection is relative to the robot's base coordinate system；
         * 4. Projection: The function projects point p onto the robot's base coordinate system (p_robot), resulting in the projected point tx_Project. tx_Project represents the coordinates of p relative to p_robot as the origin。
         * 5. Escape Direction Determination: The function calculates the angle between tx_Project and each axis selected via robchar.
         * The axis with the smallest angle to tx_Project is selected as the escape direction. The robot's end-effector will then attempt to move along this axis to resolve the collision.
         */
        public static List<double> spotagainstCollisonSrc(point p, point p_robot, ref char robDir, bool direction)
        {
            TxTransformation tx_p_robot = new TxTransformation(new TxVector(p_robot.x, p_robot.y, p_robot.z), // for translation
               new TxVector(p_robot.rx, p_robot.ry, p_robot.rz), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation tx_World = new TxTransformation(new TxVector(0, 0, 0), // for translation
                new TxVector(0, 0, 0), // for rotation
                TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation tx_P = new TxTransformation(new TxVector(p.x, p.y, p.z), // for translation
               new TxVector(p.rx, p.ry, p.rz), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);

            TxApplication.ActiveDocument.WorkingFrame = tx_p_robot;

            TxTransformation tx_Project = new TxTransformation();


            tx_Project = tx_P.LocationRelativeToWorkingFrame;

            double sinXr = 0;
            double sinYr = 0;
            double sinZr = 0;

            if ((TxrrtRobotPathPlannerForm.iterate_Count % 4 == 0) && (TxrrtRobotPathPlannerForm.iterate_Count != 0) && direction)
            {

                robDir = GetNextChar(robDir);

            }

            if (robDir == 'X')
            {

                sinXr = Math.Sqrt(1 - tx_Project[0, 0] * tx_Project[0, 0]);
                sinYr = Math.Sqrt(1 - tx_Project[0, 1] * tx_Project[0, 1]);
                sinZr = Math.Sqrt(1 - tx_Project[0, 2] * tx_Project[0, 2]);

            }

            if (robDir == 'Y')
            {
                sinXr = Math.Sqrt(1 - tx_Project[1, 0] * tx_Project[1, 0]);
                sinYr = Math.Sqrt(1 - tx_Project[1, 1] * tx_Project[1, 1]);
                sinZr = Math.Sqrt(1 - tx_Project[1, 2] * tx_Project[1, 2]);


            }

            if (robDir == 'Z')
            {
                sinXr = Math.Sqrt(1 - tx_Project[2, 0] * tx_Project[2, 0]);
                sinYr = Math.Sqrt(1 - tx_Project[2, 1] * tx_Project[2, 1]);
                sinZr = Math.Sqrt(1 - tx_Project[2, 2] * tx_Project[2, 2]);



            }


            List<double> result = new List<double>();

            result.Add(sinXr);
            result.Add(sinYr);
            result.Add(sinZr);

            TxApplication.ActiveDocument.WorkingFrame = tx_World;
            return result;


        }



    }
}
