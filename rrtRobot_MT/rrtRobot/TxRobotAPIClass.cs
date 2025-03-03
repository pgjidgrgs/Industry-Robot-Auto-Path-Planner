using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Tecnomatix.Engineering;

namespace rrtRobot
{
    public class TxRobotAPIClass
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
        
        public static void TxRobotPostureGenerate(Control control, TxRobot robot, ArrayList Solutions, double Gun_openning)
        {

            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                control.Invoke(new Action<Control, TxRobot, ArrayList,  double>(TxRobotPostureGenerate), control, robot, Solutions,  Gun_openning);
            }
            else
            {
                // Execute the main logic of setting the robot and tool poses
                if (Solutions.Count == 0)
                    return;

                TxPoseData poseData = (TxPoseData)Solutions[0];
                robot.CurrentPose = poseData;

                ITxDevice tSelectObject = robot.MountedTools[0] as ITxDevice;

                ArrayList ToolJointValue = new ArrayList();
                ToolJointValue.Add(Gun_openning);


                TxPoseData txToolPoseData = tSelectObject.CurrentPose;
                txToolPoseData.JointValues = ToolJointValue;
                tSelectObject.CurrentPose = txToolPoseData;

            }


        }

        /* public static ArrayList robotInverseCal(TxRobot robot, point target)
         * Robot inverse kinematics calculations are performed using the built-in functions of TECNOMATIX. 
         * Other inverse kinematics functions (custom-defined) can also be substituted.
         * 通过TECNOMATIX自带的函数进行机器人逆向学计算，这里也可以更换成其他的逆向学函数(自定义的)
         */

        public static ArrayList robotInverseCal(Control control, TxRobot robot, point target)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (ArrayList)control.Invoke(new Func<Control, TxRobot, point, ArrayList>(robotInverseCal), control, robot, target);
            }
            else
            {
                // The main logic of the method
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

                ArrayList solutions = robot.CalcInverseSolutions(txRobotInverseData);

                return solutions;
            }
        }
        /*  
         * Implement collision detection using TECNOMATIX's internal functions. Call TxCollisionQueryResults via a using statement to guarantee proper disposal of resources.
         * 通过TECNOMATIX自带的函数进行碰撞检测，通过using 调用TxCollisionQueryResults，并及时释放资源
         */

        public static bool Collision_Check(Control control, TxCollisionPairCreationData cd, TxCollisionQueryParams queryParams, TxCollisionRoot root, TxObjectList collisionSrc, TxObjectList collisionTar, double Clearance)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (bool)control.Invoke(new Func<Control, TxCollisionPairCreationData, TxCollisionQueryParams, TxCollisionRoot, TxObjectList, TxObjectList, double, bool>(
                    Collision_Check), control, cd, queryParams, root, collisionSrc, collisionTar, Clearance);
            }
            else
            {
                // Execute the main logic of the function
                queryParams.Mode = TxCollisionQueryParams.TxCollisionQueryMode.DefinedPairs;
                queryParams.NearMissDistance = Clearance;

                using (TxCollisionQueryResults results = root.GetCollidingObjects(queryParams))
                {
                    if (results.States.Count == 0)
                    {
                        results.States.Clear();
                        cd.Dispose();
                        return true;
                    }

                    for (int i = 0; i < results.States.Count; i++)
                    {
                        if ((results.States[i] as TxCollisionState).Type == TxCollisionState.TxCollisionStateType.Collision)
                        {
                            results.States.Clear();
                            cd.Dispose();
                            return false;
                        }
                    }
                    return true;

                }

            }
        }

        public static void DisposeTxposureData(ArrayList Solutions)
        {
            if (Solutions.Count == 0) return;

            for (int i = 0; i < Solutions.Count; i++)
            {

                (Solutions[i] as TxPoseData).Dispose();
            }

        }

    }
}
