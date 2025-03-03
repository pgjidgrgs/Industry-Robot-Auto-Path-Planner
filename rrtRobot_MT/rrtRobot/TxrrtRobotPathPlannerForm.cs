using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Ui;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;



namespace rrtRobot
{
    /* rrtRobot dll文件是基于SIMENSE Tecnomatix Process simulate 开发的一款用于工业机器人路径自动仿真的插件；
     * 生成的dll需要参照参考文件"如何激活dll在tecnomatix仿真软件功能"的参考文件进行安装和调试；
     * dll代码中使用的是基于Tecnomatix APi函数进行的开发，主要用到其碰撞检测，机器人的正逆向运动学运算，机器人posture 的改变及路径点的自动化生成；
     * 类TxRobotKinematix中包含了以上常用的函数供系统调用；
     * 类TxRobotRRTConnect类中包含了常用的路径生成算法RRT* CONNECT, 代码的可靠性已经在VTK三维中进行了验证，具体可以参考如下Github链接：
     * https://github.com/WYoseanLove/RRT-_Connect_3D
     * 系统在计算过程中的log文件保存在Documents/rrtRobot里面，也可以自己修改log 函数生成自身需要的参数信息；
     * 本程序目前只试用于机器人伺服焊钳的点焊轨迹的生成；
     * 软件界面首先需要选取需要自动生成轨迹的机器人，具体代码在函数void m_txRobotName_Picked()中定义，需要保证选取机器人安装的工具正是生成轨迹的Weld Gun;
     * 软件界面再次选取的是碰撞检测的内容，这里需要将所有用于碰撞检测的数据均添加其中；
     * 完成以上任务之后，所有的配置已经完成；
     * 选中需要计算的机器人轨迹(只能包括焊点，其他的类型点不参加计算）, 点击path connect,计算完成后，点击生成轨迹即可；
     * 
     * The rrtRobot.dll file is a plugin for industrial robot path simulation, developed using Siemens Tecnomatix Process Simulate.
     * Installation: 
     * The generated DLL requires installation and debugging according to the instructions in the reference document "How to activate the DLL in Tecnomatix simulation software."
     * Functionality: 
     * The DLL utilizes the Tecnomatix API, primarily for collision detection, robot forward and inverse kinematics, robot posture manipulation, and automated path point generation.
     * Key Class: TxRobotKinematix: This class contains commonly used functions for robot kinematics, accessible by the system.
     * Key Class: TxRobotRRTConnect: This class implements the RRT*-Connect path planning algorithm. The code's reliability has been verified using VTK 3D visualization (see GitHub link).
     * https://github.com/WYoseanLove/RRT-_Connect_3D
     * Logging: 
     * Log files are saved in Documents/rrtRobot/. The log function can be modified to output custom parameters
     * Application: 
     * This program is currently designed for generating welding trajectories for robot-mounted servo welding guns (spot welding).
     * Robot Selection (m_txRobotName_Picked()):
     * The software interface requires selecting the robot for which to generate the trajectory. Ensure the selected robot has the correct welding gun attached
     * Collision Detection Data: 
     * The interface also requires selecting all data necessary for collision detection
     * Workflow: 
     * After configuring the robot, collision data, then select the welding points (only welding points are considered; 
     * other point types are ignored), click "path connect," and after the calculation is complete, click "generate trajectory."
     */

    public partial class TxrrtRobotPathPlannerForm : TxForm
    {
        
        public static double M_PI = 3.1415926;
        public static Control mainTxControl;
        public static TxTransformation TCPLocation; //记录所选机器人的TCP Frame 值，从robot 的Tool frame 到TCP frame 的转换矩阵，用于正向运动学计算
        public static TxTransformation ToolFrameLocation; //记录所选机器人的Tool Frame 值，从robot 的Tcp frame 到Tool frame 的转换矩阵,用于逆运动学计算
        public static TxFrame baseFrame; //记录所选机器人的Base Frame 值;
        public static TxObjectList collisionTar;
        public static TxCollisionPairCreationData cd;
        public static TxCollisionPair cp;
        public static TxCollisionQueryParams queryParams;
        public static TxObjectList collisionSrc;
        public static TxCollisionRoot root;
        public static TxRobot robot;
        public static TxServoGun robServerGun;
        private int progressbarNumber = 0;
        private int progressbarCount = 0;
        public TxWeldOperation weldTargetOperation;
        public static double ToolJointOpening; // 存放的是焊钳的open的尺寸
        public static bool rrtconnectCal_ongoing = false;
        public List<Tuple<Tuple<Node3D, string>, TxPoseData>> node3D_startptpList;
        public List<Tuple<Tuple<Node3D, string>, TxPoseData>> node3D_endptpList;
        private const double influenceRadius = 10; // 障碍物影响半径
        public static int iterate_Count = 0;// 记录由于1000次rrt 迭代计算无法得到结果而跳出的次数，这个次数会决定了rrt生成的RX/RY的旋转角度
        private static int collisionCount = 0;
        public static List<List<joint>> fullpath;

        // 用于log文件txt的生成，在系统Documents/rrtRobot文件夹下面
        // 获取当前用户的Documents路径
        public static string LogfilePath;
        // 定义子文件夹名称和文件名

        public char spotagainstCollisionSrc;
        public TxrrtRobotPathPlannerForm()
        {
            GenerateLogfile();
        }
        public void GenerateLogfile()
        {
            // 组合完整的子文件夹路径和文件路径
            string documentsPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            string subFolderName = "rrtRobot";
            string fileName = "dataLog.txt";

            string subFolderPath = Path.Combine(documentsPath, subFolderName);
            LogfilePath = Path.Combine(subFolderPath, fileName);

            // 如果子文件夹不存在，则创建它
            if (!Directory.Exists(subFolderPath))
            {
                Directory.CreateDirectory(subFolderPath);
            }
            // 检查文件是否存在，如果不存在则创建并写入文件
            if (!File.Exists(LogfilePath))
            {
                File.Create(LogfilePath).Dispose();

            }

        }
        public void Form_Setup()
        {
            //对对话框控件的字体进行修改，在WPF中修改的字体无法在Tecnomatix中显示；
            m_pathGenerate.Enabled = false;
            m_pathGenerate.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            m_spotDirec.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            Robot_Name.Font = new System.Drawing.Font("Microsoft Sans Serif", 13);
            TCP.Font = new System.Drawing.Font("Microsoft Sans Serif", 13);
            Collision_Src.Font = new System.Drawing.Font("Microsoft Sans Serif", 13);
            groupBox2.Size = new System.Drawing.Size(465, 112);
            Group_Collision.Size = new System.Drawing.Size(465, 103);
            groupBox1.Size = new System.Drawing.Size(465, 92);
           
            this.Size = new System.Drawing.Size(507, 456);
            this.FormBorderStyle = FormBorderStyle.Fixed3D;
            collisionSrc = new TxObjectList();

            m_collisionListPick.Enabled = false;
            button1.Enabled = false;

        }
        private void m_txRobotName_Picked(object sender, TxObjEditBoxCtrl_PickedEventArgs args)
        {
            /* 定义用于轨迹分析和生成的机器人，并将其存储在TxRobot robot 中去，作为成员变量，供其他函数和类调用
             * 选中的机器人的工具坐标会在对话框中显示出来
             * 如果机器人安装了多个工具，则软件会自动将所有的工具全部卸掉，需要人工重新安装，确保机器人上的工具就是进行轨迹分析的焊钳，且只有一个；
             * 目前无法自动识别，安装的工具是焊钳还是其他工具如抓手等等，待找到相应的API之后进行更新；
             * ToolJointOpening 变量存储的是焊钳open的角度，这个角度会作为参数供TxRootRRTConnect等其他类进行调用；
             * 在参数ToolJointOpening赋值的过程中，使用了try,如果报错，则说明Mounted Tool 并非weld Gun, 或者是焊钳的设置有问题，需要重新检查和安装工具
             * 将机器人，和焊钳添加到碰撞检测的target中去，Src的定义需要用户来实现
             */

            robot = m_txRobotName.Object as TxRobot;
            if (robot != null)
            {

                TxFrame tcpFrame = robot.TCPF;
                TxFrame robot_J6_Frame = robot.Toolframe;
                baseFrame = robot.Baseframe;
                TCPLocation = tcpFrame.GetLocationRelativeToObject(robot_J6_Frame);
                ToolFrameLocation = robot_J6_Frame.GetLocationRelativeToObject(tcpFrame);
                m_txTCPFrame.Text = TCPLocation.Translation.ToString(2) + " " + TCPLocation.RotationRPY_XYZ.ToString(2);
                TxApplication.ActiveDocument.WorkingFrame = robot.AbsoluteLocation;
            }
            else
            {
                m_txTCPFrame.Text = string.Empty;
                TxMessageBox.Show("No Robot selected SetUp", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            ITxDevice tSelectObject = robot as ITxDevice;

            ArrayList toolJointValues = new ArrayList();

            if (robot.MountedTools.Count == 0)
            {
                TxMessageBox.Show("No Weld Gun Added in Picked Robot Tool Box !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            tSelectObject = robot.MountedTools[0] as ITxDevice;
            if (tSelectObject == null)
            {
                TxMessageBox.Show("No Weld Gun Added in Picked Robot Tool Box !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            if (robot.MountedTools.Count > 1)
            {
                for (int i = 0; i < robot.MountedTools.Count; i++)
                {
                    robot.UnmountTool(robot.MountedTools[i] as ITxLocatableObject);
                    i--;
                }

                TxMessageBox.Show("Weld Gun not set as the first Mounted Tools, please re-mount the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;


            }

            TxObjectList x = tSelectObject.PoseList;
            try
            {

                for (int i = 0; i < x.Count; i++)
                {

                    string poseName = ((TxPose)x[i]).Name;
                    poseName = poseName.ToLower();
                    if (poseName == "open")
                    {
                        toolJointValues = ((TxPose)x[i]).PoseData.JointValues;
                        break;
                    }
                    else
                        continue;

                }
                ToolJointOpening = (double)Convert.ToInt16(toolJointValues[0]);

                if (ToolJointOpening == 0)
                {
                    for (int i = 0; i < robot.MountedTools.Count; i++)
                    {
                        robot.UnmountTool(robot.MountedTools[i] as ITxLocatableObject);
                        i--;
                    }
                    TxMessageBox.Show("Not Get the Gun Openning Data, please re-check the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;

                }
            }
            catch (Exception e)
            {
                TxMessageBox.Show("Weld Gun not set as System Requested Mounted Tools, please re-mount the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;

            }
            collisionTar =robot.MountedTools;
            collisionTar.Add(robot);
            m_collisionListPick.Enabled = true;
            button1.Enabled = true;
        }

        private void m_collisionListPicked(object sender, TxObjComboBoxCtrl_PickedEventArgs args)
        {
            TxObjectBase txObjects = m_collisionListPick.Object as TxObjectBase;
            if (txObjects==null)
            {
                m_collisionListPick.LoseFocus();
                if (collisionSrc.Count != 0)
                {
                    m_collisionListPick.SelectObject(collisionSrc[collisionSrc.Count - 1]);
                }
                return;
            }



            collisionSrc.Add(m_collisionListPick.Object);
            
            m_collisionListPick.AddItem(m_collisionListPick.Object.Name, m_collisionListPick.Object);
            m_collisionListPick.LoseFocus();
        }

        private void weldOperationSpotAllocate(TxWeldOperation weldOperation)
        {
            /*对所选择的进行轨迹规划的Operation 进行焊点分析，确保每个焊点均均被可达性，且干涉量为0；
             * 如果不可达，或者存在干涉的情况，则自动绕焊点的Z轴进行旋转，每旋转10deg作为一个step, 直到此焊点可达为止，即退出并进行下一个
             * 如果旋转36次之后仍未可达，则将此焊点移出Operation,不进行焊接轨迹的计算；
             * 调用RobotKinemetix.robotInverseCal()函数进行逆向学计算
             * RobotKinemetix.TxRobotPostureGenerate(),将机器人调试至焊点姿态；
             * Collision_Check(),确认是否有干涉存在；
             */

            if (weldOperation == null)
                return;

            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
            TxObjectList allWeldPointsExist = weldOperation.GetDirectDescendants(opFilter);
            bool postureOK = false;
            int LocRotationCount = 0;
            for (int i = 0; i < allWeldPointsExist.Count; i++)
            {
                double tx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.X;
                double ty = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Y;
                double tz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Z;
                double rx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.X;
                double ry = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Y;
                double rz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Z;


                //移除process Type, 并改为Line;

                ArrayList Location_paramList = ((TxWeldLocationOperation)allWeldPointsExist[i]).Parameters;

                for (int j = 0; j < Location_paramList.Count; j++) //iterate throug the arraylist
                {

                    if (Location_paramList[j].GetType() == typeof(TxRoboticStringParam)) //if param is int
                    {

                        try
                        {
                            TxRoboticStringParam stringParam = Location_paramList[j] as TxRoboticStringParam;
                            if (stringParam.Type == "ProcessType")
                            {
                                stringParam.Value = "";
                                ((TxWeldLocationOperation)allWeldPointsExist[i]).SetParameter(new TxRoboticStringParam("ProcessType", ""));
                            }



                        }
                        catch (Exception e)

                        {

                            throw;

                        }

                    }

                }

                Location_paramList = ((TxWeldLocationOperation)allWeldPointsExist[i]).Parameters;

                for (int j = 0; j < Location_paramList.Count; j++) //iterate throug the arraylist
                {

                    if (Location_paramList[j].GetType() == typeof(TxRoboticIntParam)) //if param is int
                    {

                        try
                        {

                            TxRoboticIntParam intParam = Location_paramList[j] as TxRoboticIntParam;
                            if (intParam.Type == "RRS_MOTION_TYPE")
                            {
                                intParam.Value = 2;
                                ((TxWeldLocationOperation)allWeldPointsExist[i]).SetParameter(new TxRoboticIntParam("RRS_MOTION_TYPE", 1));
                            }


                        }
                        catch (Exception e)

                        {

                            throw;

                        }

                    }




                }
             
                point weldPoc = new point(tx, ty, tz, rx, ry, rz, 0);
                ArrayList Solutions = TxRobotAPIClass.robotInverseCal(mainTxControl,robot, weldPoc);
                while (!postureOK)
                {
                    if (Solutions.Count != 0)
                    {
                        TxRobotAPIClass.TxRobotPostureGenerate(mainTxControl, robot, Solutions, 0);
                        if (TxRobotAPIClass.Collision_Check(mainTxControl, cd, queryParams, root, collisionSrc, collisionTar, 3.0))
                        {
                            postureOK = true;
                            break;
                        }


                    }
                    ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation = ((TxWeldLocationOperation)RobotLocationRotationSteps(allWeldPointsExist[i], true, 0, 0, 10, 0, 0, 0)).AbsoluteLocation;
                    tx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.X;
                    ty = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Y;
                    tz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Z;
                    rx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.X;
                    ry = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Y;
                    rz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Z;
                    Solutions = TxRobotAPIClass.robotInverseCal(mainTxControl,robot, new point(tx, ty, tz, rx, ry, rz, 0));
                    LocRotationCount++;
                    if (LocRotationCount == 36)
                    {
                        TxApplication.ActiveDocument.OperationRoot.AddObject(((TxWeldLocationOperation)allWeldPointsExist[i]));
                        break;
                    }

                    else
                        continue;



                }
                TxRobotAPIClass.DisposeTxposureData(Solutions);
                Solutions.Clear();
                postureOK = false;
                LocRotationCount = 0;

            }

            /*
           * 清空log文件
           */
            FileStream stream = File.Open(LogfilePath, FileMode.OpenOrCreate, FileAccess.Write);
            stream.Seek(0, SeekOrigin.Begin);
            stream.SetLength(0);
            stream.Close();


        }
        public TxObjectBase RobotLocationRotationSteps(ITxObject RobLocation, bool Weld_Via, double Steps_X, double Steps_Y, double Steps_Z, double Move_Steps_X, double Move_Steps_Y, double Move_Steps_Z)
        {
            /*
             以原有的自身坐标系为起始坐标系；
             绕Z轴旋转Steps_Z角度，值为度数，不是弧度；
             绕旋转之后的新坐标系Y轴旋转Steps_Y角度，值为度数，不是弧度；
             绕旋转之后的新坐标系X轴旋转Steps_X角度，值为度数，不是弧度；

             */

            TxVector rotation = new TxVector();
            rotation.X = Steps_X * M_PI / 180;
            rotation.Y = Steps_Y * M_PI / 180;
            rotation.Z = Steps_Z * M_PI / 180;


            TxTransformation txTransformation = new TxTransformation();

            txTransformation.Translation.X = Move_Steps_X;
            txTransformation.Translation.Y = Move_Steps_Y;
            txTransformation.Translation.Z = Move_Steps_Z;

            txTransformation.RotationRPY_XYZ = rotation;

            TxVector Translation = new TxVector();
            Translation.X = Move_Steps_X;
            Translation.Y = Move_Steps_Y;
            Translation.Z = Move_Steps_Z;

            TxTransformation loca_Translate = new TxTransformation(Translation, TxTransformation.TxTransformationType.Translate);


            if (Weld_Via)
            {
                TxTransformation weldFrameLocation = ((TxWeldLocationOperation)RobLocation).AbsoluteLocation;
                ((TxWeldLocationOperation)RobLocation).AbsoluteLocation = weldFrameLocation * txTransformation * loca_Translate;
                return ((TxWeldLocationOperation)RobLocation);

            }
            else
            {
                TxTransformation weldFrameLocation = ((TxRoboticViaLocationOperation)RobLocation).AbsoluteLocation;
                ((TxRoboticViaLocationOperation)RobLocation).AbsoluteLocation = weldFrameLocation * txTransformation * loca_Translate;
                return ((TxRoboticViaLocationOperation)RobLocation);

            }



        }
        public double GetRandomDouble(double minValue, double maxValue, double lowerLimit, double UpperLimit)
        {
            if (minValue < lowerLimit) minValue = lowerLimit;
            if (maxValue > UpperLimit) maxValue = UpperLimit;

            Random random = new Random();

            return random.NextDouble() * (maxValue - minValue) + minValue;
        }
        private bool getptpPassthroughPoints(joint p_start, joint p_end, ref point passPoint, out TxPoseData robotPosture)
        {
            List<double> jointschange = TxRobotptpPathCal.calculateJointsChange(p_start, p_end, robot);

            (double value, int index) result = TxRobotptpPathCal.FindLargestAbsoluteWithIndex(jointschange);

            double ptpTime = TxRobotptpPathCal.calculatePTPtime(mainTxControl,jointschange, robot);

            int jointDiv = (int)Math.Abs((result.value / M_PI) * 180);

            int i = jointDiv / 2;

            TxPoseData robotcurrentPosedata = TxRobotptpPathCal.calCurrentRobotPosedata(mainTxControl, p_start, p_end, robot, (ptpTime / jointDiv) * (i + 1));

            double Gun_open = p_start.Sever_Gun + (p_end.Sever_Gun - p_start.Sever_Gun) * (i + 1) / jointDiv;

            joint p = new joint((double)robotcurrentPosedata.JointValues[0],
                (double)robotcurrentPosedata.JointValues[1],
                (double)robotcurrentPosedata.JointValues[2],
                (double)robotcurrentPosedata.JointValues[3],
                (double)robotcurrentPosedata.JointValues[4],
                (double)robotcurrentPosedata.JointValues[5],
                Gun_open);



            int count = 0;

            double step = M_PI / 180;

            int step_count = 1;

            joint q = p;
            robotPosture = new TxPoseData();
            while (!TxRobotRRTConnectJoint.collisioncheckforSingleJoint(mainTxControl,ref q))
            {

                q.j1 = GetRandomDouble(p.j1 - step_count * step * 2, p.j1 + step_count * step * 2, robot.Joints[0].LowerSoftLimit, robot.Joints[0].UpperSoftLimit);
                q.j2 = GetRandomDouble(p.j2 - step_count * step * 4, p.j2 + step_count * step * 4, robot.Joints[1].LowerSoftLimit, robot.Joints[1].UpperSoftLimit);
                q.j3 = GetRandomDouble(p.j3 - step_count * step * 4, p.j3 + step_count * step * 4, robot.Joints[2].LowerSoftLimit, robot.Joints[2].UpperSoftLimit);
                q.j4 = GetRandomDouble(p.j4 - step_count * step * 5, p.j4 + step_count * step * 5, robot.Joints[3].LowerSoftLimit, robot.Joints[3].UpperSoftLimit);
                q.j5 = GetRandomDouble(p.j5 - step_count * step * 4, p.j5 + step_count * step * 4, robot.Joints[4].LowerSoftLimit, robot.Joints[4].UpperSoftLimit);

                q.j6 = GetRandomDouble(p.j6 - step_count * step * 10, p.j6 + step_count * step * 10, robot.Joints[5].LowerSoftLimit, robot.Joints[5].UpperSoftLimit);

                count++;
                if (count % 500 == 0)
                {
                    step_count++;

                }


                if (count > 5000)
                {

                    TxRobotRRTConnectJoint.LograndNodeInformation(passPoint, "bybass Point not calculated out and add failed: ");
                    return false;
                }

            }


            ArrayList robJointValue = new ArrayList();

            for (i = 0; i < q.ToArray().Length; i++)
            {
                robJointValue.Add(q.ToArray()[i]);

            }

            robotPosture.JointValues = robJointValue;

            robot.CurrentPose = robotPosture;

            passPoint = new point(

                robot.TCPF.AbsoluteLocation.Translation.X,
                robot.TCPF.AbsoluteLocation.Translation.Y,
                robot.TCPF.AbsoluteLocation.Translation.Z,
                robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                p.Sever_Gun

                );

            TxRobotRRTConnectJoint.LograndNodeInformation(passPoint, "bybass Point added: ");
            return true;
        }

        private void m_pathGenerate_Click(object sender, EventArgs e)
        {
            /* RRT CONNECT计算结束，生辰轨迹点；
            * 首先将没有计算出的焊点移出现有的轨迹
            * 按照焊点顺序和fullpath 的顺序依次生成轨迹坐标

            */
            if (fullpath.Count == 0)
            {
                TxMessageBox.Show("No Path Generated !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));

            TxObjectList allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            if (fullpath.Count != node3D_startptpList.Count)
            {
                // 首先将没算完成的焊点移出去再生成轨迹
                string lastpathweldName = "";
                for (int i = fullpath.Count; i >= 0; i--)
                {
                    if (node3D_startptpList[i].Item1.Item2 != "bypass")
                    {
                        lastpathweldName = node3D_startptpList[i].Item1.Item2;
                        break;
                    }
                }

                for (int i = allWeldPointsExist.Count - 1; i >= 0; i--)
                {
                    if (allWeldPointsExist[i].Name != lastpathweldName)
                    {
                        TxApplication.ActiveDocument.OperationRoot.AddObject(allWeldPointsExist[i]);
                        allWeldPointsExist.Remove(allWeldPointsExist[i]);
                    }
                    else
                        break;

                }

            }


            //确认每个焊点是否都进行了进行robot teach 
            for (int i = 0; i < allWeldPointsExist.Count; i++)
            {
                if ((allWeldPointsExist[i] as TxWeldLocationOperation).RobotConfigurationData == null)
                {
                    TxMessageBox.Show("please Teach the weld spot target point as reference for Robot ConfigurationData !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }
                if ((allWeldPointsExist[i] as TxWeldLocationOperation).RobotExternalAxesData == null)
                {
                    TxMessageBox.Show("please Setup ServerGun Joint Value !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }


            }

            int pathindex = 0;
            for (int i = 0; i < allWeldPointsExist.Count; i++)
            {
                if (pathindex >= fullpath.Count) break;

                if (i == (allWeldPointsExist.Count - 1)) continue;

                for (int j = 0; j < fullpath[pathindex].Count; j++)
                {

                    TxPoseData robotPosture = new TxPoseData();
                    ArrayList robJointValue = new ArrayList();

                    for (int k = 0; k < fullpath[pathindex][j].ToArray().Length; k++)
                    {
                        robJointValue.Add(fullpath[pathindex][j].ToArray()[k]);

                    }

                    robotPosture.JointValues = robJointValue;

                    robot.CurrentPose = robotPosture;


                    point p = new point(

                     robot.TCPF.AbsoluteLocation.Translation.X,
                     robot.TCPF.AbsoluteLocation.Translation.Y,
                     robot.TCPF.AbsoluteLocation.Translation.Z,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                   fullpath[pathindex][j].Sever_Gun
                );



                    TxRoboticViaLocationOperation RobFramepostLocation = TxRobotPathOptimizePtp.addrobotPathViaLoc("LocTemp" + allWeldPointsExist[i].Name + j.ToString(), new point(p.x, p.y, p.z,
                       p.rx, p.ry, p.rz, p.Gun_Open), weldTargetOperation, robot);

                    weldTargetOperation.MoveChildAfter((TxWeldLocationOperation)allWeldPointsExist[i + 1], RobFramepostLocation);


                    TxRobotConfigurationData txRobotConfigurationData = robot.GetPoseConfiguration(robotPosture);
                    RobFramepostLocation.RobotConfigurationData = txRobotConfigurationData;
                }
                if ((pathindex + 1) >= node3D_startptpList.Count) continue;
                if (allWeldPointsExist[i + 1].Name != node3D_startptpList[pathindex + 1].Item1.Item2) i--;

                pathindex++;
            }

            TxRobotPathOptimizePtp.OperationOptimize(ref weldTargetOperation, robot);
            rrtCalThread = null;

            GC.Collect();
        }

        private Task rrtCalThread;

        private  void button1_Click(object sender, EventArgs e)
        {

            if (mainTxControl == null)  mainTxControl = this;
            //创建碰撞干涉的检查类组;
            root = TxApplication.ActiveDocument.CollisionRoot;
            for (int i = 0; i < root.PairList.Count; i++)
            {
                if (root.PairList[i].Name == "cp1")
                {
                    root.PairList[i].Delete();
                    i--;
                }

                else
                    continue;
            }

            cd = new TxCollisionPairCreationData("cp1", collisionSrc, collisionTar, 3.0);
            if (cd.FirstList.Count == 0 || cd.SecondList.Count == 0)
            {
                TxMessageBox.Show("No collision src or target list are setted up successfully", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            cp = root.CreateCollisionPair(cd);

            queryParams = new TxCollisionQueryParams();

            if (rrtconnectCal_ongoing)// 在计算未完成的时候，强行终止目前的计算
            {
                rrtconnectCal_ongoing = false;
                UpdateProgressBar((int)PathprogressBar.Maximum);
                button1.Text = "Path Connect";
                m_pathGenerate.Enabled = true;
                return;
               
            }

            /*
             * 1. 首先确认operation的根目录下是否选择的焊接程序，如果有选择的，自动规划已选择的焊点程序
             * 2. 确认选择的焊点程序中每个焊点均是可达的，且没有干涉，如果有干涉，则重新定义焊点的方向直至不干涉；
             * 3. 如果无法达到，且无法避免干涉，则将此焊点移除选择的焊点程序，并放到根目录下；           
             */

            ITxObject selectItem = TxApplication.ActiveSelection.GetLastPickedItem() as ITxObject;
            if (selectItem == null || selectItem.GetType().Name != "TxWeldOperation")
            {
                TxMessageBox.Show("Please select the Weld OP firstly before connect!", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            progressbarCount = 0;
            //确认operation的根目录下是否存在没有分配的焊点，如果有则确认是否具备机器人的可达性           
            TxOperationRoot txOperationRoot = TxApplication.ActiveDocument.OperationRoot;
            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
            weldTargetOperation = selectItem as TxWeldOperation;
            TxObjectList allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            if (allWeldPointsExist.Count == 0)
            {
                TxMessageBox.Show("No Weld Points founded under the Robot Welding Operation !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;

            }

            // 确定选择的焊点轨迹，每个焊点均能够达到，不能达到则改变方向或者移出来
            weldOperationSpotAllocate(weldTargetOperation);
            allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);
            progressbarCount = allWeldPointsExist.Count - 1;
            fullpath = new List<List<joint>>();
            node3D_startptpList = new List<Tuple<Tuple<Node3D, string>, TxPoseData>>(); 
            node3D_endptpList = new List<Tuple<Tuple<Node3D, string>, TxPoseData>>();
            for (int i = 0; i < allWeldPointsExist.Count - 1; i++)
            {

                double x = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.X;
                double y = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Y;
                double z = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Z;
                double rx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.X;
                double ry = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Y;
                double rz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Z;



                TxPoseData spotPoseData = robot.GetPoseAtLocation(allWeldPointsExist[i] as ITxRoboticLocationOperation);
               
                Node3D node3D_start = new Node3D(x, y, z, rx, ry, rz);
                node3D_startptpList.Add(Tuple.Create(Tuple.Create(node3D_start, allWeldPointsExist[i].Name), spotPoseData));

                x = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.Translation.X;
                y = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.Translation.Y;
                z = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.Translation.Z;
                rx = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.RotationRPY_XYZ.X;
                ry = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.RotationRPY_XYZ.Y;
                rz = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.RotationRPY_XYZ.Z;


                Node3D node3D_goal = new Node3D(x, y, z, rx, ry, rz);
                spotPoseData = robot.GetPoseAtLocation(allWeldPointsExist[i+1] as ITxRoboticLocationOperation);
                node3D_endptpList.Add(Tuple.Create(Tuple.Create(node3D_goal, allWeldPointsExist[i+1].Name), spotPoseData));

            }

            rrtconnectCal_ongoing = true;
            button1.Text = "Stop";

            PathprogressBar.Value = 0;
            m_collisionListPick.Enabled = false;
            m_txRobotName.Enabled = false;
            if (collisionSrc.Count != 0)
            {
                m_collisionListPick.SelectObject(collisionSrc[collisionSrc.Count - 1]);
            }
            /* rrtconnectCalOnGoing_Async()异步启动rrt计算。
             * Tecnomatix不支持多线程开发，所以为了达到进度条的显示和软件的可视化，采用异步机制；
             * 之前采用多线程，process simulate 软件总是莫名退出，提示内存读写冲突；
             */
            TxRobotRRTConnectJoint.logpathGenerateOK("Path Calculate Start !");
            rrtconnectCalOnGoingPtp_Async();
            TxRobotRRTConnectJoint.logpathGenerateOK("Path Calculate end !");
            //rrtCalThread = Task.Run(()=>rrtconnectCalOnGoingPtp_Async());

            //await rrtCalThread;


        }
        private void rrtconnectCalOnGoingPtp_Async()
        {
            try
            {

                for (int i = 0; i < node3D_startptpList.Count; i++)
                {
                    
                    if (!rrtconnectCal_ongoing)
                    {
                        break;
                    }


                    List<joint> path = new List<joint>();
                    Node3D node3D_start = node3D_startptpList[i].Item1.Item1;
                    Node3D node3D_goal = node3D_endptpList[i].Item1.Item1;
                    TxPoseData robstartPosedata = node3D_startptpList[i].Item2;
                    TxPoseData robendPosedata = node3D_endptpList[i].Item2;


                    TxRobotRRTConnectJoint rrt = new TxRobotRRTConnectJoint();
                    rrt.j1Llimit = robot.Joints[0].LowerSoftLimit;
                    rrt.j1Ulimit = robot.Joints[0].UpperSoftLimit;

                    rrt.j2Llimit = robot.Joints[1].LowerSoftLimit;
                    rrt.j2Ulimit = robot.Joints[1].UpperSoftLimit;

                    rrt.j3Llimit = robot.Joints[2].LowerSoftLimit;
                    rrt.j3Ulimit = robot.Joints[2].UpperSoftLimit;

                    rrt.j4Llimit = robot.Joints[3].LowerSoftLimit;
                    rrt.j4Ulimit = robot.Joints[3].UpperSoftLimit;

                    rrt.j5Llimit = robot.Joints[4].LowerSoftLimit;
                    rrt.j5Ulimit = robot.Joints[4].UpperSoftLimit;

                    rrt.j6Llimit = robot.Joints[5].LowerSoftLimit;
                    rrt.j6Ulimit = robot.Joints[5].UpperSoftLimit;


                    joint start_p = new joint(

                      (double)robstartPosedata.JointValues[0],
                      (double)robstartPosedata.JointValues[1],
                      (double)robstartPosedata.JointValues[2],
                      (double)robstartPosedata.JointValues[3],
                      (double)robstartPosedata.JointValues[4],
                      (double)robstartPosedata.JointValues[5],
                      ToolJointOpening

                        );

                    joint end_p = new joint(

                      (double)robendPosedata.JointValues[0],
                      (double)robendPosedata.JointValues[1],
                      (double)robendPosedata.JointValues[2],
                      (double)robendPosedata.JointValues[3],
                      (double)robendPosedata.JointValues[4],
                      (double)robendPosedata.JointValues[5],
                      ToolJointOpening

                     );


                    rrt.rrt_connectJointPtp(mainTxControl, start_p, end_p);

                    if (!TxRobotRRTConnectJoint.currentpathdone) //表示当下的计算没有产生合适的路径而退出
                    {
                        iterate_Count++;
                        /*
                         * 当迭代1000次之后，需要取中间点，之前的算法是取start 点和end 点的中点;
                         * 改变策略，之前从start 点运动到end 点 找到干涉的姿态后，记录干涉的点；
                         * 以干涉点为起始点，通过generatePassThroughPoints()函数改变干涉嗲拿的姿态并加入到轨迹中进行运算
                        */
                        point bypassPoint = new point((node3D_goal.x + node3D_start.x) / 2, (node3D_goal.y + node3D_start.y) / 2, (node3D_goal.z + node3D_start.z) / 2,
                                  node3D_goal.rx, node3D_goal.ry, node3D_goal.rz, ToolJointOpening);
                        TxPoseData bypassPosture = robstartPosedata;
                        bool bypassPointOK = getptpPassthroughPoints(start_p, end_p, ref bypassPoint, out bypassPosture);
                        if (bypassPointOK)
                        {
                            Node3D p_start_Node = new Node3D(bypassPoint.x, bypassPoint.y, bypassPoint.z, bypassPoint.rx, bypassPoint.ry, bypassPoint.rz);
                            node3D_startptpList.Insert(i + 1, Tuple.Create(Tuple.Create(p_start_Node, "bypass"), bypassPosture));

                            node3D_endptpList.Insert(i, Tuple.Create(Tuple.Create(p_start_Node, "bypass"), bypassPosture));

                            i--;
                            rrt.Dispose();
                            continue;

                        }
                        else
                        {
                            i--;
                            rrt.Dispose();
                            continue;
                        }


                    }

                    /*
                    * 如果迭代次数超过1000，而造成退出，则在退出的起始点和终止点增加一个中点，然后再进行计算
                    */
                    if(rrtconnectCal_ongoing)
                    {
                        path = rrt.path_points_start;
                        fullpath.Add(path);
                    }
                  
                    rrt.Dispose();

                    TxRobotRRTConnectJoint.currentpathdone = true;// 记录当前的轨迹已经计算结束，无论是正常结束还是手动结束
                    if (node3D_endptpList[i].Item1.Item2 != "bypass") progressbarNumber++;
                    if (rrtconnectCal_ongoing)
                        UpdateProgressBar((int)((PathprogressBar.Maximum * ((double)progressbarNumber / progressbarCount))));

                }


                SetButtonState(m_pathGenerate, true);
                SetButtonState(button1, false);
                SetButtonText(button1, "Path Connect");
                rrtconnectCal_ongoing = false;



            }
            catch (Exception ex) {

                TxRobotRRTConnectJoint.logpathGenerateOK(ex.Message);


            }
            
            
            

        }
        private void UpdateProgressBar(int value)
        {
            if (PathprogressBar.InvokeRequired)
            {
                // Use Invoke to update the progress bar on the UI thread
                PathprogressBar.Invoke(new Action(() => PathprogressBar.Value = value));
            }
            else
            {
                // Directly update the progress bar if already on UI thread
                PathprogressBar.Value = value;
            }
        }

        private void SetButtonState(System.Windows.Forms.Button myButton,bool enabled)
        {
            if (myButton.InvokeRequired)
            {
                myButton.Invoke(new Action(() => myButton.Enabled = enabled));
            }
            else
            {
                myButton.Enabled = enabled;
            }
        }

        private void SetButtonText(System.Windows.Forms.Button myButton, string text)
        {
            if (myButton.InvokeRequired)
            {
                myButton.Invoke(new Action(() => myButton.Text = text));
            }
            else
            {
                myButton.Text = text;
            }
        }

    }
}
