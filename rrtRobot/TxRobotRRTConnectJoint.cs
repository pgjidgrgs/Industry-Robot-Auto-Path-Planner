using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;
namespace rrtRobot
{
    public struct joint
    {
        public double j1 { get; set; }
        public double j2 { get; set; }
        public double j3 { get; set; }
        public double j4 { get; set; }
        public double j5 { get; set; }
        public double j6 { get; set; }


        public double Sever_Gun { get; set; }
        public joint(double J1, double J2, double J3, double J4, double J5, double J6, double Gun_Opening)
        {
            j1 = J1;
            j2 = J2;
            j3 = J3;

            j4 = J4;
            j5 = J5;
            j6 = J6;

            Sever_Gun = Gun_Opening;

        }
        public double[] ToArray()
        {
            return new[] { j1, j2, j3, j4, j5, j6 };
        }

    };

    public struct point
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double rx { get; set; }
        public double ry { get; set; }
        public double rz { get; set; }

        public double Gun_Open { get; set; }


        public int fromStartorEnd { get; set; } // 定义这个点是从起始点延申出来的，还是终止点延申出来的，起始点为0，终止点为1

        public point(double X, double Y, double Z, double RX, double RY, double RZ, double Gun_Opening)
        {
            x = X;
            y = Y;
            z = Z;

            rx = RX;
            ry = RY;
            rz = RZ;

            Gun_Open = Gun_Opening;
            fromStartorEnd = 0;
        }
        public double[] ToArray()
        {
            return new[] { x, y, z, rx, ry, rz };
        }

    };
    public class Node3D_joint
    {
        public joint loc;
        public double cost;
        public Node3D_joint parent;
        public double step_size;


    }
    public class Node3D
    {
        public double x;
        public double y;
        public double z;
        public double rx;
        public double ry;
        public double rz;
        public double cost;
        public Node3D parent;

        public Node3D(double X, double Y, double Z, double RX, double RY, double RZ)
        {
            this.x = X;
            this.y = Y;
            this.z = Z;
            this.rx = RX;
            this.ry = RY;
            this.rz = RZ;

        }

    }

    /* 
     * The TxRobotRRTConnectJoint class is used to calculate the transition point trajectory between solder joints. The algorithm is based on bidirectional RRT tree expansion using the start and end point robot's six axis values.
     * The reference GitHub for the RRT* Connect algorithm is https://github.com/WYoseanLove/RRT-_Connect_3D.
     * During the RRT tree expansion, the robot's six axis values are randomly generated and checked for collisions. If the collision detection is passed, the point is considered valid until a valid path is found.
     * Collision detection first performs single-point detection, and then PTP (Point-to-Point) interpolation. The interpolation function isValidforstepCorss(Control control, joint step, joint near) interpolates based on time. It divides the PTP time based on the angle change; if the angle change is less than 100, it divides and interpolates according to 100.
     * The TxRobotptpPathCal class records the PTP interpolation algorithm for a six-axis robot.
     * The RRT Connect algorithm is enhanced with the calculation of APF (Artificial Potential Field) attractive and repulsive forces to influence the generation of random tree points. The attractive force attracts random points towards the end point and the nearest end node tree point. The repulsive force field records all previous interference points from collision detection, and the repulsive force field is calculated based on the distance between new tree points and these interference points."
     * 
     * TxRobotRRTConnectJoint 类用于计算焊点之间的过渡点轨迹，算法基于起始点和终止点机器人的六个轴值进行rrt树的双向扩展。
     * rrt* connect 算法的参考Github https://github.com/WYoseanLove/RRT-_Connect_3D
     * 在rrt树的扩展中，随机生成机器人的6个轴值，并对其进行碰撞检测，如果通过碰撞检测，则视为有效点，直至扩展出有效的路径；
     * 碰撞检测首先进行单点检测，然后进行ptp的插补运算，插补运算函数isValidforstepCorss(Control control, joint step, joint near)，是基于时间进行插补的，将ptp的时间按照角度变化量进行插补，如果角度变化量小于100，则按照100进行时间分割并插补；
     * TxRobotptpPathCal类记录了六轴机器人ptp的插补运算算法；
     * 在rrt connect 算法上增加了apf引力场和排斥力场的计算，用于干涉随机树点的生成，引力场将随机点朝终止点和最近的endnode树点进行吸引，排斥力场记录了之前碰撞检测的所有干涉点，新的树点与干涉点之间的距离进行排斥力场的计算；
     * 
     */

    public partial class TxRobotRRTConnectJoint : TxrrtRobotPathPlannerForm
    {
        private int connected = 0;
        private int state = 1;
        private int sub_state = 0;
        private const double Max_step_szie = Math.PI / 9;
        private const double Min_step_szie = Math.PI / 1800;
        private double Start_Lower_Step_size = Min_step_szie;
        private double Start_Upper_Step_size = Max_step_szie;
        private double End_Lower_Step_size = Min_step_szie;
        private double End_Upper_Step_size = Max_step_szie;
        private double start_step_size = M_PI / 18;
        private double end_step_size = M_PI / 18;

        private double k_att = 1.0;
        private double k_rep = 5.0;

        private double circle_radius_1 = 20;

        public List<joint> path_points_start = new List<joint>(500);
        private int pathcount_start = 0;
        private List<joint> path_points_end = new List<joint>(500);
        private int pathcount_end = 0;
        private Random rd;
        private List<Node3D_joint> start_nodes = new List<Node3D_joint>(10000);
        private int nodecount_start = 0;
        private List<Node3D_joint> end_nodes = new List<Node3D_joint>(10000);
        private int nodecount_end = 0;
        public static List<joint> obs;

        private int IterationCounts = 0;//记录rrtconnect 的迭代次数；
        private const int stepAdjustCyle = 500;
        public static bool currentpathdone = false;

        public double j1Llimit, j2Llimit, j3Llimit, j4Llimit, j5Llimit, j6Llimit;
        public double j1Ulimit, j2Ulimit, j3Ulimit, j4Ulimit, j5Ulimit, j6Ulimit;
        //private const double influenceRadius = Math.PI / 9; // 障碍物影响半径

        //public static List<joint> InterpolJoints=new List<joint>();
        public static void logpathGenerateOK(string str)
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);
            sw.WriteLine(DateTime.Now.ToLocalTime().ToString() + str);
            sw.Close();

        }
        public static void LograndNodeInformation(point p, string information)
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);

            sw.WriteLine(DateTime.Now.ToLocalTime().ToString() + " " + information + ": " + p.x.ToString() + " "
           + p.y.ToString() + " "
           + p.z.ToString() + " "
           + (p.rx * 180 / M_PI).ToString() + " "
           + (p.ry * 180 / M_PI).ToString() + " "
           + (p.rz * 180 / M_PI).ToString() + " "
           + (p.Gun_Open).ToString() + " ");

            sw.Close();
        }
        public static void LoginterPoljoints(List<joint> InterpolJoints)
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);
            for(int i=0;i< InterpolJoints.Count;i++)
            {
                sw.WriteLine("InterpolJoints " +(i + 1).ToString() + ": " + (InterpolJoints[i].j1 * 180 / M_PI).ToString() + " "
          + (InterpolJoints[i].j2 * 180 / M_PI).ToString() + " "
          + (InterpolJoints[i].j3 * 180 / M_PI).ToString() + " "
          + (InterpolJoints[i].j4 * 180 / M_PI).ToString() + " "
          + (InterpolJoints[i].j5 * 180 / M_PI).ToString() + " "
          + (InterpolJoints[i].j6 * 180 / M_PI).ToString() + " "
          + (InterpolJoints[i].Sever_Gun).ToString() + " ");

            }

            sw.WriteLine();

            sw.Close();
        }

        public double dist(joint p1, joint p2)  // To calculate the distance between two points
        {

            return Math.Sqrt(Math.Pow(p2.j1 - p1.j1, 2) + Math.Pow(p2.j2 - p1.j2, 2) + Math.Pow(p2.j3 - p1.j3, 2)
                + Math.Pow(p2.j4 - p1.j4, 2) + Math.Pow(p2.j5 - p1.j5, 2) + Math.Pow(p2.j6 - p1.j6, 2));

        }

        public int Nearest_Node(int fromstart2end, Node3D_joint rand)
        {
            double min = 999.0;
            int index = -1;

            if (fromstart2end == 1)
            {
                for (int i = 0; i < nodecount_start; i++)
                {

                    if (dist(rand.loc, start_nodes[i].loc) < min)
                    {
                        min = dist(rand.loc, start_nodes[i].loc);
                        index = i;
                    }

                }

            }
            else
            {
                for (int i = 0; i < nodecount_end; i++)
                {

                    if (dist(rand.loc, end_nodes[i].loc) < min)
                    {
                        min = dist(rand.loc, end_nodes[i].loc);
                        index = i;
                    }

                }



            }




            return index;
        }

        public joint step_func(joint near, joint rand, double size_step)
        {
            double j1 = rand.j1 - near.j1;
            double j2 = rand.j2 - near.j2;
            double j3 = rand.j3 - near.j3;
            double j4 = rand.j4 - near.j4;
            double j5 = rand.j5 - near.j5;
            double j6 = rand.j6 - near.j6;



            double d = Math.Sqrt(Math.Pow(j1, 2) + Math.Pow(j2, 2) + Math.Pow(j3, 2) +
                Math.Pow(j4, 2) + Math.Pow(j5, 2) + Math.Pow(j6, 2));

            joint step = new joint(near.j1 + (size_step) * (j1 / d),
                near.j2 + (size_step) * (j2 / d),
                near.j3 + (size_step) * (j3 / d),
                near.j4 + (size_step) * (j4 / d),
                near.j5 + (size_step) * (j5 / d),
                near.j6 + (size_step) * (j6 / d),
                rand.Sever_Gun
                 );

            return step;
        }
        /*
        public bool isValid(Control control, joint step, joint near, bool stepToNear)
        {

            if (!collisioncheckforSingleJoint(control, ref step)) return false;

            if (stepToNear)
            {
                if (!isValidforstepCorss(control,near, step))
                    return false;
            }
            else
            {
                if (!isValidforstepCorss(control, step, near))
                    return false;
            }


            return true;
        }

        public static bool collisioncheckforSingleJoint(Control control, ref joint step)
        {
            // 如果返回false 说明有干涉，返回true说明没有干涉
            using (TxPoseData robotPosture = new TxPoseData())
            {

                ArrayList robJointValue = new ArrayList();

                for (int i = 0; i < step.ToArray().Length; i++)
                {
                    robJointValue.Add(step.ToArray()[i]);

                }

                robotPosture.JointValues = robJointValue;

                ArrayList solutions = new ArrayList();

                solutions.Add(robotPosture);

                TxRobotAPIClass.TxRobotPostureGenerate(control, TxrrtRobotPathPlannerForm.robot, solutions, step.Sever_Gun);

                if (TxRobotAPIClass.Collision_Check(control, cd, queryParams, root, collisionSrc, collisionTar, 5.0)) return true;

                int gun_open_step = 30;
                for (int i = gun_open_step - 1; i >= 1; i--)
                {
                    TxRobotAPIClass.TxRobotPostureGenerate(control, TxrrtRobotPathPlannerForm.robot, solutions, step.Sever_Gun * 1.0 * i / gun_open_step);
                    if (TxRobotAPIClass.Collision_Check(control, cd, queryParams, root, collisionSrc, collisionTar, 5.0))
                    {
                        step.Sever_Gun = step.Sever_Gun * 1.0 * i / gun_open_step;
                        robJointValue.Clear();

                        return true;
                    }

                }
                robJointValue.Clear();

                obs.Add(step);

                return false;

            }

        }
        
        public static bool isValidforstepCorss(Control control, joint step, joint near)  // 将从step 到near中间的干涉点找出并返回
        {
            List<double> jointschange = TxRobotptpPathCal.calculateJointsChange(step, near, robot);

            double servoGunjointChange= TxRobotptpPathCal.calculateServoGunJointChange(step, near, robot);



            (double value, int index) result = TxRobotptpPathCal.FindLargestAbsoluteWithIndex(jointschange);

            double ptpTime = TxRobotptpPathCal.calculatePTPtime(control, jointschange, robot);

            double ServoGunTime = TxRobotptpPathCal.calculateServoPTPtime(servoGunjointChange, robServerGun);

            if (ptpTime <= ServoGunTime)
                ptpTime = ServoGunTime;

            int jointDiv = (int)(ptpTime / 0.05);
            

            for (int i = 0; i < jointDiv; i++)
            {

                using (TxPoseData robotcurrentPosedata = TxRobotptpPathCal.calCurrentRobotPosedata(control, step, near, robot, (ptpTime / jointDiv) * (i + 1),ptpTime))
                {
                    //double Gun_open = step.Sever_Gun + (near.Sever_Gun - step.Sever_Gun) * (i + 1) / jointDiv;


                    double Gun_open = TxRobotptpPathCal.calCurrentServoGunJointData(ptpTime, (ptpTime / jointDiv) * (i + 1), robServerGun, servoGunjointChange, step.Sever_Gun);


                    joint p = new joint((double)robotcurrentPosedata.JointValues[0],
                        (double)robotcurrentPosedata.JointValues[1],
                        (double)robotcurrentPosedata.JointValues[2],
                        (double)robotcurrentPosedata.JointValues[3],
                        (double)robotcurrentPosedata.JointValues[4],
                        (double)robotcurrentPosedata.JointValues[5],
                        Gun_open);

                    if (!collisioncheckforSingleJoint(control, ref p))
                    {

                        jointschange.Clear();

                        return false;
                    }

                }
            }
            jointschange.Clear();

            return true;
        }
        

        /// 从 step 到 near 的轨迹检测：
        /// 1) 任何一次单点 collisioncheckforSingleJoint 失败，立即返回 false；
        /// 2) 如果整条路径都过了单点检测，但某次检测修改了 step，则用新 step 重新检测；
        /// 3) 如果整条路径都过了，且 step 前后相同，则返回 true。
        /// </summary>
        public static bool isValidforstepCorss(Control control, ref joint step, joint near)
        {
            const int MAX_ITERS = 10;    // 防止万一死循环
            int iters = 0;

            while (iters++ < MAX_ITERS)
            {
                // 1) 先拷贝一个快照，用于后面判断 step 有没有被修改
                joint originalStep = step;

                // 2) 计算关节差分和总时间
                List<double> jointsChange = TxRobotptpPathCal.calculateJointsChange(step, near, robot);
                double servoGunChange = TxRobotptpPathCal.calculateServoGunJointChange(step, near, robot);

                (double maxDelta, int idx) = TxRobotptpPathCal.FindLargestAbsoluteWithIndex(jointsChange);
                double ptpTime = TxRobotptpPathCal.calculatePTPtime(control, jointsChange, robot);
                double servoTime = TxRobotptpPathCal.calculateServoPTPtime(servoGunChange, robServerGun);
                if (ptpTime < servoTime) ptpTime = servoTime;

                int divisions = (int)(ptpTime / 0.05);

                bool needRecheck = false;

                for (int i = 0; i < divisions; i++)
                {
                    // 3) 插值出当前时刻的机器人姿态
                    double t = ptpTime / divisions * (i + 1);
                    using (var pose = TxRobotptpPathCal.calCurrentRobotPosedata(
                                          control, step, near, robot, t, ptpTime))
                    {
                        double gun = TxRobotptpPathCal.calCurrentServoGunJointData(
                                          ptpTime, t, robServerGun, servoGunChange, step.Sever_Gun);

                        // 用一个临时 joint p 来做单点检测
                        joint p = new joint(
                            (double)pose.JointValues[0],
                            (double)pose.JointValues[1],
                            (double)pose.JointValues[2],
                            (double)pose.JointValues[3],
                            (double)pose.JointValues[4],
                            (double)pose.JointValues[5],
                            gun
                        );

                        // 4) 单点检测，如果失败，整个函数直接 false
                        if (!collisioncheckforSingleJoint(control, ref p))
                        {
                            jointsChange.Clear();
                            return false;
                        }

                        // 5) 如果 collisioncheckforSingleJoint 在内部“微调”了 p，
                        //    并且 p 与我们传入的 step 不同，就认为要用新 p 重新跑
                        if (!p.Equals(originalStep))
                        {
                            step = p;        // 把 step 更新成最新的无碰撞点
                            needRecheck = true;
                            break;           // 退出 for，重新 while 迭代
                        }
                    }
                }

                jointsChange.Clear();

                // 如果本次迭代里没有触发“step 被改动”，说明整条路径都干净，且 step 保持不变
                if (!needRecheck)
                {
                    return true;
                }

                // 否则跳回 while，用新的 step 再跑一次
            }

            // 超过最大迭代次数仍然没稳定下来，就当作 false
            return false;
        }
        */

        /// 顶层接口：
        /// 1) 先单点剔除碰撞并微调 step；
        /// 2) 再做整段插值碰撞检测；
        ///    只要 collisioncheck 过程中微调了 step，就重跑一遍，直到“整条跑完都没改动 step” 或者碰撞失败。
        /// </summary>
        public bool isValid(Control control, joint step, joint near, bool stepToNear)
        {
            // 单点检测并尝试调整 step
            if (!collisioncheckforSingleJoint(control, ref step))
                return false;

            // 跨点路径检测，只允许微调 step
            if (!isValidforstepCorss(control, ref step, near, stepToNear))
                return false;

            return true;
        }

        /// <summary>
        /// 原样保留：对单独一个姿态做碰撞检测，
        /// 如果检测通过返回 true；否则尝试微调 step.Sever_Gun，
        /// 可调通则 true，否则 false。
        /// </summary>
        public static bool collisioncheckforSingleJoint(Control control, ref joint step)
        {
            using (var robotPosture = new TxPoseData())
            {
                var arr = new ArrayList(step.ToArray());
                robotPosture.JointValues = arr;

                var sols = new ArrayList { robotPosture };
                TxRobotAPIClass.TxRobotPostureGenerate(
                    control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, step.Sever_Gun);

                if (TxRobotAPIClass.Collision_Check(
                    control, cd, queryParams, root, collisionSrc, collisionTar, 5.0))
                    return true;

                const int N = 30;
                for (int i = N - 1; i >= 1; i--)
                {
                    double g = step.Sever_Gun * i / (double)N;
                    TxRobotAPIClass.TxRobotPostureGenerate(
                        control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, g);
                    if (TxRobotAPIClass.Collision_Check(
                        control, cd, queryParams, root, collisionSrc, collisionTar, 5.0))
                    {
                        step.Sever_Gun = g;
                        return true;
                    }
                }

                obs.Add(step);
                return false;
            }
        }

        /// <summary>
        /// 整段路径检测：
        ///   stepToNear=false → 从 step→near，step 是 “起点”，
        ///   stepToNear=true  → 从 near→step，step 是 “终点”。
        /// 只要 collisioncheckforSingleJoint 微调了 step，就把新值写回 step 并重跑一遍，
        /// 直到“一整条跑下来都没再改动 step” 才算 true，
        /// 或者某次单点碰撞失败直接 false。
        /// </summary>
        public static bool isValidforstepCorss(
            Control control,
            ref joint step,
            joint near,
            bool stepToNear
        )
        {
            const int MAX_ITERS = 10;
            int iter = 0;
            while (iter++ < MAX_ITERS)
            {
                //joint before = step;    // 备份本轮进入时的 step

                // 决定插值的 start/end
                joint start = stepToNear ? near : step;
                joint end = stepToNear ? step : near;

                // 计算 PTP 时间和分段数
                var deltas = TxRobotptpPathCal.calculateJointsChange(start, end, robot);
                double dg = TxRobotptpPathCal.calculateServoGunJointChange(start, end, robot);
                double t1 = TxRobotptpPathCal.calculatePTPtime(control, deltas, robot);
                double t2 = TxRobotptpPathCal.calculateServoPTPtime(dg, robServerGun);
                double T = Math.Max(t1, t2);
                int N = (int)(T / 0.01);

                bool bumped = false;
                //InterpolJoints.Add(start);
                //InterpolJoints.Add(end);
                for (int i = 1; i <= N; i++)
                {
                    double t = T * i / N;
                    using (var pose = TxRobotptpPathCal.calCurrentRobotPosedata(
                        control, start, end, robot, t, T))
                    {
                        double gun = TxRobotptpPathCal.calCurrentServoGunJointData(
                            T, t, robServerGun, dg, start.Sever_Gun);

                        joint p = new joint(
                            (double)pose.JointValues[0],
                            (double)pose.JointValues[1],
                            (double)pose.JointValues[2],
                            (double)pose.JointValues[3],
                            (double)pose.JointValues[4],
                            (double)pose.JointValues[5],
                            gun
                        );
                        //InterpolJoints.Add(p);
                        joint before = p;    // 备份本轮进入时的 step
                        // 单点碰撞
                        if (!collisioncheckforSingleJoint(control, ref p))
                        {
                            //InterpolJoints.Clear();
                            return false;
                        }
                            

                        // 如果碰撞检测微调了 p，就把它当做新的 step 然后跳出重跑
                        if (!JointEquals(p, before))
                        {
                            step.Sever_Gun = p.Sever_Gun;
                            bumped = true;
                            break;
                        }
                    }
                }

                if (!bumped)
                {

                    // 打印出插补点；
                    //LoginterPoljoints(InterpolJoints);
                    //InterpolJoints.Clear();
                    return true;    // 本轮没有被微调，说明完整路径都 OK

                }
            }

            // 超过最大迭代次数仍未“收敛”，认为失败
            //InterpolJoints.Clear();
            return false;
        }
        // 只用于最终的轨迹优化
        public static bool isValidforstepCorss(
           Control control,
           joint start,
           joint end
          )
        {
            
            // 计算 PTP 时间和分段数
            var deltas = TxRobotptpPathCal.calculateJointsChange(start, end, robot);
            double dg = TxRobotptpPathCal.calculateServoGunJointChange(start, end, robot);
            double t1 = TxRobotptpPathCal.calculatePTPtime(control, deltas, robot);
            double t2 = TxRobotptpPathCal.calculateServoPTPtime(dg, robServerGun);
            double T = Math.Max(t1, t2);
            int N = (int)(T / 0.05);
        

            for (int i = 1; i <= N; i++)
            {
                double t = T * i / N;
                using (var pose = TxRobotptpPathCal.calCurrentRobotPosedata(
                    control, start, end, robot, t, T))
                {
                    double gun = TxRobotptpPathCal.calCurrentServoGunJointData(
                        T, t, robServerGun, dg, start.Sever_Gun);

                    joint p = new joint(
                        (double)pose.JointValues[0],
                        (double)pose.JointValues[1],
                        (double)pose.JointValues[2],
                        (double)pose.JointValues[3],
                        (double)pose.JointValues[4],
                        (double)pose.JointValues[5],
                        gun
                    );
                    
                    if (!collisioncheckforSingleJoint(control, ref p))
                        return false;

                    
                }
            }


            return true;

        }



        static bool JointEquals(joint a, joint b)
        {
            const double EPS = 1e-6;
            return
              Math.Abs(a.j1 - b.j1) < EPS && Math.Abs(a.j2 - b.j2) < EPS &&
              Math.Abs(a.j3 - b.j3) < EPS && Math.Abs(a.j4 - b.j4) < EPS &&
              Math.Abs(a.j5 - b.j5) < EPS && Math.Abs(a.j6 - b.j6) < EPS &&
              Math.Abs(a.Sever_Gun - b.Sever_Gun) < EPS;
        }




        public void minimal_cost(Control control, Node3D_joint step, int threadholdIter)
        {

            double new_cost;
            double min_cost = step.cost;
            int index = -1;

            if (state == 1)
            {
                circle_radius_1 = 2 * start_step_size;
                for (int i = 0; i < nodecount_start; i++)
                {
                    if (dist(start_nodes[i].loc, step.loc) < circle_radius_1 && isValid(control, step.loc, start_nodes[i].loc, true))
                    {
                        new_cost = dist(start_nodes[i].loc, step.loc) + start_nodes[i].cost;
                        if (new_cost < min_cost)
                        {
                            min_cost = new_cost;
                            index = i;
                        }
                    }

                }
                if (min_cost < step.cost)
                {
                    step.parent = start_nodes[index];
                    step.cost = min_cost;
                }

            }
            else
            {
                circle_radius_1 = 2 * end_step_size;
                for (int i = 0; i < nodecount_end; i++)
                {
                    if (dist(end_nodes[i].loc, step.loc) < circle_radius_1 && isValid(control, step.loc, end_nodes[i].loc, false))
                    {
                        new_cost = dist(end_nodes[i].loc, step.loc) + end_nodes[i].cost;
                        if (new_cost < min_cost)
                        {
                            min_cost = new_cost;
                            index = i;
                        }
                    }

                }
                if (min_cost < step.cost)
                {
                    step.parent = end_nodes[index];
                    step.cost = min_cost;
                }


            }

        }
        public void path_Points(int index_1, int index_2) // final path points will be keep at path_points_start and path_points_end List
        {
            pathcount_start = 0;
            pathcount_end = 0;
            Node3D_joint n1, n2;
            double d = Min_step_szie;

            n2 = start_nodes[index_1 - 1];
            n1 = n2.parent;

            path_points_start.Add(n2.loc);
            pathcount_start++;

            while (n1.parent != null)
            {
                if (dist(n1.loc, path_points_start[pathcount_start - 1]) < d)
                {
                    n1 = n1.parent;
                }
                else
                {
                    n2 = n1;
                    n1 = n2.parent;

                    path_points_start.Add(n2.loc);
                    pathcount_start++;
                }
            }
            n2 = end_nodes[index_2 - 1];
            n1 = n2.parent;

            path_points_end.Add(n2.loc);
            pathcount_end++;
            while (n1.parent != null)
            {
                if (dist(n1.loc, path_points_end[pathcount_end - 1]) < d / 4)
                {
                    n1 = n1.parent;
                }
                else
                {
                    n2 = n1;
                    n1 = n2.parent;
                    path_points_end.Add(n2.loc);
                    pathcount_end++;
                }
            }



        }
        public double GetRandomDouble(double minValue, double maxValue, double lowerLimit, double UpperLimit)
        {
            if (minValue < lowerLimit) minValue = lowerLimit;
            if (maxValue > UpperLimit) maxValue = UpperLimit;



            return rd.NextDouble() * (maxValue - minValue) + minValue;
        }

        private double[] getAttractiveforceField(joint q, joint goal, double k_att)
        {
            double[] gradient = new double[6];

            // 计算吸引势场
            for (int i = 0; i < 6; i++) // 对位置和旋转分量进行吸引势场计算
            {
                gradient[i] = k_att * (goal.ToArray()[i] - q.ToArray()[i]);

            }
            return gradient;

        }
        private double[] ArtificialPotentialField(Control control, joint q, joint near, joint nearfromGoalNodes, joint goal, double step_size)
        {
            double[] gradient = new double[6];
            double[] gradientfromEndNode = new double[6];
            double k_att = 1.0;

            // 计算吸引势场
            gradient = getAttractiveforceField(q, goal, k_att);
            gradientfromEndNode = getAttractiveforceField(q, nearfromGoalNodes, k_att * 3);
            double k_rep = 3.0;
            double safedistance = 5.0;
            double nearmissdata = getcollisonNearmissdata(control, q, cd, queryParams, root, collisionSrc, collisionTar, safedistance);
            double[] RepulsiveForce = new double[6] { 0, 0, 0, 0, 0, 0 };
            if (nearmissdata == 10000)
            {
                RepulsiveForce = new double[6] { 0, 0, 0, 0, 0, 0 };

            }
            else
            {

                if (nearmissdata == 0)
                {
                    RepulsiveForce = getAttractiveforceField(q, near, k_rep);

                }
                else
                {


                    RepulsiveForce = getAttractiveforceField(q, near, k_rep * (1 - nearmissdata / safedistance));

                }


            }


            for (int i = 0; i < RepulsiveForce.Count(); i++)
            {

                gradient[i] += RepulsiveForce[i] + gradientfromEndNode[i];
            }


            return Normalize(gradient);
        }
        private double[] ApfCalculateMethod(Control control, joint q, joint nearfromGoalNodes, joint goal, double step_size, List<joint> obsList, double k_att, double k_rep, double d0)
        {

            double[] gradient = new double[6];
            double[] gradientfromGoalNodes = new double[6];
            double[] RepulsiveForce = new double[6] { 0, 0, 0, 0, 0, 0 };
            //计算吸引势场
            gradient = getAttractiveforceField(q, goal, k_att);
            //计算与对手nodelist里面最近点的吸引势场
            gradientfromGoalNodes = getAttractiveforceField(q, nearfromGoalNodes, 3 * k_att);
            //计算与obs的排斥力场
            foreach (var obs in obsList)
            {

                double distance = dist(q, obs);
                if (distance > d0) continue;
                if (distance == 0)
                {
                    for (int i = 0; i < RepulsiveForce.Length; i++)
                    {
                        RepulsiveForce[i] = 1000000; // 或者任何其他计算方式
                    }
                    continue;
                }

                for (int i = 0; i < 6; i++) // 对位置和旋转分量进行吸引势场计算
                {
                    RepulsiveForce[i] += k_rep * (1 / distance - 1 / d0) * (1 / (distance * distance)) * (q.ToArray()[i] - obs.ToArray()[i]) / distance;

                }


            }

            for (int i = 0; i < RepulsiveForce.Count(); i++)
            {

                gradient[i] += RepulsiveForce[i] + gradientfromGoalNodes[i];

            }


            return Normalize(gradient);

        }

        private static double[] Normalize(double[] vector)
        {
            double norm = Math.Sqrt(vector.Sum(x => x * x));
            return vector.Select(x => x / norm).ToArray();
        }

        private double getcollisonNearmissdata(Control control, joint step, TxCollisionPairCreationData cd, TxCollisionQueryParams queryParams, TxCollisionRoot root, TxObjectList collisionSrc, TxObjectList collisionTar, double Clearance)
        {

            TxPoseData robotPosture = new TxPoseData();
            ArrayList robJointValue = new ArrayList();

            for (int i = 0; i < step.ToArray().Length; i++)
            {
                robJointValue.Add(step.ToArray()[i]);

            }

            robotPosture.JointValues = robJointValue;

            ArrayList solutions = new ArrayList();

            solutions.Add(robotPosture);
            TxRobotAPIClass.TxRobotPostureGenerate(control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, solutions, step.Sever_Gun);

            queryParams.Mode = TxCollisionQueryParams.TxCollisionQueryMode.DefinedPairs;

            queryParams.NearMissDistance = Clearance;

            using (TxCollisionQueryResults results = root.GetCollidingObjects(queryParams))
            {
                double nearmissdata = double.MaxValue;

                if (results.States.Count == 0)
                {
                    results.States.Clear();
                    cd.Dispose();
                    return 10000;
                }

                for (int i = 0; i < results.States.Count; i++)
                {
                    if ((results.States[i] as TxCollisionState).Type == TxCollisionState.TxCollisionStateType.Collision)
                    {
                        results.States.Clear();
                        cd.Dispose();
                        return 0;

                    }
                    if ((results.States[i] as TxCollisionState).Type == TxCollisionState.TxCollisionStateType.NearMiss)
                    {
                        if (nearmissdata >= (results.States[i] as TxCollisionState).Distance)
                            nearmissdata = (results.States[i] as TxCollisionState).Distance;
                    }

                }
                cd.Dispose();
                return nearmissdata;

            }
        }

        public double getMaxdistanceinNodeList(List<Node3D_joint> start_nodes, int lastsamples)// 只计算最后10个点
        {
            if (start_nodes.Count == 1) return 0;

            double maxdistance = double.MaxValue;
            int start_index = 0;
            if (start_nodes.Count <= lastsamples) start_index = 0;
            else start_index = start_nodes.Count - lastsamples;

            for (int i = start_index; i < start_nodes.Count; i++)
            {
                for (int j = i + 1; j < start_nodes.Count; j++)
                {
                    double distance = CalculateDistance(start_nodes[i].loc, start_nodes[j].loc);
                    if (distance < maxdistance) maxdistance = distance;

                }


            }
            return maxdistance;

        }


        public double avgValidNodestepsize(List<Node3D_joint> start_nodes, int lastsamples)
        {
            double avgvalidnodestepsize = 0;

            if (start_nodes.Count == 1) return Min_step_szie;

            int start_index = 0;
            if (start_nodes.Count <= lastsamples) start_index = 0;
            else start_index = start_nodes.Count - lastsamples;

            double sumStepsize = 0;

            for (int i = start_index; i < start_nodes.Count; i++)
            {
                sumStepsize += start_nodes[i].step_size;

            }


            avgvalidnodestepsize = sumStepsize / (start_nodes.Count - start_index);

            return avgvalidnodestepsize;
        }

        static double CalculateDistance(joint j1, joint j2)
        {
            double sum = 0;

            for (int i = 0; i < 6; i++)
            {
                sum += (j1.ToArray()[i] - j2.ToArray()[i]) * (j1.ToArray()[i] - j2.ToArray()[i]);
            }

            return Math.Sqrt(sum);
        }
        public void rrt_connectJointPtp(Control control, joint p_start, joint p_end)
        {
            connected = 0;
            state = 1;
            sub_state = 0;
            nodecount_start = 0;
            nodecount_end = 0;

            Node3D_joint start_node = new Node3D_joint();
            Node3D_joint end_node = new Node3D_joint();
            Node3D_joint rand_node = new Node3D_joint();
            int index;


            Node3D_joint step_node;
            Node3D_joint sub_step_node;

            rd = new Random(unchecked((int)DateTime.Now.Ticks));

            start_node.loc = new joint(p_start.j1, p_start.j2, p_start.j3, p_start.j4, p_start.j5, p_start.j6, p_start.Sever_Gun);

            start_node.parent = new Node3D_joint();
            start_node.cost = 0;
            end_node.loc = new joint(p_end.j1, p_end.j2, p_end.j3, p_end.j4, p_end.j5, p_end.j6, p_end.Sever_Gun);

            end_node.parent = new Node3D_joint();
            end_node.cost = 0;

            start_nodes.Add(start_node);

            nodecount_start++;

            end_nodes.Add(end_node);
            nodecount_end++;
            currentpathdone = false;

            int gun_open_splict = 30;

            obs = new List<joint>();

            while (!collisioncheckforSingleJoint(control, ref p_start))
            {

                return;
            }

            start_node.loc.Sever_Gun = p_start.Sever_Gun;


            while (!collisioncheckforSingleJoint(control, ref p_end))
            {

                return;
            }

            end_node.loc.Sever_Gun = p_end.Sever_Gun;
            int threshold = 1000;
            double p_end2p_start = CalculateDistance(p_end, p_start);
            Start_Upper_Step_size = p_end2p_start / 5;
            //Start_Upper_Step_size = p_end2p_start ;
            if (Start_Upper_Step_size > Max_step_szie) Start_Upper_Step_size = Max_step_szie;
             End_Upper_Step_size = p_end2p_start / 5;
            // End_Upper_Step_size = p_end2p_start ;
            if (End_Upper_Step_size > Max_step_szie) End_Upper_Step_size = Max_step_szie;


            Start_Lower_Step_size = p_end2p_start / 100;
            if (Start_Lower_Step_size < Min_step_szie) Start_Lower_Step_size = Min_step_szie;

            End_Lower_Step_size = p_end2p_start / 100;
            if (End_Lower_Step_size < Min_step_szie) End_Lower_Step_size = Min_step_szie;

            double start_Lower_Temp_step_size = Start_Lower_Step_size;
            double end_Lower_Temp_step_size = End_Lower_Step_size;

            double start_Upper_Temp_step_size = Start_Upper_Step_size;
            double end_Upper_Temp_step_size = End_Upper_Step_size;

            double pre_StartstepAdjustcycleNodecount = 0;
            double pre_StartstepAdjustcycleNoderatio = 0;
            double pre_StartstepAdjustcycleNodeMaxdis = 0;

            double pre_EndstepAdjustcycleNodecount = 0;
            double pre_EndstepAdjustcycleNoderatio = 0;
            double pre_EndstepAdjustcycleNodeMaxdis = 0;

            double current_StartstepAdjustcycleNodecount = 0;
            double current_StartstepAdjustcycleNoderatio = 0;
            double current_StartstepAdjustcycleNodeMaxdis = 0;

            double current_EndstepAdjustcycleNodecount = 0;
            double current_EndstepAdjustcycleNoderatio = 0;
            double current_EndstepAdjustcycleNodeMaxdis = 0;

            int start_interationCount = 0;
            int end_interationCount = 0;
            bool stepsizeadjusted = false;
            while (connected != 1)
            {
                Application.DoEvents();
                if (!rrtconnectCal_ongoing)
                {
                    break;
                }
                if (IterationCounts > 1000)
                {
                    stepsizeadjusted = true;
                    if (IterationCounts % stepAdjustCyle == 0)
                    {
                        //every 500 iteration cycle run the stepsize adjustmen function

                        // start_step_size;
                        double temp = avgValidNodestepsize(start_nodes, 20);

                        current_StartstepAdjustcycleNodecount = start_nodes.Count - pre_StartstepAdjustcycleNodecount;
                        current_StartstepAdjustcycleNoderatio = current_StartstepAdjustcycleNodecount / start_interationCount;
                        current_StartstepAdjustcycleNodeMaxdis = getMaxdistanceinNodeList(start_nodes, 20);

                        if (temp > start_Lower_Temp_step_size * 5 || //如果平均步长大于阈值 start_Lower_Temp_step_size * 5
                            (pre_StartstepAdjustcycleNoderatio > 0 && current_StartstepAdjustcycleNoderatio >= pre_StartstepAdjustcycleNoderatio * 2) ||
                            (pre_StartstepAdjustcycleNodeMaxdis > 0 && current_StartstepAdjustcycleNodeMaxdis >= pre_StartstepAdjustcycleNodeMaxdis * 2)
                            )
                        {
                            Start_Upper_Step_size = start_Upper_Temp_step_size;
                            // logpathGenerateOK(" start step size enter narrow space");

                        }
                        else if (temp <= start_Lower_Temp_step_size * 5)
                        {

                            Start_Upper_Step_size = start_Lower_Temp_step_size * 4;

                            //logpathGenerateOK(" start step size exit narrow space");
                        }

                        pre_StartstepAdjustcycleNodeMaxdis = current_StartstepAdjustcycleNodeMaxdis;
                        pre_StartstepAdjustcycleNodecount = current_StartstepAdjustcycleNodecount;
                        pre_StartstepAdjustcycleNoderatio = current_StartstepAdjustcycleNoderatio;

                        start_interationCount = 0;

                        // end_step_size;

                        temp = avgValidNodestepsize(end_nodes, 20);


                        current_EndstepAdjustcycleNodecount = end_nodes.Count - pre_EndstepAdjustcycleNodecount;
                        current_EndstepAdjustcycleNoderatio = current_EndstepAdjustcycleNodecount / end_interationCount;
                        current_EndstepAdjustcycleNodeMaxdis = getMaxdistanceinNodeList(end_nodes, 20);

                        if (temp > end_Lower_Temp_step_size * 5 || //如果平均步长大于阈值 End_Lower_Temp_step_size * 5
                            (pre_EndstepAdjustcycleNoderatio > 0 && current_EndstepAdjustcycleNoderatio >= pre_EndstepAdjustcycleNoderatio * 2) ||
                            (pre_EndstepAdjustcycleNodeMaxdis > 0 && current_EndstepAdjustcycleNodeMaxdis >= pre_EndstepAdjustcycleNodeMaxdis * 2)
                            )
                        {
                            End_Upper_Step_size = end_Upper_Temp_step_size;
                            //logpathGenerateOK(" end step size enter narrow space");
                        }
                        else if (temp <= end_Lower_Temp_step_size * 5)
                        {

                            End_Upper_Step_size = end_Lower_Temp_step_size * 4;

                            //logpathGenerateOK(" end step size exit narrow space");
                        }

                        pre_EndstepAdjustcycleNodeMaxdis = current_EndstepAdjustcycleNodeMaxdis;
                        pre_EndstepAdjustcycleNodecount = current_EndstepAdjustcycleNodecount;
                        pre_EndstepAdjustcycleNoderatio = current_EndstepAdjustcycleNoderatio;

                        end_interationCount = 0;


                    }

                }

                start_step_size = GetRandomDouble(Start_Lower_Step_size, Start_Upper_Step_size, Start_Lower_Step_size, Start_Upper_Step_size);
                end_step_size = GetRandomDouble(End_Lower_Step_size, End_Upper_Step_size, End_Lower_Step_size, End_Upper_Step_size);
                sub_state = 0;
                IterationCounts++;
                if ((IterationCounts / threshold) == 10) return; //如果迭代次数超过10000次则退出
                if (state == 1)
                {
                    if (stepsizeadjusted) start_interationCount++;
                    double rand_node_gun_open = 0;

                    rand_node_gun_open = ToolJointOpening - rd.Next(0, gun_open_splict) * (ToolJointOpening / gun_open_splict);

                    rand_node.loc = new joint(GetRandomDouble(start_nodes[0].loc.j1 - M_PI / 2, start_nodes[0].loc.j1 + M_PI / 2, j1Llimit, j1Ulimit),
                        GetRandomDouble(start_nodes[0].loc.j2 - M_PI / 2, start_nodes[0].loc.j2 + M_PI / 2, j2Llimit, j2Ulimit),
                        GetRandomDouble(start_nodes[0].loc.j3 - M_PI / 2, start_nodes[0].loc.j3 + M_PI / 2, j3Llimit, j3Ulimit),
                        GetRandomDouble(start_nodes[0].loc.j4 - M_PI, start_nodes[0].loc.j4 + M_PI, j4Llimit, j4Ulimit),
                        GetRandomDouble(start_nodes[0].loc.j5 - M_PI / 2, start_nodes[0].loc.j5 + M_PI / 2, j5Llimit, j5Ulimit),
                        GetRandomDouble(start_nodes[0].loc.j6 - M_PI, start_nodes[0].loc.j6 + M_PI, j6Llimit, j6Ulimit),
                        rand_node_gun_open);

                    index = Nearest_Node(1, rand_node);
                    int index_fromEndNodes = Nearest_Node(2, rand_node);

                    if (index_fromEndNodes < 0) index_fromEndNodes = 0;

                    if (index < 0)
                    {
                        continue;
                    }
                    
                    if (dist(start_nodes[index].loc, rand_node.loc) < start_step_size)
                    {

                        continue;
                    }
                    else
                    {
                        step_node = new Node3D_joint();

                        (step_node.loc) = step_func(start_nodes[index].loc, rand_node.loc, start_step_size);

                        //double[] apf_direction = ArtificialPotentialField(control, step_node.loc, start_nodes[index].loc, end_nodes[index_fromEndNodes].loc, p_end, start_step_size);
                        double[] apf_direction = ApfCalculateMethod(control, step_node.loc, end_nodes[index_fromEndNodes].loc, p_end, start_step_size, obs, k_att, k_rep, 2* start_step_size);
                        double[] q_rand_array = step_node.loc.ToArray();
                        for (int i = 0; i < 6; i++) // 对位置和旋转分量进行调整
                        {
                            q_rand_array[i] += start_step_size * apf_direction[i];
                        }

                        joint q_rand_modified = new joint(q_rand_array[0], q_rand_array[1], q_rand_array[2], q_rand_array[3], q_rand_array[4], q_rand_array[5], step_node.loc.Sever_Gun);

                        step_node.loc = q_rand_modified;


                    }
                    bool isvalidcorss = isValid(control, step_node.loc, start_nodes[index].loc, true);
                    if (isvalidcorss == false)
                    {
                        // 如果扩展失败，尝试使用局部路径规划
                        if (LocalPathPlanningWithAPF(control, ref step_node.loc, p_start,p_end,start_nodes[index].loc, end_nodes[index_fromEndNodes].loc, true,obs,k_att,k_rep,  start_step_size, 5,start_step_size/2))
                        {
                            isvalidcorss = true;
                        }

                    }
                    if (isvalidcorss == true)
                    {

                        step_node.parent = start_nodes[index];
                        step_node.cost = start_nodes[index].cost + start_step_size;
                        minimal_cost(control, step_node, IterationCounts);
                        step_node.step_size = start_step_size;

                        start_nodes.Add(step_node);
                        nodecount_start++;
                    }
                    else continue;
                    state = 2;
                    int end_substate = 0;

                    while (sub_state != 1)
                    {
                        if (!rrtconnectCal_ongoing)
                        {
                            break;
                        }

                        index = Nearest_Node(state, step_node);
                        if (index < 0) continue;
                        if (isValid(control, step_node.loc, end_nodes[index].loc, false) || (!rrtconnectCal_ongoing))
                        {

                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");

                            path_Points(nodecount_start, index + 1);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_joint();

                            (sub_step_node.loc) = step_func(end_nodes[index].loc, step_node.loc, end_step_size);


                        }
                        if (isValid(control, sub_step_node.loc, end_nodes[index].loc, false) == false)
                        {
                            sub_state = 1;
                            continue;
                        }
                        else
                        {
                            sub_step_node.parent = end_nodes[index];
                            sub_step_node.cost = end_nodes[index].cost + end_step_size;
                            minimal_cost(control, sub_step_node, IterationCounts);
                            sub_step_node.step_size = end_step_size;

                            end_nodes.Add(sub_step_node);
                            nodecount_end++;
                            end_substate++;
                            if (end_substate > 5)
                            {
                                sub_state = 1;
                                end_substate = 0;
                            }
                        }

                    }

                }
                if (state == 2)
                {
                    if (stepsizeadjusted) end_interationCount++;
                    double rand_node_gun_open = 0;

                    rand_node_gun_open = ToolJointOpening - rd.Next(0, gun_open_splict) * (ToolJointOpening / gun_open_splict);
                    rand_node.loc = new joint(GetRandomDouble(end_nodes[0].loc.j1 - M_PI / 2, end_nodes[0].loc.j1 + M_PI / 2, j1Llimit, j1Ulimit),
                        GetRandomDouble(end_nodes[0].loc.j2 - M_PI / 2, end_nodes[0].loc.j2 + M_PI / 2, j2Llimit, j2Ulimit),
                        GetRandomDouble(end_nodes[0].loc.j3 - M_PI / 2, end_nodes[0].loc.j3 + M_PI / 2, j3Llimit, j3Ulimit),
                        GetRandomDouble(end_nodes[0].loc.j4 - M_PI, end_nodes[0].loc.j4 + M_PI, j4Llimit, j4Ulimit),
                        GetRandomDouble(end_nodes[0].loc.j5 - M_PI / 2, end_nodes[0].loc.j5 + M_PI / 2, j5Llimit, j5Ulimit),
                        GetRandomDouble(end_nodes[0].loc.j6 - M_PI, end_nodes[0].loc.j6 + M_PI, j6Llimit, j6Ulimit),
                        rand_node_gun_open);
                    index = Nearest_Node(state, rand_node);

                    int index_fromEndNodes = Nearest_Node(1, rand_node);
                    if (index_fromEndNodes < 0) index_fromEndNodes = 0;
                    if (index < 0) continue;
                   
                    if (dist(end_nodes[index].loc, rand_node.loc) < end_step_size) continue;
                    else
                    {
                        step_node = new Node3D_joint();



                        (step_node.loc) = step_func(end_nodes[index].loc, rand_node.loc, end_step_size);


                        //double[] apf_direction = ArtificialPotentialField(control, step_node.loc, end_nodes[index].loc, start_nodes[index_fromEndNodes].loc, p_start, end_step_size);
                        double[] apf_direction = ApfCalculateMethod(control, step_node.loc, start_nodes[index_fromEndNodes].loc, p_start, end_step_size, obs, k_att, k_rep, end_step_size*2);
                        double[] q_rand_array = step_node.loc.ToArray();
                        for (int i = 0; i < 6; i++) // 对位置和旋转分量进行调整
                        {
                            q_rand_array[i] += end_step_size * apf_direction[i];
                        }

                        joint q_rand_modified = new joint(q_rand_array[0], q_rand_array[1], q_rand_array[2], q_rand_array[3], q_rand_array[4], q_rand_array[5], step_node.loc.Sever_Gun);

                        step_node.loc = q_rand_modified;

                    }
                    bool isvalidcorss = isValid(control, step_node.loc, end_nodes[index].loc, false);
                    if (isvalidcorss == false)
                    {
                        if (LocalPathPlanningWithAPF(control, ref step_node.loc, p_start,p_end,start_nodes[index_fromEndNodes].loc, end_nodes[index].loc, false,obs,k_att,k_rep, end_step_size, 5,end_step_size/2))
                        {
                            isvalidcorss = true;
                        }
                    }
                    if (isvalidcorss == true)
                    {

                        step_node.parent = end_nodes[index];
                        step_node.cost = end_nodes[index].cost + end_step_size;
                        minimal_cost(control, step_node, IterationCounts);
                        step_node.step_size = end_step_size;
                        end_nodes.Add(step_node);
                        // TxRobotAPIClass.drawingthePathLine(step_node.parent.loc, step_node.loc, xend, "endNode" + end_nodes.Count.ToString(), robot);

                        nodecount_end++;

                    }
                    else continue;

                    state = 1;
                    int start_substate = 0;
                    while (sub_state != 1)
                    {
                        if (!rrtconnectCal_ongoing) break;

                        index = Nearest_Node(state, step_node);
                        if (index < 0) continue;
                        if (isValid(control, step_node.loc, start_nodes[index].loc, true) || (!rrtconnectCal_ongoing))
                        {
                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");
                            path_Points(index + 1, nodecount_end);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_joint();

                            (sub_step_node.loc) = step_func(start_nodes[index].loc, step_node.loc, start_step_size);

                        }

                        if (isValid(control, sub_step_node.loc, start_nodes[index].loc, true) == false)
                        {


                            sub_state = 1;
                            continue;
                        }
                        else
                        {
                            start_substate++;
                            sub_step_node.parent = start_nodes[index];
                            sub_step_node.cost = start_nodes[index].cost + start_step_size;
                            minimal_cost(control, sub_step_node, IterationCounts);

                            start_nodes.Add(sub_step_node);
                            //TxRobotAPIClass.drawingthePathLine(sub_step_node.parent.loc, sub_step_node.loc, xstart, "startNode" + start_nodes.Count.ToString(), robot);
                            sub_step_node.step_size = start_step_size;
                            nodecount_start++;
                            if (start_substate > 5)
                            {
                                sub_state = 1;
                                start_substate = 0;
                            }

                        }

                    }




                }

            }
            path_points_start.Reverse();

            path_points_start.AddRange(path_points_end);
            start_nodes.Clear();
            end_nodes.Clear();
            logpathGenerateOK(" Path Generate OK");
            currentpathdone = true;// 记录当前的轨迹已经计算结束，无论是正常结束还是手动结束
        }
        // 新增的方法：局部路径规划
        // 新增的方法：局部路径规划
        public bool LocalPathPlanningWithAPF(Control control, ref joint current, joint p_start, joint p_goal, joint NearStartNodes, joint NearGoalNodes, bool fromstart2end, List<joint> obsList, double k_att, double k_rep, double influenceRadius, int maxIterations, double learningRate)
        {
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (fromstart2end) //表示从start向end去扩展
                {
                    // 计算当前位置的势场梯度  
                    double[] gradient = ApfCalculateMethod(control, current, NearGoalNodes, p_goal, learningRate, obsList, k_att, k_rep, influenceRadius);

                    // 沿着梯度方向调整位置
                    current.j1 += learningRate * gradient[0];
                    current.j2 += learningRate * gradient[1];
                    current.j3 += learningRate * gradient[2];
                    current.j4 += learningRate * gradient[3];
                    current.j5 += learningRate * gradient[4];
                    current.j6 += learningRate * gradient[5];

                    // 检查调整后的新位置是否有效
                    if (isValid(control, current, NearStartNodes, true))
                    {
                        return true; // 找到有效路径
                    }
                }
                else
                {
                    // 计算当前位置的势场梯度  
                    double[] gradient = ApfCalculateMethod(control, current, NearStartNodes, p_start, learningRate, obsList, k_att, k_rep, influenceRadius);

                    // 沿着梯度方向调整位置
                    current.j1 += learningRate * gradient[0];
                    current.j2 += learningRate * gradient[1];
                    current.j3 += learningRate * gradient[2];
                    current.j4 += learningRate * gradient[3];
                    current.j5 += learningRate * gradient[4];
                    current.j6 += learningRate * gradient[5];

                    // 检查调整后的新位置是否有效
                    if (isValid(control, current, NearGoalNodes, false))
                    {
                        return true; // 找到有效路径
                    }



                }


            }


            return false; // 未能找到有效路径
        }

    }
}



