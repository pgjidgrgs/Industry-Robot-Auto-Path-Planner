using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.CommandParameters;
using Tecnomatix.Engineering.PrivateImplementationDetails;

namespace rrtRobot
{

    public class TxRobotPathOptimize
    {
        static int FindIndex(TxObjectList list, string name)
        {
            for (int i = 0; i < list.Count; i++)
            {
                // 从 ArrayList 中获取对象并进行类型转换
                ITxObject target = list[i] as ITxObject;
                if (target != null && target.Name == name)
                {
                    return i;
                }
            }
            return -1; // 如果未找到匹配项，则返回 -1
        }
        public static bool OperationOptimize(ref TxWeldOperation _robweldOperation, TxRobot robot)
        {
            TxTypeFilter opFilter = new TxTypeFilter();
            opFilter.AddIncludedType(typeof(TxWeldLocationOperation));
            //opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotWeldOpLocation = _robweldOperation.GetAllDescendants(opFilter);
            //筛选出轨迹路径中的焊点和过渡点位
            TxTypeFilter opFilterpass = new TxTypeFilter();
            opFilterpass.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotviaOpLocation = _robweldOperation.GetAllDescendants(opFilterpass);
            opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotOpLocation = _robweldOperation.GetAllDescendants(opFilter);
            /*
             * _robotWeldOpLocation包括所有的焊点信息
             * _robotviaOpLocation包括所有的过渡点信息；
             * _robotOpLocation包括所有的焊点+过渡点的信息；
             * 
             */
            for (int i = 0; i < _robotWeldOpLocation.Count - 1; i++)
            {

                //第一个焊点在轨迹中的位置
                int _startIndex = FindIndex(_robotOpLocation, _robotWeldOpLocation[i].Name);
                //紧邻的第二个焊点在轨迹中的位置；
                int _endIndex = FindIndex(_robotOpLocation, _robotWeldOpLocation[i + 1].Name);
                _startIndex += 1; //紧邻第一个焊点的过渡点在轨迹中的位置；
                _endIndex -= 1;//紧邻第二个焊点的过渡点在轨迹中的位置；
                               //如果前一个点为焊点轨迹，或者后一个点为焊点轨迹，则改变这个过渡点的rx/ry/rz数值
                (_robotOpLocation[_startIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = (_robotOpLocation[_startIndex - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X;
                (_robotOpLocation[_startIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = (_robotOpLocation[_startIndex - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y;
                (_robotOpLocation[_startIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = (_robotOpLocation[_startIndex - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z;

                (_robotOpLocation[_endIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = (_robotOpLocation[_endIndex + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X;
                (_robotOpLocation[_endIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = (_robotOpLocation[_endIndex + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y;
                (_robotOpLocation[_endIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = (_robotOpLocation[_endIndex + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z;
                for (int j = _startIndex + 1; j < _endIndex; j++) // j从前往后，k是从后往前
                {

                    for (int k = _endIndex - 1; k > j; k--)
                    {

                        point start = new point(

                       (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                       );

                        point end = new point(

                           (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                         );

                        point p = new point();
                        if (TxRobotRRTConnect.isValidforstepCorss(start, end,out p))
                        {

                            while (j != k - 1)
                            {
                                //将j后面的过渡点删掉，由于删除了过渡点，则_endIndex 和k值也需要相应的减少index;
                                _robotOpLocation[j + 1].Delete();
                                _robotOpLocation.RemoveAt(j + 1);
                                k--;
                                _endIndex--;

                            }
                            // j 移动到删除点之后的，再次寻找是否可以找到删除的点;
                            j = k + 1;
                            break;
                        }
                        else
                            continue;

                    }

                }
            }

            //进行每个过渡点的robot teach 
            TxPoseData txPoseDataSpot = new TxPoseData();
            for (int i = 0; i < _robotOpLocation.Count; i++)
            {

                if (_robotOpLocation[i].GetType() == typeof(TxWeldLocationOperation))
                {

                    txPoseDataSpot = robot.GetPoseAtLocation(_robotOpLocation[i] as ITxRoboticLocationOperation);

                    continue;
                }
                else
                {
                    point pathPoint = new point(

                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                       );
                    point pre_pathPoint, after_pathPoint;
                    if (_robotOpLocation[i - 1].GetType()== typeof(TxWeldLocationOperation))
                    {
                        pre_pathPoint = new point(

                       (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.Translation.X,
                      (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.Translation.Y,
                      (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.Translation.Z,
                      (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                      (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                      (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                      (_robotOpLocation[i - 1] as TxWeldLocationOperation).RobotExternalAxesData[0].JointValue
                       );


                    }
                    else
                    {
                        pre_pathPoint = new point(

                      (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                     (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                     (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                     (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                     (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                     (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                     (_robotOpLocation[i - 1] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                      );

                    }

                    if (_robotOpLocation[i + 1].GetType() == typeof(TxWeldLocationOperation))
                    {
                        after_pathPoint = new point(

                      (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.Translation.X,
                     (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.Translation.Y,
                     (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.Translation.Z,
                     (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                     (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                     (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                     (_robotOpLocation[i + 1] as TxWeldLocationOperation).RobotExternalAxesData[0].JointValue
                      );


                    }
                    else
                    {

                        after_pathPoint = new point(

                      (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                     (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                     (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                     (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                     (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                     (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                     (_robotOpLocation[i + 1] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                      );

                    }

                       

                    ArrayList Solutions = RobotKinemetix.robotInverseCal(robot, pathPoint);

                    if (Solutions.Count == 0) continue;


                    int besSolution = ChooseBestInverseSolution_ST_Value(Solutions, txPoseDataSpot,robot, ref pathPoint,pre_pathPoint,after_pathPoint);
                    /*
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X = pathPoint.x;
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y = pathPoint.y;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z = pathPoint.z;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = pathPoint.rx;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = pathPoint.ry;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = pathPoint.rz;
                    */

                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).Locate(new TxTransformation(
                        
                        new TxVector(pathPoint.x, pathPoint.y, pathPoint.z),
                        new TxVector(pathPoint.rx, pathPoint.ry, pathPoint.rz),
                        TxTransformation.TxRotationType.RPY_XYZ
                        ));
                    
                    Solutions = RobotKinemetix.robotInverseCal(robot, pathPoint);
                    if (besSolution > Solutions.Count) continue;

                    TxPoseData txPoseData = (TxPoseData)Solutions[besSolution];


                    TxRobotConfigurationData locationConfigData = robot.GetPoseConfiguration(txPoseData);

                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).RobotConfigurationData = locationConfigData;

                    //txPoseDataSpot = txPoseData;
                    //RobotKinemetix.DisposeTxposureData(Solutions);
                    //Solutions.Clear();


                }



            }


            return true;

        }

        //比较两个角度，并考虑角度的周期性问题(360deg) Compare two angles, taking into account their periodicity (360 degrees).
        private static double AngleDifference(double angle1, double angle2)
        {
            double diff = Math.Abs(angle1 - angle2);
            return Math.Min(diff, 2 * Math.PI - diff);
        }
        //计算某个逆向解与上一个点的总关节角度变化  Calculate the total joint angle change between a given inverse kinematic solution and the previous point.
        private static double CalculateTotalDifference(List<double> current_Solution, List<double> pre_Solution)
        {

            double totalDifference = 0;

            for (int i = 0; i < current_Solution.Count; i++)
            {
                totalDifference += AngleDifference(current_Solution[i], pre_Solution[i]);

            }

            return totalDifference;


        }
        /*static int ChooseBestInverseSolution(ArrayList current_Solution, TxPoseData pre_txPoseData,bool AllJointsorNot)
         * 从所有逆向解列表中选出最优的逆向解，并定义为机器人在该点处的姿态：
         * 1. List<double> current_Solution 所有逆向解的列表；
         * 2. pre_txPoseData 与之前的逆向解进行对比，并挑选出最合适的逆向解；
         * 3. AllJointsorNot 是否考虑所有的关节角度；
         *    如果为true, 则计算所有的6个关节角度，并选出关节角度之和变化最小的逆向解；
         *    如果为false, 则只计算1轴，4轴和6轴，此时已默认机器人的ST值进行了定义
         * The optimal inverse kinematic solution is selected from the list of available solutions and defined as the robot's pose at that point.
         * 1. Inverse Kinematic Solutions: List<double> current_Solution contains all calculated inverse kinematic solutions.
         * 2. Comparison with Previous Solution: pre_txPoseData represents the previous inverse kinematic solution. The optimal solution from current_Solution is selected based on its similarity to pre_txPoseData.
         * 3. Joint Angle Consideration (AllJointsorNot): This parameter determines which joint angles are considered when selecting the optimal solution.
         *    true: All six joint angles are considered. The solution with the minimum total change in joint angles (sum of absolute differences) compared to pre_txPoseData is selected.
         *    false: Only joints 1, 4, and 6 are considered. This assumes that the robot's ST values (likely referring to specific robot parameters or configurations) have already been defined.
         * 
         */
        private static int ChooseBestInverseSolution(ArrayList current_Solution, TxPoseData pre_txPoseData,bool AllJointsorNot)
        {

            double minDifference = double.MaxValue;
            int bestSulotion = int.MaxValue;

            List<double> pre_Solution = new List<double>();
            if(AllJointsorNot)
            {
                pre_Solution.Add((double)pre_txPoseData.JointValues[0]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[1]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[2]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[3]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[4]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[5]);
            }
            else
            {
                pre_Solution.Add((double)pre_txPoseData.JointValues[0]);

                pre_Solution.Add((double)pre_txPoseData.JointValues[3]);
                
                pre_Solution.Add((double)pre_txPoseData.JointValues[5]);
            }

            for (int i = 0; i < current_Solution.Count; i++)
            {
                List<double> varSolution = new List<double>();
                if(AllJointsorNot)
                {
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[0]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[1]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[2]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[3]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[4]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[5]);
                }
                else
                {
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[0]);
                   
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[3]);
                   
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[5]);

                }
                
                double totalDifference = CalculateTotalDifference(varSolution, pre_Solution);

                if (totalDifference < minDifference)
                {
                    minDifference = totalDifference;
                    bestSulotion = i;
                }

            }



            return bestSulotion;
        }
        /*static ArrayList filterRobotConfigSolution(TxRobotConfigurationData pre_locationConfigData, ArrayList current_Solution,TxRobot robot)
         * 从所有的逆向解列表中挑选出来与之前逆向解ST值相同的逆向解，并加入到新的逆向解清单作为筛选出的逆向解；
         * 1. TxRobotConfigurationData pre_locationConfigData 前一个机器人姿态的ST值；
         * 2. List<double> current_Solution 所有逆向解的列表；
         * 3. robot 参与计算和定义逆向解姿态的机器人；
         * 
         * This process selects inverse kinematic solutions from a list that match the ST values of a previous solution and adds them to a new list of filtered solutions.
         * 1. Previous Robot Pose: TxRobotConfigurationData pre_locationConfigData stores the ST values (likely representing specific robot configuration parameters) of the previous robot pose.
         * 2. Inverse Kinematic Solutions: List<double> current_Solution contains all calculated inverse kinematic solutions.
         * 3. Robot: robot specifies the robot used for calculating and defining the inverse kinematic poses.
         */
        private static ArrayList filterRobotConfigSolution(TxRobotConfigurationData pre_locationConfigData, ArrayList current_Solution,TxRobot robot)
        {
            ArrayList filter_Solutions = new ArrayList();

            for (int i = 0; i < current_Solution.Count; i++)
            {

                TxRobotConfigurationData locationConfigData = robot.GetPoseConfiguration((TxPoseData)current_Solution[i]);
                if (
                    (locationConfigData.OverheadState == pre_locationConfigData.OverheadState)
                    &&
                    (
                    (locationConfigData.JointsConfigurations[2] as TxJointConfigurationData).State
                    == (pre_locationConfigData.JointsConfigurations[2] as TxJointConfigurationData).State
                    )
                    )
                {
                    filter_Solutions.Add((TxPoseData)current_Solution[i]);
                }


            }


            return filter_Solutions; 


        }
        private static int choseBestInverseSolutionIteration = 0; //计算逆向解优化的次数；Determine the number of times the inverse kinematics solution was optimized.
        /*static int ChooseBestInverseSolution_ST_Value(ArrayList current_Solution, TxPoseData pre_txPoseData, TxRobot robot, ref point pathPoint,point pre_Via, point after_Via)
         * 从筛选过的逆向解列表中挑出最优的逆向解
         * 1 current_Solution 定义了机器人路径点计算出的所有逆向解列表；
         * 2 pre_txPoseData 机器人前一个路径点的逆向解对应的姿态；
         * 3 robot-当前计算和规划路径的机器人；
         * 4 pathPoint 当前计算的路径点的坐标值，并根据最优姿态修改并返回此路径点的坐标；
         * 5 pre_Via，after_Via 定义了前一个路径点和后一个路径点的坐标值，用于计算当此路径点坐标值优化后再次进行碰撞检测，确保优化后的pathpoint是有效的；
         * 代码采用递归的形式，连续迭代5次来优化过度点的位置坐标，保证此位置点与上一个位置点的ST值相同，来达到使路径相对平滑的功能；
         * 
         * This code selects the optimal inverse kinematics solution from a filtered list. Let's break down the bullet points and then discuss the overall function:
         * 1 current_Solution (1): This variable holds a list of all calculated inverse kinematics solutions for a robot's path point. Inverse kinematics is the process of determining the joint angles needed to reach a desired end-effector pose (position and orientation).
         * 2 pre_txPoseData (2): This stores the pose (position and orientation) corresponding to the inverse kinematics solution of the previous path point. This is crucial for ensuring smooth transitions between points.
         * 3 robot (3): This represents the robot for which the path is being planned. Different robots have different kinematic structures, affecting the inverse kinematics calculations.
         * 4 pathPoint (4): This variable initially holds the coordinates of the current path point being calculated. The function modifies these coordinates based on the optimal pose found, and then returns the updated coordinates.
         * 5 pre_Via, after_Via (5): These variables store the coordinates of the path points before and after the current path point. They're used for collision detection after optimizing the pathPoint coordinates. This ensures that the optimized point doesn't cause a collision with the environment.
         * The code uses a recursive approach, iterating up to 5 times. 
         * The goal is to refine the pathPoint coordinates to maintain the same "ST value" (presumably some measure of smoothness or transition) between consecutive points. 
         * This iterative refinement aims to create a smoother overall robot path. 
         * The nature of the "ST value" isn't explicitly defined, but it likely represents a metric related to the change in joint angles or velocities between consecutive points.
         */
        private static int ChooseBestInverseSolution_ST_Value(ArrayList current_Solution, TxPoseData pre_txPoseData, TxRobot robot, ref point pathPoint,point pre_Via, point after_Via)
        {
            int bestSulotion = int.MaxValue;

            TxRobotConfigurationData pre_locationConfigData = robot.GetPoseConfiguration(pre_txPoseData);
            ArrayList filter_Solutions=new ArrayList();

            filter_Solutions = filterRobotConfigSolution(pre_locationConfigData, current_Solution, robot);
            if (filter_Solutions.Count!=0)
            {
                int bestIndex= ChooseBestInverseSolution(filter_Solutions, pre_txPoseData,false);
                bestSulotion = current_Solution.IndexOf(filter_Solutions[bestIndex]);
            }
            else
            {
                // if the IK results only exist two results, random modify the via loc and let it have at least 4 results
                optimizePathPointSTValue(ref pathPoint, pre_Via, after_Via, robot);
                current_Solution = RobotKinemetix.robotInverseCal(robot, pathPoint);
                choseBestInverseSolutionIteration++;
                if(choseBestInverseSolutionIteration==5)
                {
                    bestSulotion = ChooseBestInverseSolution(current_Solution, pre_txPoseData,true);
                    choseBestInverseSolutionIteration = 0;
                    return bestSulotion;
                }
                bestSulotion = ChooseBestInverseSolution_ST_Value(current_Solution, pre_txPoseData, robot, ref pathPoint, pre_Via, after_Via);
               

            }

            choseBestInverseSolutionIteration = 0;
            return bestSulotion;

        }

        /*TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxWeldOperation weldOperation, TxRobot robot)
         * 在当前选择计算的焊接轨迹中增加过度点；
         * 1. name:定义过度点的名称；
         * 2. p：过度点的坐标值；
         * 3. weldOperation：计算或增加过度点的轨迹；
         * 4. robot-当前计算和规划路径的机器人；
         * 每个新增加的过度点都定义了相对应的Tooling设备，Tooling的角度值，以及过度点的运动类型，PTP or Line
         * 
         * This code snippet describes adding via points (intermediate points) to a welding trajectory. Let's break down each point:
         * name (1): This assigns a unique identifier or name to the via point. This is useful for tracking and debugging.
         * p (2): This represents the 3D Cartesian coordinates (x, y, z) of the via point in the robot's workspace.
         * weldOperation (3): This likely refers to the welding operation associated with the via point. This could include details like welding parameters (e.g., current, speed, etc.) or instructions for the robot's welding tool. The description implies that this element is either calculated to determine the via point or is used to add the via point to the existing welding trajectory.
         * robot (4): Similar to the previous example, this specifies the robot for which the path is being planned.
         * Tooling, Angle, and Motion Type (5): Each via point is further defined by:
         * Tooling: Specifies the end-effector or tool being used at this via point. This might be a specific welding torch or other tool attached to the robot.
         * Angle: This refers to the orientation of the tooling at the via point. Accurate orientation is critical for successful welding.
         * Motion Type (PTP or Line): This dictates the type of robot motion used to reach the via point
         * PTP (Point-to-Point): The robot moves directly to the point, ignoring the path taken. This is generally faster but less precise.
         * Line: The robot moves along a straight line to the point. This provides more control over the path but might be slower.
         */
        public static TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxWeldOperation weldOperation, TxRobot robot)
        {
            TxRoboticViaLocationOperationCreationData roboticViaLocationOperationCreationData = new TxRoboticViaLocationOperationCreationData(name, "", 2.0);

            TxVector tran = new TxVector(p.x, p.y, p.z);

            TxVector rot = new TxVector(p.rx, p.ry, p.rz);

            roboticViaLocationOperationCreationData.AbsoluteLocation = new TxTransformation(tran, rot, TxTransformation.TxRotationType.RPY_XYZ);

            TxRoboticViaLocationOperation RobFramepostLocation = weldOperation.CreateRoboticViaLocationOperation(roboticViaLocationOperationCreationData);
            TxRobotExternalAxisData[] externalAxisData = new TxRobotExternalAxisData[1];
            externalAxisData[0] = new TxRobotExternalAxisData();
            externalAxisData[0].Device = robot.MountedTools[0] as TxServoGun;
            externalAxisData[0].Joint = (robot.MountedTools[0] as TxServoGun).Joints.Last();
            externalAxisData[0].JointValue = p.Gun_Open;

            RobFramepostLocation.RobotExternalAxesData = externalAxisData;
            RobFramepostLocation.SetParameter(new TxRoboticIntParam("RRS_MOTION_TYPE", 2));


            return RobFramepostLocation;
        }

        static double RandomStep() => (new Random().NextDouble() - 0.5) * 2; //Generate the float data which is between -1 and +1; 

        /*static void optimizePathPointSTValue(ref point pathPoint, point pre_Via, point after_Via, TxRobot robot)
         * 采用random函数在pathpoint的附近寻找逆向解为4个的点位坐标，并更新到pathpoint中去；
         * 1 robot-当前计算和规划路径的机器人；
         * 2 pathPoint 当前计算的路径点的坐标值，并根据最优姿态修改并返回此路径点的坐标；
         * 3 pre_Via，after_Via 定义了前一个路径点和后一个路径点的坐标值，用于计算当此路径点坐标值优化后再次进行碰撞检测，确保优化后的pathpoint是有效的；
         * The function uses the random function to search for a new coordinate position near the pathPoint. This new point must have an inverse kinematic solution with 4 degrees of freedom. The coordinates of this new point then update the pathPoint.
         * robot (1): This refers to the robot for which the path is being calculated and planned；
         * pathPoint (2): This is the current path point's coordinate value. The function modifies this coordinate value based on an optimal pose (orientation and position) and returns the updated coordinates；
         * pre_Via, after_Via (3): These variables store the coordinates of the path point before and after the current pathPoint. They're used to perform collision detection after the pathPoint is optimized. This ensures that the optimized pathPoint is collision-free with respect to its neighboring points on the path."
         */
        public static void optimizePathPointSTValue(ref point pathPoint, point pre_Via, point after_Via, TxRobot robot)
        {

            double TranslatestepSize = 1;
            double rotateStepSize = 0.01;
            int maxIterations = 100;

            point UpdatedPathPoint = pathPoint;

            for(int i=0;i< maxIterations;i++)
            {
                UpdatedPathPoint.x = UpdatedPathPoint.x + RandomStep() * TranslatestepSize;
                UpdatedPathPoint.y = UpdatedPathPoint.y + RandomStep() * TranslatestepSize;
                UpdatedPathPoint.z = UpdatedPathPoint.z + RandomStep() * TranslatestepSize;

                List<double> resultendVEC = new List<double>();

                resultendVEC = TxRobotRRTConnect.getLocVectorafterRotateDeg(UpdatedPathPoint, new List<double> { RandomStep()* rotateStepSize, RandomStep() * rotateStepSize, RandomStep() * rotateStepSize*10 });

                UpdatedPathPoint.rx= resultendVEC[0];
                UpdatedPathPoint.ry = resultendVEC[1];
                UpdatedPathPoint.rz = resultendVEC[2];


                ArrayList Solutions = RobotKinemetix.robotInverseCal(robot, UpdatedPathPoint);

                if (Solutions.Count<4)
                    continue;

                if (!TxRobotRRTConnect.collisioncheckforSinglePoint(ref UpdatedPathPoint))
                    continue;

                if (!TxRobotRRTConnect.isValidforstepCorss(pre_Via, UpdatedPathPoint, out point p))
                    continue;


                if (!TxRobotRRTConnect.isValidforstepCorss( UpdatedPathPoint, after_Via,out p))
                    continue;



                pathPoint = UpdatedPathPoint;
                break;

            }




        }

    }
}
