using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using aco.common;
using aco.common.Logs;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 粒子群模型
    /// </summary>
    [Serializable]
    public class PSOModel
    {
        private Random r = new Random();
        FileLogger logger = FileLogger.Create(LoggerSetting.DefaultSystemLogName);
        //初始化每个粒子时的最大尝试次数
        private readonly int RetryCount = 1000;

        /// <summary>
        /// 粒子群模型默认构造函数
        /// PSOGoal,PSOGlobal,PSOMotion需在后续过程赋值
        /// </summary>
        public PSOModel()
        {
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// PSOGlobal,PSOMotion,权重策略需在后续过程赋值
        /// </summary>
        /// <param name="goal">粒子群目标函数</param>
        public PSOModel(PSOGoal goal)
        {
            this.Goal = goal;
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// 粒子群运动模型使用默认构造
        /// 目标函数PSOGoal需在后续过程赋值
        /// </summary>
        /// <param name="psog">全局对象</param>
        public PSOModel(PSOGlobal psog)
        {
            this.Global = psog;
            this.Motion = new PSOMotion(this.Global);
            this.WeightStrategy = new WeightStrategy(this.Global);
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// 粒子群运动模型使用默认构造
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="posList">预先指定的粒子初始位置列表</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, List<double[]> posList = null) : this(psog)
        {
            this.Goal = goal;
            this.ParticularPositions = posList;
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// 运动模型使用指定的惯性权重iw构造
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="iw">惯性权重ω</param>
        /// <param name="posList">预先指定的粒子初始位置列表</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, double iw, List<double[]> posList = null)
        {
            this.Goal = goal;
            this.Global = psog;
            this.Motion = new PSOMotion(this.Global, iw);
            this.ParticularPositions = posList;
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="motion">粒子运动模型</param>
        /// <param name="posList">预先指定的粒子初始位置列表</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, PSOMotion motion, List<double[]> posList = null)
        {
            this.Goal = goal;
            this.Global = psog;
            this.Motion = motion;
            this.WeightStrategy = new WeightStrategy(this.Global);
            this.ParticularPositions = posList;
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="motion">粒子运动模型</param>
        /// <param name="strategy">权重策略</param>
        /// <param name="posList">预先指定的粒子初始位置列表</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, PSOMotion motion, WeightStrategy strategy, List<double[]> posList = null) : this(goal, psog, motion, posList)
        {
            this.WeightStrategy = strategy;
        }

        /// <summary>
        /// 目标函数
        /// </summary>
        public PSOGoal Goal
        {
            get;
            set;
        }

        /// <summary>
        /// 全局对象
        /// </summary>
        public PSOGlobal Global
        {
            get;
            set;
        }

        /// <summary>
        /// 粒子运动模型
        /// </summary>
        public PSOMotion Motion
        {
            get;
            set;
        }

        /// <summary>
        /// 权重策略
        /// </summary>
        public WeightStrategy WeightStrategy
        {
            get;
            set;
        }

        /// <summary>
        /// 预先指定的粒子初始位置列表
        /// </summary>
        public List<double[]> ParticularPositions
        {
            get;
            set;
        }

        /// <summary>
        /// 初始化粒子群的速度和位置
        /// </summary>
        /// <param name="cv">辅助验证类对象</param>
        /// <returns>初始化的粒子群列表</returns>
        private List<Particle> Init(ConstraintVerification cv = null)
        {
            List<Particle> pList = new List<Particle>();
            if (this.Global != null && this.Global.ParticleNum > 0)
            {
                logger.WriteLine("开始初始化.");

                int foodCnt = this.Global.FoodNum;
                var fixDict = this.Global.FixPositionDict;
                try
                {
                    for (int i = 0; i < this.Global.ParticleNum; i++)
                    {
                        logger.WriteLine(string.Format("正在初始化第{0}/{1}个粒子.", (i + 1), this.Global.ParticleNum));

                        Particle p = new Particle(this.Global);
                        bool flag = false;

                        int tryIdx = 0;
                        //确保粒子位置的合法性
                        while (!flag && tryIdx < this.RetryCount)
                        {
                            //重置位置值
                            for (int k = 0; k < p.Position.Length; k++)
                            {
                                p.Position[k] = 0;
                            }
                            //开始赋值
                            for (int j = 0; j < foodCnt; j++)
                            {
                                //使用初始化粒子位置列表
                                if (this.ParticularPositions != null && i < this.ParticularPositions.Count)
                                {
                                    p.Velocity[j] = r.NextDouble() * p.MaxVelocity;
                                    p.Position[j] = this.ParticularPositions[i][j];
                                }
                                //判断该食物位置是否固定
                                else if (fixDict != null && fixDict.ContainsKey(j))
                                {
                                    p.Velocity[j] = 0;
                                    p.Position[j] = fixDict[j];
                                }
                                else
                                {
                                    double currPosSum = p.Position.Sum();
                                    p.Velocity[j] = r.NextDouble() * p.MaxVelocity;
                                    if (j < foodCnt - 1)
                                    {
                                        p.Position[j] = r.NextDouble(this.Global.SearchRangeLowerLimit, this.Global.SearchRangeUpperLimit - currPosSum);
                                    }
                                    //最后一个组分
                                    else
                                    {
                                        p.Position[j] = this.Global.SearchRangeUpperLimit - p.Position.Sum();
                                    }
                                }
                            }
                            //判断最后一个组分
                            //if (fixDict != null && fixDict.ContainsKey(foodCnt - 1))
                            //{
                            //    p.Velocity[foodCnt - 1] = 0;
                            //    p.Position[foodCnt - 1] = fixDict[foodCnt - 1];
                            //}
                            //else
                            //{
                            //    p.Velocity[foodCnt - 1] = r.NextDouble() * p.MaxVelocity;
                            //    p.Position[foodCnt - 1] = this.Global.SearchRangeUpperLimit - p.Position.Sum();
                            //}



                            flag = IsValid(p, cv);
                            tryIdx++;
                        }

                        //如果粒子位置合法,添加至列表
                        if (flag)
                        {
                            //初次,当前位置即为最优位置
                            Array.ConstrainedCopy(p.Position, 0, p.BestPosition, 0, p.Position.Length);
                            pList.Add(p);
                        }
                        else
                        {
                            logger.WriteLine(string.Format("第{0}轮搜索: 尝试寻找{1}次未找到合适位置的粒子,放弃本次搜索,将开始下一轮搜索.", i, this.RetryCount));
                        }
                    }
                    //计算初始的全局最优解
                    if (pList != null && pList.Count > 0)
                    {
                        this.UpdateGlobalBest(pList);
                    }
                }
                catch (Exception ex)
                {
                    logger.WriteLine(ex);
                }

            }
            //更新粒子数目为列表中的实际粒子树
            this.Global.ParticleNum = pList.Count;
            return pList;
        }

        /// <summary>
        /// 更新粒子历史最优解
        /// </summary>
        /// <param name="p">要更新的粒子</param>
        public void UpdateParticleHistoryBest(ref Particle p)
        {
            if (this.Goal != null && this.Global != null)
            {
                //根据粒子当前位置计算结果
                double gCurr = this.Goal.Calc(p.Position);
                //根据粒子粒子历史最优位置计算结果
                double gBest = this.Goal.Calc(p.BestPosition);

                //按极小值优化
                if (this.Global.Direction == OptimizationDirection.MinValue)
                {
                    if (gCurr < gBest)
                    {
                        Array.ConstrainedCopy(p.Position, 0, p.BestPosition, 0, p.Position.Length);
                    }
                }
                //按极大值优化
                else if (this.Global.Direction == OptimizationDirection.MaxValue)
                {
                    if (gCurr > gBest)
                    {
                        Array.ConstrainedCopy(p.Position, 0, p.BestPosition, 0, p.Position.Length);
                    }
                }
            }
        }

        /// <summary>
        /// 计算粒子群全局最优解,并记入PSOGlobal
        /// </summary>
        /// <param name="pList">当前的粒子群</param>
        public void UpdateGlobalBest(List<Particle> pList)
        {
            if (this.Goal != null && this.Global != null)
            {
                List<Tuple<double[], double>> rDict = new List<Tuple<double[], double>>();
                for (int i = 0; i < pList.Count; i++)
                {
                    var p = pList[i];
                    double ret = this.Goal.Calc(p.BestPosition);
                    rDict.Add(new Tuple<double[], double>(p.BestPosition, ret));
                }
                //按计算结果升序排列
                rDict = rDict.Where(t => t.Item2 > 0).ToList();//过滤掉0解
                rDict = rDict.OrderBy(t => t.Item2).ToList();
                //是否首次赋值
                bool isFirst = true;
                if (this.Global.GlobalBestPosition != null && this.Global.GlobalBestPosition.Length > 0)
                {
                    isFirst = false;
                }

                if (isFirst)
                {
                    this.Global.GlobalBestPosition = new double[this.Global.FoodNum];
                }
                double bestVal = isFirst ? double.NaN : this.Goal.Calc(this.Global.GlobalBestPosition);

                //按照优化方向给出最优解
                if (this.Global.Direction == OptimizationDirection.MinValue)
                {
                    if (isFirst || rDict[0].Item2 < bestVal)
                    {
                        Array.ConstrainedCopy(rDict[0].Item1, 0, this.Global.GlobalBestPosition, 0, this.Global.FoodNum);
                    }
                }
                else if (this.Global.Direction == OptimizationDirection.MaxValue)
                {
                    if (isFirst || rDict[0].Item2 > bestVal)
                    {
                        Array.ConstrainedCopy(rDict[rDict.Count - 1].Item1, 0, this.Global.GlobalBestPosition, 0, this.Global.FoodNum);
                    }
                }
            }
        }

        /// <summary>
        /// 粒子群算法模型求解
        /// </summary>
        /// <param name="cv">辅助校验对象</param>
        /// <returns>求解结果</returns>
        public PSOResult Solve(ConstraintVerification cv = null)
        {
            PSOResult result = null;

            if (this.Global != null && this.Goal != null && this.Motion != null)
            {

                //初始化粒子群
                var pList = this.Init(cv);

                //初始化粒子列表为空
                if (pList == null || pList.Count == 0)
                {
                    logger.WriteLine("初始化失败,未能开始搜索.");
                    return null;
                }

                //模拟各粒子寻找食物的运动
                if (this.Global.Iterations > 0)
                {
                    //寻找次数
                    int cnt = this.Global.Iterations;

                    for (int i = 0; i < cnt; i++)
                    {
                        try
                        {
                            logger.WriteLine(string.Format("开始第{0}次搜索:", i));
                            //第i次寻找
                            for (int j = 0; j < pList.Count; j++)
                            {
                                //第j个粒子
                                var p = pList[j];
                                //寻找食物,更新粒子速度和位置
                                Particle np = this.Motion.SearchFood(p);

                                logger.WriteLine(string.Format("第{0}个粒子:", (j + 1)));
                                //验证有效性
                                bool flag = IsValid(np, cv);
                                //更新该粒子历史最优解
                                if (flag)
                                {
                                    this.UpdateParticleHistoryBest(ref np);
                                }
                                pList[j] = np;
                            }
                            logger.WriteCutOffLine();

                            //更新惯性权重
                            double niw = this.WeightStrategy.GetValue(i);
                            this.Motion.UpdateInertiaWeight(niw);

                            //更新全局最优解
                            this.UpdateGlobalBest(pList);

                            //定时回收内存
                            if (i % 100 == 0)
                            {
                                GC.Collect();
                            }
                        }
                        catch (Exception ex)
                        {
                            logger.WriteLine(string.Format("在第{0}次搜索中发生异常:", i))
                                .WriteLine(ex);
                        }
                    }
                }

                result = new PSOSolution(this).GetResult();
                logger.WriteLine(string.Format("{0}\r\n运算完成.", result.ToString()));
                logger.WriteCutOffLine();
            }
            return result;
        }



        /// <summary>
        /// 判断粒子运动位置是否合法
        /// </summary>
        /// <param name="p">粒子</param>
        /// <param name="cv">辅助校验类对象</param>
        /// <returns>是否合法</returns>
        private bool IsValid(Particle p, ConstraintVerification cv = null)
        {
            bool flag = false;
            //如果t不为空,要对粒子位置进行合法性判定
            if (cv != null)
            {
                flag = p.CheckPosition(cv);
            }
            else
            {
                flag = true;
            }
            return flag;
        }

    }

}
