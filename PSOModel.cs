using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using aco.common;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 粒子群模型
    /// </summary>
    public class PSOModel
    {
        private Random r = new Random();

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
        public PSOModel(PSOGoal goal, PSOGlobal psog) : this(psog)
        {
            this.Goal = goal;
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// 运动模型使用指定的惯性权重iw构造
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="iw">惯性权重ω</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, double iw)
        {
            this.Goal = goal;
            this.Global = psog;
            this.Motion = new PSOMotion(this.Global, iw);
            this.WeightStrategy = new WeightStrategy(this.Global);
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="motion">粒子运动模型</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, PSOMotion motion)
        {
            this.Goal = goal;
            this.Global = psog;
            this.Motion = motion;
            this.WeightStrategy = new WeightStrategy(this.Global);
        }

        /// <summary>
        /// 粒子群模型构造函数
        /// </summary>
        /// <param name="goal">目标函数</param>
        /// <param name="psog">全局对象</param>
        /// <param name="motion">粒子运动模型</param>
        /// <param name="strategy">权重策略</param>
        public PSOModel(PSOGoal goal, PSOGlobal psog, PSOMotion motion, WeightStrategy strategy) : this(goal, psog, motion)
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
        /// 初始化粒子群的速度和位置
        /// </summary>
        /// <param name="cv">辅助验证类对象</param>
        /// <returns>初始化的粒子群列表</returns>
        private List<Particle> Init(ConstraintVerification cv = null)
        {
            List<Particle> pList = new List<Particle>();
            if (this.Global != null && this.Global.ParticleNum > 0)
            {
                int foodCnt = this.Global.FoodNum;
                for (int i = 0; i < this.Global.ParticleNum; i++)
                {
                    Particle p = new Particle(this.Global);
                    bool flag = false;

                    //确保粒子位置的合法性
                    while (!flag)
                    {
                        //重置位置值
                        for (int k = 0; k < p.Position.Length; k++)
                        {
                            p.Position[k] = 0;
                        }
                        //开始赋值
                        for (int j = 0; j < foodCnt - 1; j++)
                        {
                            double currPosSum = p.Position.Sum();
                            p.Velocity[j] = r.NextDouble() * p.MaxVelocity;
                            p.Position[j] = r.NextDouble(this.Global.SearchRangeLowerLimit, this.Global.SearchRangeUpperLimit - currPosSum);
                        }
                        p.Velocity[foodCnt - 1] = r.NextDouble() * p.MaxVelocity;
                        p.Position[foodCnt - 1] = this.Global.SearchRangeUpperLimit - p.Position.Sum();
                        flag = IsValid(p, cv);
                    }
                    //初次,当前位置即为最优位置
                    Array.ConstrainedCopy(p.Position, 0, p.BestPosition, 0, p.Position.Length);
                    pList.Add(p);
                }
                //计算初始的全局最优解
                this.UpdateGlobalBest(pList);
            }
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

                //模拟各粒子寻找食物的运动
                if (this.Global.Iterations > 0)
                {
                    //寻找次数
                    int cnt = this.Global.Iterations;

                    for (int i = 0; i < cnt; i++)
                    {
                        //第i次寻找
                        for (int j = 0; j < pList.Count; j++)
                        {
                            //第j个粒子
                            var p = pList[j];
                            //寻找食物,更新粒子速度和位置
                            Particle np = this.Motion.SearchFood(p);

                            #region 测试
                            //double psum = np.Position.Sum();
                            //if (psum > 3003 || psum < 2997)
                            //{
                            //    string[] pstrs = np.Position.ToList().Select(tp => tp.ToString()).ToArray();
                            //    string nps = string.Format("[{0}.{1}] sum({2}) = {3}", i, j, string.Join(",", pstrs), psum);
                            //}
                            #endregion

                            //验证有效性
                            bool flag = IsValid(np, cv);
                            //更新该粒子历史最优解
                            if (flag)
                            {
                                this.UpdateParticleHistoryBest(ref np);
                            }
                            pList[j] = np;
                        }
                        //更新惯性权重
                        double niw = this.WeightStrategy.GetValue(i);
                        this.Motion.UpdateInertiaWeight(niw);

                        //更新全局最优解
                        this.UpdateGlobalBest(pList);
                    }

                }
                result = new PSOSolution(this).GetResult();
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
