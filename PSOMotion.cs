using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 粒子运动模型
    /// </summary>
    public class PSOMotion
    {
        private PSOGlobal g;
        private Random ran1 = new Random();
        private Random ran2 = new Random();

        public PSOMotion(PSOGlobal psog)
        {
            this.g = psog;
            this.InertiaWeight = 1;
            this.AccFactorA = 2;
            this.AccFactorB = 2;
        }

        public PSOMotion(PSOGlobal psog, double iw) : this(psog)
        {
            this.InertiaWeight = iw;
        }

        public PSOMotion(PSOGlobal psog, double iw, double fa, double fb)
        {
            this.g = psog;
            this.InertiaWeight = iw;
            this.AccFactorA = fa;
            this.AccFactorB = fb;
        }

        /// <summary>
        /// 初始惯性权重ω
        /// 取值范围[0,1]
        /// ω值变大,增强全局寻优,减弱局部寻优;
        /// ω值变小,增强局部寻优,减弱全局寻优;
        /// 通常当粒子的vmax较小时,取ω趋近于1,可采用不同策略动态调整权重值
        /// </summary>
        public double InertiaWeight { get; set; }

        /// <summary>
        /// 加速系统c1
        /// 取值范围[0,4]
        /// 常用值2
        /// </summary>
        public double AccFactorA { get; set; }

        /// <summary>
        /// 加速系统c2
        /// 取值范围[0,4]
        /// 通常与c1相等
        /// </summary>
        public double AccFactorB { get; set; }

        public void UpdateInertiaWeight(double niw)
        {
            this.InertiaWeight = niw;
        }

        /// <summary>
        /// 粒子搜索新食物,获取其新的速度和位置
        /// </summary>
        /// <param name="p">粒子状态</param>
        public Particle SearchFood(Particle p)
        {
            Particle npc = new Particle(this.g, p.MaxVelocity);
            double[] npArr = new double[g.FoodNum];
            double[] nvArr = new double[g.FoodNum];
            double[] nbpArr = new double[g.FoodNum];

            Array.ConstrainedCopy(p.BestPosition, 0, nbpArr, 0, g.FoodNum);
            npc.BestPosition = nbpArr;

            if (p != null)
            {
                double[] vs = p.Velocity;
                double[] ps = p.Position;

                for (int i = 0; i < g.FoodNum; i++)
                {
                    double nv = GetNewVelocity(vs[i], p.BestPosition[i], g.GlobalBestPosition[i], p.Position[i], p.MaxVelocity);
                    double np = GetNewPosition(p.Position[i], nv);

                    nvArr[i] = nv;
                    npArr[i] = np;
                }
            }
            npc.Velocity = nvArr;
            npc.Position = npArr;
            return npc;
        }

        /// <summary>
        /// 计算粒子最新速度
        /// </summary>
        /// <param name="v">粒子某维度上的当前速度</param>
        /// <param name="pBest">该粒子某维度上的历史最优位置</param>
        /// <param name="gBest">某维度上的全局最优位置</param>
        /// <param name="x">粒子某维度上的当前位置</param>
        /// <param name="maxV">粒子的最大速度</param>
        /// <returns>粒子最新速度</returns>
        private double GetNewVelocity(double v, double pBest, double gBest, double x, double maxV)
        {
            double nv = this.InertiaWeight * v + this.AccFactorA * ran1.NextDouble() * (pBest - x) + this.AccFactorB * ran2.NextDouble() * (gBest - x);
            if (Math.Abs(nv) > Math.Abs(maxV))
            {
                nv = (nv > 0) ? Math.Abs(maxV) : (Math.Abs(maxV) * (-1));
            }
            return nv;
        }

        /// <summary>
        /// 计算粒子最新位置
        /// </summary>
        /// <param name="x">粒子某维度上的当前位置</param>
        /// <param name="nv">计算出的粒子某维度上的新速度</param>
        /// <returns>粒子最新位置</returns>
        private double GetNewPosition(double x, double nv)
        {
            double np = x + nv;
            np = Math.Min(g.SearchRangeUpperLimit, np);
            np = Math.Max(g.SearchRangeLowerLimit, np);
            return np;
        }
    }
}
