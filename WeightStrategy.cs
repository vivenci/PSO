using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 惯性权重运算策略
    /// </summary>
    [Serializable]
    public class WeightStrategy
    {
        private PSOGlobal g;
        private Random r = new Random();

        public WeightStrategy(PSOGlobal psog)
        {
            this.g = psog;
            this.Strategy = StrategyType.Typical;
        }

        public WeightStrategy(PSOGlobal psog, StrategyType strategy)
        {
            this.g = psog;
            this.Strategy = strategy;
        }

        public StrategyType Strategy
        {
            get;
            set;
        }

        /// <summary>
        /// 根据不同策略计算第i次运算时的权重值
        /// </summary>
        /// <param name="i">迭代次数索引</param>
        /// <returns>权重值</returns>
        public double GetValue(int i)
        {
            double wi = 1;

            if (this.g != null)
            {
                switch (this.Strategy)
                {
                    case StrategyType.Typical:
                        wi = g.MaxWeight - ((g.MaxWeight - g.MinWeight) * (i + 1) / g.Iterations);
                        break;
                    case StrategyType.LinearDifferential:
                        wi = g.MaxWeight - ((g.MaxWeight - g.MinWeight) * Math.Pow((i + 1), 2) / Math.Pow(g.Iterations, 2));
                        break;
                    case StrategyType.FITD:
                        double tmp = (i + 1) / g.Iterations;
                        if (tmp >= 0 && tmp <= 0.5)
                        {
                            wi = tmp + 0.4;
                        }
                        else if (tmp > 0.5 && tmp <= 1)
                        {
                            wi = 1.4 - tmp;
                        }
                        break;
                    case StrategyType.Random:
                        wi = (r.NextDouble() + 0.5) / 2;
                        break;
                    default:
                        wi = g.MaxWeight - ((g.MaxWeight - g.MinWeight) * (i + 1) / g.Iterations);
                        break;
                }
            }
            return wi;
        }

    }

    public enum StrategyType
    {
        Typical,            //典型线性递减策略
        LinearDifferential, //线性微分递减策略
        FITD,               //先增后减策略
        Random              //随机调整策略
    }
}
