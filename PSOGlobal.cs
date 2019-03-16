using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    [Serializable]
    public class PSOGlobal
    {
        private readonly int DEFAULT_ITERATIONS = 2000;
        //食物数目默认值
        private int DEFAULT_FOOD_NUM = 3;
        //粒子数目默认值
        private int DEFAULT_PARTICLE_NUM = 30;
        private double DEFAULT_SEARCH_LOWER_LIMIT = 0;
        private double DEFAULT_SEARCH_UPPER_LIMIT = 1000;
        //惯性权重
        private double DEFAULT_WEIGHT_MIN = 0.1;
        private double DEFAULT_WEIGHT_MAX = 1;
        //默认粒子运动最大速度
        private static double DEFAULT_PARTICLE_MAX_VELOCITY = 2;
        //误差(比例值)
        private static double DEFAULT_ERR = 0.001;

        public PSOGlobal()
        {
            this.FoodNum = DEFAULT_FOOD_NUM;
            this.ParticleNum = DEFAULT_PARTICLE_NUM;
            this.SearchRangeLowerLimit = DEFAULT_SEARCH_LOWER_LIMIT;
            this.SearchRangeUpperLimit = DEFAULT_SEARCH_UPPER_LIMIT;
            this.Direction = OptimizationDirection.MinValue;
            this.Iterations = DEFAULT_ITERATIONS;
            this.MinWeight = DEFAULT_WEIGHT_MIN;
            this.MaxWeight = DEFAULT_WEIGHT_MAX;
        }

        /// <summary>
        /// 构造模型全局配置
        /// </summary>
        /// <param name="foodNum">食物数目(目标数目)</param>
        /// <param name="lowerLimit">粒子可搜索的最小位置</param>
        /// <param name="upperLimit">粒子可搜索的最大位置</param>
        public PSOGlobal(int foodNum, double lowerLimit, double upperLimit) : this()
        {
            this.FoodNum = foodNum;
            this.SearchRangeLowerLimit = lowerLimit;
            this.SearchRangeUpperLimit = upperLimit;
        }

        /// <summary>
        /// 构造模型全局配置
        /// </summary>
        /// <param name="foodNum">食物数目(目标数目)</param>
        /// <param name="pNum">用于搜索的粒子数目</param>
        /// <param name="lowerLimit">粒子可搜索的最小位置</param>
        /// <param name="upperLimit">粒子可搜索的最大位置</param>
        public PSOGlobal(int foodNum, int pNum, double lowerLimit, double upperLimit) : this(foodNum, lowerLimit, upperLimit)
        {
            this.ParticleNum = pNum;
        }

        /// <summary>
        /// 构造模型全局配置
        /// </summary>
        /// <param name="foodNum">食物数目(目标数目)</param>
        /// <param name="pNum">用于搜索的粒子数目</param>
        /// <param name="lowerLimit">粒子可搜索的最小位置</param>
        /// <param name="upperLimit">粒子可搜索的最大位置</param>
        /// <param name="iteration">进行搜索的次数</param>
        public PSOGlobal(int foodNum, int pNum, double lowerLimit, double upperLimit, int iteration) : this(foodNum, pNum, lowerLimit, upperLimit)
        {
            this.Iterations = iteration;
        }

        /// <summary>
        /// 构造模型全局配置
        /// </summary>
        /// <param name="foodNum">食物数目(目标数目)</param>
        /// <param name="pNum">用于搜索的粒子数目</param>
        /// <param name="lowerLimit">粒子可搜索的最小位置</param>
        /// <param name="upperLimit">粒子可搜索的最大位置</param>
        /// <param name="iteration">进行搜索的次数</param>
        /// <param name="minW">惯性权重最小值</param>
        /// <param name="maxW">惯性权重最大值</param>
        public PSOGlobal(int foodNum, int pNum, double lowerLimit, double upperLimit, int iteration, double minW, double maxW) : this(foodNum, pNum, lowerLimit, upperLimit, iteration)
        {
            this.MinWeight = minW;
            this.MaxWeight = maxW;
        }

        /// <summary>
        /// 构造模型全局配置
        /// </summary>
        /// <param name="foodNum">食物数目(目标数目)</param>
        /// <param name="pNum">用于搜索的粒子数目</param>
        /// <param name="lowerLimit">粒子可搜索的最小位置</param>
        /// <param name="upperLimit">粒子可搜索的最大位置</param>
        /// <param name="iteration">进行搜索的次数</param>
        /// <param name="od">优化方向</param>
        public PSOGlobal(int foodNum, int pNum, double lowerLimit, double upperLimit, int iteration, OptimizationDirection od) : this(foodNum, pNum, lowerLimit, upperLimit, iteration)
        {
            this.Direction = od;
        }

        /// <summary>
        /// 构造模型全局配置
        /// </summary>
        /// <param name="foodNum">食物数目(目标数目)</param>
        /// <param name="pNum">用于搜索的粒子数目</param>
        /// <param name="lowerLimit">粒子可搜索的最小位置</param>
        /// <param name="upperLimit">粒子可搜索的最大位置</param>
        /// <param name="iteration">进行搜索的次数</param>
        /// <param name="minW">惯性权重最小值</param>
        /// <param name="maxW">惯性权重最大值</param>
        /// <param name="od">优化方向</param>
        public PSOGlobal(int foodNum, int pNum, double lowerLimit, double upperLimit, int iteration, double minW, double maxW, OptimizationDirection od) : this(foodNum, pNum, lowerLimit, upperLimit, iteration, minW, maxW)
        {
            this.Direction = od;
        }

        /// <summary>
        /// 默认粒子最大运动速度
        /// </summary>
        public static double DefaultParticleMaxVelocity
        {
            get
            {
                return DEFAULT_PARTICLE_MAX_VELOCITY;
            }
            set
            {
                DEFAULT_PARTICLE_MAX_VELOCITY = value;
            }
        }

        /// <summary>
        /// 默认允许误差
        /// </summary>
        public static double DefaultErr
        {
            get
            {
                return DEFAULT_ERR;
            }
            set
            {
                DEFAULT_ERR = value;
            }
        }


        /// <summary>
        /// 食物数量
        /// </summary>
        public int FoodNum
        {
            get;
            private set;
        }

        /// <summary>
        /// 粒子的数量
        /// </summary>
        public int ParticleNum { get; set; }

        /// <summary>
        /// 搜索范围
        /// 各粒子的位置最大值
        /// </summary>
        public double SearchRangeUpperLimit { get; set; }

        /// <summary>
        /// 搜索范围
        /// 各粒子的位置最小值
        /// </summary>
        public double SearchRangeLowerLimit { get; set; }

        /// <summary>
        /// 惯性权重最大值
        /// </summary>
        public double MaxWeight { get; set; }

        /// <summary>
        /// 惯性权重最小值
        /// </summary>
        public double MinWeight { get; set; }

        /// <summary>
        /// 最大迭代次数
        /// </summary>
        public int Iterations { get; set; }

        /// <summary>
        /// 优化方向
        /// </summary>
        public OptimizationDirection Direction { get; set; }

        /// <summary>
        /// 全局最优位置
        /// </summary>
        public double[] GlobalBestPosition { get; set; }

        /// <summary>
        /// 用来描述固定食物位置状态的字典
        /// [key]:固定食物的索引
        /// [value]:固定食物的固定位置
        /// </summary>
        public Dictionary<int, double> FixPositionDict { get; set; }

    }

    /// <summary>
    /// 优化方向
    /// </summary>
    public enum OptimizationDirection
    {
        MinValue = 0,
        MaxValue
    }
}
