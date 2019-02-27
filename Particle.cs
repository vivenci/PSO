using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 粒子类
    /// </summary>
    public class Particle
    {
        private int foodNum;

        public Particle(int num)
        {
            this.foodNum = num;
            this.MaxVelocity = PSOGlobal.DefaultParticleMaxVelocity;
            this.Init();
        }

        public Particle(PSOGlobal g)
        {
            this.foodNum = g.FoodNum;
            this.MaxVelocity = PSOGlobal.DefaultParticleMaxVelocity;
            this.Init();
            //this.CheckFun = g.CheckFun;
        }

        public Particle(PSOGlobal g, double maxV) : this(g)
        {
            this.MaxVelocity = maxV;
        }

        /// <summary>
        /// 粒子在各维度上的速度
        /// </summary>
        public double[] Velocity
        {
            get;
            set;
        }

        /// <summary>
        /// 粒子在各维度上的位置
        /// </summary>
        public double[] Position
        {
            get;
            set;
        }

        /// <summary>
        /// 粒子的最优位置
        /// </summary>
        public double[] BestPosition
        {
            get;
            set;
        }

        /// <summary>
        /// 粒子的最大运动速度(须为正数)
        /// </summary>
        public double MaxVelocity { get; set; }

        /// <summary>
        /// 检测粒子当前位置是否越界
        /// </summary>
        /// <param name="cv">检测类的实例</param>
        /// <returns>是否越界</returns>
        public bool CheckPosition(ConstraintVerification cv)
        {
            if (cv != null)
            {
                return cv.Verification(this.Position);
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// 检测粒子当前位置是否越界的方法
        /// (从外部根据实际情况定义)
        /// 
        /// double[]:粒子当前位置
        /// object:额外引入的对象,包含辅助检测的属性
        /// boo1:是否检测通过
        /// </summary>
        //public Func<double[], object, bool> CheckFun
        //{
        //    get;
        //    set;
        //}

        private void Init()
        {
            if (this.foodNum > 0)
            {
                this.Velocity = new double[foodNum];
                this.Position = new double[foodNum];
                this.BestPosition = new double[foodNum];
            }
        }
    }
}
